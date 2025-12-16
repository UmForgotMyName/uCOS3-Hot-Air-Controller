/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  * @Modified by    : Group 5 ENGG*4420 Real-Time Systems
  ******************************************************************************
  */

#include "main.h"
#include "stm32f4xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "gpio.h"
#include "includes.h"

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

/* =========================================================
 *               Basic app configuration
 * =======================================================*/
#define APP_CFG_GUICLK_TASK_STK_SIZE   (256u)
#define APP_CFG_GUICLK_TASK_PRIO       (10u)
#define APP_CFG_STARTUP_TASK_STK_SIZE   (256u)
#define APP_CFG_STARTUP_TASK_PRIO       (4u)
#define APP_CFG_COMM_TASK_STK_SIZE      (256u)
#define APP_CFG_COMM_TASK_PRIO          (12u)
#define APP_CFG_BTN_TASK_STK_SIZE       (192u)
#define APP_CFG_BTN_TASK_PRIO           (11u)
#define APP_CFG_PLANT_TASK_STK_SIZE     (256u)
#define APP_CFG_PLANT_TASK_PRIO         (8u)
#define APP_CFG_CONTROL_TASK_STK_SIZE   (256u)
#define APP_CFG_CONTROL_TASK_PRIO       (7u)
#define APP_CFG_CLOCK_TASK_STK_SIZE     (192u)
#define APP_CFG_CLOCK_TASK_PRIO         (13u)
#define APP_CFG_GUI_TASK_STK_SIZE       (512u)
#define APP_CFG_GUI_TASK_PRIO           (10u)

/* =========================================================
 *               LCD theme + simple colors
 * =======================================================*/
#define COL_BG         LCD_COLOR_WHITE
#define COL_TEXT       LCD_COLOR_BLACK
#define COL_SUBTEXT    LCD_COLOR_DARKGRAY
#define COL_PV         LCD_COLOR_BLUE
#define COL_SP         LCD_COLOR_RED
#define COL_GRAPH_BG   LCD_COLOR_WHITE

/* -------------------- Graph helpers (history + drawing) -------------------- */
#define GUI_MAX_SAMPLES   240

static float    gui_pv_hist[GUI_MAX_SAMPLES];
static float    gui_sp_hist[GUI_MAX_SAMPLES];
static uint16_t gui_hist_n = 0;

/* ===== Voltage axis range for mapping ===== */
#define GUI_VMIN   0.0f
#define GUI_VMAX   5.0f

/* Clamp Y to rect (y0..y0+h-1) */
static inline int clamp_y_px(int y, int y0, int h)
{
    if (y < y0) {
        return y0;
    }
    int yb = y0 + h - 1;
    if (y > yb) {
        return yb;
    }
    return y;
}

/* Map voltage [VMIN,VMAX] to pixel Y inside rect (y0..y0+h-1) */
static int map_v_to_y(float v, int y0, int h)
{
    if (v < GUI_VMIN) {
        v = GUI_VMIN;
    }
    if (v > GUI_VMAX) {
        v = GUI_VMAX;
    }
    float t = (v - GUI_VMIN) / (GUI_VMAX - GUI_VMIN);
    return y0 + h - 1 - (int)(t * (float)h + 0.5f);
}

/* Push newest PV and SP samples into a rolling window */
static void GUI_PushSample(float pv_v, float sp_v)
{
    // keep a simple rolling buffer of recent samples (shift when full)
    if (gui_hist_n < GUI_MAX_SAMPLES) {
        gui_pv_hist[gui_hist_n] = pv_v;
        gui_sp_hist[gui_hist_n] = sp_v;
        gui_hist_n++;
    } else {
        memmove(&gui_pv_hist[0], &gui_pv_hist[1], (GUI_MAX_SAMPLES-1)*sizeof(float));
        memmove(&gui_sp_hist[0], &gui_sp_hist[1], (GUI_MAX_SAMPLES-1)*sizeof(float));
        gui_pv_hist[GUI_MAX_SAMPLES-1] = pv_v;
        gui_sp_hist[GUI_MAX_SAMPLES-1] = sp_v;
    }
}

/* Draw sparkline on graph */
static void GUI_DrawSparkline(const float *hist, uint16_t n,
                              int x0, int y0, int w, int h, uint32_t color)
{
    if (n < 2 || w <= 1) {
        return;
    }

    // Visible window logic stays the same
    uint16_t vis;
    if (n < GUI_MAX_SAMPLES) {
        vis = n;
    } else {
        vis = GUI_MAX_SAMPLES;
    }
    uint16_t start;
    if (n > GUI_MAX_SAMPLES) {
        start = (uint16_t)(n - GUI_MAX_SAMPLES);
    } else {
        start = 0;
    }

    // If we have more samples than pixels, decimate so we draw ≤ 1 sample per pixel
    uint16_t stride = 1;
    if (vis > (uint16_t)w) {
        // ceil(vis / w) without floats: (vis + w - 1)/w
        stride = (uint16_t)((vis + (uint16_t)w - 1) / (uint16_t)w);
    }

    // Effective number of plotted points after decimation
    uint16_t pts = (uint16_t)((vis + stride - 1) / stride);
    if (pts < 2) {
        pts = 2;  /* ensure we draw at least a segment */
    }

    // use floating spacing across width so points span the whole graph
    float dx = (float)(w - 1) / (float)(pts - 1);

    BSP_LCD_SetTextColor(color);

    // First point
    uint16_t i0 = start;
    int px = x0;
    int py = map_v_to_y(hist[i0], y0, h);

    for (uint16_t j = 1; j < pts; j++) {
        uint16_t idx = (uint16_t)(start + j * stride);
        if (idx >= (uint16_t)(start + vis)) {
            idx = (uint16_t)(start + vis - 1);
        }

        int x = x0 + (int)(j * dx + 0.5f);
        if (x > x0 + w - 1) {
            x = x0 + w - 1;
        }

        int y = map_v_to_y(hist[idx], y0, h);

        // draw line between consecutive decimated points (clamped to rect)
        BSP_LCD_DrawLine(px, clamp_y_px(py, y0, h), x, clamp_y_px(y, y0, h));
        px = x; py = y;
    }
}

/* Horizontal setpoint line across the graph */
static void GUI_DrawRefLine(float sp_v, int x0, int y0, int w, int h)
{
    int yy = clamp_y_px(map_v_to_y(sp_v, y0, h), y0, h);
    BSP_LCD_SetTextColor(COL_SP);
    BSP_LCD_DrawLine(x0, yy, x0 + w - 1, yy);
}

/* =========================================================
 *               Shared state for control loop
 * =======================================================*/
typedef enum { MODE_MANUAL=0, MODE_AUTO=1 } ctrl_mode_t;

typedef struct {
    float pv_volt; // plant output (volts)
    float pv_degC; // plant output (deg C)
    float cv_volt; // controller output (V)
    float sp_degC; // setpoint (deg C)

    /* PID params */
    float Kc;
    float Ti_s;
    float Td_s;

    /* PID integrator/prev error */
    float e_prev;
    float e_int;

    ctrl_mode_t mode;
    float manual_cv_volt;

    uint32_t clock_s; // increments each second
} shared_t;

static shared_t g = {0}; // guarded by g_mutex
static OS_MUTEX g_mutex; // guard shared state
static OS_MUTEX lcd_mutex; // ensure only 1 GUI task touches LCD at a time

/* =========================================================
 *               RTOS objects & forward decls
 * =======================================================*/
void SystemClock_Config(void);

/* Tasks */
static void StartupTask (void *p_arg);
static void CommTask    (void *p_arg);
static void BtnTask     (void *p_arg);
static void GuiTask     (void *p_arg);
static void PlantTask   (void *p_arg);
static void ControlTask (void *p_arg);
static void ClockTask   (void *p_arg);
static void GuiClockTask(void *p_arg);


/* Helpers */
static float TempToVoltage(float temp);
static float VoltageToTemp(float volt);
static void  parse_command_line(const char *s);

/* TCBs & stacks */
static OS_TCB  StartupTaskTCB;  static CPU_STK StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE];
static OS_TCB  CommTaskTCB;     static CPU_STK CommTaskStk[APP_CFG_COMM_TASK_STK_SIZE];
static OS_TCB  BtnTaskTCB;      static CPU_STK BtnTaskStk[APP_CFG_BTN_TASK_STK_SIZE];
static OS_TCB  GuiTaskTCB;      static CPU_STK GuiTaskStk[APP_CFG_GUI_TASK_STK_SIZE];
static OS_TCB  PlantTaskTCB;    static CPU_STK PlantTaskStk[APP_CFG_PLANT_TASK_STK_SIZE];
static OS_TCB  ControlTaskTCB;  static CPU_STK ControlTaskStk[APP_CFG_CONTROL_TASK_STK_SIZE];
static OS_TCB  ClockTaskTCB;    static CPU_STK ClockTaskStk[APP_CFG_CLOCK_TASK_STK_SIZE];
static OS_TCB  GuiClockTaskTCB; static CPU_STK GuiClockTaskStk[APP_CFG_GUICLK_TASK_STK_SIZE];

/* Simple command queue */
static OS_Q CommQ;

/* BSP handles (for IRQ disables) */
extern LTDC_HandleTypeDef  hltdc;
extern DMA2D_HandleTypeDef hdma2d;

/* =========================================================
 *                        main function
 * =======================================================*/
int main(void)
{
    OS_ERR err;
    HAL_Init();

    BSP_ClkInit();
    BSP_IntInit();
    BSP_OS_TickInit();

    Mem_Init();
    CPU_IntDis();
    CPU_Init();
    Math_Init();

    OSInit(&err);
    if (err != OS_ERR_NONE) { while (1){} }

    App_OS_SetAllHooks();

    OSQCreate(&CommQ, "CommQ", 8, &err);
    if (err != OS_ERR_NONE) { while (1){} }

    OSMutexCreate(&g_mutex,  "Shared", &err);
    OSMutexCreate(&lcd_mutex,"LCD",    &err);

    /* Safe defaults */
    g.mode   = MODE_AUTO;
    g.sp_degC= 35.0f;
    g.Kc     = 1.0f;
    g.Ti_s   = 10.0f;
    g.Td_s   = 0.0f;

    OSTaskCreate(&StartupTaskTCB, "Startup", StartupTask, 0,
                 APP_CFG_STARTUP_TASK_PRIO,
                 &StartupTaskStk[0], StartupTaskStk[APP_CFG_STARTUP_TASK_STK_SIZE/10],
                 APP_CFG_STARTUP_TASK_STK_SIZE, 0, 0, 0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);
    if (err != OS_ERR_NONE) { while (1){} }

    OSStart(&err);
    while (1) {}
}

/* =========================================================
 *                     Startup task
 * =======================================================*/
static void StartupTask(void *p_arg)
{
    OS_ERR err; (void)p_arg;

    OS_TRACE_INIT();
    BSP_OS_TickEnable();

    MX_GPIO_Init();
    MX_USB_DEVICE_Init();

    printf("READY\r\n");

    BSP_LED_Init();

    /* ---------- Memory & graphics  ---------- */
    MX_FMC_Init();
    MX_DMA2D_Init();
    MX_I2C3_Init();
    MX_LTDC_Init();
    MX_SPI5_Init();

    /* ---------- LCD init ---------- */
    BSP_LCD_Init();
    BSP_LCD_LayerDefaultInit(LCD_BACKGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_LayerDefaultInit(LCD_FOREGROUND_LAYER, LCD_FRAME_BUFFER);
    BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
    BSP_LCD_DisplayOn();

    BSP_LCD_Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetTextColor(LCD_COLOR_BLACK);
    BSP_LCD_DisplayStringAt(4, 4, (uint8_t*)"HELLO", LEFT_MODE);

    /* ---------- Create the rest of tasks ---------- */
    OSTaskCreate(&CommTaskTCB, "Comm", CommTask, 0, APP_CFG_COMM_TASK_PRIO,
                 &CommTaskStk[0], CommTaskStk[APP_CFG_COMM_TASK_STK_SIZE/10],
                 APP_CFG_COMM_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);

    OSTaskCreate(&BtnTaskTCB, "Btn", BtnTask, 0, APP_CFG_BTN_TASK_PRIO,
                 &BtnTaskStk[0], BtnTaskStk[APP_CFG_BTN_TASK_STK_SIZE/10],
                 APP_CFG_BTN_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);

    /* HIL MODE: PlantTask disabled - plant model runs externally in LabVIEW
     * The STM32 sends CV over USB, LabVIEW computes plant response and sends PV back.
     * To re-enable embedded plant simulation, uncomment the following block:
     */
    /*
    OSTaskCreate(&PlantTaskTCB, "Plant", PlantTask, 0, APP_CFG_PLANT_TASK_PRIO,
                 &PlantTaskStk[0], PlantTaskStk[APP_CFG_PLANT_TASK_STK_SIZE/10],
                 APP_CFG_PLANT_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);
    */

    OSTaskCreate(&ControlTaskTCB, "Control", ControlTask, 0, APP_CFG_CONTROL_TASK_PRIO,
                 &ControlTaskStk[0], ControlTaskStk[APP_CFG_CONTROL_TASK_STK_SIZE/10],
                 APP_CFG_CONTROL_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);

    OSTaskCreate(&ClockTaskTCB, "Clock", ClockTask, 0, APP_CFG_CLOCK_TASK_PRIO,
                 &ClockTaskStk[0], ClockTaskStk[APP_CFG_CLOCK_TASK_STK_SIZE/10],
                 APP_CFG_CLOCK_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);

    OSTaskCreate(&GuiTaskTCB, "GUI", GuiTask, 0, APP_CFG_GUI_TASK_PRIO,
                 &GuiTaskStk[0], GuiTaskStk[APP_CFG_GUI_TASK_STK_SIZE/10],
                 APP_CFG_GUI_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);

    OSTaskCreate(&GuiClockTaskTCB, "GUI Clock", GuiClockTask, 0, APP_CFG_GUICLK_TASK_PRIO,
                 &GuiClockTaskStk[0], GuiClockTaskStk[APP_CFG_GUICLK_TASK_STK_SIZE/10],
                 APP_CFG_GUICLK_TASK_STK_SIZE, 0,0,0,
                 OS_OPT_TASK_STK_CHK|OS_OPT_TASK_STK_CLR, &err);

    /* LED blink */
    while (DEF_TRUE) {
        BSP_LED_Toggle(0);
        OSTimeDlyHMSM(0,0,1,0, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}



/* =========================================================
 *                    Comm + button tasks
 * =======================================================*/
static void CommTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR err;
    void *p_msg; OS_MSG_SIZE sz; CPU_TS ts;

    while (DEF_TRUE) {
        p_msg = OSQPend(&CommQ, 0, OS_OPT_PEND_BLOCKING, &sz, &ts, &err);
        if (err == OS_ERR_NONE && p_msg && sz > 0) {
            parse_command_line((const char*)p_msg);
        }
    }
}

static void BtnTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR err;

    while (DEF_TRUE) {
        OSTimeDlyHMSM(0,0,0,10, OS_OPT_TIME_HMSM_STRICT, &err);
        if (HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET) {
            while (HAL_GPIO_ReadPin(KEY_BUTTON_GPIO_PORT, KEY_BUTTON_PIN) == GPIO_PIN_SET) {}
            OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
            if (g.mode == MODE_AUTO) {
                g.mode = MODE_MANUAL;
            } else {
                g.mode = MODE_AUTO;
            }
            OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);
        }
    }
}

/* =========================================================
 *                     Plant & control
 * =======================================================*/

/* PlantTask: Discrete first-order plant simulation
 * Transfer function: y[k] = 0.119217*u[k] + 0.904837*y[k-1]
 *
 * NOTE: This task is DISABLED in HIL mode. When running Hardware-in-the-Loop
 * testing, the plant model executes externally in LabVIEW. The STM32 sends
 * control output (CV) over USB and receives process variable (PV) back.
 *
 * To switch modes:
 *   - HIL Mode:      Comment out OSTaskCreate for PlantTask in StartupTask
 *   - Embedded Mode: Uncomment OSTaskCreate for PlantTask in StartupTask
 */
static void PlantTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR err;
    const float a1 = 0.904837f;
    const float b0 = 0.119217f;
    float y_prev = 0.0f;
    int print_count = 0;

    while (DEF_TRUE) {
        float u_now;
        OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
        u_now = g.cv_volt;
        OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);

        float y_now = b0*u_now + a1*y_prev;

        OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
        g.pv_volt = y_now;
        g.pv_degC = VoltageToTemp(y_now);
        OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);

        if (print_count > 5){
            printf("Plant: u=%.3fV  y=%.3fV / %.2fC\r\n", u_now, y_now, VoltageToTemp(y_now));
            print_count = 0;
        }
        print_count++;

        y_prev = y_now;
        OSTimeDlyHMSM(0,0,0,200, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

static void ControlTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR err;

    const float Ts = 0.200f;
    const float U_MIN = 0.0f, U_MAX = 5.0f;

    float e_prev = 0.0f;
    float Iacc   = 0.0f;

    while (DEF_TRUE) {
        float spC, pvC, Kc, Ti_s, Td_s, man_v; ctrl_mode_t mode;
        OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
        spC=g.sp_degC; 
        pvC=g.pv_degC; 
        Kc=g.Kc; 
        Ti_s=g.Ti_s; 
        Td_s=g.Td_s;
        mode=g.mode; 
        man_v=g.manual_cv_volt;
        OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);

        float e = spC - pvC;
        float Iterm;
        if (Ti_s > 1e-6f) {
            Iterm = Iacc + (Ts / Ti_s) * e;
        } else {
            Iterm = Iacc;
        }

        float Dterm;
        if (Td_s > 1e-6f) {
            Dterm = (e - e_prev) / Ts;
        } else {
            Dterm = 0.0f;
        }

        float u;
        if (mode == MODE_AUTO) {
            u = Kc * (e + Iterm + Td_s * Dterm);
        } else {
            u = man_v;
        }

        if (u > U_MAX) {
            if (mode == MODE_AUTO && Kc > 1e-6f) {
                Iterm -= (u - U_MAX) / Kc;
            }
            u = U_MAX;
        }
        if (u < U_MIN) {
            if (mode == MODE_AUTO && Kc > 1e-6f) {
                Iterm += (U_MIN - u) / Kc;
            }
            u = U_MIN;
        }

        Iacc = Iterm; e_prev = e;

        OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
        g.cv_volt = u;
        g.e_int   = Iacc;
        g.e_prev  = e_prev;
        OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);

        /* HIL MODE: Send CV to external plant (LabVIEW) over USB VCP.
         * Format: "CV=X.XXXX\r\n" - LabVIEW parses this, computes plant response,
         * and sends back "pv=X.XXXX" which is handled by parse_command_line().
         */
        printf("CV=%.4f\r\n", u);

        OSTimeDlyHMSM(0,0,0,200, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* =========================================================
 *                        Clock task
 * =======================================================*/
static void ClockTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR err;

    uint32_t last_ms = HAL_GetTick();   /* HAL millisecond counter */
    uint32_t acc_ms  = 0;

    while (DEF_TRUE) {
        uint32_t now = HAL_GetTick();
        uint32_t dt  = now - last_ms;
        last_ms = now;

        acc_ms += dt;
        while (acc_ms >= 1000u) {
            acc_ms -= 1000u;
            OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
            g.clock_s += 1;
            OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);
        }

        OSTimeDlyHMSM(0,0,0,50, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* ================================ GuiClockTask ================================ */
static void GuiClockTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR   err;
    shared_t s;

    const uint16_t W = BSP_LCD_GetXSize();
    const int pad     = 4;

    const int mode_w  = 52;
    const int mode_h  = 14;
    const int mode_x  = pad;
    const int mode_y  = pad;

    const int clock_w = 68;
    const int clock_h = 14;
    const int clock_x = W - pad - clock_w - 50;
    const int clock_y = pad;

    BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
    OSMutexPend(&lcd_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);

    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_DrawRect(mode_x-1, mode_y-1, mode_w+2, mode_h+2);
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(mode_x, mode_y, mode_w, mode_h);

    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_DrawRect(clock_x-1, clock_y-1, clock_w+2, clock_h+2);
    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(clock_x, clock_y, clock_w, clock_h);
    OSMutexPost(&lcd_mutex, OS_OPT_POST_NONE, &err);

    char last_mode[8]  = "";
    char last_clock[16]= "";
    uint32_t last_sec  = 0;

    while (DEF_TRUE) {
        // read mode + seconds
        OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
        s = g;
        OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);

        char now_mode[8];
        if (s.mode == MODE_AUTO) {
            strcpy(now_mode, "AUTO");
        } else {
            strcpy(now_mode, "MAN");
        }

        // HH:MM:SS string
        char clk[16];
        uint32_t sec = s.clock_s;
        uint32_t hh = (sec / 3600u) % 24u;
        uint32_t mm = (sec / 60u) % 60u;
        uint32_t ss = sec % 60u;
        snprintf(clk, sizeof clk, "%02lu:%02lu:%02lu",
                 (unsigned long)hh, (unsigned long)mm, (unsigned long)ss);

        OSMutexPend(&lcd_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);

        if (strcmp(now_mode, last_mode) != 0) {
            BSP_LCD_SetTextColor(COL_BG);
            BSP_LCD_FillRect(mode_x, mode_y, mode_w, mode_h);
            BSP_LCD_SetTextColor(COL_TEXT);
            BSP_LCD_DisplayStringAt(mode_x+2, mode_y, (uint8_t*)now_mode, LEFT_MODE);
            strcpy(last_mode, now_mode);
        }

        if (sec != last_sec || strcmp(clk, last_clock) != 0) {
            BSP_LCD_SetTextColor(COL_BG);
            BSP_LCD_FillRect(clock_x, clock_y, clock_w, clock_h);
            BSP_LCD_SetTextColor(COL_TEXT);
            BSP_LCD_DisplayStringAt(clock_x+2, clock_y, (uint8_t*)clk, LEFT_MODE);
            strcpy(last_clock, clk);
            last_sec = sec;
        }

        OSMutexPost(&lcd_mutex, OS_OPT_POST_NONE, &err);

        OSTimeDlyHMSM(0,0,0,150, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}

/* =========================================================
 *                    Minimal GUI task
 * =======================================================*/
static void GuiTask(void *p_arg)
{
    (void)p_arg;
    OS_ERR   err;
    shared_t s;

    /* Layout */
    const uint16_t W = BSP_LCD_GetXSize();
    const uint16_t H = BSP_LCD_GetYSize();

    const int pad      = 4;

    const int top_row_h = 14;
    const int graph_x  = pad;
    const int graph_y  = pad + top_row_h + 2;
    const int graph_w  = W - 2*pad;
    int graph_h;
    if (H > 240) {
        graph_h = (H/3);
    } else {
        graph_h = (H/2);
    }  /* modest height */

    const int line_h   = 14;
    const int gap_tv   = 2;
    const int block_gap= 6;
    const int y0       = graph_y + graph_h + pad;

    const int ref_t_y  = y0;
    const int ref_v_y  = ref_t_y + line_h + gap_tv;
    const int out_t_y  = ref_v_y + line_h + block_gap;
    const int out_v_y  = out_t_y + line_h + gap_tv;
    const int ts_t_y   = out_v_y + line_h + block_gap;
    const int ts_v_y   = ts_t_y  + line_h + gap_tv;

    BSP_LCD_SelectLayer(LCD_FOREGROUND_LAYER);
    OSMutexPend(&lcd_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);

    BSP_LCD_SetTextColor(COL_BG);
    BSP_LCD_FillRect(4, 4, 80, 14);

    /* Static row titles */
    BSP_LCD_SetTextColor(COL_TEXT);
    BSP_LCD_DisplayStringAt(pad, ref_t_y, (uint8_t*)"Reference", LEFT_MODE);
    BSP_LCD_DisplayStringAt(pad, out_t_y, (uint8_t*)"Output",    LEFT_MODE);
    BSP_LCD_DisplayStringAt(pad, ts_t_y,  (uint8_t*)"Ts",        LEFT_MODE);

    BSP_LCD_SetTextColor(COL_GRAPH_BG);
    BSP_LCD_FillRect(graph_x, graph_y, graph_w, graph_h);
    BSP_LCD_SetTextColor(LCD_COLOR_LIGHTGRAY);
    BSP_LCD_DrawRect(graph_x-1, graph_y-1, graph_w+2, graph_h+2);

    OSMutexPost(&lcd_mutex, OS_OPT_POST_NONE, &err);

    // Last-printed strings (avoid redundant LCD changes)
    char last_ref[40]   = "";
    char last_out[40]   = "";
    char last_ts [16]   = "";

    while (DEF_TRUE) {
        OSMutexPend(&g_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);
        s = g;
        OSMutexPost(&g_mutex, OS_OPT_POST_NONE, &err);

        // Prepare values
        const float sp_v = TempToVoltage(s.sp_degC);
        GUI_PushSample(s.pv_volt, sp_v);

        char now_ref[40], now_out[40], now_ts[16];
        snprintf(now_ref, sizeof now_ref, "%0.3f V / %0.1f C", sp_v,      s.sp_degC);
        snprintf(now_out, sizeof now_out, "%0.3f V / %0.1f C", s.pv_volt, s.pv_degC);
        snprintf(now_ts,  sizeof now_ts,  "200 ms");

        // LCD updates
        OSMutexPend(&lcd_mutex, 0, OS_OPT_PEND_BLOCKING, 0, &err);

        // GRAPH: clear interior only, then draw SP line + PV sparkline
        BSP_LCD_SetTextColor(COL_GRAPH_BG);
        BSP_LCD_FillRect(graph_x, graph_y, graph_w, graph_h);

        GUI_DrawRefLine(sp_v,                      graph_x, graph_y, graph_w, graph_h);
        GUI_DrawSparkline(gui_pv_hist, gui_hist_n, graph_x, graph_y, graph_w, graph_h, COL_PV);

        // TEXT ROWS: redraw only on change (overwrite with blanks first)
        BSP_LCD_SetTextColor(COL_SUBTEXT);

        if (strcmp(now_ref, last_ref) != 0) {
            static char blanks_ref[48]; memset(blanks_ref, ' ', sizeof(blanks_ref)-1); blanks_ref[sizeof(blanks_ref)-1]=0;
            BSP_LCD_DisplayStringAt(pad, ref_v_y, (uint8_t*)blanks_ref, LEFT_MODE);
            BSP_LCD_DisplayStringAt(pad, ref_v_y, (uint8_t*)now_ref, LEFT_MODE);
            strcpy(last_ref, now_ref);
        }
        if (strcmp(now_out, last_out) != 0) {
            static char blanks_out[48]; memset(blanks_out, ' ', sizeof(blanks_out)-1); blanks_out[sizeof(blanks_out)-1]=0;
            BSP_LCD_DisplayStringAt(pad, out_v_y, (uint8_t*)blanks_out, LEFT_MODE);
            BSP_LCD_DisplayStringAt(pad, out_v_y, (uint8_t*)now_out, LEFT_MODE);
            strcpy(last_out, now_out);
        }
        if (strcmp(now_ts, last_ts) != 0) {
            static char blanks_ts[24]; memset(blanks_ts, ' ', sizeof(blanks_ts)-1); blanks_ts[sizeof(blanks_ts)-1]=0;
            BSP_LCD_DisplayStringAt(pad, ts_v_y, (uint8_t*)blanks_ts, LEFT_MODE);
            BSP_LCD_DisplayStringAt(pad, ts_v_y, (uint8_t*)now_ts, LEFT_MODE);
            strcpy(last_ts, now_ts);
        }

        OSMutexPost(&lcd_mutex, OS_OPT_POST_NONE, &err);

        OSTimeDlyHMSM(0,0,0,300, OS_OPT_TIME_HMSM_STRICT, &err);
    }
}


/* =========================================================
 *                     Command parser
 * =======================================================*/
static void trim_line(char *s) {
    size_t n = strlen(s);
    while (n && (s[n-1]=='\r'||s[n-1]=='\n'||s[n-1]==' '||s[n-1]=='\t')) { s[--n]=0; }
}
static int ends_s(const char *p){ size_t n=strlen(p); return n>0 && (p[n-1]=='s'||p[n-1]=='S'); }

static void parse_command_line(const char *in)
{
    OS_ERR err;
    char buf[96];
    strncpy(buf, in, sizeof(buf)-1); buf[sizeof(buf)-1]=0; trim_line(buf);

    if (!strncmp(buf,"mode=",5)) {
        const char *v = buf+5;
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        if (!strcasecmp(v, "auto")) {
            g.mode = MODE_AUTO;
        } else {
            g.mode = MODE_MANUAL;
        }
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK %s\r\n", buf);
        return;
    }
    if (!strncmp(buf,"sp=",3)) {
        float sp = strtof(buf+3,NULL);
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        g.sp_degC = sp;
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK sp=%.2f C\r\n", sp);
        return;
    }
    /* HIL MODE: Receive process variable (PV) from external plant simulation.
     * LabVIEW sends "pv=X.XXXX" after computing plant response from CV.
     * This updates g.pv_volt and g.pv_degC for the next PID cycle.
     */
    if (!strncmp(buf,"pv=",3)) {
        float pv = strtof(buf+3,NULL);
        if (pv < 0.0f) pv = 0.0f;
        if (pv > 5.0f) pv = 5.0f;
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        g.pv_volt = pv;
        g.pv_degC = VoltageToTemp(pv);
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK pv=%.4f V (%.2f C)\r\n", pv, VoltageToTemp(pv));
        return;
    }
    if (!strncmp(buf,"kc=",3)) {
        float kc = strtof(buf+3,NULL);
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        g.Kc = kc;
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK kc=%.3f\r\n", kc);
        return;
    }
    if (!strncmp(buf,"ti=",3)) {
        const char *p = buf+3; float ti = strtof(p,NULL);
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        g.Ti_s = ti; (void)ends_s(p);
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK ti=%.2fs\r\n", ti);
        return;
    }
    if (!strncmp(buf,"td=",3)) {
        const char *p = buf+3; float td = strtof(p,NULL);
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        g.Td_s = td;
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK td=%.2fs\r\n", td);
        return;
    }
    if (!strncmp(buf,"man=",4)) {
        float mv = strtof(buf+4,NULL);
        if (mv < 0) {
            mv = 0;
        }
        if (mv > 5) {
            mv = 5;
        }
        OSMutexPend(&g_mutex,0,OS_OPT_PEND_BLOCKING,0,&err);
        g.manual_cv_volt = mv;
        OSMutexPost(&g_mutex,OS_OPT_POST_NONE,&err);
        printf("OK man=%.3f V\r\n", mv);
        return;
    }
    printf("ERR unknown cmd: %s\r\n", buf);
}

/* =========================================================
 *                 Temp/Volt conversions
 * =======================================================*/
static float TempToVoltage(float temp){
    return -0.0015f*powf(temp,2.0f) + 0.3319f*temp - 6.9173f;
}
static float VoltageToTemp(float volt){
    return 0.3053f*powf(volt,2.0f) + 2.2602f*volt + 25.287f;
}

/* =========================================================
 *                      System clock
 * =======================================================*/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_O; RCC_ClkInitTypeDef RCC_C;
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_O.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_O.HSEState = RCC_HSE_ON;
    RCC_O.PLL.PLLState = RCC_PLL_ON;
    RCC_O.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_O.PLL.PLLM = 4; RCC_O.PLL.PLLN = 168; RCC_O.PLL.PLLP = RCC_PLLP_DIV2; RCC_O.PLL.PLLQ = 7;
    if (HAL_RCC_OscConfig(&RCC_O) != HAL_OK) { while(1){} }

    RCC_C.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_C.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_C.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_C.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_C.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_C, FLASH_LATENCY_5) != HAL_OK) { while(1){} }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM1) { HAL_IncTick(); }
}

#ifdef __GNUC__
int _write(int file, char *ptr, int len)
{
    (void)file;
    int sent = 0;
    while (sent < len) {
        uint16_t chunk;
        if ((len - sent) > 64) {
            chunk = 64;
        } else {
            chunk = (uint16_t)(len - sent);
        }
        while (CDC_Transmit_HS((uint8_t *)&ptr[sent], chunk) == USBD_BUSY) {
        }
        sent += chunk;
    }
    return sent;
}
#else
int fputc(int ch, FILE *f) {
    (void)f;
    uint8_t b = (uint8_t)ch;
    while (CDC_Transmit_HS(&b, 1) == USBD_BUSY) {}
    return ch;
}
#endif

void App_USB_PostLine(const char *line)
{
    static char copy[96];
    OS_ERR err;
    size_t n = strlen(line); if (n >= sizeof(copy)) n = sizeof(copy)-1;
    memcpy(copy, line, n); copy[n]=0;
    OSQPost(&CommQ, (void*)copy, n+1, OS_OPT_POST_FIFO, &err);
}

void _Error_Handler(char *file, int line)
{
    (void)file; (void)line;
    __disable_irq();
    while (1) {
    }
}

#ifdef  USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line)
{
    (void)file; (void)line;
    __disable_irq();
    while (1) { }
}
#endif

