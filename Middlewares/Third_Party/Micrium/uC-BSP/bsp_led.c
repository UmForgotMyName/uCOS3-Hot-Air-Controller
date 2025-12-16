/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                    MICRIUM BOARD SUPPORT PACKAGE
*                                         STM32F429I-DISC1
*
* Filename : bsp_led.c
* Modified by   : Kevin Dong for ENGG4420
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                            INCLUDE FILES
*********************************************************************************************************
*/

#include  <lib_def.h>
#include  "bsp_led.h"
#include  "stm32f4xx_hal.h"


/*
*********************************************************************************************************
*                                               DEFINES
*********************************************************************************************************
*/
#define  BSP_GPIOG_LED3                        DEF_BIT_13               // Kevin: adapt to SMT32F429I-DISC1 board
#define  BSP_GPIOG_LED4                        DEF_BIT_14               // Kevin: adapt to SMT32F429I-DISC1 board


/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*********************************************************************************************************
**                                         GLOBAL FUNCTIONS
*********************************************************************************************************
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                           BSP_LED_Init()
*
* Description : Initializes the required pins that control the LEDs.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_Init (void)
{
    GPIO_InitTypeDef  gpio_init;
    __HAL_RCC_GPIOG_CLK_ENABLE();                               /* Enable GPIO clock                       */

                                                                /* Configure GPIOG for LED3 and LED4                    */
    gpio_init.Pin   = (BSP_GPIOG_LED3 | BSP_GPIOG_LED4);
    gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
    gpio_init.Pull  = GPIO_PULLUP;
    gpio_init.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOG, &gpio_init);

    BSP_LED_Off(0);
    BSP_LED_On(0);

}


/*
*********************************************************************************************************
*                                              BSP_LED_Off()
*
* Description : Turn OFF any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    turns OFF ALL the LEDs
*                       3    turns OFF user LED3
*                       4    turns OFF user LED4
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_Off (CPU_INT08U  led)
{
    switch (led) {
        case 0u:
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED3, GPIO_PIN_SET);
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED4, GPIO_PIN_SET);
             break;


        case 3u:
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED3, GPIO_PIN_SET);
             break;


        case 4u:
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED4, GPIO_PIN_SET);
             break;

        default:
             break;
    }
}


/*
*********************************************************************************************************
*                                             BSP_LED_On()
*
* Description : Turn ON any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    turns ON ALL  LEDs
*                       3    turns ON user LED3
*                       4    turns ON user LED4
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_On (CPU_INT08U led)
{
    switch (led) {
    	case 0u:
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED3, GPIO_PIN_RESET);
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED4, GPIO_PIN_RESET);
             break;


        case 3u:
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED3, GPIO_PIN_RESET);
             break;


        case 4u:
             HAL_GPIO_WritePin(GPIOG, BSP_GPIOG_LED4, GPIO_PIN_RESET);
             break;

        default:
             break;
    }
}


/*
*********************************************************************************************************
*                                            BSP_LED_Toggle()
*
* Description : TOGGLE any or all the LEDs on the board.
*
* Argument(s) : led     The ID of the LED to control:
*
*                       0    TOGGLE ALL the LEDs
*                       3    TOGGLE user LED3
*                       4    TOGGLE user LED4
*
* Return(s)   : none.
*
* Caller(s)   : Application.
*
* Note(s)     : none.
*********************************************************************************************************
*/

void  BSP_LED_Toggle (CPU_INT08U  led)
{

    switch (led) {
        case 0u:
        	 HAL_GPIO_TogglePin(GPIOG, (BSP_GPIOG_LED3  |  BSP_GPIOG_LED4));
             break;

        case 3u:
        	 HAL_GPIO_TogglePin(GPIOG, (BSP_GPIOG_LED3));
             break;

        case 4u:
        	 HAL_GPIO_TogglePin(GPIOG, (BSP_GPIOG_LED4));

        default:
             break;
    }
}
