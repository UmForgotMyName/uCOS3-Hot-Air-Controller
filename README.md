# RTOS PID Temperature Controller

<p align="center">
  <img src="https://img.shields.io/badge/Platform-STM32F429ZI-blue" alt="Platform">
  <img src="https://img.shields.io/badge/RTOS-µC/OS--III-green" alt="RTOS">
  <img src="https://img.shields.io/badge/Language-C-orange" alt="Language">
  <img src="https://img.shields.io/badge/Control-PID-red" alt="Control">
  <img src="https://img.shields.io/badge/Mode-HIL--Ready-purple" alt="HIL">
</p>

A real-time PID temperature controller implemented on the STM32F429ZI Discovery board using Micrium's µC/OS-III RTOS. This project demonstrates deterministic task scheduling for industrial control applications with a 200 ms control period and Hardware-in-the-Loop (HIL) capability for validation against external plant simulations.

---

## 📋 Table of Contents

- [Overview](#overview)
- [Features](#features)
- [HIL Architecture](#hil-architecture)
- [System Architecture](#system-architecture)
- [Hardware Requirements](#hardware-requirements)
- [Software Dependencies](#software-dependencies)
- [Task Structure](#task-structure)
- [PID Controller](#pid-controller)
- [Plant Model](#plant-model)
- [Communication Interface](#communication-interface)
- [HIL Protocol](#hil-protocol)
- [GUI Display](#gui-display)
- [Building & Flashing](#building--flashing)
- [Usage](#usage)
- [Project Structure](#project-structure)
- [License](#license)

---

## Overview

This project implements a closed-loop PID temperature controller for a hot air plant. The system runs on µC/OS-III, a preemptive real-time kernel, ensuring deterministic timing for all control and I/O operations. The controller supports both automatic (PID) and manual control modes, with real-time parameter tuning via USB Virtual COM Port (VCP).

**Current Mode: Hardware-in-the-Loop (HIL)**

The embedded plant simulation task (`PlantTask`) is disabled. The STM32 controller sends control output (CV) over USB and receives process variable (PV) from an external plant simulation (e.g., LabVIEW, Python, MATLAB).

### Key Accomplishments

- **Deterministic Control**: 200 ms control period with guaranteed deadline compliance
- **Multi-task Architecture**: 7 concurrent RTOS tasks with priority-based scheduling
- **Real-time HMI**: Live graphical display showing setpoint, process variable, and trend history
- **Remote Tuning**: USB CDC interface for runtime PID parameter adjustment
- **True HIL Capability**: Controller runs on real hardware, plant model runs externally with physical USB interface

---

## Features

| Feature | Description |
|---------|-------------|
| **PID Control** | Proportional-Integral-Derivative controller with anti-windup |
| **HIL Mode** | External plant simulation via USB VCP (LabVIEW/Python/MATLAB) |
| **Auto/Manual Modes** | Seamless switching between automatic PID and manual output control |
| **Real-time Display** | LCD shows setpoint, output, mode, elapsed time, and live trend graph |
| **USB Communication** | Virtual COM Port for CV/PV exchange and parameter tuning |
| **Mutex Protection** | Thread-safe access to shared control variables |
| **Temperature Conversion** | Nonlinear voltage-to-temperature mapping |

---

## HIL Architecture

The system operates in **Hardware-in-the-Loop** mode where the PID controller runs on real STM32 hardware while the plant model executes on an external PC:

```
┌──────────────────────────────────────┐         USB VCP          ┌──────────────────────────────────────┐
│           STM32F429ZI                │        (COM Port)        │         External PC                  │
│         (Real Hardware)              │                          │    (LabVIEW / Python / MATLAB)       │
│                                      │                          │                                      │
│  ┌────────────────────────────────┐  │                          │  ┌────────────────────────────────┐  │
│  │        ControlTask             │  │                          │  │       Plant Model              │  │
│  │          (PID)                 │  │                          │  │                                │  │
│  │                                │  │    "CV=2.5432\r\n"       │  │  y[k] = b0*u + a1*y[k-1]       │  │
│  │  e = SP - PV                   │  │  ════════════════════►   │  │                                │  │
│  │  CV = Kc*(P + I + D)           │  │                          │  │  b0 = 0.119217                 │  │
│  │                                │  │    "pv=1.8765\r\n"       │  │  a1 = 0.904837                 │  │
│  │  printf("CV=%.4f\r\n", u)      │  │  ◄════════════════════   │  │                                │  │
│  └────────────────────────────────┘  │                          │  └────────────────────────────────┘  │
│              │                       │                          │              │                       │
│              ▼                       │                          │              ▼                       │
│  ┌────────────────────────────────┐  │                          │  ┌────────────────────────────────┐  │
│  │        CommTask                │  │                          │  │       VISA Serial              │  │
│  │   parse_command_line()         │  │                          │  │    Read/Write Loop             │  │
│  │   handles "pv=X.XXXX"          │  │                          │  │                                │  │
│  └────────────────────────────────┘  │                          │  └────────────────────────────────┘  │
│                                      │                          │                                      │
│  ┌────────────────────────────────┐  │                          │                                      │
│  │   PlantTask (DISABLED)         │  │                          │                                      │
│  │   /* Commented out for HIL */  │  │                          │                                      │
│  └────────────────────────────────┘  │                          │                                      │
└──────────────────────────────────────┘                          └──────────────────────────────────────┘
```

### Switching Between Modes

**To enable HIL mode (current):**
- `PlantTask` creation is commented out in `StartupTask`
- Controller outputs CV via `printf("CV=%.4f\r\n", u)`
- External plant sends PV via `pv=X.XXXX` command

**To enable embedded simulation mode:**
- Uncomment `OSTaskCreate(&PlantTaskTCB, ...)` in `StartupTask`
- `PlantTask` will update `g.pv_volt` internally

---

## System Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        µC/OS-III Kernel                             │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │ Startup  │  │  Plant   │  │ Control  │  │   Comm   │             │
│  │  Task    │  │  Task    │  │  Task    │  │   Task   │             │
│  │ (Prio 4) │  │ (Prio 8) │  │ (Prio 7) │  │ (Prio 12)│             │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘             │
│                                                                     │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌──────────┐             │
│  │   GUI    │  │ GuiClock │  │  Clock   │  │  Button  │             │
│  │  Task    │  │   Task   │  │   Task   │  │   Task   │             │
│  │ (Prio 10)│  │ (Prio 10)│  │ (Prio 13)│  │ (Prio 11)│             │
│  └──────────┘  └──────────┘  └──────────┘  └──────────┘             │
│                                                                     │
│  ┌─────────────────────────────────────────────────────────────┐    │
│  │              Shared State (Mutex Protected)                 │    │
│  │  • PV (Process Variable)  • SP (Setpoint)  • CV (Output)    │    │
│  │  • Kc, Ti, Td (PID Gains) • Mode (Auto/Manual)              │    │
│  └─────────────────────────────────────────────────────────────┘    │
│                                                                     │
├─────────────────────────────────────────────────────────────────────┤
│                    STM32F429ZI HAL Layer                            │
│        (GPIO, LCD, USB CDC, TIM, I2C, SPI, FMC, LTDC)               │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Hardware Requirements

| Component | Specification |
|-----------|---------------|
| **MCU Board** | STM32F429I-Discovery |
| **Processor** | ARM Cortex-M4 @ 168 MHz |
| **Flash** | 2 MB |
| **RAM** | 256 KB SRAM + 64 MB SDRAM |
| **Display** | 2.4" QVGA TFT LCD (240×320) |
| **Interface** | USB Micro-B (VCP) |

---

## Software Dependencies

- **RTOS**: Micrium µC/OS-III v3.06+
- **HAL**: STM32F4xx HAL Driver
- **BSP**: STM32F429I-Discovery Board Support Package
- **USB**: STM32 USB Device Library (CDC Class)
- **Toolchain**: ARM GCC / STM32CubeIDE

---

## Task Structure

| Task | Priority | Period | Stack Size | Responsibility | Status |
|------|----------|--------|------------|----------------|--------|
| `StartupTask` | 4 | — | 256 | System initialization, task creation | ✅ Active |
| `ControlTask` | 7 | 200 ms | 256 | PID computation, CV output to USB | ✅ Active |
| `PlantTask` | 8 | 200 ms | 256 | Discrete plant simulation | ⏸️ Disabled (HIL) |
| `GuiTask` | 10 | 300 ms | 512 | LCD trend graph, numeric display | ✅ Active |
| `GuiClockTask` | 10 | 150 ms | 256 | Mode indicator, elapsed time | ✅ Active |
| `BtnTask` | 11 | 10 ms | 192 | Auto/Manual mode toggle | ✅ Active |
| `CommTask` | 12 | Event | 256 | USB command parsing (incl. PV input) | ✅ Active |
| `ClockTask` | 13 | 50 ms | 192 | System time tracking | ✅ Active |

---

## PID Controller

### Algorithm

The controller implements the velocity form of the PID algorithm with anti-windup:

```
e(k) = SP - PV

P = Kc × e(k)
I = I_acc + (Ts / Ti) × e(k)
D = Td × (e(k) - e(k-1)) / Ts

CV = Kc × (e + I + Td × D)
```

### Anti-Windup

When the output saturates (0V – 5V), the integrator is back-calculated to prevent windup:

```c
if (u > U_MAX) {
    Iterm -= (u - U_MAX) / Kc;
    u = U_MAX;
}
```

### Default Parameters

| Parameter | Symbol | Default Value |
|-----------|--------|---------------|
| Proportional Gain | Kc | 1.0 |
| Integral Time | Ti | 10.0 s |
| Derivative Time | Td | 0.0 s |
| Sample Period | Ts | 200 ms |
| Setpoint | SP | 35.0 °C |

---

## Plant Model

The hot air plant is modeled as a first-order discrete transfer function:

$$y[k] = 0.119217 \cdot u[k] + 0.904837 \cdot y[k-1]$$

**In HIL mode**, this model runs externally in LabVIEW/Python/MATLAB. The embedded `PlantTask` is disabled.

**Temperature Conversion Functions:**

```c
// Temperature (°C) → Voltage (V)
V = -0.0015×T² + 0.3319×T - 6.9173

// Voltage (V) → Temperature (°C)
T = 0.3053×V² + 2.2602×V + 25.287
```

---

## Communication Interface

### USB VCP Commands

Connect via serial terminal (115200 baud) and send commands:

| Command | Example | Description |
|---------|---------|-------------|
| `sp=<value>` | `sp=40.5` | Set temperature setpoint (°C) |
| `pv=<value>` | `pv=1.8765` | **HIL:** Receive plant output voltage (0–5V) |
| `kc=<value>` | `kc=1.5` | Set proportional gain |
| `ti=<value>` | `ti=8.0` | Set integral time (seconds) |
| `td=<value>` | `td=0.1` | Set derivative time (seconds) |
| `mode=<auto\|manual>` | `mode=manual` | Switch control mode |
| `man=<value>` | `man=2.5` | Set manual output voltage (0–5V) |

### STM32 Output (HIL Mode)

The controller outputs CV every 200ms:
```
CV=2.5432
CV=2.5501
CV=2.5589
...
```

### Response Format

```
OK sp=40.50 C
OK pv=1.8765 V (32.45 C)
OK kc=1.500
ERR unknown cmd: xyz
```

---

## HIL Protocol

### Communication Sequence

```
Time(ms)   0         200       400       600       800
           │          │          │          │          │
STM32      ├──────────┼──────────┼──────────┼──────────┤
           │ PID calc │ PID calc │ PID calc │ PID calc │
           │ Send CV  │ Send CV  │ Send CV  │ Send CV  │
           └────┬─────┴────┬─────┴────┬─────┴────┬─────┘
                │          │          │          │
           CV=2.10    CV=2.35    CV=2.48    CV=2.52
                │          │          │          │
                ▼          ▼          ▼          ▼
    ════════════════════════════════════════════════════  USB
                │          │          │          │
                ▼          ▼          ▼          ▼
LabVIEW    ├──────────┼──────────┼──────────┼──────────┤
           │ Plant    │ Plant    │ Plant    │ Plant    │
           │ Compute  │ Compute  │ Compute  │ Compute  │
           │ Send PV  │ Send PV  │ Send PV  │ Send PV  │
           └────┬─────┴────┬─────┴────┬─────┴────┬─────┘
                │          │          │          │
           pv=0.25    pv=0.53    pv=0.89    pv=1.18
                │          │          │          │
                ▼          ▼          ▼          ▼
    ════════════════════════════════════════════════════  USB
                │          │          │          │
                ▼          ▼          ▼          ▼
STM32      Receives PV, uses in next PID cycle
```

### Python HIL Test Script

```python
import serial
import time

# Connect to STM32 USB VCP
ser = serial.Serial('COM3', 115200, timeout=0.1)

# Plant model state
y_prev = 0.0
b0, a1 = 0.119217, 0.904837

print("HIL Plant Simulation Started")
print("=" * 40)

while True:
    line = ser.readline().decode().strip()
    if line.startswith("CV="):
        # Parse CV from STM32
        cv = float(line.split("=")[1])
        
        # Compute plant response
        y = b0 * cv + a1 * y_prev
        y_prev = y
        
        # Send PV back to STM32
        ser.write(f"pv={y:.4f}\r\n".encode())
        
        print(f"CV={cv:.4f} V → Plant → PV={y:.4f} V")
```

### LabVIEW Implementation Notes

1. **VISA Configure Serial Port**: COM port, 115200 baud, 8N1
2. **While Loop** (200ms period):
   - VISA Read until `\r\n`
   - Parse `CV=X.XXXX` → extract float
   - Compute: `y = 0.119217*CV + 0.904837*y_prev`
   - VISA Write: `pv=Y.YYYY\r\n`
   - Shift register for `y_prev`

---

## GUI Display

The LCD provides real-time visualization:

```
┌────────────────────────────────────────┐
│ AUTO                        00:05:23   │
├────────────────────────────────────────┤
│                                        │
│    ═══════════════════════════════     │  ← Setpoint line (red)
│        ╱╲    ╱╲                        │
│       ╱  ╲  ╱  ╲                       │  ← PV trend (blue)
│      ╱    ╲╱    ╲                      │
│     ╱              ╲                   │
│                                        │
├────────────────────────────────────────┤
│ Reference                              │
│ 2.500 V / 35.0 C                       │
│                                        │
│ Output                                 │
│ 2.234 V / 33.8 C                       │
│                                        │
│ Ts                                     │
│ 200 ms                                 │
└────────────────────────────────────────┘
```

---

## Building & Flashing

### Using STM32CubeIDE

1. Import the project into STM32CubeIDE
2. Build: `Project → Build All` (Ctrl+B)
3. Flash: `Run → Debug` or use the `.launch` configuration files

### Using Makefile

```bash
cd Debug
make all
```

### Flashing with ST-Link

```bash
st-flash write Debug/ENGG4420_Lab3-uCOS-v2.bin 0x8000000
```

---

## Usage

### HIL Mode (Default)

1. **Connect Hardware**: Plug in STM32F429I-Discovery via USB
2. **Flash Firmware**: Upload the binary using ST-Link
3. **Start External Plant**: Run LabVIEW VI or Python script (see [HIL Protocol](#hil-protocol))
4. **Open Terminal**: Optionally monitor with serial terminal (115200 baud)
5. **Observe Closed-Loop**: STM32 outputs `CV=X.XXXX`, receives `pv=X.XXXX`
6. **Tune Controller**: Send commands like `sp=45` or `kc=2.0`
7. **Toggle Mode**: Press the blue user button to switch Auto↔Manual

### Embedded Simulation Mode

To test without external plant:

1. Uncomment `OSTaskCreate(&PlantTaskTCB, ...)` in `main.c` → `StartupTask()`
2. Rebuild and flash
3. Monitor telemetry: `Plant: u=X.XXXV y=X.XXXV / XX.XXC`

---

## Project Structure

```
uCOS3-Hot-Air-Controller/
├── Src/
│   ├── main.c              # Application entry, task definitions, PID logic
│   ├── gpio.c              # GPIO initialization
│   ├── usbd_cdc_if.c       # USB CDC interface callbacks
│   ├── stm32f4xx_it.c      # Interrupt handlers
│   └── ...
├── Inc/
│   ├── main.h              # Main header
│   ├── app_cfg.h           # Task priorities and stack sizes
│   ├── os_cfg.h            # µC/OS-III kernel configuration
│   └── ...
├── Middlewares/
│   ├── Third_Party/Micrium/# µC/OS-III kernel source
│   └── ST/STM32_USB_Device_Library/
├── Drivers/
│   ├── CMSIS/              # ARM CMSIS headers
│   └── STM32F4xx_HAL_Driver/# STM32 HAL
├── Debug/
│   └── makefile            # Build configuration
├── STM32F429ZI_FLASH.ld    # Linker script
└── README.md
```

---

## References

- [µC/OS-III Documentation](https://doc.micrium.com)
- [STM32F429I-Discovery User Manual](https://www.st.com/resource/en/user_manual/um1670-discovery-kit-with-stm32f429zi-mcu-stmicroelectronics.pdf)
- ENGG*4420 Real-Time Systems Design — Lab 3: Hot Air Plant Control with RTOS

---