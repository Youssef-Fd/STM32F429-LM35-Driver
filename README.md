---

# STM32F429 LM35 Temperature Driver

A custom bare-metal embedded driver for the LM35 analog temperature sensor on the STM32F429. This project implements a modular HAL (Hardware Abstraction Layer) for ADC and UART to sample thermal data and stream it to a serial terminal.

## Features

**Custom HAL Implementation**: Register-level drivers for ADC and USART without using standard ST libraries.
***10-bit ADC Sampling**: Configured for continuous conversion mode on Channel 0.
**Integer-based Precision**: Calculates temperature with two decimal places using integer math to avoid heavy float processing.
***Interrupt-Driven UART**: Non-blocking transmission using `TXE` (Transmit Data Register Empty) interrupts.
***Serial Monitoring**: Continuous data stream at 9600 Baud.

## Tech Stack

* **Language**: Embedded C
* **Architecture**: ARM Cortex-M4
* **Peripherals**: ADC1, USART1, GPIO
* **Toolchain**: STM32CubeIDE

## Prerequisites

Before you begin, ensure you have the following:

* **Hardware**: STM32F429ZI Discovery Board.
* **Sensor**: LM35 Linear Temperature Sensor.
* **Debugger**: On-board ST-LINK/V2.
* **Terminal**: Serial monitor (PuTTY, TeraTerm, or STM32CubeIDE Console).

## Installation

1. Clone the repository:

```bash
git clone https://github.com/Youssef-Fd/STM32F429-LM35-Driver.git
cd STM32F429-LM35-Driver

```

2. Open **STM32CubeIDE** and select `File > Import`.
3. Choose `Existing Projects into Workspace` and select the cloned folder.
4. Clean and Build the project.
5. Connect your board and click **Resume/Run**.

## Hardware Connection

The LM35 output is connected to the STM32's Analog-to-Digital converter.

| LM35 Pin | STM32F429 Pin | Function |
| --- | --- | --- |
| **VCC** | 3.3V or 5V | Power Supply |
| **OUT** | **PA0** | ADC1_IN0 (Analog Input) |
| **GND** | GND | Ground |
| **-** | **PA9** | USART1_TX (Serial Out) |
| **-** | **PA10** | USART1_RX (Serial In) |

## Driver API Reference

### ADC HAL (`ADC_HAL.h`)

* `HAL_ADC_Init()`: Configures ADC1 for 10-bit resolution, scan mode, and PA0 as analog input.
* `HAL_ADC_Start(channel)`: Triggers the conversion on the specified channel.
* `HAL_ADC_Get_Value()`: Returns the raw 10-bit value from the Data Register (`DR`).

### UART HAL (`USART_HAL.h`)

* `hal_uart_init(handle)`: Sets baud rate (9600), word length, and stop bits.
* `hal_uart_tx(handle, buffer, len)`: Initiates interrupt-driven transmission.
* `USART1_IRQHandler()`: Handles the state machine for background data transfer.

## Core Conversion Logic

The code converts the 10-bit ADC value to millivolts and then to Celsius ():

```c
// 1023 represents the max value for 10-bit resolution
temperature_C = (ADC_value * 3300) / 1023; 

uint32_t Temp_int = temperature_C / 100;
uint32_t Temp_dec = temperature_C % 100;

sprintf(temperature, "Temperature: %u.%02u C\r\n", Temp_int, Temp_dec);

```

## Troubleshooting

### 1. Temperature reads 0.00

* Ensure the LM35 **VCC** and **GND** are not swapped.
* Check if **PA0** is firmly connected to the center pin of the LM35.

### 2. Garbage characters in Serial Monitor

* Verify the Baud Rate is set to **9600** in your terminal software.
* Ensure the STM32 clock configuration matches the UART baud rate calculation in `hal_uart_set_baud_rate`.

### 3. Build Errors

* Check that `Common_BASES.h` is included in your include path, as it defines the register base addresses used in the HAL.

## Acknowledgments

* This project uses a custom register-level approach inspired by the STM32 Reference Manual (RM0090).
* Special thanks to the open-source community for embedded driver patterns.

---
