# STM32F429 LM35 Temperature Driver

A custom bare-metal embedded driver for the LM35 analog temperature sensor on the STM32F429. This project implements a modular HAL (Hardware Abstraction Layer) for ADC and UART to sample thermal data and stream it to a serial terminal.

## Features

- **Custom HAL Implementation**: Register-level drivers for ADC and USART without using standard ST libraries.
- **10-bit ADC Sampling**: Configured for continuous conversion mode on Channel 0.
- **Integer-based Precision**: Calculates temperature with two decimal places using integer math to avoid heavy float processing.
- **Interrupt-Driven UART**: Non-blocking transmission using `TXE` (Transmit Data Register Empty) interrupts.
- **Serial Monitoring**: Continuous data stream at 9600 Baud.

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
git clone [https://github.com/Youssef-Fd/STM32F429-LM35-Driver.git](https://github.com/Youssef-Fd/STM32F429-LM35-Driver.git)
cd STM32F429-LM35-Driver
