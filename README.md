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

2. **Open STM32CubeIDE**:
* Go to `File` > `Import`.
* Select `General` > `Existing Projects into Workspace`.
* Browse to the `STM32F429-LM35-Driver` folder and click `Finish`.


3. **Build the Project**: Click the **Hammer icon** in the toolbar to compile the code.
4. **Flash the Board**: Connect your STM32F429ZI Discovery board via USB and click the **Run (Play) icon**.

---

## Hardware Connection

To get accurate readings, ensure the LM35 sensor is connected to the correct Analog-to-Digital (ADC) pins on the Discovery board.

| LM35 Pin | STM32F429 Pin | Role |
| --- | --- | --- |
| **+Vs (VCC)** | 3.3V or 5V | Power Supply |
| **Vout (OUT)** | **PA0** | Analog Signal Output (ADC1_IN0) |
| **GND** | GND | Common Ground |
| **UART TX** | **PA9** | Connect to Serial-to-USB RX |
| **UART RX** | **PA10** | Connect to Serial-to-USB TX |

---

## Driver API Reference

### ADC HAL (`ADC_HAL.h`)

* `HAL_ADC_Init()`: Configures ADC1 for 10-bit resolution, enables the clock for GPIOA, and sets PA0 to Analog mode.
* `HAL_ADC_Start(uint8_t channel)`: Triggers a conversion on the specified ADC channel.
* `HAL_ADC_Get_Value()`: Returns the 10-bit raw digital value from the Data Register.

### UART HAL (`USART_HAL.h`)

* `hal_uart_init(handle)`: Configures Baud rate, Word length, and Stop bits.
* `hal_uart_tx(handle, buffer, len)`: Initiates a non-blocking transfer using the `TXE` interrupt.
* `hal_uart_handle_interrupt(handle)`: The core ISR logic that manages the byte-by-byte transmission state machine.

---

## Troubleshooting

* **Reading stays at 0.00**: Verify that the LM35 is receiving power. These sensors are sensitive to orientation; check the pinout carefully.
* **Serial Terminal is empty**: Ensure your terminal (PuTTY/TeraTerm) is set to **9600 Baud** and you are connected to the correct COM port.
* **Garbage Characters**: This usually indicates a clock mismatch. Verify that your `BRR` (Baud Rate Register) calculation matches your system clock speed.
