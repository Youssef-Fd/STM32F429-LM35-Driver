---

# STM32F429 LM35 Temperature Monitor

This project is a bare-metal embedded application that reads analog temperature data from an **LM35 sensor** using the **STM32F429ZI** microcontroller. Instead of relying on heavy standard libraries, this project implements a **custom, lightweight Hardware Abstraction Layer (HAL)** to interface directly with the ARM Cortex-M4 registers.

---

## 🚀 Project Overview

The goal of this project was to create a high-efficiency temperature monitoring system. The workflow follows three main stages:

1. **Sensing**: The LM35 sensor outputs a linear voltage proportional to the temperature.
2. **Conversion**: The STM32’s internal **ADC (Analog-to-Digital Converter)** translates this analog voltage into a 10-bit digital value.
3. **Communication**: The processed temperature data is sent to a PC via **USART1** using an interrupt-driven state machine.

---

## 🛠️ Technical Implementation

### 1. Custom Peripheral Drivers (Bare-Metal)

Unlike standard projects that use `STM32CubeHAL`, I developed custom drivers in `ADC_HAL.c` and `UART_HAL.c`. This provides:

* **Lower Memory Footprint**: Only the necessary registers are toggled.
* **Better Performance**: Minimal overhead in interrupt handling.
* **Direct Control**: Granular control over clock gating and peripheral timing.

### 2. ADC Configuration (The Input)

The ADC is configured in **10-bit resolution mode**.

* **Pin**: Port A, Pin 0 (`PA0`).
* **Sampling**: Uses continuous conversion mode with a 3-cycle sampling time for rapid updates.
* **Logic**: The `HAL_ADC_WaitForConversion()` function ensures the data is ready before the CPU attempts to read the register, preventing "stale" data readings.

### 3. UART Interrupt State Machine (The Output)

To prevent the CPU from "freezing" while waiting for serial data to send, I implemented an **Interrupt-Driven USART**.

* When a string is sent, the `TXE` (Transmit Data Register Empty) interrupt is triggered.
* The `USART1_IRQHandler` handles the character-by-character transfer in the background.
* This allows the main loop to continue running other tasks while data is being transmitted.

---

## 🧪 The Math: From Volts to Celsius

The LM35 sensor provides  per . To convert the raw 10-bit digital value back into a human-readable temperature, the following logic is used:

1. **Calculate Voltage**:
Calculate Voltage:$$Voltage (mV) = \frac{ADC\_Value \times 3300}{1023}$$(Where 3300 is the $3.3V$ reference in $mV$, and 1023 is the max value for 10-bit resolution).
2. **Calculate Temperature**:


3. **Integer Precision**:
To avoid using slow floating-point math, the code calculates the temperature multiplied by 100 to extract both the **Integer** and **Decimal** parts for display (e.g., ).

---

## 🔌 Hardware Setup

| LM35 Pin | STM32 Pin | Role |
| --- | --- | --- |
| **+Vs** | 3.3V / 5V | Power Supply |
| **Vout** | **PA0** | Analog Signal Output |
| **GND** | GND | Ground |

---

## 📁 Project Structure

* `main.c`: The core application loop and system initialization.
* `ADC_HAL.c / .h`: Register definitions and functions for Analog-to-Digital conversion.
* `UART_HAL.c / .h`: USART configuration, baud rate calculation, and interrupt handling logic.
* `GPIO_HAL.h`: Definitions for Port and Pin modes (Analog vs. Alternate Function).
* `Common_BASES.h`: Contains the base memory addresses for all peripherals used.

---

## 📝 How to Use

1. Connect the LM35 to **PA0** on your STM32F429.
2. Flash the code using **STM32CubeIDE**.
3. Open a Serial Terminal (like PuTTY) set to **9600 Baud**.
4. Observe the real-time temperature updates every 1 second.

---

### What I can do for you next:

Would you like me to help you **add a "Calibration Offset"** to the code to account for small inaccuracies in your specific LM35 sensor?
