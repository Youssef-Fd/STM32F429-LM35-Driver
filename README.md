# STM32 Temperature Measurement using Custom ADC & UART HAL

## Project Overview

This project implements a **temperature measurement system** on an **STM32 microcontroller** using **custom-written HAL drivers** (no STM32Cube HAL usage).

The system:

* Reads an **analog temperature sensor** via **ADC**
* Converts the ADC value to **temperature in °C**
* Sends the temperature periodically over **UART (USART1)** using **interrupt-based transmission**

The goal of this project is to **understand low-level peripheral programming**, register manipulation, and **driver abstraction** in embedded systems.

---

## Key Learning Objectives

* Direct register-level programming (bare-metal)
* Writing reusable **HAL-like drivers**
* ADC configuration and polling-based conversion
* UART communication using **interrupts**
* NVIC interrupt handling
* Modular embedded software design

---

## Hardware & Tools

* **Microcontroller**: STM32 (tested on STM32F4 series)
* **Temperature Sensor**: Analog (e.g. LM35)
* **IDE**: STM32CubeIDE
* **Communication**: UART (USART1 @ 9600 baud)
* **Language**: C

---

## Project Structure

```
├── main.c
├── ADC_HAL.c
├── ADC_HAL.h
├── UART_HAL.c
├── UART_HAL.h
├── GPIO_HAL.h
├── Common_BASES.h
└── README.md
```

---

## System Architecture

### ADC Driver (ADC_HAL)

* Configured in **10-bit resolution**
* Continuous conversion mode
* Polling-based conversion (EOC flag)
* Channel selected dynamically via `HAL_ADC_Start(channel)`

**Main ADC APIs:**

```c
HAL_ADC_Init();
HAL_ADC_Start(channel);
HAL_ADC_WaitForConversion();
HAL_ADC_Get_Value();
```

---

### UART Driver (UART_HAL)

* Fully custom UART driver
* Interrupt-driven transmission and reception
* Supports:

  * TXE (Transmit Data Register Empty)
  * TC (Transmission Complete)
  * RXNE (Receive Not Empty)
  * Error handling (PE, FE, NE, ORE)

**UART Configuration:**

* Baud rate: `9600`
* Word length: `8 bits`
* Stop bits: `1`
* Parity: `None`
* Oversampling: `16`

**Main UART APIs:**

```c
hal_uart_init();
hal_uart_tx();
hal_uart_rx();
hal_uart_handle_interrupt();
```

---

### GPIO Configuration

* USART1 pins configured manually:

  * **PA9** → TX
  * **PA10** → RX
* GPIO set to **Alternate Function mode**
* ADC pin configured as **Analog mode**

---

## Main Application Flow

1. Initialize GPIO, UART, and ADC
2. Start ADC conversion on selected channel
3. Wait for conversion completion
4. Read ADC value
5. Convert ADC value to temperature (°C)
6. Send temperature over UART
7. Repeat every 1 second

---

## Temperature Conversion Logic

```c
temperature_C = (ADC_value * 3300) / 1023;
Temp_int = temperature_C / 100;
Temp_dec = temperature_C % 100;
```


