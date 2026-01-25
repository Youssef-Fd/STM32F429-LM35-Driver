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

# USART Interrupt Handling (UART HAL)

The UART interrupt mechanism is based on the **STM32 interrupt vector table** (RM0090) and a **centralized interrupt handler** implemented in `hal_uart_handle_interrupt()`.

---

## Interrupt Vector Table Reference

The STM32 interrupt vector table defines which **ISR (Interrupt Service Routine)** is executed when a peripheral raises an interrupt.

📷 Reference from RM0090 (STM32F42x / STM32F43x):

> Image path in repository:

```
interrupt_vector_table.png
```

Each USART peripheral maps to a fixed IRQ number and ISR name:

| Peripheral | IRQ Number | ISR Name          |
| ---------- | ---------- | ----------------- |
| USART1     | 37         | USART1_IRQHandler |
| USART2     | 38         | USART2_IRQHandler |
| USART3     | 39         | USART3_IRQHandler |

These IRQ numbers are defined in the driver:

```c
#define USART1_IRQ_NUM  37
#define USART2_IRQ_NUM  38
#define USART3_IRQ_NUM  39
```

---

## ISR Entry Point (Vector → Driver)

For each enabled USART, the vector table jumps to the corresponding ISR. The ISR **must be named exactly as defined by the vector table**.

Example (USART1):

```c
void USART1_IRQHandler(void)
{
    hal_uart_handle_interrupt(&huart1);
}
```

The ISR itself is intentionally minimal. All interrupt logic is handled inside the UART driver.

---

## Central Interrupt Handler

All USART-related interrupts are processed by:

```c
void hal_uart_handle_interrupt(uart_handle_t *huart);
```

This function:

* Reads **status flags** from `USART_SR`
* Checks **interrupt enable bits** in `CR1` and `CR3`
* Dispatches handling to internal sub-handlers
* Manages TX/RX state machines
* Handles UART error conditions

---

## Interrupt Types Handled

### 1. Error Interrupts

The following UART errors are handled:

| Error         | Status Flag | Enable Bit |
| ------------- | ----------- | ---------- |
| Parity Error  | PE          | PEIE (CR1) |
| Framing Error | FE          | EIE (CR3)  |
| Noise Error   | NE          | EIE (CR3)  |
| Overrun Error | ORE         | EIE (CR3)  |

Error flags are cleared by reading `SR` followed by `DR`:

```c
static void hal_uart_clear_error_flag(uart_handle_t *huart)
{
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;
    tmp = huart->Instance->DR;
}
```

On fatal error, the driver calls `hal_uart_error_cb()` which indicates the error by blinking LED **PG13**.

---

### 2. RXNE Interrupt (Receive Data Register Not Empty)

Triggered when new data is received.

Conditions:

* RXNE flag set in `SR`
* RXNE interrupt enabled in `CR1`

Handled by:

```c
hal_uart_handle_RXNE_interrupt(huart);
```

RX behavior:

* Reads `DR` (clears RXNE)
* Handles parity / no-parity modes
* Stores received data in buffer
* Decrements RX counter
* Disables RX interrupts when transfer completes
* Calls user RX complete callback

---

### 3. TXE Interrupt (Transmit Data Register Empty)

Triggered when UART is ready to transmit the next byte.

Conditions:

* TXE flag set in `SR`
* TXE interrupt enabled in `CR1`

Handled by:

```c
hal_uart_handle_TXE_interrupt(huart);
```

TXE behavior:

* Loads next byte into `DR`
* Decrements TX counter
* Disables TXE interrupt when buffer is empty
* Enables TC interrupt

---

### 4. TC Interrupt (Transmission Complete)

Triggered when the **entire frame** has been transmitted (including stop bits).

Handled by:

```c
hal_uart_handle_TC_interrupt(huart);
```

TC behavior:

* Disables TC interrupt
* Resets TX state to READY
* Calls user TX complete callback

---

## Interrupt Execution Flow

```
USART Hardware Event
        ↓
NVIC IRQ Trigger
        ↓
USARTx_IRQHandler()
        ↓
hal_uart_handle_interrupt()
        ↓
 ├─ Error handling
 ├─ RXNE handling
 ├─ TXE handling
 └─ TC handling
```

