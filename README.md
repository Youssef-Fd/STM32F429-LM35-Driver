## Project Overview

This project implements a **temperature measurement system** on an **STM32 microcontroller** using **custom-written HAL drivers** (no STM32Cube HAL usage).

The system:

* Reads an **analog temperature sensor** via **ADC**
* Converts the ADC value to **temperature in degrees Celsius**
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

* **Microcontroller**: STM32F429ZIT6 (STM32F4 series)
* **Temperature Sensor**: LM35 Analog Temperature Sensor
* **IDE**: STM32CubeIDE
* **Communication**: UART (USART1 @ 9600 baud)
* **Language**: C
* **Terminal Software**: PuTTY or equivalent serial monitor

---

## Project Structure

```
STM32_LM35_Project/
├── Src/
│   ├── main.c
│   ├── ADC_HAL.c
│   └── UART_HAL.c
├── Inc/
│   ├── ADC_HAL.h
│   ├── USART_HAL.h
│   ├── GPIO_HAL.h
│   └── Common_BASES.h
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
void HAL_ADC_Init(void);
void HAL_ADC_Start(uint8_t channel);
void HAL_ADC_WaitForConversion(void);
uint16_t HAL_ADC_Get_Value(void);
```

**ADC Configuration Details:**

* Resolution: 10-bit (0-1023 range)
* Clock Prescaler: /4 (ADC clock = 21 MHz)
* Sample Time: 3 ADC clock cycles
* Conversion Sequence: 1 channel (Channel 0 on PA0)

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

* Baud rate: 9600
* Word length: 8 bits
* Stop bits: 1
* Parity: None
* Oversampling: 16x

**Main UART APIs:**

```c
void hal_uart_init(uart_handle_t *uart_handle);
void hal_uart_tx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len);
void hal_uart_rx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len);
void hal_uart_handle_interrupt(uart_handle_t *huart);
```

---

### GPIO Configuration

* USART1 pins configured as Alternate Function:
  * **PA9** - TX (Transmit)
  * **PA10** - RX (Receive)
* ADC pin configured as Analog mode:
  * **PA0** - ADC input

---

## Main Application Flow

1. Initialize GPIO, UART, and ADC
2. Start ADC conversion on channel 0 (PA0)
3. Wait for conversion completion (EOC flag)
4. Read ADC value (0-1023)
5. Convert ADC value to temperature in degrees Celsius
6. Send temperature over UART using interrupt-driven transmission
7. Wait 1 second
8. Repeat every second indefinitely

---

## Temperature Conversion Logic

```c
temperature_mC = (ADC_value * 3300) / 1023;
Temp_int = temperature_mC / 100;
Temp_dec = temperature_mC % 100;
```

Where:
* ADC_value: Raw 10-bit ADC result (0-1023)
* temperature_mC: Temperature in millidegrees Celsius
* Temp_int: Integer part (degrees)
* Temp_dec: Decimal part (hundredths of degree)

Output format: "Temperature: XX.YY C"

---

# USART Interrupt Handling (UART HAL)

The UART interrupt mechanism is based on the **STM32 interrupt vector table** and a **centralized interrupt handler** implemented in `hal_uart_handle_interrupt()`.

---

## Interrupt Vector Table

The STM32 interrupt vector table defines which **ISR (Interrupt Service Routine)** is executed when a peripheral raises an interrupt.

![image alt](https://github.com/Youssef-Fd/STM32F429-LM35-Driver/blob/7800e0e9b77388264d055e0965b20e8faa2495cd/Interrupt_vector%20table.png)

Each USART peripheral maps to a fixed IRQ number and ISR name:

| Peripheral | IRQ Number | ISR Name          |
|-----------|-----------|-------------------|
| USART1    | 37        | USART1_IRQHandler |
| USART2    | 38        | USART2_IRQHandler |
| USART3    | 39        | USART3_IRQHandler |
| UART4     | 52        | UART4_IRQHandler  |
| UART5     | 53        | UART5_IRQHandler  |
| USART6    | 71        | USART6_IRQHandler |

These IRQ numbers are defined in the driver:

```c
#define USART1_IRQ_NUM  37
#define USART2_IRQ_NUM  38
#define USART3_IRQ_NUM  39
```

---

## ISR Entry Point (Vector - Driver)

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
|---------------|-------------|-----------|
| Parity Error  | PE          | PEIE (CR1)|
| Framing Error | FE          | EIE (CR3) |
| Noise Error   | NE          | EIE (CR3) |
| Overrun Error | ORE         | EIE (CR3) |

Error flags are cleared by reading `SR` followed by `DR`:

```c
static void hal_uart_clear_error_flag(uart_handle_t *huart)
{
    volatile uint32_t tmp;
    tmp = huart->Instance->SR;  /* Read Status Register */
    tmp = huart->Instance->DR;  /* Read Data Register to clear flags */
}
```

When an error is detected:
* Error code is accumulated in huart->ErrorCode
* Error callback is invoked (if registered)
* TX/RX state machines are reset to READY

---

### 2. RXNE Interrupt (Receive Data Register Not Empty)

Triggered when new data is received.

Conditions:

* RXNE flag set in `SR`
* RXNE interrupt enabled in `CR1`

Handled by:

```c
static void hal_uart_handle_RXNE_interrupt(uart_handle_t *huart)
```

RX behavior:

* Reads `DR` (clears RXNE)
* Handles parity / no-parity modes:
  * No Parity: Reads all 8 data bits
  * With Parity: Masks to 7 bits (parity is MSB)
* Stores received data in buffer
* Decrements RX counter
* When complete:
  * Disables RXNE interrupt
  * Disables parity error interrupt
  * Disables general error interrupts
  * Sets RX state to READY
  * Calls user RX complete callback

---

### 3. TXE Interrupt (Transmit Data Register Empty)

Triggered when UART is ready to transmit the next byte.

Conditions:

* TXE flag set in `SR`
* TXE interrupt enabled in `CR1`

Handled by:

```c
static void hal_uart_handle_TXE_interrupt(uart_handle_t *huart)
```

TXE behavior:

* Loads next byte from buffer into `DR`
* Increments buffer pointer
* Decrements TX counter
* If more data to send:
  * TXE interrupt remains enabled
  * UART will trigger TXE again when ready
* If last byte was sent:
  * Disables TXE interrupt
  * Enables TC interrupt to wait for transmission complete

---

### 4. TC Interrupt (Transmission Complete)

Triggered when the **entire frame** has been transmitted (including stop bits).

Handled by:

```c
static void hal_uart_handle_TC_interrupt(uart_handle_t *huart)
```

TC behavior:

* Disables TC interrupt
* Resets TX state to READY
* Calls user TX complete callback (if registered)

**Important Distinction:**
* TXE: Indicates shift register is ready for new data (buffer empty)
* TC: Indicates complete frame has left the UART (transmission complete)

---

## Interrupt Execution Flow

```
USART Hardware Event (RX, TX, or Error)
        |
        v
NVIC IRQ Trigger
        |
        v
CPU Executes USARTx_IRQHandler()
        |
        v
hal_uart_handle_interrupt()
        |
        +--- Check Parity Error (PE) Flag
        |    - If set and enabled, clear and log error
        |
        +--- Check Framing Error (FE) Flag
        |    - If set and enabled, clear and log error
        |
        +--- Check Noise Error (NE) Flag
        |    - If set and enabled, clear and log error
        |
        +--- Check Overrun Error (ORE) Flag
        |    - If set and enabled, clear and log error
        |
        +--- Check RXNE Interrupt
        |    - If flag set and enabled, call RXNE handler
        |    - Receive data into buffer
        |
        +--- Check TXE Interrupt
        |    - If flag set and enabled, call TXE handler
        |    - Transmit next byte
        |
        +--- Check TC Interrupt
        |    - If flag set and enabled, call TC handler
        |    - Complete transmission
        |
        +--- Check Accumulated Errors
        |    - If errors detected, invoke error callback
        |
        v
Return from Interrupt
```

---

## State Machines

### TX State Machine

```
State: READY
    |
    +--- Application calls hal_uart_tx()
    |    - Set state to BUSY_TX
    |    - Enable TXE interrupt
    |
    v
State: BUSY_TX
    |
    +--- TXE Interrupt fires repeatedly
    |    - Send one byte from buffer
    |    - Increment buffer pointer
    |    - Decrement TX counter
    |    - If counter > 0: Stay in BUSY_TX
    |    - If counter = 0: Enable TC interrupt
    |
    v
State: BUSY_TX (Waiting for TC)
    |
    +--- TC Interrupt fires
    |    - Disable TC interrupt
    |    - Set state to READY
    |    - Call TX completion callback
    |
    v
State: READY
```

### RX State Machine

```
State: READY
    |
    +--- Application calls hal_uart_rx()
    |    - Set state to BUSY_RX
    |    - Enable RXNE interrupt
    |
    v
State: BUSY_RX
    |
    +--- RXNE Interrupt fires repeatedly
    |    - Read byte from DR
    |    - Store in buffer
    |    - Increment buffer pointer
    |    - Decrement RX counter
    |    - If counter > 0: Stay in BUSY_RX
    |    - If counter = 0: Disable interrupts
    |
    v
State: BUSY_RX (Reception Complete)
    |
    +--- Last byte received
    |    - Set state to READY
    |    - Call RX completion callback
    |
    v
State: READY
```

---

## Timing Characteristics

### ADC Conversion Time

* Sample Time: 3 ADC clock cycles
* Conversion Time: 12 ADC clock cycles, fixed hardware characteristic 
* Total: 15 ADC cycles = 714 nanoseconds (at 21 MHz ADC clock)

### UART Transmission Time

* Baud Rate: 9600 bps = 104 microseconds per bit
* Frame Time: 1 start + 8 data + 1 stop = 10 bits = 1.04 milliseconds per character
* Example "Temperature: 25.00 C\r\n" (22 characters): 23 milliseconds

---

## Register Quick Reference

### ADC Registers

* **CR1**: Control register 1 (resolution, EOCIE, SCAN mode)
* **CR2**: Control register 2 (ADON, CONT, SWSTART)
* **SMPR2**: Sample time register 2 (channel 0-9 sample times)
* **SQR1**: Regular sequence register 1 (sequence length in bits 23:20)
* **SQR3**: Regular sequence register 3 (channel selection in bits 4:0)
* **SR**: Status register (EOC flag in bit 1)
* **DR**: Data register (raw conversion result)

### USART Registers

* **SR**: Status register (TXE bit 7, TC bit 6, RXNE bit 5, error bits 0-3)
* **DR**: Data register (8-bit TX/RX data)
* **BRR**: Baud rate register (0x0683 for 9600 baud at 42 MHz clock)
* **CR1**: Control register 1 (UE bit 13, TE bit 3, RE bit 2, interrupt enables)
* **CR2**: Control register 2 (stop bits in bits 13:12)
* **CR3**: Control register 3 (error interrupt enable in bit 0)

---

## Debugging and Troubleshooting

### No UART Output

1. Verify USART1 clock is enabled: RCC->APB2ENR bit 4
2. Check PA9/PA10 configured as Alternate Function 7
3. Confirm baud rate register: BRR = 0x0683 for 9600
4. Verify USART1_IRQHandler is in vector table
5. Use oscilloscope to probe PA9 for signal activity

### ADC Reads Zero or Maximum Values

1. Confirm PA0 is in analog mode: MODER[1:0] = 11
2. Verify ADC clock prescaler is set (/4)
3. Check sample time is adequate: SMPR2[2:0] >= 3
4. Verify ADC conversion is triggered: Check SR.STRT bit
5. Check reference voltage present on ADC supply

### Garbled Serial Data

1. Verify baud rate matches terminal (9600 bps)
2. Check for stack overflow in interrupt handlers
3. Reduce transmission frequency if necessary
4. Add delays between consecutive transmissions
5. Verify USB-to-serial adapter quality

### Transmission Interrupted

1. Check NVIC interrupt priority settings
2. Verify no critical sections block interrupts excessively
3. Monitor TX state machine for stuck states
4. Check for recursive interrupt conditions

---

## Expected Output

When system operates correctly, terminal should display:

![image alt](https://github.com/Youssef-Fd/STM32F429-LM35-Driver/blob/f88eb2df29aa58a7038ee1ff83154efd15771400/LM35_TEST.png)

Each line updates approximately every 1 second. Temperature values will vary based on sensor readings.

---

## References

* STM32F429 Reference Manual (RM0090)
* STM32F429 Datasheet
* ARM Cortex-M4 Generic User Guide
* LM35 Precision Centigrade Temperature Sensors Datasheet

---
