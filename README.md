# STM32F429 LM35 Temperature Monitoring System

## Project Overview

This project implements a comprehensive **temperature measurement system** on an **STM32F429** microcontroller using **custom-written HAL drivers** without relying on STM32Cube HAL libraries.

The system reads an analog temperature sensor (LM35) via the ADC peripheral and transmits temperature data periodically over UART (USART1) using interrupt-driven transmission.

### Key Features

- Direct register-level programming (bare-metal approach)
- Custom Hardware Abstraction Layer (HAL) drivers
- 10-bit ADC resolution with continuous conversion mode
- Interrupt-driven UART transmission with complete error handling
- Polling-based ADC conversion with software trigger
- Professional-grade code structure and documentation

---

## Learning Objectives

Upon completing this project, you will understand:

- Direct hardware register manipulation for STM32 peripherals
- Design and implementation of reusable HAL-like drivers
- ADC peripheral configuration and conversion management
- UART communication using interrupt-driven architecture
- NVIC interrupt controller programming
- State machine design for asynchronous communication
- Error detection and handling in embedded systems
- Modular embedded software architecture

---

## Hardware Requirements

- **Microcontroller**: STM32F429ZIT6 (STM32F4 series)
- **Temperature Sensor**: LM35 Analog Temperature Sensor
- **IDE**: STM32CubeIDE
- **Communication Interface**: USB-to-Serial adapter
- **Terminal Software**: PuTTY or equivalent serial monitor
- **Development Board**: STM32F429 Discovery board (optional)

### Pin Connections

| Component | STM32 Pin | Function |
|-----------|-----------|----------|
| LM35 Output | PA0 | ADC Channel 0 |
| UART TX | PA9 | USART1 Transmit |
| UART RX | PA10 | USART1 Receive |
| GND | GND | Common Ground |
| VCC | 3.3V | Power Supply |

---

## Project Structure

```
STM32_LM35_Project/
├── Src/
│   ├── main.c              (Main application)
│   ├── ADC_HAL.c           (ADC driver implementation)
│   ├── UART_HAL.c          (UART driver implementation)
│   └── startup_stm32f429zitx.s
├── Inc/
│   ├── ADC_HAL.h           (ADC driver header)
│   ├── USART_HAL.h         (UART driver header)
│   ├── GPIO_HAL.h          (GPIO definitions)
│   └── Common_BASES.h      (RCC and NVIC definitions)
├── Drivers/
│   └── CMSIS/
└── README.md
```

---

## System Architecture

### Block Diagram

```
LM35 Sensor (Analog Output)
    |
    v
PA0 (ADC Input)
    |
    v
ADC1 Peripheral (10-bit conversion)
    |
    v
Temperature Calculation (Integer Math)
    |
    v
USART1 (Interrupt-Driven TX)
    |
    v
Serial Output (9600 baud)
    |
    v
PuTTY Terminal Display
```

---

## Peripheral Configuration

### ADC (Analog-to-Digital Converter)

**Purpose**: Convert analog temperature sensor output to digital value

**Configuration**:
- Resolution: 10-bit (0-1023 range)
- Clock Prescaler: /4 (21 MHz ADC clock from 84 MHz APB2)
- Conversion Mode: Continuous software-triggered
- Sample Time: 3 ADC clock cycles (fastest)
- Channel: Channel 0 (PA0)
- Data Alignment: Right-aligned
- Sequence Length: 1 conversion

**Key Registers**:
- CR1: Resolution, scan mode, EOCIE (EOC interrupt enable)
- CR2: ADC enable, continuous mode, software start trigger
- SMPR2: Sample time for channel 0
- SQR1: Conversion sequence length
- SQR3: Channel selection
- SR: Status register (EOC flag)
- DR: Data register (conversion result)

**Conversion Flow**:
1. Call HAL_ADC_Start(0) - Initiates software trigger
2. Poll ADC1->SR EOC flag - Wait for conversion complete
3. Call HAL_ADC_Get_Value() - Read 10-bit result (0-1023)
4. Conversion time: 15 ADC cycles = 714 nanoseconds at 21 MHz

### UART (Universal Asynchronous Receiver/Transmitter)

**Purpose**: Serial communication for temperature data transmission

**Configuration**:
- Baud Rate: 9600 bits per second
- Data Bits: 8
- Stop Bits: 1
- Parity: None
- Oversampling: 16x (default)
- Hardware: USART1 on APB2 clock domain

**Pin Configuration**:
- PA9: TX (Transmit) - Alternate Function 7
- PA10: RX (Receive) - Alternate Function 7

**Key Registers**:
- SR: Status register (TXE, TC, RXNE, PE, FE, NE, ORE flags)
- DR: Data register (8-bit transmit/receive)
- BRR: Baud rate register (0x0683 for 9600 baud)
- CR1: Control register 1 (UE, TE, RE, interrupt enables)
- CR2: Control register 2 (stop bits, clock)
- CR3: Control register 3 (error interrupt enable)

**Interrupt-Driven Transmission**:
1. Application calls hal_uart_tx() with data buffer
2. TXE interrupt fires when TX data register is empty
3. Driver sends one byte per TXE interrupt
4. After last byte, TC interrupt fires when frame complete
5. Driver sets TX state to READY and calls completion callback

**Error Handling**:
- Parity Error (PE): Character received with wrong parity bit
- Framing Error (FE): Missing or incorrect stop bit
- Noise Error (NE): Noise detected during reception
- Overrun Error (ORE): New data received before previous data read

### GPIO (General Purpose Input/Output)

**Purpose**: Configure pins for UART and ADC functionality

**Configuration**:
- GPIOA Clock: Enabled via RCC->AHB1ENR
- PA0: Analog mode (ADC input)
- PA9: Alternate Function 7 (USART1 TX)
- PA10: Alternate Function 7 (USART1 RX)

---

## Temperature Conversion

### LM35 Sensor Specifications

- Output Voltage: 10 millivolts per degree Celsius
- Temperature Range: -55°C to +150°C
- Accuracy: ±0.5°C
- Output Impedance: 0.1 ohms

### Conversion Formula

```c
/* Raw ADC value to temperature in millidegrees Celsius */
temperature_mC = (ADC_value * 3300) / 1023;

/* Extract integer and decimal parts */
temperature_C_int = temperature_mC / 100;
temperature_C_dec = temperature_mC % 100;

/* Display format: "Temperature: XX.YY C" */
```

### Example Calculations

| ADC Value | Voltage | Temperature |
|-----------|---------|-------------|
| 0         | 0.0V    | 0.00°C      |
| 102       | 0.33V   | 3.27°C      |
| 204       | 0.66V   | 6.55°C      |
| 512       | 1.65V   | 16.37°C     |
| 1023      | 3.3V    | 32.75°C     |

---

## UART Interrupt Architecture

### Interrupt Vector Table

The STM32F4 uses a fixed interrupt vector table that maps peripheral interrupts to handler functions.

| Peripheral | IRQ Number | Handler Name |
|------------|-----------|--------------|
| USART1     | 37        | USART1_IRQHandler |
| USART2     | 38        | USART2_IRQHandler |
| USART3     | 39        | USART3_IRQHandler |
| UART4      | 52        | UART4_IRQHandler |
| UART5      | 53        | UART5_IRQHandler |
| USART6     | 71        | USART6_IRQHandler |

### ISR Entry Point

```c
void USART1_IRQHandler(void)
{
    hal_uart_handle_interrupt(&huart1);
}
```

The handler entry point is minimal by design. All complex interrupt logic is delegated to the central UART driver function.

### Central Interrupt Handler Flow

```
USART1_IRQHandler()
    |
    v
hal_uart_handle_interrupt(&huart1)
    |
    +---> Check and Handle Parity Error (PE)
    |
    +---> Check and Handle Framing Error (FE)
    |
    +---> Check and Handle Noise Error (NE)
    |
    +---> Check and Handle Overrun Error (ORE)
    |
    +---> Check and Handle RXNE Interrupt (Receive)
    |
    +---> Check and Handle TXE Interrupt (Transmit)
    |
    +---> Check and Handle TC Interrupt (Transmission Complete)
    |
    v
Dispatch to appropriate sub-handler or callback
```

### Interrupt Types and Handlers

**Error Interrupts**:
- Read SR register to check error flags
- Read DR register to clear error conditions
- Update huart->ErrorCode with error type
- Reset TX/RX state machines
- Invoke error callback (if registered)

**RXNE Interrupt** (Receive Data Register Not Empty):
- Triggered when data received in DR
- Read byte from DR (clears RXNE flag)
- Store in user buffer
- Decrement RX counter
- When complete, disable interrupts and call RX callback

**TXE Interrupt** (Transmit Data Register Empty):
- Triggered when TX register can accept new byte
- Send next byte from application buffer
- Increment pointer and decrement counter
- When last byte sent, disable TXE and enable TC

**TC Interrupt** (Transmission Complete):
- Triggered when entire frame transmitted
- Disable TC interrupt
- Reset TX state to READY
- Call TX completion callback

---

## Application Flow

### Initialization Sequence

1. Enable RCC clocks for GPIOA, USART1, and ADC1
2. Configure GPIO pins:
   - PA0 as analog input
   - PA9/PA10 as alternate function
3. Initialize UART with parameters
4. Enable USART1 interrupt in NVIC
5. Initialize ADC with 10-bit resolution
6. Enable global interrupts

### Main Loop Operation

```
while(1)
{
    1. Start ADC conversion on channel 0
    2. Poll EOC flag until conversion complete
    3. Read 10-bit ADC result (0-1023)
    4. Calculate temperature using integer math
    5. Format temperature string: "Temperature: XX.YY C\r\n"
    6. Transmit via UART using interrupt-driven hal_uart_tx()
    7. Wait 1 second (application blocks during delay)
    8. Repeat
}
```

### Expected UART Output

```
Temperature: 25.00 C
Temperature: 25.04 C
Temperature: 24.98 C
Temperature: 25.02 C
...
```

---

## Driver API Reference

### ADC Driver (ADC_HAL)

```c
void HAL_ADC_Init(void);
```
Initializes ADC1 peripheral. Must be called once during startup.

```c
void HAL_ADC_Start(uint8_t channel);
```
Starts conversion on specified channel (0-18). Sets channel in SQR3 and triggers SWSTART.

```c
void HAL_ADC_WaitForConversion(void);
```
Blocks until EOC flag is set. Returns when conversion complete.

```c
uint16_t HAL_ADC_Get_Value(void);
```
Returns raw ADC conversion result (0-1023 for 10-bit resolution).

### UART Driver (UART_HAL)

```c
void hal_uart_init(uart_handle_t *uart_handle);
```
Initializes UART peripheral with parameters from uart_handle->Init structure.

```c
void hal_uart_tx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len);
```
Starts non-blocking interrupt-driven transmission. Returns immediately; interrupt handler manages actual transmission.

```c
void hal_uart_rx(uart_handle_t *uart_handle, uint8_t *buffer, uint32_t len);
```
Starts non-blocking interrupt-driven reception into user buffer.

```c
void hal_uart_handle_interrupt(uart_handle_t *huart);
```
Central interrupt handler. Called from USART1_IRQHandler(). Processes all interrupt types and manages state machines.

---

## Timing Analysis

### ADC Conversion Timing

- Sample Time: 3 ADC clock cycles
- Conversion Time: 12 ADC clock cycles
- Total: 15 ADC clock cycles = 714 nanoseconds (at 21 MHz)
- With polling loop overhead: Approximately 1-2 microseconds

### UART Transmission Timing

- Baud Rate: 9600 bps
- Time per bit: 104 microseconds
- Frame time (1 start + 8 data + 1 stop): 1.04 milliseconds
- Example string "Temperature: 25.00 C\r\n" (22 chars): 23 milliseconds

### Application Loop Timing

- ADC conversion: 1-2 microseconds
- Temperature calculation: Less than 1 microsecond
- UART transmission (non-blocking): Returns immediately
- Main loop iteration: Less than 1 millisecond
- Delay(1000): 1 second blocking delay
- Overall: One temperature reading per second

---

## Debugging and Troubleshooting

### No UART Output

1. Verify USART1 clock is enabled in RCC->APB2ENR
2. Check PA9/PA10 are configured as alternate function mode
3. Confirm baud rate register is set correctly (0x0683 for 9600)
4. Verify USART1_IRQHandler is properly linked in vector table
5. Use oscilloscope to probe PA9 for signal activity

### ADC Reads Zero or Max Values

1. Check PA0 is in analog mode (MODER[1:0] = 11)
2. Verify ADC clock prescaler is set (/4)
3. Confirm sample time is adequate (3+ cycles for sensors)
4. Check ADC conversion is actually triggered (SR register)
5. Verify reference voltage is present on ADC supply

### Garbled or Missing Characters

1. Verify baud rate matches terminal settings (9600)
2. Check for stack overflow (interrupt handlers use stack space)
3. Reduce transmission frequency or optimize string formatting
4. Add delays between transmissions if necessary
5. Check USB-to-serial adapter quality and drivers

### Interrupted Transmissions

1. Verify interrupt priority allows USART1 to preempt other ISRs
2. Ensure no critical sections disable interrupts for extended periods
3. Check that callbacks are not creating recursive interrupt conditions
4. Monitor TX state machine for stuck states

---

## Performance Metrics

| Metric | Value |
|--------|-------|
| ADC Resolution | 10-bit (0-1023) |
| ADC Conversion Time | 714 ns per sample |
| Temperature Precision | 0.1°C (1 LSB = 3.22 mV) |
| UART Baud Rate | 9600 bps |
| UART Transmission Time | 1.04 ms per character |
| Temperature Update Rate | 1 Hz (configurable) |
| Interrupt Latency | <1 microsecond |
| Code Size | ~8 KB (approximation) |

---

## Design Patterns Used

### 1. Hardware Abstraction Layer (HAL)

All peripheral access is abstracted through driver functions, making code portable and maintainable.

### 2. State Machine

UART TX/RX operations use state machines to manage complex asynchronous communication:
- READY: No operation in progress
- BUSY_TX: Transmission in progress
- BUSY_RX: Reception in progress

### 3. Interrupt-Driven I/O

UART transmission is fully interrupt-driven, allowing the application to continue executing while data transmits in the background.

### 4. Callback Functions

Applications can register completion callbacks for TX/RX operations:
```c
huart1.tx_cmp_cb = my_tx_complete_callback;
```

---

## Future Enhancements

Possible extensions to this project:

1. **DMA Support**: Implement DMA-based ADC and UART transfers for higher throughput
2. **Temperature Buffering**: Store readings in circular buffer for data logging
3. **LCD Display**: Add I2C LCD for local temperature display
4. **Wireless Transmission**: Add RF module for remote monitoring
5. **Data Logging**: Store temperature history in flash memory
6. **Multi-Channel ADC**: Read multiple sensors simultaneously
7. **CRC Checksum**: Add error detection to transmitted frames
8. **Low Power Modes**: Implement sleep modes for battery operation

---

## References

- STM32F429 Reference Manual (RM0090)
- STM32F429 Datasheet
- ARM Cortex-M4 Generic User Guide
- LM35 Precision Centigrade Temperature Sensors Datasheet

---

## License

This project is provided as-is for educational and reference purposes.

---

## Author Notes

This project demonstrates professional-grade embedded systems development without reliance on vendor HAL libraries. Understanding register-level programming is essential for:

- Optimizing performance in critical systems
- Debugging complex hardware issues
- Porting code to different microcontroller families
- Developing custom peripheral drivers
- Understanding how higher-level libraries work internally

The concepts presented here are fundamental to embedded systems programming and apply across different microcontroller platforms.

---

## Contact and Support

For questions or issues regarding this project, refer to the inline code documentation and the reference materials listed above.

Last Updated: January 2026
