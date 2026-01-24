/*
 * UART_HAL.h
 *
 *  Created on: Jan 19, 2026
 *      Author: Lenovo
 */

#ifndef USART_HAL_H_
#define USART_HAL_H_

#include "Common_BASES.h"

/************* NVIC (Nested Vectored Interrupt Controller) *************/

#define USART1_IRQ_NUM       37
#define USART2_IRQ_NUM       38
#define USART3_IRQ_NUM       39
#define UART4_IRQ_NUM        52
#define UART5_IRQ_NUM        53
#define USART6_IRQ_NUM       71
#define UART7_IRQ_NUM        82
#define UART8_IRQ_NUM        83


/* UART possible error codes */
#define HAL_UART_ERROR_NONE  ((uint32_t)0x00000000) /* No error */
#define HAL_UART_ERROR_PE    ((uint32_t)0x00000001) /* Parity error */
#define HAL_UART_ERROR_NE    ((uint32_t)0x00000002) /* Noise error */
#define HAL_UART_ERROR_FE    ((uint32_t)0x00000004) /* Frame error */
#define HAL_UART_ERROR_ORE   ((uint32_t)0x00000008) /* Overrun error */
#define HAL_UART_ERROR_DMA   ((uint32_t)0x00000010) /* DMA transfer error */


/* Different USART and UART peripheral base addresses */
typedef struct {
    volatile uint32_t SR;    /* USART Status register    */
    volatile uint32_t DR;    /* USART Data register      */
    volatile uint32_t BRR;   /* USART Baud rate register */
    volatile uint32_t CR1;   /* USART Control register 1 */
    volatile uint32_t CR2;   /* USART Control register 2 */
    volatile uint32_t CR3;   /* USART Control register 1  */
    volatile uint32_t GTPR;  /* USART Guard time and prescaler register */
} USART_TypeDef;

#define USART_1    ((USART_TypeDef *)0x40011000)
#define USART_2    ((USART_TypeDef *)0x40004400)
#define USART_3    ((USART_TypeDef *)0x40004800)
#define USART_4    ((USART_TypeDef *)0x40004C00)
#define USART_5    ((USART_TypeDef *)0x40005000)
#define USART_6    ((USART_TypeDef *)0x40011400)
#define USART_7    ((USART_TypeDef *)0x40007800)
#define USART_8    ((USART_TypeDef *)0x40007C00)

/*Macros to enable the clocks for various USART/UART */
#define __HAL_RCC_USART1_CLK_ENABLE()          ( RCC->APB2ENR |=  ( 1 << 4))
#define __HAL_RCC_USART2_CLK_ENABLE()          ( RCC->APB1ENR |=  ( 1 << 17))
#define __HAL_RCC_USART3_CLK_ENABLE()          ( RCC->APB1ENR |=  ( 1 << 18))
#define __HAL_RCC_UART4_CLK_ENABLE()           ( RCC->APB1ENR |=  ( 1 << 19))
#define __HAL_RCC_UART5_CLK_ENABLE()           ( RCC->APB1ENR |=  ( 1 << 20))
#define __HAL_RCC_USART6_CLK_ENABLE()          ( RCC->APB2ENR |=  ( 1 << 5))
#define __HAL_RCC_USART7_CLK_ENABLE()          ( RCC->APB2ENR |=  ( 1 << 30))
#define __HAL_RCC_USART8_CLK_ENABLE()          ( RCC->APB2ENR |=  ( 1 << 31))


/******************************************************************************/
/*                                                                            */
/*                                 UART                                       */
/*                        Register Bit Definintions                           */
/*                                                                            */
/******************************************************************************/

/****************** Bit definition for USART_SR register ******************/
#define USART_REG_SR_CTS_FLAG                  ( (uint32_t) ( 1 << 9 ) )  /* CTS flag */
#define USART_REG_SR_LBD_FLAG                  ( (uint32_t) ( 1 << 8 ) )  /* LIN Break Detection Flag */
#define USART_REG_SR_TXE_FLAG                  ( (uint32_t) ( 1 << 7 ) )  /* Transmit Data Register Empty flag */
#define USART_REG_SR_TC_FLAG                   ( (uint32_t) ( 1 << 6 ) )  /* Transmission Complete flag */
#define USART_REG_SR_RXNE_FLAG                 ( (uint32_t) ( 1 << 5 ) )  /* Read Data Register Not Empty flag */
#define USART_REG_SR_IDLE_FLAG                 ( (uint32_t) ( 1 << 4 ) )  /* IDLE line detected flag */
#define USART_REG_SR_ORE_FLAG                  ( (uint32_t) ( 1 << 3 ) )  /* OverRun Error  flag */
#define USART_REG_SR_NE_FLAG                   ( (uint32_t) ( 1 << 2 ) )  /* Noise Error Flag */
#define USART_REG_SR_FE_FLAG                   ( (uint32_t) ( 1 << 1 ) )  /* Framing Error flag */
#define USART_REG_SR_PE_FLAG                   ( (uint32_t) ( 1 << 0 ) )  /* Parity Error flag */

/****************** Bit definition for USART_BRR register ******************/
#define USART_REG_BRR_MANTISSA                 ( (uint32_t) ( 1 << 4 ) )  /* Mantissa of USARTDIV */
#define USART_REG_BRR_FRACTION                 ( (uint32_t) ( 1 << 0 ) )  /* Fraction of USARTDIV */

#define USART_BAUD_9600                        ( (uint32_t)9600 )
#define USART_BAUD_19200                       ( (uint32_t)19200 )
#define USART_BAUD_115200                      ( (uint32_t)115200 )
#define USART_BAUD_921600                      ( (uint32_t)921600 )


/****************** Bit definition for USART_CR1 register ******************/
#define USART_REG_CR1_OVER8                    ( (uint32_t) ( 1 << 15 ) ) /* USART Oversampling by 8 enable */
#define USART_OVER8_ENABLE                     1
#define USART_OVER16_ENABLE                    0

#define USART_REG_CR1_USART_EN                 ( (uint32_t) ( 1 << 13 ) ) /* USART Enable */

#define USART_REG_CR1_UART_WL                  ( (uint32_t) ( 1 << 12 ) ) /* usart Word length */
#define USART_WL_1S8B                          0
#define USART_WL_1S9B                          1

#define USART_REG_CR1_PCE_EN                   ( (uint32_t) ( 1 << 10 ) ) /* Parity Control Enable */
#define UART_PARITY_NONE                       ((uint32_t)0x00000000)

#define USART_REG_CR1_PS                       ( (uint32_t) ( 1 << 9 ) ) /* Parity Selection */
#define USART_even_parity                      0
#define USART_odd_parity                       1

#define USART_REG_CR1_PEIE_INT_EN              ( (uint32_t) ( 1 << 8 ) ) /* PE Interrupt Enable */
#define USART_REG_CR1_TXE_INT_EN               ( (uint32_t) ( 1 << 7 ) ) /* TXE Interrupt Enable */
#define USART_REG_CR1_TCIE_INT_EN              ( (uint32_t) ( 1 << 6 ) ) /* Transmission Complete Interrupt Enable */
#define USART_REG_CR1_RXNE_INT_EN              ( (uint32_t) ( 1 << 5 ) ) /* RXNE Interrupt Enable */
#define USART_REG_CR1_IDLEIE_INT_EN            ( (uint32_t) ( 1 << 4 ) ) /* IDLE Interrupt Enable */

#define USART_REG_CR1_TE                       ( (uint32_t) ( 1 << 3 ) ) /* Transmitter Enable */
#define USART_REG_CR1_RE                       ( (uint32_t) ( 1 << 2 ) ) /* Receiver Enable */
#define USART_REG_CR1_RWU                      ( (uint32_t) ( 1 << 1 ) ) /* Receiver wakeup */
#define USART_REG_CR1_SBK                      ( (uint32_t) ( 1 << 0 ) ) /* Send Break */

/****************** Bit definition for USART_CR2 register ******************/
#define USART_REG_CR2_LINEN_EN                 ( (uint32_t) ( 1 << 14 ) ) /* LIN mode enable */

#define USART_REG_CR2_STOP_Bit                 ( (uint32_t) ( 0x3 << 12 ) ) /* STOP[1:0] bits */
#define USART_CR2_1StopBit                     0
#define USART_CR2_0_5StopBit                   1
#define USART_CR2_2StopBit                     2
#define USART_CR2_1_5StopBit                   3

#define USART_REG_CR2_CLKEN_EN                 ( (uint32_t) ( 1 << 11 ) )  /* Clock Enable*/

#define USART_REG_CR2_CPOL                     ( (uint32_t) ( 1 << 10 ) )  /* Clock Polarity */
#define USART_REG_CR2_CPHA                     ( (uint32_t) ( 1 << 9 ) )   /* Clock Phase */
#define USART_REG_CR2_LBCL                     ( (uint32_t) ( 1 << 8 ) )   /* Last Bit Clock pulse */

#define USART_REG_CR2_LBDIE                    ( (uint32_t) ( 1 << 6 ) )  /* LIN Break Detection Interrupt Enable */
#define USART_REG_CR2_LBDL                     ( (uint32_t) ( 1 << 5 ) )  /* LIN Break Detection Length */

#define USART_REG_CR2_ADD                      ( (uint32_t) ( 1 << 0 ) )  /* Address of the USART node */

/****************** Bit definition for USART_CR3 register ******************/
#define USART_REG_CR3_ONEBIT                   ( (uint32_t)( 1 << 11) ) /* USART One bit method enable */
#define USART_REG_CR3_CTSIE                    ( (uint32_t)( 1 << 10) ) /* CTS Interrupt Enable */
#define USART_REG_CR3_CTSE                     ( (uint32_t)( 1 << 9) )  /* CTS Enable */
#define USART_REG_CR3_RTSE                     ( (uint32_t)( 1 << 8) )  /* RTS Enable */

#define USART_REG_CR3_DMAT_EN                  ( (uint32_t)( 1 << 7) ) /* DMA Enable Transmitter */
#define USART_REG_CR3_DMAR_EN                  ( (uint32_t)( 1 << 6) ) /* DMA Enable Receiver */

#define USART_REG_CR3_SCEN_EN                  ( (uint32_t)( 1 << 5) ) /* Smartcard mode enable */
#define USART_REG_CR3_NACK_EN                  ( (uint32_t)( 1 << 4) ) /* Smartcard NACK enable */

#define USART_REG_CR3_HDSEL                    ( (uint32_t)( 1 << 3) ) /* Half-Duplex Selection */
#define USART_REG_CR3_IRLP                     ( (uint32_t)( 1 << 2) ) /* IrDA Low-Power  */

#define USART_REG_CR3_IREN_EN                  ( (uint32_t)( 1 << 1) ) /* IrDA mode Enable */

#define USART_REG_CR3_ERROR_IN_EN              ( (uint32_t)( 1 << 0) ) /* Error Interrupt Enable */

/****************** Bit definition for USART_GTPR register ******************/
#define USART_REG_GTPR_GT                      ( (uint32_t)0x0000FF00 )  /* Guard time value mask */
#define USART_REG_GTPR_PSC                     ( (uint32_t)0x000000FF )  /* Prescaler value mask  */

/* Shift amounts for easier value setting */
#define USART_REG_GTPR_GT_POS                  (8)
#define USART_REG_GTPR_PSC_POS                 (0)


typedef enum
{
  HAL_UART_STATE_RESET      = 0x00, /* Peripheral is not yet Initialized */
  HAL_UART_STATE_READY      = 0x01, /* Peripheral Initialized and ready for use */
  HAL_UART_STATE_BUSY       = 0x02, /* an internal process is ongoing */
  HAL_UART_STATE_BUSY_TX    = 0x12, /* Data Transmission process is ongoing */
  HAL_UART_STATE_BUSY_RX    = 0x22, /* Data Reception process is ongoing */
  HAL_UART_STATE_BUSY_TX_RX = 0x32  /* Data Transmission and Reception process is ongoing */
} hal_uart_state_t;

typedef struct
{

  uint32_t BaudRate;       /* This member configures the UART communication baud rate */
  uint32_t WordLength;     /* Specifies the number of data bits transmitted or received in a frame */
  uint32_t StopBits;       /* Specifies the number of stop bits transmitted */
  uint32_t Parity;         /* Specifies the parity mode. */
  uint32_t Mode;           /* Specifies whether the Receive or Transmit mode is enabled or disabled */
  uint32_t OverSampling;   /* Specifies whether the Over sampling 8 is enabled or disabled */

} uart_init_t;

/* ----------------------- Application callbacks typedef -------------------------*/
typedef void               (* TX_COMP_CB_t) (void *ptr);
typedef void               (* RX_COMP_CB_t) (void *ptr);

typedef struct
{

  USART_TypeDef      *Instance;    /* UART registers base address */
  uart_init_t        Init;         /* UART communication parameters */
  uint8_t            *pTxBuffPtr;  /* Pointer to UART Tx transfer Buffer */
  uint16_t           TxXferSize;   /* UART Tx Transfer size */
  uint16_t           TxXferCount;  /* UART Tx Transfer Counter */
  uint8_t            *pRxBuffPtr;  /* Pointer to UART Rx transfer Buffer */
  uint16_t           RxXferSize;   /* UART Rx Transfer size */
  uint16_t           RxXferCount;  /* UART Rx Transfer Counter */
  hal_uart_state_t   rx_state;     /* UART communication state */
  hal_uart_state_t   tx_state;     /* UART communication state */
  uint32_t           ErrorCode;    /* UART Error code */
  TX_COMP_CB_t       tx_cmp_cb;    /* Application call back when tx completed */
  RX_COMP_CB_t       rx_cmp_cb;    /* Application callback when RX Completed */

} uart_handle_t;



/******************************************************************************/
/*                                                                            */
/*                          Driver exposed APIs                               */
/*                                                                            */
/******************************************************************************/

/**
 * @brief API to do UART Peripheral initialization
 * @param *handle : pointer to the handle structure of the UART peripheral
 * @retval None
 */
void hal_uart_init(uart_handle_t *handle);

/**
 * @brief API to do UART data Transmission
 * @param *uart_handle : pointer to the handle structure of the UART Peripheral
 * @param *buffer : holds the pointer to the TX buffer
 * @param len : len of the data to be TXed
 * @retval None
 */
void hal_uart_tx(uart_handle_t *handle, uint8_t *buffer, uint32_t len);

/**
 * @brief API to do UART data Reception
 * @param *handle : pointer to the handle structure of the UART peripheral
 * @param *buffer : holds the pointer to the RX buffer
 * @param len : len of the data to be RXed
 * @retval None
 */
void hal_uart_rx(uart_handle_t *handle, uint8_t *buffer, uint32_t len);

/**
 * @brief This API handles UART interrupt request.
 * @param huart: pointer to a uart_handle_t structure that contains
 * the configuration information for the specified UART module.
 * @retval None
 */
void hal_uart_handle_interrupt(uart_handle_t *huart);


#endif /* USART_HAL_H_ */
