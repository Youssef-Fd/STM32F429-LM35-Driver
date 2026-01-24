/*
 * ADC_HAL.h
 *
 *  Created on: Jan 21, 2026
 *      Author: Lenovo
 */

#ifndef ADC_HAL_H_
#define ADC_HAL_H_

#include "Common_BASES.h"

/* -------------------------------------------------------------------------- *
 * Peripheral Type Definitions
 * -------------------------------------------------------------------------- */

typedef struct
{
    volatile uint32_t SR;     /* ADC status register,                         Address offset: 0x00 */
    volatile uint32_t CR1;    /* ADC control register 1,                      Address offset: 0x04 */
    volatile uint32_t CR2;    /* ADC control register 2,                      Address offset: 0x08 */
    volatile uint32_t SMPR1;  /* ADC sample time register 1,                  Address offset: 0x0C */
    volatile uint32_t SMPR2;  /* ADC sample time register 2,                  Address offset: 0x10 */
    volatile uint32_t JOFR1;  /* ADC injected channel data offset register 1, Address offset: 0x14 */
    volatile uint32_t JOFR2;  /* ADC injected channel data offset register 2, Address offset: 0x18 */
    volatile uint32_t JOFR3;  /* ADC injected channel data offset register 3, Address offset: 0x1C */
    volatile uint32_t JOFR4;  /* ADC injected channel data offset register 4, Address offset: 0x20 */
    volatile uint32_t HTR;    /* ADC watchdog higher threshold register,      Address offset: 0x24 */
    volatile uint32_t LTR;    /* ADC watchdog lower threshold register,       Address offset: 0x28 */
    volatile uint32_t SQR1;   /* ADC regular sequence register 1,             Address offset: 0x2C */
    volatile uint32_t SQR2;   /* ADC regular sequence register 2,             Address offset: 0x30 */
    volatile uint32_t SQR3;   /* ADC regular sequence register 3,             Address offset: 0x34 */
    volatile uint32_t JSQR;   /* ADC injected sequence register,              Address offset: 0x38*/
    volatile uint32_t JDR1;   /* ADC injected data register 1,                Address offset: 0x3C */
    volatile uint32_t JDR2;   /* ADC injected data register 2,                Address offset: 0x40 */
    volatile uint32_t JDR3;   /* ADC injected data register 3,                Address offset: 0x44 */
    volatile uint32_t JDR4;   /* ADC injected data register 4,                Address offset: 0x48 */
    volatile uint32_t DR;     /* ADC regular data register,                   Address offset: 0x4C */
} ADC_TypeDef;

typedef struct
{
    volatile uint32_t CSR;    /* ADC Common status register,                  Address offset: ADC1 base address + 0x300 */
    volatile uint32_t CCR;    /* ADC common control register,                 Address offset: ADC1 base address + 0x304 */
    volatile uint32_t CDR;    /* ADC common regular data register for dual  AND triple modes,                            Address offset: ADC1 base address + 0x308 */
} ADC_Common_TypeDef;

/* -------------------------------------------------------------------------- *
 * Peripheral Base Addresses
 * -------------------------------------------------------------------------- */

#define ADC1                           ((ADC_TypeDef *) 0x40012000 )
#define ADC2                           ((ADC_TypeDef *) 0x40012100 )
#define ADC3                           ((ADC_TypeDef *) 0x40012200 )

#define ADC_COMMON_BASE                (0x40012000UL + 0x300UL)
#define ADC                            ((ADC_Common_TypeDef *) ADC_COMMON_BASE)

#define RCC_APB2ENR_ADC1               (1 << 8)
#define RCC_APB2ENR_ADC2               (1 << 9)
#define RCC_APB2ENR_ADC3               (1 << 10)

#define __HAL_RCC_ADC1_CLK_ENABLE()    ( RCC->APB2ENR |=  RCC_APB2ENR_ADC1)
#define __HAL_RCC_ADC2_CLK_ENABLE()    ( RCC->APB2ENR |=  RCC_APB2ENR_ADC2)
#define __HAL_RCC_ADC3_CLK_ENABLE()    ( RCC->APB2ENR |=  RCC_APB2ENR_ADC3)

/************* NVIC (Nested Vectored Interrupt Controller) *************/
#define ADC_IRQn              18    /* ADC1, ADC2 and ADC3 global Interrupts */


/******************************************************************************/
/*                                                                            */
/*                                 ADC                                       */
/*                        Register Bit Definintions                           */
/*                                                                            */
/******************************************************************************/

/****************** Bit definition for ADC_SR register ******************/
#define ADC_REG_SR_AWD_FLAG                    ( (uint32_t) ( 1 << 0 ) ) //Analog watchdog flag
#define ADC_REG_SR_EOC_FLAG                    ( (uint32_t) ( 1 << 1 ) ) //Regular channel end of conversion
#define ADC_REG_SR_JEOC_FLAG                   ( (uint32_t) ( 1 << 2 ) ) //Injected channel end of conversion
#define ADC_REG_SR_JSTRT_FLAG                  ( (uint32_t) ( 1 << 3 ) ) //Injected channel start flag
#define ADC_REG_SR_STRT_FLAG                   ( (uint32_t) ( 1 << 4 ) ) //Regular channel start flag
#define ADC_REG_SR_OVR_FLAG                    ( (uint32_t) ( 1 << 5 ) ) //Overrun Flag

/****************** Bit definition for ADC_CR1 register ******************/
#define ADC_CR1_AWDCH                         ( (uint32_t) ( 1 << 0 ) )  /* Analog watchdog channel select bits */
#define ADC_CR1_EOCIE                         ( (uint32_t) ( 1 << 5 ) )  /* Enable/Disable the end of conversion interrupt */
#define ADC_CR1_AWDIE                         ( (uint32_t) ( 1 << 6 ) )  /* Analog watchdog interrupt enable */
#define ADC_CR1_JEOCIE                        ( (uint32_t) ( 1 << 7 ) )  /* Interrupt enable for injected channels */
#define ADC_CR1_SCAN_Mode                     ( (uint32_t) ( 1 << 8 ) )  /* SCAN mode enabled */

#define ADC_CR1_RES                           ( (uint32_t) ( 1 << 24) )  /* Resolution bits */
#define ADC_CR1_12B_RES                       0                          /* 12 Bits resolution */
#define ADC_CR1_10B_RES                       1                          /* 10 Bits resolution */
#define ADC_CR1_8B_RES                        2                          /* 8 Bits resolution */
#define ADC_CR1_6B_RES                        3                          /* 6 Bits resolution */

#define ADC_CR1_OVRIE                         ( (uint32_t) ( 1 << 26) )  /* overrun interrupt enable */

/****************** Bit definition for ADC_CR2 register ******************/
#define ADC_CR2_ADON                         ( (uint32_t) ( 1 << 0 ) )   /* Converter ON / OFF */
#define ADC_CR2_CONT                         ( (uint32_t) ( 1 << 1 ) )   /* Continuous Conversion */
#define ADC_CR2_EOCS                         ( (uint32_t) ( 1 << 10 ) )  /* End of conversion selection */
#define ADC_CR2_ALIGN                        ( (uint32_t) ( 1 << 11 ) )  /* Data alignment */
#define ADC_CR2_SWSTART                      ( (uint32_t) ( 1 << 30 ) )  /* Start Conversion of regular channels */

/****************** Bit definition for ADC_CCR register ******************/
#define ADC_CCR_ADCPRE                       ( (uint32_t) ( 1 << 16 ) )     /* ADCPRE[1:0] bits (ADC prescaler 4) */

/****************** Bit definition for ADC_CCR register ******************/
#define ADC_SMPR2_SMP0                       ( (uint32_t) ( 1 << 0 ) )   /* Channel 0 Sample time selection */
#define ADC_SMPR2_SMP0_3Cycle                0
#define ADC_SMPR2_SMP0_15Cycle               1
#define ADC_SMPR2_SMP0_28Cycle               2
#define ADC_SMPR2_SMP0_56Cycle               3

#define ADC_SMPR2_SMP1                       ( (uint32_t) ( 1 << 3 ) )   /* Channel 1 Sample time selection */
#define ADC_SMPR2_SMP2                       ( (uint32_t) ( 1 << 6 ) )   /* Channel 2 Sample time selection */
#define ADC_SMPR2_SMP3                       ( (uint32_t) ( 1 << 9 ) )   /* Channel 3 Sample time selection */

/****************** Bit definition for ADC_SQR1 register ******************/
#define ADC_SQR1_L                            ( (uint32_t) ( 1 << 20 ) )  /* Regular channel sequence length:
                                                                             These bits are set to define the total number of conversions */
#define ADC_SQ1_L_1_Conversion                0
#define ADC_SQ1_L_2_Conversion                1
#define ADC_SQ1_L_3_Conversion                2
#define ADC_SQ1_L_4_Conversion                3



/******************************************************************************/
/*                                                                            */
/*                          Driver exposed APIs                               */
/*                                                                            */
/******************************************************************************/

/**
 * @brief API to initialize the ADC peripheral.
 *
 * This function configures the ADC hardware, including:
 *  - Enabling the ADC and GPIO clocks
 *  - Configuring the GPIO pin in analog mode
 *  - Setting ADC resolution, sample time, and channel
 *  - Enabling ADC end-of-conversion interrupt (EOC)
 *  - Configuring the NVIC for ADC interrupts
 *
 * @retval None
 */
void HAL_ADC_Init(void);


 /**
  * @brief  Start ADC conversion on a selected channel.
  * This function selects the ADC channel and starts the conversion  *
  * @param  channel: ADC channel number betweenn 0 and 18
  * @retval None
  */
void HAL_ADC_Start(uint8_t channel);


/**
 * @brief  Wait until ADC conversion is complete.
 * This function blocks execution until the End Of Conversion (EOC) flag is set.
 * @retval None
 */
void HAL_ADC_WaitForConversion(void);



/**
 * @brief  Get the converted ADC value by Reading the ADC data register (DR).
 * @retval uint16_t Converted ADC value
 */
uint16_t HAL_ADC_Get_Value(void);

#endif /* ADC_HAL_H_ */
