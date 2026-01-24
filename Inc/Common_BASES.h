/*
 * Common_BASES.h
 *
 *  Created on: Jan 21, 2026
 *      Author: Lenovo
 */

#ifndef COMMON_BASES_H_
#define COMMON_BASES_H_

#include <stdint.h>


typedef struct {
    volatile uint32_t CR;
    volatile uint32_t PLLCFGR;
    volatile uint32_t CFGR;
    volatile uint32_t CIR;
    volatile uint32_t AHB1RSTR;
    volatile uint32_t AHB2RSTR;
    volatile uint32_t AHB3RSTR;
    volatile uint32_t RESERVED0;
    volatile uint32_t APB1RSTR;
    volatile uint32_t APB2RSTR;
    volatile uint32_t RESERVED1[2];
    volatile uint32_t AHB1ENR;
    volatile uint32_t AHB2ENR;
    volatile uint32_t AHB3ENR;
    volatile uint32_t RESERVED2;
    volatile uint32_t APB1ENR;
    volatile uint32_t APB2ENR;
} RCC_TypeDef;


typedef struct
{
    volatile uint32_t ISER[8];   /* Interrupt Set Enable Registers */
    uint32_t RESERVED0[24];
    volatile uint32_t ICER[8];   /* Interrupt Clear Enable Registers */
    uint32_t RESERVED1[24];
    volatile uint32_t ISPR[8];   /* Interrupt Set Pending Registers */
    uint32_t RESERVED2[24];
    volatile uint32_t ICPR[8];   /* Interrupt Clear Pending Registers */
    uint32_t RESERVED3[24];
    volatile uint32_t IABR[8];   /* Interrupt Active Bit Registers */
    uint32_t RESERVED4[56];
    volatile uint8_t  IP[240];   /* Interrupt Priority Registers (8-bit each) */

} NVIC_TypeDef;

#define RCC                       ((RCC_TypeDef *)0x40023800)

#define NVIC                      ((NVIC_TypeDef *)0xE000E100)

#define NVIC_ENABLE_IRQ(IRQn)     ( NVIC->ISER[(IRQn) / 32] = (1U << ((IRQn) % 32)) )
#define NVIC_DISABLE_IRQ(IRQn)    ( NVIC->ICER[(IRQn) / 32] = (1U << ((IRQn) % 32)) )


#endif /* COMMON_BASES_H_ */
