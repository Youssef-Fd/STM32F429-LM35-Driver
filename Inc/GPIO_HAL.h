/*
 * GPIO_HAL.h
 *
 *  Created on: Jan 21, 2026
 *      Author: Lenovo
 */

#ifndef GPIO_HAL_H_
#define GPIO_HAL_H_

#include "Common_BASES.h"


/* -------------------------------------------------------------------------- *
 * Peripheral Type Definitions
 * -------------------------------------------------------------------------- */

typedef struct {
    volatile uint32_t MODER;
    volatile uint32_t OTYPER;
    volatile uint32_t OSPEEDR;
    volatile uint32_t PUPDR;
    volatile uint32_t IDR;
    volatile uint32_t ODR;
    volatile uint32_t BSRR;
    volatile uint32_t LCKR;
    volatile uint32_t AFRL;
    volatile uint32_t AFRH;
} GPIO_TypeDef;

/* -------------------------------------------------------------------------- *
 * Peripheral Base Addresses
 * -------------------------------------------------------------------------- */

#define GPIOA     ((GPIO_TypeDef *)0x40020000)
#define GPIOB     ((GPIO_TypeDef *)0x40020400)
#define GPIOC     ((GPIO_TypeDef *)0x40020800)
#define GPIOD     ((GPIO_TypeDef *)0x40020C00)
#define GPIOE     ((GPIO_TypeDef *)0x40021000)
#define GPIOF     ((GPIO_TypeDef *)0x40021400)
#define GPIOG     ((GPIO_TypeDef *)0x40021800)
#define GPIOH     ((GPIO_TypeDef *)0x40021C00)
#define GPIOI     ((GPIO_TypeDef *)0x40022000)
#define GPIOJ     ((GPIO_TypeDef *)0x40022400)
#define GPIOK     ((GPIO_TypeDef *)0x40022800)


#define RCC_AHB1ENR_GPIOAEN  (1 << 0)
#define RCC_AHB1ENR_GPIOBEN  (1 << 1)
#define RCC_AHB1ENR_GPIOCEN  (1 << 2)
#define RCC_AHB1ENR_GPIODEN  (1 << 3)
#define RCC_AHB1ENR_GPIOEEN  (1 << 4)
#define RCC_AHB1ENR_GPIOFEN  (1 << 5)
#define RCC_AHB1ENR_GPIOGEN  (1 << 6)
#define RCC_AHB1ENR_GPIOHEN  (1 << 7)
#define RCC_AHB1ENR_GPIOIEN  (1 << 8)

/* -------------------------------------------------------------------------- *
 * GPIO Mode Definitions (MODER Register)
 * -------------------------------------------------------------------------- */
#define GPIO_MODE_INPUT       0x00
#define GPIO_MODE_OUTPUT      0x01
#define GPIO_MODE_AF          0x02
#define GPIO_MODE_ANALOG      0x03


#endif /* GPIO_HAL_H_ */
