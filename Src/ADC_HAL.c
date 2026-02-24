/*
 * ADC_HAL.c
 *
 *  Created on: Jan 21, 2026
 *      Author: Youssef Fadel
 */

#include "ADC_HAL.h"
#include "GPIO_HAL.h"

void HAL_ADC_Init(void){

	__HAL_RCC_ADC1_CLK_ENABLE(); // Enable ADC1 clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN; //GPIOA Clock enable

	/* Clear and Set prescaler 4 (/4) in the common control register */
	ADC->CCR &= ~(3 << 16);
	ADC->CCR |= ADC_CCR_ADCPRE;

	/* Configure rsolution and Scan mode*/
	ADC1->CR1 &= ~(3 << 24);
	ADC1->CR1 |= (ADC_CR1_10B_RES << 24); // 10-bits resolution
	ADC1->CR1 |=  ADC_CR1_SCAN_Mode; // Enable scan mode

	/* Enable continous conversion and EOC flag and right alignement*/
	ADC1->CR2 |= ADC_CR2_CONT; //Enable conctinous conversion
	ADC1->CR2 |= ADC_CR2_EOCS; //End of conversion selection

	/* Confifure sampling time ( 3 cycles for simplicity */
	ADC1->SMPR2 &= ~(7 << 0);
	ADC1->SMPR2 |= (ADC_SMPR2_SMP0_3Cycle << 0);

	/* Sequence Length in  ADC regular sequence register 1 */
	ADC1->SQR1 |= (ADC_SQ1_L_1_Conversion << 20); // L=0 means 1 conversion in sequence

	/* Configure GPIOA to analoge for ADC1*/
	GPIOA->MODER |= (GPIO_MODE_ANALOG << 0);

	/* Disable interrupts for polling Mode */
	ADC1->CR1 &= ~ADC_CR1_EOCIE;

	/* Enable the ADC by Setting the ADON */
	ADC1->CR2 |= ADC_CR2_ADON;

}


void HAL_ADC_Start(uint8_t channel) {

    /* Set the channel in the 1st slot of the sequence (SQ1) */
    ADC1->SQR3 &= ~(0x1F << 0); //Clear the SQ1[4:0]
    ADC1->SQR3 |= (channel << 0); //Choose the channel desired

    ADC1->SR = 0; //Clearstatus register
    ADC1->CR2 |= ADC_CR2_SWSTART; //Start conversion

}


void HAL_ADC_WaitForConversion(void) {

	/* Wait for the EOC bit in SR to be set */
    while(!(ADC1->SR & ADC_REG_SR_EOC_FLAG));

}


uint16_t HAL_ADC_Get_Value(void) {

    return (uint16_t) ADC1->DR;
}
