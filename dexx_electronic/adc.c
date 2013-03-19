/*
 * adc.c
 *
 *  Created on: 04-10-2012
 *      Author: dexx
 */
#include "adc.h"

void adc_init(void) {
	gpio_pin_cfg(GPIOB, 0, GPIO_CRx_MODE_CNF_IN_ANALOG_value);

//RCC
	// ADC CLOCK ON
	bitband_t m_BITBAND_PERIPH(&RCC->APB2ENR,9) = 1; // RCC->APB2ENR RCC_APB2ENR_ADC1EN <= bit9;
	// ADC Prescaler (2)
	bitband_t m_BITBAND_PERIPH(&RCC->CFGR,14) = 1; // RCC->CFGR bit 14 => RCC_CFGR_ADCPRE_0;;
	bitband_t m_BITBAND_PERIPH(&RCC->CFGR,15) = 1; // RCC->CFGR bit 14 => RCC_CFGR_ADCPRE_0;;
/*
//CR1
	//Discontinius mode enable
	bitband_t m_BITBAND_PERIPH(&ADC1->CR1,11) = 1; //ADC1->CR1 ADC_CR1_DISCEN bit 15;
	// Channel 8 (ADC_CR1_DISCNUM = 000 ) 1channel
	bitband_t m_BITBAND_PERIPH(&ADC1->CR1,13) = 0; //ADC1->CR1 ADC_CR1_DISCNUM_0 bit 13;
	bitband_t m_BITBAND_PERIPH(&ADC1->CR1,14) = 0; //ADC1->CR1 ADC_CR1_DISCNUM_1 bit 14;
	bitband_t m_BITBAND_PERIPH(&ADC1->CR1,15) = 0; //ADC1->CR1 ADC_CR1_DISCNUM_2 bit 15;

	// Channel 8
	bitband_t m_BITBAND_PERIPH(&ADC1->SQR3,0) = 0; //ADC1->SQR3 |= ADC_SQR3_SQ1_0; bit 0;
	bitband_t m_BITBAND_PERIPH(&ADC1->SQR3,0) = 0; //ADC1->SQR3 |= ADC_SQR3_SQ1_1; bit 1;
	bitband_t m_BITBAND_PERIPH(&ADC1->SQR3,0) = 0; //ADC1->SQR3 |= ADC_SQR3_SQ1_2; bit 2;
	bitband_t m_BITBAND_PERIPH(&ADC1->SQR3,0) = 1; //ADC1->SQR3 |= ADC_SQR3_SQ1_3; bit 3;

//CR2
	//Bit 22 SWSTART: Start conversion of regular channels
	bitband_t m_BITBAND_PERIPH(&ADC1->CR2,22) = 1; //ADC1->CR2 ADC_CR2_SWSTART bit 22;
	// Set ADC ON
	bitband_t m_BITBAND_PERIPH(&ADC1->CR2,0) = 1; // ADC1->CR2 ADC_CR2_ADON <= bit1;
*/
	ADC1 ->CR2 = 0x005E7003;	//Switch on the ADC and enable continuous conversion
	ADC1 ->SQR1 = 0x0000;	 	//set sequence length to one
	ADC1 ->SQR2 = 0x0000;	 	//select conversion on channel zero
	ADC1 ->SQR3 = 0x0008;
	ADC1 ->CR2 |= 0x005E7003; 	//rewrite on bit
	ADC1 ->CR1 = 0x000100;		//Start conversion of regular	channels, enable ADC

}

uint16_t adc_read(void) {

	// Set ADC ON
	bitband_t m_BITBAND_PERIPH(&ADC1->CR2,0) = 1; // ADC1->CR2 ADC_CR2_ADON <= bit1;
	//Bit 22 SWSTART: Start conversion of regular channels
	bitband_t m_BITBAND_PERIPH(&ADC1->CR2,22) = 1; //ADC1->CR2 ADC_CR2_SWSTART bit 22;

//	while(m_BITBAND_PERIPH(&ADC1->SR,1)!=1)
//	{

//	}
	return ADC1 ->DR;

}
