/*
 * tim3.c
 *
 *  Created on: 30-09-2012
 *      Author: dexx
 */
#include "tim3.h"
#include "config.h"

//#define TIM3_REWRITE_DEFAULT 1
/*
 * Timer 3, no remap, channel 4 PWM mode
 */
void tim3_init(uint16_t proc) {

	/*
	 * AFIOEN - Alternative function enable
	 * IOPBEN - I/O Port B clock enable
	 */
	RCC ->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPBEN; //PC + 22


	/*
	 * APB1 - Peripheral clock enable register
	 * TM3EN (1) - Timer 3 clock enable
	 */
	//		RCC ->APB1ENR |= RCC_APB1ENR_TIM3EN; // PC + 16
	(bitband_t m_BITBAND_PERIPH(&RCC->APB1ENR,1)) = 1; //12

	gpio_pin_cfg(GPIOB, 1, GPIO_CRx_MODE_CNF_ALT_PP_50M_value);

	//TIM3 Partial remap setup IN REG AFIO->MAPR bit_11=1 bit_10=0

//	 OPTION 1
//	 uint32_t tmp;
//	 tmp = AFIO->MAPR;
//	 tmp &= ~(AFIO_MAPR_TIM3_REMAP_0 | AFIO_MAPR_TIM3_REMAP_1);
//	 tmp |= (AFIO_MAPR_TIM3_REMAP_PARTIALREMAP);
//	 AFIO->MAPR = tmp;
//
//


//	 OPTION 2

#ifdef TIM3_REWRITE_DEFAULT
	// NO REMAP <= default
	(bitband_t m_BITBAND_PERIPH(&AFIO->MAPR,10)) = 0; //bit 10
	(bitband_t m_BITBAND_PERIPH(&AFIO->MAPR,11)) = 0; //bit 11

	//Clock division
	bitband_t m_BITBAND_PERIPH(&TIM3->CR1,8) = 0; // TIM_CR1_CKD_0
	bitband_t m_BITBAND_PERIPH(&TIM3->CR1,9) = 0; // TIM_CR1_CKD_1

	// Count UP
	bitband_t m_BITBAND_PERIPH(&TIM3->CR1,4) = 0; // TIM_CR1_DIR
#endif
	// Prescaler
	TIM3 ->PSC = (uint16_t) (FREQUENCY / 1000000) - 1;

	// Period
	TIM3 ->ARR = 5000;

	volatile uint32_t *tmp_ptr;
	uint32_t tmp_value;

	// CHANNEL 4 Settings
	// PWM mode 1 - TIM_CCMR2_OC3M
	// Preload Enable - TIM_CCMR2_OC3PE
	tmp_ptr = &TIM3 ->CCMR2;
	tmp_value = *tmp_ptr;
	tmp_value &= ~(TIM_CCMR2_OC4M ); // wyzeruj bity OC1M
	tmp_value |= (TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4PE ); // ustaw tylko bity 1 i 2
	*tmp_ptr = tmp_value;

	// Output Enable
	bitband_t m_BITBAND_PERIPH(&TIM3->CCER,12) = 1; //TIM_CCER_CC4E

	// Polarity high
	bitband_t m_BITBAND_PERIPH(&TIM3->CCER,13) = 0; //TIM_CCER_CC4P

	// Duty Cycle Channel 4
	TIM3 ->CCR4 = proc;

	//ARR PreloadConfig
	//TIM3 ->CR1 |= TIM_CR1_ARPE;
	//bitband_t m_BITBAND_PERIPH(&TIM3->CR1,8) = 1;

	// Enable TIM3
	//TIM3 ->CR1 |= TIM_CR1_CEN;
	//bitband_t m_BITBAND_PERIPH(&TIM3->CR1,1) = 1;

	TIM3 ->CR1 |= TIM_CR1_CEN | TIM_CR1_ARPE;
}

