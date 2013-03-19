/*
 * usart.c
 *
 *  Created on: 11.03.2013
 *      Author: pldraspa
 */

#include "usart1.h"
/**
 * \todo Interrupt based receiver
 */


void usart1_init(void) {

//	bitband_t m_BITBAND_PERIPH(&RCC->APB2ENR,RCC_APB2ENR_USART1EN) = 1; // ADC1->CR2 ADC_CR2_ADON <= bit1;

	RCC ->APB2ENR |= RCC_APB2ENR_USART1EN | RCC_APB2ENR_IOPAEN; // USART1 clock enable, IO PORTA clock enable

	/* Settings description:
	 * PORTA 9 (USART1 TxD)
	 * GPIOA->CRH MODE 11 (Output mode, max speed 50MHz)
	 * GPIOA->CRH CNF  10 (Alternate function output Open-drain)
	 *
	 * PORTA 10 (USART1 RxD)
	 * GPIOA->CRH MODE 00 (Input mode)
	 * GPIOA->CRH CNF  10 (Input with pull-up / pull-down)
	 */
	GPIOA ->CRH &= ~(0xFF << 4); // Clear current value for PORTA 9 and PORTA10
	GPIOA ->CRH |= (0x8b << 4); // (0x8B << 4) = 1000 1010 0000

	// Calculate bound rate
	USART1 ->BRR = FREQUENCY / USART1_BOUNDRATE;

	USART1 ->CR1 = 0;
	USART1 ->CR2 = 0;
	USART1 ->CR3 = 0;

	// UE - USART ENABLE, TE - Transmitter Enable, RE - Reciver Enable
	USART1 ->CR1 |= USART_CR1_UE | USART_CR1_TE	| USART_CR1_RE;


	// Set initial values
	for(i_rx = 0; i_rx <USART1_RX_BUFFER_SIZE; i_rx++)
		b_rx[i_rx] = 0; //input frame buffer
	i_rx = 0;
}

void usart1_putc(char znak) {

	USART1 ->DR = znak & 0xff;
	while(!(USART1->SR & USART_SR_TC));

}


