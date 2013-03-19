/*
 * usart1.h
 *
 *  Created on: 11.03.2013
 *      Author: pldraspa
 */


// \todo unlink useless headers

#ifndef USART1_H_
#define USART1_H_

#include <stdint.h>
#include "inc/stm32f10x.h"
#include "hdr/hdr_gpio.h"
#include "hdr/hdr_bitband.h"
#include "hdr/hdr_rcc.h"
#include "hdr/hdr_usart.h"
#include "config.h"


// \todo Replace USART1 RX buffer with DMA
#define USART1_RX_BUFFER_SIZE 250 // Sets maximum size for rx buffer;
#define USART1_BOUNDRATE 9600


uint8_t i_rx; 		// receive buffer index
uint8_t b_rx[255]; //input frame buffer



void usart1_init(void);
void usart1_putc(char);


#endif /* USART1_H_ */
