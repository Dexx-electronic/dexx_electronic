/*
 * usart1.h
 *
 *  Created on: 11.03.2013
 *      Author: pldraspa
 */

#ifndef USART1_H_
#define USART1_H_

#include <stdint.h>
#include "inc/stm32f10x.h"
#include "hdr/hdr_gpio.h"
#include "hdr/hdr_bitband.h"
#include "hdr/hdr_rcc.h"
#include "hdr/hdr_usart.h"
#include "config.h"


#define USART1_BOUNDRATE 9600




void usart1_init(void);
void usart1_putc(char);


#endif /* USART1_H_ */
