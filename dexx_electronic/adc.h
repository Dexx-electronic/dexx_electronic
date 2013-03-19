/*
 * adc.h
 *
 *  Created on: 04-10-2012
 *      Author: dexx
 */

#ifndef ADC_H_
#define ADC_H_

#include <stdint.h>
#include "inc/stm32f10x.h"
#include "hdr/hdr_gpio.h"
#include "hdr/hdr_bitband.h"
#include "config.h"

void adc_init(void);
uint16_t adc_read(void);


#endif /* ADC_H_ */
