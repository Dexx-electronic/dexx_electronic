/** \file main.c
 * \brief Template for telemetry system
 * \details  *
 * \author Dexx, http://www.dexx.ovh.org/
 * \date 2013.01
 * \todo 1. interrupt based UART connection
 * \todo 2. frysky protocol parser
 * \todo 3. LCD handling
 * \todo 4.
 */

/******************************************************************************
* project: Dexx-electronic
* chip: STM32F103RB
* compiler: arm-none-eabi-gcc
*
* prefix: (none)
*
* available global functions:
* 	int main(void)
*
* available local functions:
* 	static void flash_latency(uint32_t frequency)
* 	static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
* 	static void system_init(void)
*	static void exti_init(void)
*
* available interrupt handlers:
******************************************************************************/

/*
 * \todo CHECK MAIN STACK SIZE
 */


/*
+=============================================================================+
| includes
+=============================================================================+
*/

#include <stdint.h>

#include "inc/stm32f10x.h"
#include "config.h"
#include "hdr/hdr_rcc.h"
#include "hdr/hdr_gpio.h"
#include "gpio.h"
#include "tim3.h"

/*
+=============================================================================+
| module variables
+=============================================================================+
*/

/*
+=============================================================================+
| local functions' declarations
+=============================================================================+
*/

static void flash_latency(uint32_t frequency);
static uint32_t pll_start(uint32_t crystal, uint32_t frequency);
static void system_init(void);
static void exti_init(void);
/*
+=============================================================================+
| global functions
+=============================================================================+
*/

/*------------------------------------------------------------------------*//**
* \brief main code block
* \details
*//*-------------------------------------------------------------------------*/
uint16_t i, c_interrupts;
int main(void)
{
	c_interrupts = 0;
	volatile uint32_t count = 0;
	pll_start(CRYSTAL, FREQUENCY);
	system_init();


	uint8_t index = 0;
	while (1)
	{
		count = c_interrupts ;//SysTick->VAL;
	}
}

/*
+=============================================================================+
| local functions
+=============================================================================+
*/

//	\todo move to external file.

/*------------------------------------------------------------------------*//**
* \brief Configures Flash latency
* \details Configures Flash latency (wait-states) which allows the chip to run
* at higher speeds
*
* \param [in] frequency defines the target frequency of the core
*//*-------------------------------------------------------------------------*/

static void flash_latency(uint32_t frequency)
{
	uint32_t wait_states;

	if (frequency < 24000000ul)				// 0 wait states for core speed below 24MHz
		wait_states = 0;
	else if (frequency < 48000000ul)		// 1 wait state for core speed between 24MHz and 48MHz
		wait_states = 1;
	else									// 2 wait states for core speed over 48MHz
		wait_states = 2;

	FLASH->ACR |= wait_states;				// set the latency
}

/*------------------------------------------------------------------------*//**
* \brief Starts the PLL
* \details Configure and enable PLL to achieve some frequency with some crystal.
* Before the speed change Flash latency is configured via flash_latency(). PLL
* parameter mul is based on both function parameters. The PLL is set up,
* started and connected. APB1 clock ratio is set to 1:2 (max freq = 36MHz)
*
* \param [in] crystal is the frequency of the crystal resonator connected to the
* STM32F103RB
* \param [in] frequency is the desired target frequency after enabling the PLL
*
* \return real frequency that was set
*//*-------------------------------------------------------------------------*/

static uint32_t pll_start(uint32_t crystal, uint32_t frequency)
{
	uint32_t mul;

	RCC_CR_HSEON_bb = 1;					// enable HSE clock
	flash_latency(frequency);				// configure Flash latency for desired frequency

	mul = frequency / crystal;				// PLL multiplier calculation

	if (mul > 16)							// max PLL multiplier is 16
		mul = 16;

	frequency = crystal * mul;

	RCC->CFGR |= ((mul - 2) << RCC_CFGR_PLLMUL_bit) | RCC_CFGR_PLLSRC | RCC_CFGR_PPRE1_DIV2;	// configuration of PLL: HSE x (mul), APB1 clk = /2

	while (!RCC_CR_HSERDY_bb);				// wait for stable clock

	RCC_CR_PLLON_bb = 1;					// enable PLL
	while (!RCC_CR_PLLRDY_bb);				// wait for PLL lock

	RCC->CFGR |= RCC_CFGR_SW_PLL;			// change SYSCLK to PLL
	while (((RCC->CFGR) & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);	// wait for switch

	return frequency;
}


/*------------------------------------------------------------------------*//**
* \brief Configures External interrupt
* \details Configures PIN 0 from PORTC as pulled up input and as external
* interrupt source.
*//*-------------------------------------------------------------------------*/

static void exti_init(void)
{
	// \todo bitband where possible
	// \todo move to external  file

	// Enable Alternative Function IO clock
	// Enable Port C clock
	RCC ->APB2ENR |= RCC_APB2ENR_AFIOEN | RCC_APB2ENR_IOPCEN; // Alternative Function IO clock enable

	GPIOC->CRL &= ~((uint32_t)(0x0F)); // Clear PC0 configuration
	GPIOC->CRL |= GPIO_CRL_CNF0_1; // Input with pull up or pull down (MODE 00 CNF10)
	GPIOC->ODR |= GPIO_ODR_ODR0; // Pull up


	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PC; //EXTI0 to port PC0
	EXTI->IMR |= EXTI_IMR_MR0; // Interrupt mask register - interrupt mask on line x (0 - masked)
	//EXTI->EMR |= EXTI_EMR_MR0; // Event mask register - Event request from masked line (0 - masked)
	EXTI->FTSR |= EXTI_FTSR_TR0; // Falling trigger selection register - (0 -falling trigger disabled)
	EXTI->RTSR |= EXTI_RTSR_TR0; // Rising trigger selecton register - (0 - Rising trigger disabled)

	/*
	 * ISER - Interrupt set enable register
	 */
	NVIC->ISER[0] |= NVIC_ISER_SETENA_6;
}

/*------------------------------------------------------------------------*//**
* \brief Initializes system
* \details
*//*-------------------------------------------------------------------------*/

static void system_init(void)
{
	gpio_init();
	tim3_init(1000);
	adc_init();
	usart1_init();
	SysTick_Config(5000);
	exti_init();
}


void frysky_parser(void)
{
	uint8_t i_parser = 0;

	if(i_parser == i_rx) // if parser had interpreted whole buffer
		i_rx = 0;			// move buffer index to first position
}
/*
+=============================================================================+
| ISRs
+=============================================================================+
*/
void SysTick_Handler(void)
{
	TIM3->CCR4 = ADC1 ->DR; // Przepisz wartość otrzymana z ADC do rejestru odpowiadojącego za wypełnienie

}

void EXTI0_IRQHandler(void)
{
	c_interrupts = c_interrupts + 1;
	EXTI->PR |= EXTI_PR_PR0; // \todo check whether it is necessary
}


void USART1_IRQHandler(void)
{
	while(1);
// \todo USART1	if(interrupt flag)
	{
		b_rx[i_rx++] = USART1 ->DR;
	}

}
/******************************************************************************
* END OF FILE
******************************************************************************/
