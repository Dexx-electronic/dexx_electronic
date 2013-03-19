/** \file hdr_timx.h
 * \brief Header with definition of bits in TIMx control registers
 * \author Dexx, http://www.dexx.ovh.org/
 * \date 2012-09-30
 */

/******************************************************************************
* chip: STM32F1x
* compiler: arm-none-eabi-gcc (Sourcery CodeBench Lite 2011.09-69) 4.6.1
******************************************************************************/

#ifndef HDR_TIMX_H_
#define HDR_TIMX_H_

/*
+=============================================================================+
| global definitions
+=============================================================================+
*/

/*
+-----------------------------------------------------------------------------+
| TIMx control register 1 (TIMx_CR1)
+-----------------------------------------------------------------------------+
*/

#define TIMx_CR1_CEN_bit					0
#define TIMx_CR1_CEN_0_bit					0

#define TIMx_CR1_UDIS_bit					1
#define TIMx_CR1_UDIS_0_bit					1

#define TIMx_CR1_URS_bit					2
#define TIMx_CR1_URS_0_bit					2

#define TIMx_CR1_OPM_bit					3
#define TIMx_CR1_OPM_0_bit					3

#define TIMx_CR1_CMS_bit					4
#define TIMx_CR1_CMS_0_bit					4
#define TIMx_CR1_CMS_1_bit					5

#define TIMx_CR1_DIR_bit					6
#define TIMx_CR1_DIR_0_bit					6

#define TIMx_CR1_ARPE_bit					7
#define TIMx_CR1_ARPE_0_bit					7

#define TIMx_CR1_CKD_bit					8
#define TIMx_CR1_CKD_0_bit					8
#define TIMx_CR1_CKD_1_bit					9



/*
+-----------------------------------------------------------------------------+
| TIMx slave mode control register (TIMx_SMCR)
+-----------------------------------------------------------------------------+
*/

#define TIMx_SMCR_CCDS_bit					3
#define TIMx_SMCR_CCDS_0_bit				3

#define TIMx_SMCR_MMS_bit					4
#define TIMx_SMCR_MMS_0_bit					4
#define TIMx_SMCR_MMS_1_bit					5
#define TIMx_SMCR_MMS_2_bit					6

#define TIMx_CR2_TI1S_bit					7
#define TIMx_CR2_TI1S_0_bit					7




/*
+-----------------------------------------------------------------------------+
| TIMx slave mode control register (TIMx_SMCR)
+-----------------------------------------------------------------------------+
*/

/*
Bit 15 ETP: External trigger polarity
	This bit selects whether ETR or ETR is used for trigger operations
		0: ETR is non-inverted, active at high level or rising edge
		1: ETR is inverted, active at low level or falling edge
*/
#define TIMx_SMCR_ETP_bit					15
#define TIMx_SMCR_ETP_0_bit					15

/*
Bit 14 ECE: External clock enable
	This bit enables External clock mode 2
		0: External clock mode 2 disabled
		1: External clock mode 2 enabled. The counter is clocked by any active edge on the ETRF signal.

	1: Setting the ECE bit has the same effect as selecting external clock mode 1 with TRGI connected to ETRF (SMS=111 and TS=111).
	2: It is possible to simultaneously use external clock mode 2 with the following slave modes: reset mode, gated mode and trigger mode. Nevertheless, TRGI must not be connected to ETRF in this case (TS bits must not be 111).
	3: If external clock mode 1 and external clock mode 2 are enabled at the same time, the external clock input is ETRF.
*/
#define TIMx_SMCR_ECE_bit					14
#define TIMx_SMCR_ECE_0_bit					14

/*
Bits 13:12 ETPS: External trigger prescaler
	External trigger signal ETRP frequency must be at most 1/4 of CK_INT frequency. A prescaler can be enabled to reduce ETRP frequency. It is useful when inputting fast external clocks.

	00: Prescaler OFF
	01: ETRP frequency divided by 2
	10: ETRP frequency divided by 4
	11: ETRP frequency divided by 8
*/
#define TIMx_SMCR_ECE_bit					12
#define TIMx_SMCR_ECE_0_bit					12
#define TIMx_SMCR_ECE_1_bit					13

/*
Bits 11:8 ETF[3:0]: External trigger filter
	This bit-field then defines the frequency used to sample ETRP signal and the length of the
	digital filter applied to ETRP. The digital filter is made of an event counter in which N events
	are needed to validate a transition on the output:

	1000: fSAMPLING=fDTS/8, N=6
	0000: No filter, sampling is done at fDTS
	0001: fSAMPLING=fCK_INT, N=2
	1001: fSAMPLING=fDTS/8, N=8
	1010: fSAMPLING=fDTS/16, N=5
	0010: fSAMPLING=fCK_INT, N=4
	1011: fSAMPLING=fDTS/16, N=6
	0011: fSAMPLING=fCK_INT, N=8
	0100: fSAMPLING=fDTS/2, N=6
	1100: fSAMPLING=fDTS/16, N=8
	0101: fSAMPLING=fDTS/2, N=8
	1101: fSAMPLING=fDTS/32, N=5
	1110: fSAMPLING=fDTS/32, N=6
	0110: fSAMPLING=fDTS/4, N=6
	0111: fSAMPLING=fDTS/4, N=8
	1111: fSAMPLING=fDTS/32, N=8
*/
#define TIMx_SMCR_ETF_bit					8
#define TIMx_SMCR_ETF_0_bit					8
#define TIMx_SMCR_ETF_1_bit					9
#define TIMx_SMCR_ETF_2_bit					10
#define TIMx_SMCR_ETF_3_bit					11

/*
Bit 7 MSM: Master/Slave mode
	0: No action
	1: The effect of an event on the trigger input (TRGI) is delayed to allow a perfect
	synchronization between the current timer and its slaves (through TRGO). It is useful if we
	want to synchronize several timers on a single external event.
*/
#define TIMx_MSM_ETF_bit					7
#define TIMx_MSM_ETF_0_bit					7




/*
Bits 6:4 TS: Trigger selection
	This bit-field selects the trigger input to be used to synchronize the counter.

	000: Internal Trigger 0 (ITR0).
	001: Internal Trigger 1 (ITR1).
	010: Internal Trigger 2 (ITR2).
	011: Internal Trigger 3 (ITR3).
	100: TI1 Edge Detector (TI1F_ED)
	101: Filtered Timer Input 1 (TI1FP1)
	110: Filtered Timer Input 2 (TI2FP2)
	111: External Trigger input (ETRF)

	See Table 71: TIMx internal trigger connection on page 315 for more details on ITRx
	meaning for each Timer.
	Note: These bits must be changed only when they are not used (e.g. when SMS=000) to
	avoid wrong edge detections at the transition.
*/
#define TIMx_SMCR_TS_bit					4
#define TIMx_SMCR_TS_0_bit					4
#define TIMx_SMCR_TS_1_bit					5
#define TIMx_SMCR_TS_2_bit					6

/*
Bit 3 OCCS: OCREF clear selection
	This bit is used to select the OCREF clear source

	0: OCREF_CLR_INT is connected to the OCREF_CLR input
	1: OCREF_CLR_INT is connected to ETRF
*/
#define TIMx_SMCR_OCCS_bit					3
#define TIMx_SMCR_OCCS_0_bit				3

/*
Bits 2:0 SMS: Slave mode selection
	When external signals are selected the active edge of the trigger signal (TRGI) is linked to
	the polarity selected on the external input (see Input Control register and Control Register
	description.

	000: Slave mode disabled - if CEN = ‘1 then the prescaler is clocked directly by the internal
		clock.
	001: Encoder mode 1 - Counter counts up/down on TI2FP2 edge depending on TI1FP1
		level.
	010: Encoder mode 2 - Counter counts up/down on TI1FP1 edge depending on TI2FP2
		level.
	011: Encoder mode 3 - Counter counts up/down on both TI1FP1 and TI2FP2 edges
		depending on the level of the other input.
	100: Reset Mode - Rising edge of the selected trigger input (TRGI) reinitializes the counter
		and generates an update of the registers.
	101: Gated Mode - The counter clock is enabled when the trigger input (TRGI) is high. The
		counter stops (but is not reset) as soon as the trigger becomes low. Both start and stop of
		the counter are controlled.
	110: Trigger Mode - The counter starts at a rising edge of the trigger TRGI (but it is not
		reset). Only the start of the counter is controlled.
	111: External Clock Mode 1 - Rising edges of the selected trigger (TRGI) clock the counter.

	Note: The gated mode must not be used if TI1F_ED is selected as the trigger input (TS=100).
	Indeed, TI1F_ED outputs 1 pulse for each transition on TI1F, whereas the gated mode
	checks the level of the trigger signal.
*/
#define TIMx_SMCR_SMS_bit					0
#define TIMx_SMCR_SMS_0_bit					0
#define TIMx_SMCR_SMS_1_bit					1
#define TIMx_SMCR_SMS_2_bit					2




//#define GPIO_CRx_MODE_CNF_mask				(GPIO_CRx_MODE_mask | (GPIO_CRx_CNF_mask << 2))

/*
+=============================================================================+
| macros
+=============================================================================+
*/



/*
+=============================================================================+
| combinations of MODE and CNF
+=============================================================================+
*/

/******************************************************************************
* END OF FILE
******************************************************************************/
#endif /* HDR_TIMX_H_ */
