/*
 * hdr_usart.h
 *
 *  Created on: 14.03.2013
 *      Author: pldraspa
 */

#ifndef HDR_USART_H_
#define HDR_USART_H_

/*------------------------------------------------------------------------*//**
* \brief Definitions of bit number for USART registers
* \details
* \param [in]
* \param [in]
* \todo Check posibility to delete this
* \todo Write all registers
*
*//*-------------------------------------------------------------------------*/




/*
 +-----------------------------------------------------------------------------+
 | USART status register (USART_SR)
 +-----------------------------------------------------------------------------+
 */


/*
 Bits 31:10 Reserved, forced by hardware to 0.
*/


/*
 Bit 9 CTS: CTS flag
 This bit is set by hardware when the nCTS input toggles, if the CTSE bit is set. It is cleared
 by software (by writing it to 0). An interrupt is generated if CTSIE=1 in the USART_CR3 register.
 	 0: No change occurred on the nCTS status line
 	 1: A change occurred on the nCTS status line
 This bit is not available for UART4 & UART5.
 */
 #define USART_SR_CTS_bit 9

/*
 Bit 8 LBD: LIN break detection flag
 This bit is set by hardware when the LIN break is detected. It is cleared by software (by
 writing it to 0). An interrupt is generated if LBDIE = 1 in the USART_CR2 register.
 	 0: LIN Break not detected
 	 1: LIN break detected
 Note: An interrupt is generated when LBD=1 if LBDIE=1
 */
#define USART_SR_LIN_bit 8

/*
 Bit 7 TXE: Transmit data register empty
 This bit is set by hardware when the content of the TDR register has been transferred into
 the shift register. An interrupt is generated if the TXEIE bit =1 in the USART_CR1 register. It
 is cleared by a write to the USART_DR register.
 	 0: Data is not transferred to the shift register
 	 1: Data is transferred to the shift register)
 Note: This bit is used during single buffer transmission.
*/
#define USART_SR_TXE_bit 7

 /*
 Bit 6 TC: Transmission complete
 This bit is set by hardware if the transmission of a frame containing data is complete and if
 TXE is set. An interrupt is generated if TCIE=1 in the USART_CR1 register. It is cleared by a
 software sequence (a read from the USART_SR register followed by a write to the
 USART_DR register). The TC bit can also be cleared by writing a '0' to it. This clearing
 sequence is recommended only for multibuffer communication.
 	 0: Transmission is not complete
 	 1: Transmission is complete
*/
#define USART_SR_TC_bit 6

 /*
 Bit 5 RXNE: Read data register not empty
 This bit is set by hardware when the content of the RDR shift register has been transferred to
 the USART_DR register. An interrupt is generated if RXNEIE=1 in the USART_CR1 register.
 It is cleared by a read to the USART_DR register. The RXNE flag can also be cleared by
 writing a zero to it. This clearing sequence is recommended only for multibuffer
 communication.
 	 0: Data is not received
 	 1: Received data is ready to be read.
 */
#define USART_SR_RXNE_bit 5

/*
Bit 4 IDLE: IDLE line detected
This bit is set by hardware when an Idle Line is detected. An interrupt is generated if the
IDLEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the
USART_SR register followed by a read to the USART_DR register).
0: No Idle Line is detected
1: Idle Line is detected
Note: The IDLE bit will not be set again until the RXNE bit has been set itself (i.e. a new idle
line occurs).
*/
#define USART_SR_IDLE_bit 4
/*
Bit 3 ORE: Overrun error
This bit is set by hardware when the word currently being received in the shift register is
ready to be transferred into the RDR register while RXNE=1. An interrupt is generated if
RXNEIE=1 in the USART_CR1 register. It is cleared by a software sequence (an read to the
USART_SR register followed by a read to the USART_DR register).
0: No Overrun error
1: Overrun error is detected
Note: When this bit is set, the RDR register content will not be lost but the shift register will be
overwritten. An interrupt is generated on ORE flag in case of Multi Buffer
communication if the EIE bit is set.
*/
#define USART_SR_ORE_bit 3

/*
Bit 2 NE: Noise error flag
This bit is set by hardware when noise is detected on a received frame. It is cleared by a
software sequence (an read to the USART_SR register followed by a read to the
USART_DR register).
0: No noise is detected
1: Noise is detected
Note: This bit does not generate interrupt as it appears at the same time as the RXNE bit
which itself generates an interrupting interrupt is generated on NE flag in case of Multi
Buffer communication if the EIE bit is set.
*/
#define USART_SR_NE_bit 2

/*
Bit 1 FE: Framing error
This bit is set by hardware when a de-synchronization, excessive noise or a break character
is detected. It is cleared by a software sequence (an read to the USART_SR register
followed by a read to the USART_DR register).
0: No Framing error is detected
1: Framing error or break character is detected
Note: This bit does not generate interrupt as it appears at the same time as the RXNE bit
which itself generates an interrupt. If the word currently being transferred causes both
frame error and overrun error, it will be transferred and only the ORE bit will be set.
An interrupt is generated on FE flag in case of Multi Buffer communication if the EIE bit
is set.
*/
#define USART_SR_FE_bit 1

/*
Bit 0 PE: Parity error
This bit is set by hardware when a parity error occurs in receiver mode. It is cleared by a
software sequence (a read to the status register followed by a read to the USART_DR data
register). The software must wait for the RXNE flag to be set before clearing the PE bit.
An interrupt is generated if PEIE = 1 in the USART_CR1 register.
	0: No parity error
	1: Parity error
*/
#define USART_SR_PE_bit 0

/*
 +-----------------------------------------------------------------------------+
 | USART Data register (USART_DR)
 +-----------------------------------------------------------------------------+
 */


/*
Bits 31:9 Reserved, forced by hardware to 0.
*/

/*

Bits 8:0 DR[8:0]: Data value
Contains the Received or Transmitted data character, depending on whether it is read from
or written to.
The Data register performs a double function (read and write) since it is composed of two
registers, one for transmission (TDR) and one for reception (RDR)
The TDR register provides the parallel interface between the internal bus and the output
shift register (see Figure 1).
The RDR register provides the parallel interface between the input shift register and the
internal bus.
When transmitting with the parity enabled (PCE bit set to 1 in the USART_CR1 register), the
value written in the MSB (bit 7 or bit 8 depending on the data length) has no effect because
it is replaced by the parity.
When receiving with the parity enabled, the value read in the MSB bit is the received parity
bit.
*/


/*
 +-----------------------------------------------------------------------------+
 | USART Baud rate register (USART_BRR)
 +-----------------------------------------------------------------------------+
 */


/*
Bits 31:16 Reserved, forced by hardware to 0.
*/

/*
Bits 15:4 DIV_Mantissa[11:0]: mantissa of USARTDIV
These 12 bits define the mantissa of the USART Divider (USARTDIV)
*/

/*
Bits 3:0 DIV_Fraction[3:0]: fraction of USARTDIV
These 4 bits define the fraction of the USART Divider (USARTDIV)
*/


/*
 +-----------------------------------------------------------------------------+
 | USART Control register 1 (USART_CR1)
 +-----------------------------------------------------------------------------+
 */


/*
Bits 31:14 Reserved, forced by hardware to 0.
*/


/*
Bit 13 UE: USART enable
When this bit is cleared the USART prescalers and outputs are stopped and the end of the
current
byte transfer in order to reduce power consumption. This bit is set and cleared by software.
  0: USART prescaler and outputs disabled
  1: USART enabled
 */
 #define USART_CR1_UE_bit 13

/*
Bit 12 M: Word length
This bit determines the word length. It is set or cleared by software.
  0: 1 Start bit, 8 Data bits, n Stop bit
  1: 1 Start bit, 9 Data bits, n Stop bit
Note: The M bit must not be modified during a data transfer (both transmission and reception)
 */
 #define USART_CR1_M_bit 12

/*
Bit 11 WAKE: Wakeup method
This bit determines the USART wakeup method, it is set or cleared by software.
  0: Idle Line
  1: Address Mark
 */
 #define USART_CR1_WAKE_bit 11

/*
Bit 10 PCE: Parity control enable
This bit selects the hardware parity control (generation and detection). When the parity
control is enabled, the computed parity is inserted at the MSB position (9th bit if M=1; 8th bit
if M=0) and parity is checked on the received data. This bit is set and cleared by software.
Once it is set, PCE is active after the current byte (in reception and in transmission).
  0: Parity control disabled
  1: Parity control enabled
 */
 #define USART_CR1_PCE_bit 10

/*
Bit 9 PS: Parity selection
This bit selects the odd or even parity when the parity generation/detection is enabled (PCE
bit set). It is set and cleared by software. The parity will be selected after the current byte.
  0: Even parity
  1: Odd parity
 */
 #define USART_CR1_PS_bit 9

/*
Bit 8 PEIE: PE interrupt enable
This bit is set and cleared by software.
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever PE=1 in the USART_SR register
 */
 #define USART_CR1_PE_bit 8

/*
Bit 7 TXEIE: TXE interrupt enable
This bit is set and cleared by software.
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever TXE=1 in the USART_SR register
 */
 #define USART_CR1_TXEIE_bit 7

/*
Bit 6 TCIE: Transmission complete interrupt enable
This bit is set and cleared by software.
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever TC=1 in the USART_SR register
 */
 #define USART_CR1_TCIE_bit 6

/*
Bit 5 RXNEIE: RXNE interrupt enable
This bit is set and cleared by software.
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever ORE=1 or RXNE=1 in the USART_SR register
 */
 #define USART_CR1_RXNEIE_bit 5

/*
Bit 4 IDLEIE: IDLE interrupt enable
This bit is set and cleared by software.
  0: Interrupt is inhibited
  1: A USART interrupt is generated whenever IDLE=1 in the USART_SR register
 */
 #define USART_CR1_IDLEIE_bit 4

/*
Bit 3 TE: Transmitter enable
This bit enables the transmitter. It is set and cleared by software.
  0: Transmitter is disabled
  1: Transmitter is enabled
Note: 1: During transmission, a �0� pulse on the TE bit (�0� followed by �1�) sends a preamble
      (idle line) after the current word, except in smartcard mode.
      2: When TE is set there is a 1 bit-time delay before the transmission starts.
 */
 #define USART_CR1_TE_bit 3

/*
Bit 2 RE: Receiver enable
This bit enables the receiver. It is set and cleared by software.
  0: Receiver is disabled
  1: Receiver is enabled and begins searching for a start bit
 */
 #define USART_CR1_RE_bit 2

/*
Bit 1 RWU: Receiver wakeup
This bit determines if the USART is in mute mode or not. It is set and cleared by software and
can be cleared by hardware when a wakeup sequence is recognized.
  0: Receiver in active mode
  1: Receiver in mute mode
Note: 1: Before selecting Mute mode (by setting the RWU bit) the USART must first receive a
        data byte, otherwise it cannot function in Mute mode with wakeup by Idle line detection.
      2: In Address Mark Detection wakeup configuration (WAKE bit=1) the RWU bit cannot
        be modified by software while the RXNE bit is set.
 */
 #define USART_CR1_RWU_bit 1

/*
Bit 0 SBK: Send break
This bit set is used to send break characters. It can be set and cleared by software. It should
be set by software, and will be reset by hardware during the stop bit of break.
  0: No break character is transmitted
  1: Break character will be transmitted
 */
 #define USART_CR1_SBK_bit 0


/*
 +-----------------------------------------------------------------------------+
 | USART Control register 2 (USART_CR2)
 +-----------------------------------------------------------------------------+
 */


/*
Bits 31:15 Reserved, forced by hardware to 0.
*/


/*
Bit 14 LINEN: LIN mode enable
This bit is set and cleared by software.
  0: LIN mode disabled
  1: LIN mode enabled
The LIN mode enables the capability to send LIN Synch Breaks (13
 */
 #define USART_CR2_LIN_bit 14

/*
Bits 13:12 STOP: STOP bits
These bits are used for programming the stop bits.
00: 1 Stop bit
01: 0.5 Stop bit
10: 2 Stop bits
11: 1.5 Stop bit
The 0.5 Stop bit and 1.5 Stop bit are not available for UART4 & UART5.
 */
 #define USART_CR2_STOP_bit 12
 #define USART_CR2_STOP_0_bit 12
 #define USART_CR2_STOP_1_bit 13

/*
Bit 11 CLKEN: Clock enable
This bit allows the user to enable the CK pin.
0: CK pin disabled
1: CK pin enabled
This bit is not available for UART4 & UART5.
 */
 #define USART_CR2_CLKEN_bit 11

/*
Bit 10 CPOL: Clock polarity
This bit allows the user to select the polarity of the clock output on the CK pin in synchronous
mode. It works in conjunction with the CPHA bit to produce the desired clock/data
relationship
  0: Steady low value on CK pin outside transmission window.
  1: Steady high value on CK pin outside transmission window.
This bit is not available for UART4 & UART5.
 */
 #define USART_CR2_CPOL_bit 10

/*
Bit 9 CPHA: Clock phase
This bit allows the user to select the phase of the clock output on the CK pin in synchronous
mode. It works in conjunction with the CPOL bit to produce the desired clock/data
relationship
  0: The first clock transition is the first data capture edge.
  1: The second clock transition is the first data capture edge.
This bit is not available for UART4 & UART5.
 */
 #define USART_CR2_CPHA_bit 9


/*
Bit 8 LBCL: Last bit clock pulse
This bit allows the user to select whether the clock pulse associated with the last data bit
transmitted (MSB) has to be output on the CK pin in synchronous mode.
  0: The clock pulse of the last data bit is not output to the CK pin
  1: The clock pulse of the last data bit is output to the CK pin
The last bit is the 8th or 9th data bit transmitted depending on the 8 or 9 bit format selected
by the M bit in the USART_CR1 register.
This bit is not available for UART4 & UART5.
*/
 #define USART_CR2_LBCL_bit 8

/*
Bit 7 Reserved, forced by hardware to 0.
*/

/*
Bit 6 LBDIE: LIN break detection interrupt enable
Break interrupt mask (break detection using break delimiter).
0: Interrupt is inhibited
1: An interrupt is generated whenever LBD=1 in the USART_SR register
*/
 #define USART_CR2_LBDIE_bit 6

/*
Bit 5 LBDL: lin break detection length
This bit is for selection between 11 bit or 10 bit break detection.
0: 10 bit break detection
1: 11 bit break detection
*/
 #define USART_CR2_LBDL_bit 5

/*
Bit 4 Reserved, forced by hardware to 0.
*/

/*
Bits 3:0 ADD[3:0]: Address of the USART node
  This bit-field gives the address of the USART node.
  This is used in multiprocessor communication during mute mode, for wake up with address
  mark detection.
 */
 #define USART_CR2_ADD_bit 0
 #define USART_CR2_ADD_0_bit 0
 #define USART_CR2_ADD_1_bit 1
 #define USART_CR2_ADD_2_bit 2

#endif
