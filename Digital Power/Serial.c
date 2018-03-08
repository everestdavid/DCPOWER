#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "LPC8xx.h"
#include "Board.h"
#include "LPC8xx_uart.h"
#include "lpc8xx_syscon.h"
#include "lpc8xx_swm.h"
#include "utilities.h"

/**
 * USART CFG register definitions
 */
#define UART_CFG_ENABLE         (0x01 << 0)
#define UART_CFG_DATALEN_7      (0x00 << 2)		/*!< UART 7 bit length mode */
#define UART_CFG_DATALEN_8      (0x01 << 2)		/*!< UART 8 bit length mode */
#define UART_CFG_DATALEN_9      (0x02 << 2)		/*!< UART 9 bit length mode */
#define UART_CFG_PARITY_NONE    (0x00 << 4)		/*!< No parity */
#define UART_CFG_PARITY_EVEN    (0x02 << 4)		/*!< Even parity */
#define UART_CFG_PARITY_ODD     (0x03 << 4)		/*!< Odd parity */
#define UART_CFG_STOPLEN_1      (0x00 << 6)		/*!< UART One Stop Bit Select */
#define UART_CFG_STOPLEN_2      (0x01 << 6)		/*!< UART Two Stop Bits Select */
#define UART_CFG_CTSEN          (0x01 << 9)		/*!< CTS enable bit */
#define UART_CFG_SYNCEN         (0x01 << 11)	/*!< Synchronous mode enable bit */
#define UART_CFG_CLKPOL         (0x01 << 12)	/*!< Un_RXD rising edge sample enable bit */
#define UART_CFG_SYNCMST        (0x01 << 14)	/*!< Select master mode (synchronous mode) enable bit */
#define UART_CFG_LOOP           (0x01 << 15)	/*!< Loopback mode enable bit */

/**
 * USART CTRL register definitions
 */
#define UART_CTRL_TXBRKEN       (0x01 << 1)		/*!< Continuous break enable bit */
#define UART_CTRL_ADDRDET       (0x01 << 2)		/*!< Address detect mode enable bit */
#define UART_CTRL_TXDIS         (0x01 << 6)		/*!< Transmit disable bit */
#define UART_CTRL_CC            (0x01 << 8)		/*!< Continuous Clock mode enable bit */
#define UART_CTRL_CLRCC         (0x01 << 9)		/*!< Clear Continuous Clock bit */
#define UART_CTRL_AUTOBAUD_EN   (0x01 << 16)	/*!< Enable Autobaud option */

/**
 * USART STAT register definitions
 */
#define UART_STAT_RXRDY         (0x01 << 0)			/*!< Receiver ready */
#define UART_STAT_RXIDLE        (0x01 << 1)			/*!< Receiver idle */
#define UART_STAT_TXRDY         (0x01 << 2)			/*!< Transmitter ready for data */
#define UART_STAT_TXIDLE        (0x01 << 3)			/*!< Transmitter idle */
#define UART_STAT_CTS           (0x01 << 4)			/*!< Status of CTS signal */
#define UART_STAT_DELTACTS      (0x01 << 5)			/*!< Change in CTS state */
#define UART_STAT_TXDISINT      (0x01 << 6)			/*!< Transmitter disabled */
#define UART_STAT_OVERRUNINT    (0x01 << 8)			/*!< Overrun Error interrupt flag. */
#define UART_STAT_RXBRK         (0x01 << 10)		/*!< Received break */
#define UART_STAT_DELTARXBRK    (0x01 << 11)		/*!< Change in receive break detection */
#define UART_STAT_START         (0x01 << 12)		/*!< Start detected */
#define UART_STAT_FRM_ERRINT    (0x01 << 13)		/*!< Framing Error interrupt flag */
#define UART_STAT_PAR_ERRINT    (0x01 << 14)		/*!< Parity Error interrupt flag */
#define UART_STAT_RXNOISEINT    (0x01 << 15)		/*!< Received Noise interrupt flag */
#define UART_STAT_ABERR         (0x01 << 16)		/*!< Autobuad error flag */

/**
 * USART INTENSET/INTENCLR register definitions
 */
#define UART_INTEN_RXRDY        (0x01 << 0)			/*!< Receive Ready interrupt */
#define UART_INTEN_TXRDY        (0x01 << 2)			/*!< Transmit Ready interrupt */
#define UART_INTEN_DELTACTS     (0x01 << 5)			/*!< Change in CTS state interrupt */
#define UART_INTEN_TXDIS        (0x01 << 6)			/*!< Transmitter disable interrupt */
#define UART_INTEN_OVERRUN      (0x01 << 8)			/*!< Overrun error interrupt */
#define UART_INTEN_DELTARXBRK   (0x01 << 11)		/*!< Change in receiver break detection interrupt */
#define UART_INTEN_START        (0x01 << 12)		/*!< Start detect interrupt */
#define UART_INTEN_FRAMERR      (0x01 << 13)		/*!< Frame error interrupt */
#define UART_INTEN_PARITYERR    (0x01 << 14)		/*!< Parity error interrupt */
#define UART_INTEN_RXNOISE      (0x01 << 15)		/*!< Received noise interrupt */
#define UART_INTEN_ABERR        (0x01 << 16)		/*!< Autobuad error interrupt */


extern osThreadId t_uart0_rxdata;

// Implementation of sendchar, hard-coded to UART0 (used by printf)
// This is for Keil projects.
int sendchar (int ch) {
  while (!((LPC_USART0->STAT) & TXRDY));   // Wait for TX Ready
  return (LPC_USART0->TXDAT  = ch);        // Write one character to TX data register
}


// Implementation of MyLowLevelPutchar, hard-coded to UART0 (used by printf)
// This is for IAR projects. Must include locally modified __write in the project.
int MyLowLevelPutchar(int ch) {
  while (!((LPC_USART0->STAT) & TXRDY));   // Wait for TX Ready
  return (LPC_USART0->TXDAT  = ch);        // Write one character to TX data register
}


// Implementation of getkey, hard-coded to UART0 (used by scanf)
// This is for Keil and LPCxpresso projects.
int getkey (void) {
  while (!((LPC_USART0->STAT) & RXRDY));   // Wait for RX Ready
  return (LPC_USART0->RXDAT );             // Read one character from RX data register
}


// Implementation of MyLowLevelGetchar, hard-coded to UART0 (used by scanf)
// This is for IAR projects. Must include locally modified __read in the project.
int MyLowLevelGetchar(void){
  while (!((LPC_USART0->STAT) & RXRDY));   // Wait for RX Ready
  return (LPC_USART0->RXDAT );             // Read one character from RX data register
}


// setup_debug_uart, hard coded to UART0, desired_baudrate/8/N/1
void setup_debug_uart() {

  // Select the baud rate
  const uint32_t desired_baud = 115200;
	
	// Turn on relevant clocks
	#if 0
	LPC_SWM->PINASSIGN[0] |= 0x0000FFFF;                    // 
  LPC_SWM->PINASSIGN[0] &= ((UART0_TX << 0) | 0xFFFFFF00);  // assign U0 TXD to P0_txdpin
  LPC_SWM->PINASSIGN[0] &= ((UART0_RX << 8) | 0xFFFF00FF);  // assign U0 RXD to P0_rxdpin

  LPC_SYSCON->SYSAHBCLKCTRL |= (UART0 | SWM);

  // Connect UART0 TXD, RXD signals to port pins
  //ConfigSWM(U0_TXD, P0_4);       // Use with USB-to-RS232 break-out cable
  //ConfigSWM(U0_RXD, P0_0);       // Use with USB-to-RS232 break-out cable
 // ConfigSWM(U0_TXD, UART0_TX);  // For MBED serial port (requires board mod.)
//  ConfigSWM(U0_RXD, UART0_RX);  // For MBED serial port (requires board mod.)
	
  // UART BRG calculation:
  // For asynchronous mode (UART mode) the BRG formula is:
  // (BRG + 1) * (1 + (m/256)) * (UARTCLKDIV) * (16 * baudrate Hz.) = MainClock Hz.
  // As long as UARTCLKDIV = AHBCLKDIV, and FRG = 1, the System Clock and the UARTn_PCLKs will be the same.
  // For this example, we set m = 0 (so FRG = 1), and UARTCLKDIV = AHBCLKDIV.
  // Then, we can use the SystemCoreClock variable, as set by the function SystemCoreClockUpdate(),
  // in our BRG calculation as follows:
  // BRG = (SystemCoreClock Hz. / (16 * desired_baud Hz.)) - 1

  // Configure the UARTCLKDIV, default for calculation below is same as AHBCLKDIV
	//LPC_SYSCON->UARTCLKDIV = LPC_SYSCON->SYSAHBCLKDIV;
	LPC_SYSCON->UARTCLKDIV = 1;
  #endif
	#if 1
	    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 0) |                 // enable clock to Switch Matrix
                                 (1 << 4);                 // enable AHB clock to the IOCON

    LPC_SWM->PINASSIGN[0] |= 0x0000FFFF;                    // 
    LPC_SWM->PINASSIGN[0] &= ((UART0_TX << 0) | 0xFFFFFF00);  // assign U0 TXD to P0_txdpin
    LPC_SWM->PINASSIGN[0] &= ((UART0_RX << 8) | 0xFFFF00FF);  // assign U0 RXD to P0_rxdpin

    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 14);                 // Enable USART0 clock
    LPC_SYSCON->UARTCLKDIV     = 1;   
	
		LPC_USART0->BRG      = (SystemCoreClock / (desired_baud << 4)) - 1;                         // set baud rate
    LPC_USART0->CFG      = (1 << 0) | (1 << 2);             // 8 bits, no Parity, 1 Stop bit

    LPC_USART0->STAT     = UART_STAT_DELTACTS
                         | UART_STAT_OVERRUNINT
                         | UART_STAT_DELTARXBRK
                         | UART_STAT_START
                         | UART_STAT_FRM_ERRINT
                         | UART_STAT_PAR_ERRINT
                         | UART_STAT_RXNOISEINT
                         | UART_STAT_ABERR;
    LPC_USART0->INTENSET = UART_INTEN_RXRDY;
    LPC_USART0->INTENCLR = UART_INTEN_TXRDY;
    LPC_USART0->CTL    &= ~UART_CTRL_TXDIS;

    NVIC_EnableIRQ(UART0_IRQn);     
	#else
  // Configure the FRG (default for calculation below is divide-by-1)
  LPC_SYSCON->UARTFRGMULT = 0;
  LPC_SYSCON->UARTFRGDIV = 255;

  // Give USART0 a reset
  LPC_SYSCON->PRESETCTRL &= (UART0_RST_N);
  LPC_SYSCON->PRESETCTRL |= ~(UART0_RST_N);

  // Get the System Clock frequency for the BRG calculation.
  //SystemCoreClockUpdate();
	
  // Write calculation result to BRG register
  LPC_USART0->BRG = (SystemCoreClock / (16 * desired_baud)) - 1;

  // Configure the USART0 CFG register:
  // 8 data bits, no parity, one stop bit, no flow control, asynchronous mode
  LPC_USART0->CFG = DATA_LENG_8|PARITY_NONE|STOP_BIT_1;

  // Configure the USART0 CTL register (nothing to be done here)
  // No continuous break, no address detect, no Tx disable, no CC, no CLRCC
  LPC_USART0->CTL = 0;

  // Clear any pending flags (Just to be safe, isn't necessary after the peripheral reset)
  LPC_USART0->STAT = 0xFFFF;

  // Don't enable the USART0 RX Ready Interrupt, this function assumes a polled use case
  LPC_USART0->INTENSET = RXRDY;	  // if a valid rx data in buffer, then enable/trigger an interrupt
  NVIC_EnableIRQ(UART0_IRQn);    //Enable The NVIC UART0 Interrupt

  // Enable USART0
  LPC_USART0->CFG |= UART_EN;
	#endif
	
}


