#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "LPC8xx.h"                     // Device header
#include "stdio.h"
#include "lpc8xx_syscon.h"
#include "lpc8xx_spi.h"
#include "spi0.h"
#include "board.h"
/**
  \fn          void spi0_ini (void)
  \brief       initialize spi0
  \param[in]   
  \returnse
	\changes[date]
*/
void spi0_ini(void)
{
	  // Setup the SPIs ...
  // Give both SPIs a reset  (see lpc8xx_syscon.h)
  LPC_SYSCON->PRESETCTRL &= (SPI0_RST_N);
  LPC_SYSCON->PRESETCTRL |= ~((SPI0_RST_N));

  // Configure the SPI master's clock divider, slave's value meaningless. (value written to DIV divides by value+1)
  // SCK = SPI_PCLK divided by 2
  LPC_SPI0->DIV = (20000-1);

  // Configure the CFG registers:
  // Enable=true, master/slave, no LSB first, CPHA=1, CPOL=1, no loop-back, SSEL active low
  LPC_SPI0->CFG = CFG_ENABLE | CFG_MASTER | CFG_CPHA | CFG_CPOL;


  // Configure the master SPI delay register (DLY), slave's value meaningless.
  // Pre-delay = 0 clocks, post-delay = 0 clocks, frame-delay = 0 clocks, transfer-delay = 0 clocks
  LPC_SPI0->DLY = 0x0000;

  // Configure the SPI control registers
  // Master: End-of-frame true, LEN = 8 bits. Slave: LEN = 8 bits
  LPC_SPI0->TXCTL = CTL_EOF | CTL_LEN(12);
  //LPC_SPI1->TXCTL = CTL_LEN(8);
}
/**
  \fn          void spi0_tx (unsigned int dat)
  \brief       transfer dac data
  \param[in]   unsigned int dat, the dac data
  \returnse    
				\b			 0, tx success
				\b			 1, tx failed
	\changes[date]
*/
#if 0
int spi0_tx (unsigned int dat) 
{
	unsigned char cnt;
	unsigned int temp;
	
	//LPC_SPI0->TXCTL &= ~(CTL_EOT);                 // Start a new transfer, clear the EOT bit
	//LPC_SPI0->TXCTL |= CTL_TXSSELN;
	LPC_GPIO_PORT->CLR0 = (1 << 20);
	//osDelay(10);
	for(cnt =0; cnt < 10; cnt++) {
		if ((LPC_SPI0->STAT & STAT_TXRDY) != 0) {
			LPC_SPI0->TXDAT = dat;
			temp = LPC_SPI0->RXDAT;
			break;
		}
		osDelay(10);
	}
	for(cnt =0; cnt < 10; cnt++) {
		if ((LPC_SPI0->STAT & STAT_TXRDY) != 0) {
			LPC_SPI0->TXCTL |= CTL_EOT;
			return 0;
		}	
		osDelay(10);
	}
	//LPC_SPI0->TXCTL &= ~CTL_TXSSELN;
	LPC_GPIO_PORT->SET0 = (1 << DACSYNC_PIN);
	
	//LPC_SPI0->TXCTL |= CTL_EOT;
	return 1; 
}
#else
int spi0_tx (unsigned int dat) 
{
	unsigned char cnt;
	unsigned char tempH, tempL;
	
	//LPC_SPI0->TXCTL &= ~(CTL_EOT);                 // Start a new transfer, clear the EOT bit
	//LPC_SPI0->TXCTL |= CTL_TXSSELN;
	tempH = dat / 256;
	tempL = dat % 256;
	tempL &= 0xFC;
	LPC_GPIO_PORT->CLR0 = (1 << DACCLK_PIN);
	LPC_GPIO_PORT->CLR0 = (1 << DACSYNC_PIN);
	//osDelay(10);
	for(cnt =0; cnt < 8; cnt++) {
		LPC_GPIO_PORT->CLR0 = (1 << DACCLK_PIN);
    if(tempH & 0x80) {
			LPC_GPIO_PORT->SET0 = (1 << DACDAT_PIN);
		}
		else {
			LPC_GPIO_PORT->CLR0 = (1 << DACDAT_PIN);
		}
	
		osDelay(1);
		LPC_GPIO_PORT->SET0 = (1 << DACCLK_PIN);
		tempH = tempH << 1;
		osDelay(1);
	}
	
	for(cnt =0; cnt < 8; cnt++) {
		LPC_GPIO_PORT->CLR0 = (1 << DACCLK_PIN);
    if(tempL & 0x80) {
			LPC_GPIO_PORT->SET0 = (1 << DACDAT_PIN);
		}
		else {
			LPC_GPIO_PORT->CLR0 = (1 << DACDAT_PIN);
		}
	
		osDelay(1);
		LPC_GPIO_PORT->SET0 = (1 << DACCLK_PIN);
		tempL = tempL << 1;
		osDelay(1);
	}
	LPC_GPIO_PORT->CLR0 = (1 << DACCLK_PIN);
	//LPC_SPI0->TXCTL &= ~CTL_TXSSELN;
	LPC_GPIO_PORT->SET0 = (1 << DACSYNC_PIN);
	osDelay(1);
	LPC_GPIO_PORT->SET0 = (1 << DACCLK_PIN);
	
	//LPC_SPI0->TXCTL |= CTL_EOT;
	return 0; 
}
#endif

