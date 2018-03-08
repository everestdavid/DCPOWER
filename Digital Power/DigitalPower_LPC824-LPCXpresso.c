/*-----------------------------------------------------------------------------
 * Name:    LED_LPC824-LPCXpresso.c
 * Purpose: LED interface for LPC82x-LPCXpresso evaluation board
 * Rev.:    1.01
 *----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2016 ARM LIMITED

   All rights reserved.
   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:
   - Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   - Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in the
     documentation and/or other materials provided with the distribution.
   - Neither the name of ARM nor the names of its contributors may be used
     to endorse or promote products derived from this software without
     specific prior written permission.
   *
   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS AND CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
   ---------------------------------------------------------------------------*/

#include "LPC8xx.h"                     // Device header
#include "Board.h"
//#include "utilities.h"

extern void setup_debug_uart(void);
extern void ConfigSWM(uint32_t func, uint32_t port_pin);

#define LED_NUM             (3)

const uint32_t led_mask[] = { 1UL << 27, 1UL << 16, 1UL << 12 };

/**
  \fn          int32_t LED_Initialize (void)
  \brief       Initialize LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t Board_Initialize (void) {

  LPC_SYSCON->SYSAHBCLKCTRL |= ( (1 << DMA_CLK_EN) |    /* Enable Clock for DMA */
																 (1 << ADC_CLK_EN) |    /* Enable Clock for ADC */
																 (1 << CMP_CLK_EN) |    /* Enable Clock for Analog Compare*/
																 //(1 << UART0_CLK_EN) | 	/* Enable Clock for UART0 */
																 (1 << UART1_CLK_EN) | 	/* Enable Clock for UART1 */
																 (1 << MRT_CLK_EN) | 		/* Enable Clock for MRT */
																 (1 << GPIO_CLK_EN) |   /* Enable Clock for GPIO */
																 //(1 << I2C0_CLK_EN) | 	/* Enable Clock for I2C0 */
																 (1 << FLASH_CLK_EN) | 	/* Enable Clock for FLASH */
																 (1 << FLASHREG_CLK_EN) |  /* Enable Clock for FLASHREG */
																 (1 << RAM01_CLK_EN) |	   /* Enable Clock for RAM0/1 */
																 (1 << SWM_CLK_EN) |	   /* Enable Clock for SWM */
																 (1 << IOCON_CLK_EN) |	   /* Enable Clock for IOCON */
																 (1 << ROM_CLK_EN) |      /* Enable Clock for ROM */
																 (1 << SPI0_CLK_EN));            /* Enable Clock for SPI0 */

  LPC_GPIO_PORT->DIR0 |= ((1 << POWER_SHUTDOWN_PIN) |           /* configure GPIO as output */
                          (1 << GAIN0_PIN) |
                          (1 << GAIN1_PIN) | 
													(1 << GAIN2_PIN) |
													(1 << VOL0_PIN) |
													(1 << VOL1_PIN) |
													(1 << VOL2_PIN) |
													(1 << VOLEN_PIN) |
													(1 << CALI_PIN) |
													(1 << DACDAT_PIN) |
													(1 << DACCLK_PIN) |
													(1 << DACSYNC_PIN)
													);
		/*
	ConfigSWM(U0_TXD, UART0_TX);	//config UART0 RX/TX pin
	ConfigSWM(U0_RXD, UART0_RX);

	ConfigSWM(U1_TXD, UART1_TX);	//config UART1 RX/TX pin
	ConfigSWM(U1_RXD, UART1_RX);	
	*/
	setup_debug_uart();
	//ConfigSWM(I2C1_SDA, I2C_SDA);	//config I2C SDA/SCL pin
	//ConfigSWM(I2C1_SCL, I2C_SCL);	

	//ConfigSWM(SPI0_MOSI, DACDAT_PIN);	//config SPI0 CS/SCK/MOSI/MISO pin
	//ConfigSWM(SPI0_SCK, DACCLK_PIN);	
	//ConfigSWM(SPI0_SSEL0, DACSYNC_PIN);	
	//ConfigSWM(SPI0_MISO, 21);	

  //LED_SetOut (0);                                 /* switch LEDs off          */
	DigPower_Shutdown(DISABLE);
	Calibrate(DISABLE);
	MainSupply_Adj_Enable(DISABLE);
	MainSupply_Adjustment(0);
	Gain_Adjustment(0);
	
	LPC_GPIO_PORT->SET0 = (1 << DACSYNC_PIN);
	LPC_GPIO_PORT->SET0 = (1 << DACCLK_PIN);
	LPC_GPIO_PORT->CLR0 = (1 << DACDAT_PIN);
  return 0;
}

/**
  \fn          int32_t DigPower_Shutdown (uint32_t shutdown)
  \brief       Turn on or off digital power supply module
  \param[in]   shutdown  =0, shutdown power supply
							           =1, turn on power supply
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t DigPower_Shutdown (uint32_t shutdown) {
	if(shutdown==POWERON) {   //shutdown
		LPC_GPIO_PORT->SET0 = (1 << POWER_SHUTDOWN_PIN);
	}	else {
		LPC_GPIO_PORT->CLR0 = (1 << POWER_SHUTDOWN_PIN);
	}
  return 0;
}
/**
  \fn          int32_t Calibrate (uint32_t cali)
  \brief       Start Calibrate
  \param[in]   cali  =0, disable calibration
							       =1, enable calibration
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t Calibrate (uint32_t cali) {
	if(cali==0) {   //disable
		LPC_GPIO_PORT->CLR0 = (1 << CALI_PIN);
	}	else {				//enable
		LPC_GPIO_PORT->SET0 = (1 << CALI_PIN);
	}
  return 0;
}
/**
  \fn          int32_t MainSupply_Adj_Enable (uint32_t enable)
  \brief       Enable the Adjustment of main supply
  \param[in]   enable = 0, disable
											= 1, enable
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t MainSupply_Adj_Enable (uint32_t enable) {
	if(enable==0) {   //disable
		LPC_GPIO_PORT->CLR0 = (1 << VOLEN_PIN);
	}	else {				//enable
		LPC_GPIO_PORT->SET0 = (1 << VOLEN_PIN);
	}
  return 0;
}
/**
  \fn          int32_t MainSupply_Adjustment (uint32_t vol)
  \brief       Adjust the Voltage of main supply
  \param[in]   vol, the voltge step of main supply
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t MainSupply_Adjustment (uint32_t vol) {
	switch(vol) {
		case MAIN_2P5V:
					//MainSupply_Adj_Enable(DISABLE);
					//LPC_GPIO_PORT->CLR0	= (1 << VOL0_PIN) | (1 << VOL1_PIN) | (1 << VOL2_PIN);
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL0_PIN) | (1 << VOL1_PIN) | (1 << VOL2_PIN);

					break;
		case MAIN_5P3V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL0_PIN) | (1 << VOL1_PIN) | (1 << VOL2_PIN);
					break;
		case MAIN_6P4V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->SET0	|= (1 << VOL0_PIN);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL1_PIN) | (1 << VOL2_PIN);
					break;		
		case MAIN_7P4V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->SET0	|= (1 << VOL1_PIN);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL0_PIN) | (1 << VOL2_PIN);
					break;	
		case MAIN_8P6V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL2_PIN);
					LPC_GPIO_PORT->SET0	|= (1 << VOL0_PIN) | (1 << VOL1_PIN);
					break;
		case MAIN_9P3V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->SET0	|= (1 << VOL2_PIN);
					LPC_GPIO_PORT->CLR0  |= (1 << VOL0_PIN) | (1 << VOL1_PIN);
					break;
		case MAIN_10P8V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL1_PIN);
					LPC_GPIO_PORT->SET0	|= (1 << VOL0_PIN) | (1 << VOL2_PIN);
					break;
		case MAIN_11P7V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL0_PIN);
					LPC_GPIO_PORT->SET0	|= (1 << VOL1_PIN) | (1 << VOL2_PIN);
					break;
		case MAIN_12P7V:
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->SET0	|= (1 << VOL0_PIN) | (1 << VOL1_PIN) | (1 << VOL2_PIN);
					break;
		default:
					//MainSupply_Adj_Enable(DISABLE);
					//LPC_GPIO_PORT->CLR0	= (1 << VOL0_PIN) | (1 << VOL1_PIN) | (1 << VOL2_PIN
					MainSupply_Adj_Enable(ENABLE);
					LPC_GPIO_PORT->CLR0	|= (1 << VOL0_PIN) | (1 << VOL1_PIN) | (1 << VOL2_PIN);

					break;
	}
  return 0;
}
/**
  \fn          int32_t Gain_Adjustment (uint32_t gain)
  \brief       Adjust the gain of voltage
  \param[in]   gain, the gain step
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
int32_t Gain_Adjustment (uint32_t gain) {
	switch(gain) {
		case 0:
					LPC_GPIO_PORT->SET0	= (1 << GAIN0_PIN) | (1 << GAIN1_PIN) | (1 << GAIN2_PIN);
					break;
		case 1:
					LPC_GPIO_PORT->CLR0	= (1 << GAIN0_PIN);
					LPC_GPIO_PORT->SET0	= (1 << GAIN1_PIN) | (1 << GAIN2_PIN);
					break;		
		case 2:
					LPC_GPIO_PORT->CLR0	= (1 << GAIN1_PIN);
					LPC_GPIO_PORT->SET0	= (1 << GAIN0_PIN) | (1 << GAIN2_PIN);
					break;	
		case 3:
					LPC_GPIO_PORT->SET0	= (1 << GAIN2_PIN);
					LPC_GPIO_PORT->CLR0	= (1 << GAIN0_PIN) | (1 << GAIN1_PIN);
					break;
		case 4:
					LPC_GPIO_PORT->CLR0	= (1 << GAIN2_PIN);
					LPC_GPIO_PORT->SET0	= (1 << GAIN0_PIN) | (1 << GAIN1_PIN);
					break;
		case 5:
					LPC_GPIO_PORT->SET0	= (1 << GAIN1_PIN);
					LPC_GPIO_PORT->CLR0	= (1 << GAIN0_PIN) | (1 << GAIN2_PIN);
					break;
		case 6:
					LPC_GPIO_PORT->SET0	= (1 << GAIN0_PIN);
					LPC_GPIO_PORT->CLR0	= (1 << GAIN1_PIN) | (1 << GAIN2_PIN);
					break;
		case 7:
					LPC_GPIO_PORT->CLR0	= (1 << GAIN0_PIN) | (1 << GAIN1_PIN) | (1 << GAIN2_PIN);
					break;
		default:
					LPC_GPIO_PORT->CLR0	= (1 << GAIN0_PIN) | (1 << GAIN1_PIN) | (1 << GAIN2_PIN);
					break;
	}
  return 0;
}

