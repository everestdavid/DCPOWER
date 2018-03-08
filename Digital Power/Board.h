/*-----------------------------------------------------------------------------
 * Name:    Board_LED.h
 * Purpose: LED interface header file
 * Rev.:    1.00
 *-----------------------------------------------------------------------------*/

/* Copyright (c) 2013 - 2014 ARM LIMITED

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

#ifndef __BOARD_H
#define __BOARD_H

//#include <LPC8xx.h>
#include <stdint.h>
#include "lpc8xx_swm.h"
//#include "utilities.h"

#define ENABLE  1
#define DISABLE 0

#define DMA_CLK_EN   29
#define ADC_CLK_EN   24
#define CMP_CLK_EN   19
#define IOCON_CLK_EN  18
#define UART1_CLK_EN  15
#define UART0_CLK_EN  14
#define MRT_CLK_EN    10
#define SCT_CLK_EN    8
#define SWM_CLK_EN    7
#define GPIO_CLK_EN   6
#define I2C0_CLK_EN   5
#define FLASH_CLK_EN   4
#define FLASHREG_CLK_EN 3
#define RAM01_CLK_EN 2
#define ROM_CLK_EN 1
#define SPI0_CLK_EN  11
/*
* Definition of the GPIO
*/
#define POWER_SHUTDOWN_PIN   22   //12  
#define GAIN0_PIN   16  
#define GAIN1_PIN   27  
#define GAIN2_PIN   26  
#define VOL0_PIN		25
#define VOL1_PIN    24
#define VOL2_PIN    15
#define VOLEN_PIN   9
#define CALI_PIN    8
#define DACDAT_PIN  18
#define DACCLK_PIN  19
#define DACSYNC_PIN  20

#define UART0_TX P0_4//P0_21             // For the uart0 serial port
#define UART0_RX P0_0//P0_22            // For the uart0 serial port

#define UART1_TX P0_2             // For the uart1 serial port
#define UART1_RX P0_3            // For the uart1 serial port

#define I2C_SDA P0_11						  //For the I2C port
#define I2C_SCL P0_10

#define RX_BUFFER_SIZE 100

#define SHUTDOWN 0x00
#define POWERON  0x80
#define NORMAL   0x00 
#define OVER_CUR  0x01

#define CURRENT_THRESHOLD 2000

#define MAIN_2P5V    0
#define MAIN_5P3V    1
#define MAIN_6P4V    2
#define MAIN_7P4V    3
#define MAIN_8P6V    4
#define MAIN_9P3V    5
#define MAIN_10P8V   6
#define MAIN_11P7V   7
#define MAIN_12P7V   8
/**
  \fn          int32_t LED_Initialize (void)
  \brief       Initialize I/O interface for LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t LED_Uninitialize (void)
  \brief       De-initialize I/O interface for LEDs
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t LED_On (uint32_t num)
  \brief       Turn on a single LED indicated by \em num
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t LED_Off (uint32_t num)
  \brief       Turn off a single LED indicated by \em num
  \param[in]   num  LED number
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t LED_SetOut (uint32_t val)
  \brief       Control all LEDs with the bit vector \em val
  \param[in]   val  each bit represents the status of one LED.
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          uint32_t LED_GetCount (void)
  \brief       Get number of available LEDs on evaluation hardware
  \return      Number of available LEDs
*/
/**
  \fn          int32_t DigPower_Shutdown (uint32_t shutdown)
  \brief       Turn on or off digital power supply module
  \param[in]   shutdown  =0, shutdown power supply
							           =1, turn on power supply
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t DigPower_Shutdown (uint32_t shutdown)
  \brief       Turn on or off digital power supply module
  \param[in]   shutdown  =0, shutdown power supply
							           =1, turn on power supply
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t MainSupply_Adj_Enable (uint32_t enable)
  \brief       Enable the Adjustment of main supply
  \param[in]   enable = 0, disable
											= 1, enable
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t MainSupply_Adjustment (uint32_t vol)
  \brief       Adjust the Voltage of main supply
  \param[in]   vol, the voltge step of main supply
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/
/**
  \fn          int32_t Gain_Adjustment (uint32_t gain)
  \brief       Adjust the gain of voltage
  \param[in]   gain, the gain step
  \returns
   - \b  0: function succeeded
   - \b -1: function failed
*/

extern int32_t  Board_Initialize   (void);
extern int32_t  LED_Uninitialize (void);
extern int32_t  LED_On           (uint32_t num);
extern int32_t  LED_Off          (uint32_t num);
extern int32_t  LED_SetOut       (uint32_t val);
extern uint32_t LED_GetCount     (void);
extern int32_t DigPower_Shutdown (uint32_t shutdown);
extern int32_t Calibrate (uint32_t cali);
extern int32_t MainSupply_Adj_Enable (uint32_t enable);
extern int32_t MainSupply_Adjustment (uint32_t vol);
extern int32_t Gain_Adjustment (uint32_t gain);
#endif /* __BOARD_H */
