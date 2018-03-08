/*
 * lpc8xx_syscon.h
 *
 *  Created on: Feb 16, 2016
 *      Author: 
 */

#ifndef LPC8XX_SYSCON_H_
#define LPC8XX_SYSCON_H_


// SYSAHBCLKCTRL register bits
#define ROM       (1<<1)
#define RAM0_1    (1<<2)
#define FLASHREG  (1<<3)
#define FLASH     (1<<4)
#define I2C0      (1<<5)
#define GPIO      (1<<6)
#define SWM       (1<<7)
#define SCT       (1<<8)
#define WKT       (1<<9)
#define MRT       (1<<10)
#define SPI0      (1<<11)
#define SPI1      (1<<12)
#define CRC       (1<<13)
#define UART0     (1<<14)
#define UART1     (1<<15)
#define UART2     (1<<16)
#define WWDT      (1<<17)
#define IOCON     (1<<18)
#define ACMP      (1<<19)
#define I2C1      (1<<20)
#define I2C2      (1<<21)
#define I2C3      (1<<22)
#define ADC       (1<<24)
#define MTB       (1<<26)
#define DMA       (1<<29)

// PRESETCTRL register bits
#define SPI0_RST_N    ~(1<<0)
#define SPI1_RST_N    ~(1<<1)
#define UARTFRG_RST_N ~(1<<2)
#define UART0_RST_N   ~(1<<3)
#define UART1_RST_N   ~(1<<4)
#define UART2_RST_N   ~(1<<5)
#define I2C0_RST_N    ~(1<<6)
#define MRT_RST_N     ~(1<<7)
#define SCT0_RST_N    ~(1<<8)
#define WKT_RST_N     ~(1<<9)
#define GPIO_RST_N    ~(1<<10)
#define FLASH_RST_N   ~(1<<11)
#define ACMP_RST_N    ~(1<<12)
#define I2C1_RST_N    ~(1<<14)
#define I2C2_RST_N    ~(1<<15)
#define I2C3_RST_N    ~(1<<16)
#define ADC_RST_N     ~(1<<24)
#define DMA_RST_N     ~(1<<29)

// STARTERP1 register bits
#define SPI0_INT_WAKEUP   (1<<0)
#define SPI1_INT_WAKEUP   (1<<1)
#define USART0_INT_WAKEUP (1<<3)
#define USART1_INT_WAKEUP (1<<4)
#define USART2_INT_WAKEUP (1<<5)
#define I2C1_INT_WAKEUP   (1<<7)
#define I2C0_INT_WAKEUP   (1<<8)
#define WWDT_INT_WAKE     (1<<12)
#define BOD_INT_WAKE      (1<<13)
#define WKT_INT_WAKEUP    (1<<15)
#define I2C2_INT_WAKEUP   (1<<21)
#define I2C3_INT_WAKEUP   (1<<22)

// PDAWAKECFG and PDRUNCFG register bits
#define IRCOUT_PD         (1<<0)
#define IRC_PD            (1<<1)
#define FLASH_PD          (1<<2)
#define BOD_PD            (1<<3)
#define ADC_PD            (1<<4)
#define SYSOSC_PD         (1<<5)
#define WDTOSC_PD         (1<<6)
#define SYSPLL_PD         (1<<7)
#define ACMP_PD           (1<<15)

// WDTOSCCTRL register bit field shifters
#define DIVSEL  0
#define FREQSEL 5

#endif /* LPC8XX_SYSCON_H_ */
