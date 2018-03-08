/*
 * lpc8xx_swm.h
 *
 *  Created on: Feb 16, 2016
 *
 */

#ifndef LPC8XX_SWM_H_
#define LPC8XX_SWM_H_

// Port pin number equates
#define P0_0      0
#define P0_1      1
#define P0_2      2
#define P0_3      3
#define P0_4      4
#define P0_5      5
#define P0_6      6
#define P0_7      7
#define P0_8      8
#define P0_9      9
#define P0_10    10
#define P0_11    11
#define P0_12    12
#define P0_13    13
#define P0_14    14
#define P0_15    15
#define P0_16    16
#define P0_17    17
#define P0_18    18
#define P0_19    19
#define P0_20    20
#define P0_21    21
#define P0_22    22
#define P0_23    23
#define P0_24    24
#define P0_25    25
#define P0_26    26
#define P0_27    27
#define P0_28    28



// Function name equates (for normal people)
#define U0_TXD        0
#define U0_RXD        1
#define U0_RTS        2
#define U0_CTS        3
#define U0_SCLK       4

#define U1_TXD        5
#define U1_RXD        6
#define U1_RTS        7
#define U1_CTS        8
#define U1_SCLK       9

#define U2_TXD        10
#define U2_RXD        11
#define U2_RTS        12
#define U2_CTS        13
#define U2_SCLK       14

#define SPI0_SCK      15
#define SPI0_MOSI     16
#define SPI0_MISO     17
#define SPI0_SSEL0    18
#define SPI0_SSEL1    19
#define SPI0_SSEL2    20
#define SPI0_SSEL3    21

#define SPI1_SCK      22
#define SPI1_MOSI     23
#define SPI1_MISO     24
#define SPI1_SSEL0    25
#define SPI1_SSEL1    26

#define SCT_PIN0      27
#define SCT_PIN1      28
#define SCT_PIN2      29
#define SCT_PIN3      30
#define SCT_OUT0      31
#define SCT_OUT1      32
#define SCT_OUT2      33
#define SCT_OUT3      34
#define SCT_OUT4      35
#define SCT_OUT5      36

#define I2C1_SDA      37
#define I2C1_SCL      38
#define I2C2_SDA      39
#define I2C2_SCL      40
#define I2C3_SDA      41
#define I2C3_SCL      42

#define ADC_PINTRIG0  43
#define ADC_PINTRIG1  44

#define ACOMP         45
#define CLKOUT        46
#define GPIO_INT_BMAT 47

#define num_funcs 48



// PINENABLE0 defines
#define ACMP_I1   (1<<0)
#define ACMP_I2   (1<<1)
#define ACMP_I3   (1<<2)
#define ACMP_I4   (1<<3)
#define SWCLK     (1<<4)
#define SWDIO     (1<<5)
#define XTALIN    (1<<6)
#define XTALOUT   (1<<7)
#define RESETN    (1<<8)
#define CLKIN     (1<<9)
#define VDDCMP    (1<<10)
#define I2C0_SDA  (1<<11)
#define I2C0_SCL  (1<<12)
#define ADC_0     (1<<13)
#define ADC_1     (1<<14)
#define ADC_2     (1<<15)
#define ADC_3     (1<<16)
#define ADC_4     (1<<17)
#define ADC_5     (1<<18)
#define ADC_6     (1<<19)
#define ADC_7     (1<<20)
#define ADC_8     (1<<21)
#define ADC_9     (1<<22)
#define ADC_10    (1<<23)
#define ADC_11    (1<<24)





#endif /* LPC8XX_SWM_H_ */
