#ifndef __I2C_H
#define __I2C_H

#define Self_Slave_address 0x50

#define OVER_TIME    0XFF
#define INVALID_STA  0XFE

void i2c0_init(void) ;
unsigned int i2c0_read(unsigned char chip_addr, unsigned int reg,  unsigned char *pdat);
unsigned int  i2c0_write(unsigned char chip_addr, unsigned int reg, unsigned char dat);
#endif  /* __I2C_H */
