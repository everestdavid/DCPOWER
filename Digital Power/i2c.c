#include "cmsis_os.h" 
#include "LPC8xx.h"
//#include <cr_section_macros.h>
//#include <stdio.h>

#include "lpc8xx_i2c.h"
#include "lpc8xx_swm.h"
#include "lpc8xx_syscon.h"
#include "utilities.h"
#include "i2c.h"

/*****************************************************************************
** Function name:		WaitI2CMasterState
**
** Description:		    Waits for I2C master pending, then compares the master
**                      state to the state parameter. If compare fails, enter
**                      a while(1) loop. If compare passes, return.
**
** parameters:
**                      ptr_LPC_I2C: A pointer to an I2C instance
**                      state: One of the 3-bit Master function state codes of the I2C
** Returned value:		None
**
*****************************************************************************/
unsigned char WaitI2CMasterState(uint32_t state) {
	unsigned int index, flag;
	flag = 1;
	for(index = 0; index<10; index++) {
	 if((LPC_I2C->STAT & STAT_MSTPEND)) {
		 flag = 0;
		 break;	
	 }
	 osDelay(50);
	}
	if(flag == 1) {
		return OVER_TIME;
	}
	
  if((LPC_I2C->STAT & MASTER_STATE_MASK) != state) { // If master state mismatch ...
    return INVALID_STA;                                          // die here and debug the problem
  }
  return 0;                                                // If no mismatch, return

}

unsigned int i2c0_read(unsigned char chip_addr, unsigned int reg, unsigned char *pdat)
{
	unsigned  int status = 0;
	status = WaitI2CMasterState(I2C_STAT_MSTST_IDLE);
	if(status == 0) { // Wait the master state to be idle
    LPC_I2C0->MSTDAT = (chip_addr<<1) | 0;    // Address with 0 for RWn bit (WRITE)
    LPC_I2C0->MSTCTL = CTL_MSTSTART;                   // Start the transaction by setting the MSTSTART bit to 1 in the Master control register.
    status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the address to be ACK'd
    if(status == 0) {
      LPC_I2C0->MSTDAT = reg;// / 256;                           // Send the reg addr to the slave
      LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
		  
      status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the addr to be ACK'd
      if(status == 0) {
				LPC_I2C0->MSTDAT = reg % 256;                           // Send the reg addr to the slave
				LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
				status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the addr to be ACK'd

				if(status == 0) {
					LPC_I2C0->MSTDAT = (chip_addr<<1) | 1;    // Address with 1 for RWn bit (read)
					LPC_I2C0->MSTCTL = CTL_MSTSTART;                   // Start the transaction by setting the MSTSTART bit to 1 in the Master control register.
					status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the address to be ACK'd
					if(status == 0) {
						LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
						status = WaitI2CMasterState(I2C_STAT_MSTST_RX);   // Wait for the data to be ACK'd	
            if(status == 0) {
							*pdat = LPC_I2C0->MSTDAT;                           // Send the reg data to the slave
						}
					}
			  }
			}
		}
	}	
	
	LPC_I2C0->MSTCTL = CTL_MSTSTOP; 
	return status;
}


unsigned int i2c0_write(unsigned char chip_addr, unsigned int reg, unsigned char dat)
{
	unsigned  int status = 0;
	status = WaitI2CMasterState(I2C_STAT_MSTST_IDLE); // Wait the master state to be idle
	if(status == 0) {
    LPC_I2C0->MSTDAT = (chip_addr<<1) | 0;    // Address with 0 for RWn bit (WRITE)
    LPC_I2C0->MSTCTL = CTL_MSTSTART;                   // Start the transaction by setting the MSTSTART bit to 1 in the Master control register.
    status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the address to be ACK'd
    if(status == 0) {
      LPC_I2C0->MSTDAT = reg ;/// 256;                           // Send the reg addr to the slave
      LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
      status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the addr to be ACK'd
      if(status == 0) {
				LPC_I2C0->MSTDAT = reg % 256;                           // Send the reg addr to the slave
				LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
				status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the addr to be ACK'd
				if(status == 0) {
					LPC_I2C0->MSTDAT = dat;                           // Send the reg data to the slave
					LPC_I2C0->MSTCTL = CTL_MSTCONTINUE;                // Continue the transaction
					status = WaitI2CMasterState(I2C_STAT_MSTST_TX);   // Wait for the data to be ACK'd	
				}	
      }				
		}
	}
	LPC_I2C0->MSTCTL = CTL_MSTSTOP;
	return status;
}


void i2c0_init(void)
{
    LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 5)         // I2C0 clock enable
                               | (1 << 7);        // enable clock to the Switch Matrix
    LPC_SYSCON->PRESETCTRL    |= (1 << 6);         // de-assert I2C0 reset

    LPC_SWM->PINENABLE0       &= ~(1 << 12);       // P0_10 is SCL
    LPC_SWM->PINENABLE0       &= ~(1 << 11);       // P0_11 is SDA

    LPC_I2C->TIMEOUT = 0x0000008F;                 // time-out of 8 x 16 I2C clock counts
  
    LPC_I2C->DIV = (SystemCoreClock / 200000)-1;   // I2C clock = 200000 Hz
    LPC_I2C->MSTTIME = 0;                          // SCL_low = SCL_high = clocks = 10usec (100kb/s)

    //NVIC_EnableIRQ(I2C_IRQn);                      // enable I2C interrupt
    LPC_I2C->CFG |= (1<<0) | (1<<3);               // enable master and time-out function
}
