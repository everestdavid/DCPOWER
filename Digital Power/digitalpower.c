/*----------------------------------------------------------------------------
 *      RL-ARM - RTX
 *----------------------------------------------------------------------------
 *      Name:    Blinky.c
 *      Purpose: RTX example program
 *----------------------------------------------------------------------------
 *      This code is part of the RealView Run-Time Library.
 *      Copyright (c) 2014 - 2016 KEIL - An ARM Company. All rights reserved.
 *---------------------------------------------------------------------------*/

#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "LPC8xx.h"                     // Device header
#include "Board.h"                  // ::Board Support:LED
#include "stdio.h"
#include "spi0.h"
#include "adc.h"
#include "i2c.h"
#include "string.h"

extern void IRC_Only_SystemInit(void);
volatile enum {false, true} handshake;
//unsigned char rx_quenue_buffer[RX_BUFFER_SIZE];

#define OS_BUF_LENG  40

unsigned char console[OS_BUF_LENG];
unsigned char parameter[OS_BUF_LENG +1];

static unsigned int rx_quenue_counter1 = 0;
//static unsigned int rx_quenue_counter2 = 0;
//static unsigned int console_index = 0;
//static unsigned char UART_RX_DAT = 0;
#define WaitForUART0txRdy  while(((LPC_USART0->STAT) & (1<<2)) == 0) ;

#define COMMAND_NUM   5
const char *command[COMMAND_NUM + 1] = {
	"power on\r\n",    //0
	"power off\r\n",   //1
	"set voltage\r\n", //2
	"get voltage\r\n", //3
	"get current\r\n",  //4
	"invalid command\r\n" //5
};
#define SUCCESS_NUM   2
const char *success[SUCCESS_NUM] = {
	"successfully!\r\n",    //0
	"failed!\r\n",   //1
};

//#define MEAS_NUM 1
const char *meas_info = "do measurement\r\n";
const char *i2c_info = "failed to write I2C memory\r\n";
typedef struct {
	/*
	* the definition of power status
	* 0, off
	* 1, on
	* 2, over current
	* others, reserved
	*/
	unsigned int status;   
	float target_voltage;
	float run_voltage;
	float run_current;
	unsigned int run_main_voltage;
} DigitalPower_Private;
DigitalPower_Private myprivate;

/*
* The declaration of RTOS
*/
//osThreadId t_ledOn;                     /* assigned task id of task: ledOn   */
osThreadId t_uart_rx_id;                    /* assigned task id of task: ledOff  */
osThreadId t_console_explain;
osThreadId t_set_voltage;
osThreadId t_uart_tx;
osThreadId t_do_measure;

typedef struct {
 unsigned char canData[OS_BUF_LENG];
} message_t;
osPoolDef(uartpool, 3, message_t); 
osPoolId uartpool;
osMessageQDef(uartqueue, 3, message_t); 
osMessageQId uartqueue;
osEvent uartevt;

osMessageQId uart_tx_quenue;
osMessageQDef (uart_tx_quenue,9, message_t);
osEvent uart_tx_evt;
osPoolDef(uart_tx_pool, 9, message_t); 
osPoolId uart_tx_pool;


osMessageQId set_voltage;
osMessageQDef (set_voltage,0x5, unsigned char);
osEvent set_voltage_evt;

osMutexId adc_mutex;
osMutexDef (adc_mutex);
osMutexId mainpower_mutex;
osMutexDef (mainpower_mutex);
osMutexId shutdown_mutex;
osMutexDef (shutdown_mutex);
osMutexId dac_mutex;
osMutexDef (dac_mutex);
osMutexId i2c_mutex;
osMutexDef (i2c_mutex);

/**
  \fn          int32_t strcmp_private (void)
  \brief       to check the command promote in UART string
  \param[in]   
  \returns
   - \b  The commnad number
   - \b 
*/
unsigned char strcmp_private(void) 
{
	unsigned char index, pt;
	unsigned char flag = 0x00;
	//unsigned char param = 0;
	//unsigned char  point = 0;
	unsigned char cnt = 0;
	for(index =0; index < COMMAND_NUM; index++) {
		for(pt =0; pt < OS_BUF_LENG; pt++) {
			if(*(command[index] + pt) ==  0x0d) {
				break;
			}
			if(*(command[index] +pt) != console[pt]) 
			{
				flag = 0xaa;
				break;
			}
			else
			{
				flag = 0x55;
			}

		}
		if(flag == 0x55) {
			for(cnt =0; cnt < OS_BUF_LENG; cnt++) {
				parameter[cnt] = 0;
			}
			if(console[pt] == 0x0d) {
				parameter[0] = 0x00;
				break;
			}
			if(console[pt] == 0x20) {
				parameter[0] = 0xaa;
			}
			pt++;
			for(cnt = 1;; pt++, cnt++) {
				if(console[pt] == 0x0d) {
					break;
				}
				else {				
					parameter[cnt] = console[pt];
				}
			}
			break;
		}
	}
	
	return index;
}
/**
  \fn          int32_t uart_rx (void const *argument)
\brief       task: uart received
  \param[in]   
  \returns
   - \b  
   - \b 
*/
void uart_rx(void const *argument) {
  unsigned char temp;
	message_t *message;
	message = (message_t*)osPoolAlloc(uartpool); 	
  for(;;) {
		osSignalWait(0x01, osWaitForever);     /* waiting for signal for ever  */
	  temp= LPC_USART0->RXDAT ;
		message->canData[rx_quenue_counter1]  = temp;
		if (temp == 0x0A) {                       // CR (carriage return) is current character. End of string.
			rx_quenue_counter1++;
			message->canData[rx_quenue_counter1] = 0x00;    // Append a NUL terminator character to rx_buffer to complete the string.
			
			rx_quenue_counter1 = 0;
			osMessagePut(uartqueue, (uint32_t)message, osWaitForever);
		}
		else {                                    // Current character is not CR, keep collecting them.
			rx_quenue_counter1++;                      // Increment array index counter.
			if (rx_quenue_counter1 == RX_BUFFER_SIZE)  // If the string overruns the buffer, stop here before all hell breaks lose.
				rx_quenue_counter1 = 0;
		}	
	}
}
/**
  \fn          int32_t uart_tx (void const *argument)
\brief       task: uart transfer
  \param[in]   
  \returns
   - \b  
   - \b 
*/
void uart_tx(void const *argument) {
	unsigned char cnt, index;
	
  for(;;) {
		uart_tx_evt = osMessageGet(uart_tx_quenue, osWaitForever);
		if(uart_tx_evt.status == osEventMessage) {
			for(index =0; index < OS_BUF_LENG; index++) {
				message_t *message = (message_t*)uart_tx_evt.value.p;
				if(message->canData[index] == 0) {
					osPoolFree(uart_tx_pool, message);
					break;
				}
				for(cnt =0; cnt < 10; cnt++) {
					if(((LPC_USART0->STAT) & (1<<2)) != 0) {
						LPC_USART0->TXDAT = (unsigned char)(message->canData[index] & 0x000000ff);
						break;
					}
					osDelay(1);
				}

			}		
		}
	}
}

/**
  \fn          int32_t set_target_voltage (void const *argument)
\brief       task: to set the target voltage
  \param[in]   
  \returns
   - \b  
   - \b 
*/
void set_target_voltage (void const *argument) {
	unsigned char flag, index, cmd_len;
	unsigned int main_vol;
	unsigned int dac_dat;
	message_t *message_uart_tx;
	message_uart_tx = (message_t*)osPoolAlloc(uart_tx_pool); 
	
	for(;;) {
		set_voltage_evt = osMessageGet(set_voltage, osWaitForever);
		//printf("target voltage = %d\r\n", set_voltage_evt.value.v);
		main_vol = set_voltage_evt.value.v + 1000;
		if(main_vol > 12000) {
			myprivate.run_main_voltage = 8;
		} 
		else {
			if(main_vol > 11000) {
				myprivate.run_main_voltage = 7;
			}
			else {
				if(main_vol > 10000) {
					myprivate.run_main_voltage = 6;
				}
				else {
					if(main_vol > 9000) {
						myprivate.run_main_voltage = 5;
					}
					else {
						if(main_vol > 8000) {
							myprivate.run_main_voltage = 4;
						}
						else {
							if(main_vol > 7000) {
								myprivate.run_main_voltage = 3;
							}
							else {
								if(main_vol > 6000) {
									myprivate.run_main_voltage = 2;
								}
								else {
									if(main_vol > 5000) {
										myprivate.run_main_voltage = 1;
									}
									else {
										myprivate.run_main_voltage = 0;
									}
								}
							}				
						}
					}
				}
			}
		}
		osMutexWait(mainpower_mutex,osWaitForever);
		MainSupply_Adjustment(myprivate.run_main_voltage);
		osMutexRelease(mainpower_mutex);
		
		osMutexWait(dac_mutex,osWaitForever);
		dac_dat = (unsigned int)((float)set_voltage_evt.value.v * 1.024 / 6);
		if(spi0_tx(dac_dat) == 0) {
			flag = 0;
		}
		else
		{
			flag = 1;
		}
		osMutexRelease(dac_mutex);
		
		for(index = 0; index < OS_BUF_LENG; index++) {
			message_uart_tx->canData[index] = 0;
		}
					
		cmd_len = strlen(success[flag]);
		for(index = 0; index < cmd_len; index++) {
			message_uart_tx->canData[index] = *(success[flag] + index);
		}
		osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever); //echo success flag

	}
}
/**
  \fn          void do_measure (void const *argument)
\brief       task: to do measurement on voltage and current
  \param[in]   
  \returns
   - \b  
   - \b 
*/
 void do_measure (void const *argument) 
{
		//unsigned int vol_diff, index, cmd_len;
		unsigned int index, cmd_len;
	  unsigned int run_vol, run_cur;
		message_t *message_uart_tx;
		message_uart_tx = (message_t*)osPoolAlloc(uart_tx_pool); 
	 //for(;;) {
		//	osSignalWait(0x01, osWaitForever); 
		  for(;;) {
				osMutexWait(adc_mutex,osWaitForever);
				run_vol = adc_vol_convert();
				run_cur = adc_current_convert();

				myprivate.run_voltage = run_vol ;
				myprivate.run_current = run_cur ;
				
				//myprivate.run_voltage = (float)run_vol * 2.5 / (float)4095;
				//myprivate.run_current = (float)run_cur * 2.5 / (float)4095;
				osMutexRelease(adc_mutex);
				#if 0
				if(myprivate.run_current > CURRENT_THRESHOLD ) {
					if(myprivate.run_main_voltage > 0) {
						myprivate.status |= OVER_CUR;
						myprivate.run_main_voltage--;
						
						osMutexWait(mainpower_mutex,osWaitForever);
						MainSupply_Adjustment(myprivate.run_main_voltage);
						osMutexRelease(mainpower_mutex);
					} 
				}
				else {
					myprivate.status |= NORMAL;
					/*
					if(myprivate.run_voltage < myprivate.target_voltage) {
						vol_diff = myprivate.target_voltage - myprivate.run_voltage;
						osMutexWait(dac_mutex,osWaitForever);
						for(index = 0; index < vol_diff; index++) {
							spi0_tx(myprivate.run_voltage + index);
						}
						osMutexRelease(dac_mutex);
					}
					else {
						vol_diff = myprivate.run_voltage - myprivate.target_voltage;
						osMutexWait(dac_mutex,osWaitForever);
						for(index = 0; index < vol_diff; index++) {
							spi0_tx(myprivate.run_voltage - index);
						}
						osMutexRelease(dac_mutex);						
					}
					*/
				}
				#endif
				for(index = 0; index < OS_BUF_LENG; index++) {
					message_uart_tx->canData[index] = 0;
				}
					
				cmd_len = strlen(meas_info);
				for(index = 0; index < cmd_len; index++) {
					message_uart_tx->canData[index] = *(meas_info + index);
				}
				//osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever); //echo success flag
				
				osDelay(50);
			}
	 //}
}
osThreadDef(do_measure, osPriorityNormal, 1, 0);

/**
  \fn          int32_t console_explain (void const *argument)
\brief       task: to explain uart console command
  \param[in]   
  \returns
   - \b  
   - \b 
*/
void console_explain (void const *argument) {
	unsigned int index = 0;
	unsigned int cmd_len = 0;
	unsigned int cmd_flag = 0;
	unsigned char cnt = 0;
	static unsigned char cmd_index = 0;
	unsigned char index_tmp;
	unsigned int vol, cur, status;
	union {
		unsigned char c[4];
		float f;
	} ctof;

	message_t *message_uart_tx;
	message_uart_tx = (message_t*)osPoolAlloc(uart_tx_pool); 	
	for(;;) {
		uartevt = osMessageGet(uartqueue, osWaitForever);
		cmd_flag = 0x00;
		if(uartevt.status == osEventMessage) {
			cmd_flag = 0x00;
			for(index = 0; index < OS_BUF_LENG; index++){
				message_t *message = (message_t*)uartevt.value.p;
				console[index] = (uint32_t)message->canData[index];
				if(index > 0) {
					if(((uint32_t)message->canData[index] == 0x0a)) {
						cmd_flag = 0xaa;
						osPoolFree(uartpool, message);
						break;
					}
				}
			}
			for(index = 0; index < OS_BUF_LENG; index++) {
				message_uart_tx->canData[index] = 0;
			}
			if(cmd_flag == 0xaa) {
				cmd_index = strcmp_private();
				cmd_len = strlen(command[cmd_index]);
				if(cmd_index < COMMAND_NUM) {
					for(index = 0; index < cmd_len; index++) {
						message_uart_tx->canData[index] = *(command[cmd_index] + index);
					}	
				}
				else
				{
					cmd_len = strlen(command[5]);
					for(index = 0; index < cmd_len; index++) {
						message_uart_tx->canData[index] = *(command[5] + index);
					}						
				}
				switch(cmd_index) {
					case 0:  // power on
					  myprivate.status = POWERON;
						DigPower_Shutdown(POWERON);		
						MainSupply_Adj_Enable(ENABLE);
						//osSignalSet(t_do_measure, 0x01); 
						t_do_measure = osThreadCreate(osThread(do_measure), NULL);
					
						index_tmp = index;
						cmd_len = strlen(success[0]);
						for(index = 0; index < cmd_len; index++) {
							message_uart_tx->canData[index + index_tmp] = *(success[0]+index);
						}		
					osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever);						
					break;
					case 1:  // power off
					  myprivate.status = SHUTDOWN;
						MainSupply_Adj_Enable(DISABLE);		
						DigPower_Shutdown(SHUTDOWN);	
						osThreadTerminate(t_do_measure);
						ctof.f = myprivate.target_voltage;
						osMutexWait(i2c_mutex,osWaitForever);
						status = i2c0_write(Self_Slave_address, 0, ctof.c[0]);
						if(status == 0) {
							status = i2c0_write(Self_Slave_address, 1, ctof.c[1]);
							if(status == 0) {
								status = i2c0_write(Self_Slave_address, 2, ctof.c[2]);
								if(status == 0) {
									status = i2c0_write(Self_Slave_address, 3, ctof.c[3]);
								}
							}
						}
						osMutexRelease(i2c_mutex);
						if(status == 0) {
							/*
								for(index = 0; index < OS_BUF_LENG; index++) {
									message_uart_tx->canData[index] = 0;
								}			
							*/
							index_tmp = index;
							cmd_len = strlen(success[0]);
							for(index = 0; index < cmd_len; index++) {
								message_uart_tx->canData[index + index_tmp] = *(success[0]+index);
								//if(message_uart_tx->canData[index] == 0x0A) break;
							}	
						}
						else
						{
							index_tmp = index;
							cmd_len = strlen(i2c_info);
							for(index = 0; index < cmd_len; index++) {
								message_uart_tx->canData[index + index_tmp] = *(i2c_info+index);
								//if(message_uart_tx->canData[index] == 0x0A) break;
							}								
						}
						osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever);
					break;
					case 2:
						if(parameter[0] == 0xaa) {
							//message_uart_tx->canData[index] = 0;
							osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever);
							
							vol = 0;
							for(cnt=0; cnt < 5;cnt++) {
								if(parameter[cnt+1] == 0) {
									break;
								}	
								if(parameter[cnt+1] == 0x20) {
									continue;
								}
								if(parameter[cnt+1] >= 0x30 && parameter[cnt+1] <= 0x39) {
									vol = vol * 10;
									vol += (parameter[cnt+1]-0x30);
								}								
							}	
							//osDelay(100);
							myprivate.target_voltage = (float)vol/1000;
							osMessagePut(set_voltage, vol, osWaitForever);			
						}
					break;
					case 3:
						vol = myprivate.run_voltage * 1000;
						message_uart_tx->canData[index++] = vol /1000 + 0x30;
						vol = vol % 1000;
						message_uart_tx->canData[index++] = vol /100 + 0x30;
						vol = vol % 100;
						message_uart_tx->canData[index++] = vol /10 + 0x30;	
						vol = vol % 10;
						message_uart_tx->canData[index++] = vol  + 0x30;
						message_uart_tx->canData[index++] = '\r';
					  message_uart_tx->canData[index++] = '\n';
					  message_uart_tx->canData[index++] = 0;
						osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever);
					break;
					case 4:
						cur = myprivate.run_current * 1000;
						message_uart_tx->canData[index++] = cur /1000 + 0x30;
						cur = cur % 1000;
						message_uart_tx->canData[index++] = cur /100 + 0x30;
						cur = cur % 100;
						message_uart_tx->canData[index++] = cur /10 + 0x30;	
						cur = cur % 10;
						message_uart_tx->canData[index++] = cur  + 0x30;
						message_uart_tx->canData[index++] = '\r';
					  message_uart_tx->canData[index++] = '\n';
					  message_uart_tx->canData[index++] = 0;					
						osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever);
					break;
					default: 
						osMessagePut(uart_tx_quenue, (uint32_t)message_uart_tx, osWaitForever);
					break;
				}
				osDelay (10); 
			}
		}
	}
}

osThreadDef(uart_rx, osPriorityHigh, 1, 0);
osThreadDef(console_explain, osPriorityAboveNormal, 1, 0);
osThreadDef(set_target_voltage, osPriorityNormal, 1, 0);
osThreadDef(uart_tx, osPriorityNormal, 1, 0);

/*****************************************************************************
** Function name:		UART0_IRQHandler
**
** Description:		    UART0 interrupt service routine.
**                      This ISR reads one received char from the UART0 RXDAT register,
**                      appends it to the rx_buffer array, and echos it back via the
**                      UART0 transmitter. If the char. is 0xD (carriage return),
**                      a new line char (0xA) is appended to the array and echoed,
**                      then a NUL char (0x0) is appended to the array to terminate the string
**                      for future use.
**
** Parameters:			None
** Returned value:		void
**
*****************************************************************************/
void UART0_IRQHandler(void) {
	unsigned char temp;
	temp = LPC_USART0->RXDAT ;
	osSignalSet(t_uart_rx_id, 0x01); 
}
//osThreadDef(ledOn,  osPriorityNormal, 1, 0);


/*----------------------------------------------------------------------------
  Main: Initialize and start RTX Kernel
 *---------------------------------------------------------------------------*/
int main (void) {

  //SystemCoreClockUpdate();
	//osKernelInitialize ();
	//IRC_Only_SystemInit();
	union {
		unsigned char c[4];
		float f;
	} ctof;
	unsigned char dat, status;
	
	
	rx_quenue_counter1 = 0;
  //rx_quenue_counter2 = 0;
	i2c0_init();
	adc_ini();
	Board_Initialize();                                   /* Initialize LEDs     */
	spi0_ini();

	status = 0;
	if(i2c0_read(Self_Slave_address, 0x0000, &dat)== 0) {
		ctof.c[0] = dat;
		if(i2c0_read(Self_Slave_address, 0x0001, &dat)== 0) {
			ctof.c[1] = dat;
			if(i2c0_read(Self_Slave_address, 0x0002, &dat)== 0) {
				ctof.c[2] = dat;
				if(i2c0_read(Self_Slave_address, 0x0003, &dat)== 0) {
					ctof.c[3] = dat;
				}
				else {
					status = 1;
				}
			}
      else {
				status = 1;
      }				
		}
    else {
			status = 1;
		}
	}
	else {
		status = 1;
	}
	if(status == 0) {
		myprivate.target_voltage = ctof.f;
		printf("success load i2c memory!\r\n");
	} 
	else {
		myprivate.target_voltage = 3300;
		printf("fail to load i2c memory!\r\n");
	}
	handshake = false;

	uartqueue = osMessageCreate(osMessageQ(uartqueue),NULL);
	uartpool = osPoolCreate(osPool(uartpool));
	
	set_voltage = osMessageCreate(osMessageQ(set_voltage),NULL);
	uart_tx_quenue = osMessageCreate(osMessageQ(uart_tx_quenue),NULL);
	uart_tx_pool = osPoolCreate(osPool(uart_tx_pool));
	
	adc_mutex = osMutexCreate(osMutex(adc_mutex));
	mainpower_mutex = osMutexCreate(osMutex(mainpower_mutex));
	dac_mutex = osMutexCreate(osMutex(dac_mutex));
	i2c_mutex = osMutexCreate(osMutex(i2c_mutex));
	
  t_uart_rx_id = osThreadCreate(osThread(uart_rx), NULL);  /* start task 'ledOff' */
	t_console_explain = osThreadCreate(osThread(console_explain), NULL); 
	t_set_voltage  = osThreadCreate(osThread(set_target_voltage), NULL); 
	t_uart_tx = osThreadCreate(osThread(uart_tx), NULL);  
	//t_do_measure = osThreadCreate(osThread(do_measure), NULL);  

	//osKernelStart(); 
  osDelay(osWaitForever);
}
