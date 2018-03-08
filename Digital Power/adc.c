#include "cmsis_os.h"                   // ARM::CMSIS:RTOS:Keil RTX
#include "LPC8xx.h"                     // Device header
#include "lpc8xx_adc.h"
#include "lpc8xx_swm.h"
#include "lpc8xx_syscon.h"
#include "adc.h"

#define SOFTWARE_TRIGGER 1
volatile enum {false, true} seqa_handshake, seqb_handshake;
uint32_t current_seqa_ctrl, current_seqb_ctrl;
/**
  \fn          void adc_ini (void)
\brief         initialize adc into software trigger mode,
							 only ADC_9 and ADC_10 used for analog input
  \param[in]   
  \returns
   - \b  
   - \b 
*/
void adc_ini(void) 
{
  uint32_t current_clkdiv; 

  // Step 1. Power up and reset the ADC, and enable clocks to peripherals (see lpc8xx_syscon.h)
  LPC_SYSCON->PDRUNCFG &= ~(ADC_PD);
  LPC_SYSCON->PRESETCTRL &= (ADC_RST_N);
  LPC_SYSCON->PRESETCTRL |= ~(ADC_RST_N);
  LPC_SYSCON->SYSAHBCLKCTRL |= (ADC | GPIO | SWM);

 
  // Step 2. Perform a self-calibration
  // Choose a CLKDIV divider value that yields about 500 KHz, we will use the SystemCoreClock variable for the calculation.
  //SystemCoreClockUpdate();
  current_clkdiv = (SystemCoreClock / 500000) - 1;

  // Start the self-calibration
  // Calibration mode = true, low power mode = false, CLKDIV = appropriate for 500,000 Hz
  LPC_ADC->CTRL = ( (1<<ADC_CALMODE) | (0<<ADC_LPWRMODE) | (current_clkdiv<<ADC_CLKDIV) );

  // Poll the calibration mode bit until it is cleared
  while (LPC_ADC->CTRL & (1<<ADC_CALMODE));

  // Step 3. Configure the SWM (see utilities_lib and lpc8xx_swm.h)
  // ADC9 - ADC10 pins to be ADC analog functions
  LPC_SWM->PINENABLE0 &= ~(ADC_9|ADC_10); // Fixed pin analog functions enabled
  LPC_SWM->PINENABLE0 |= (ADC_0|ADC_1|ADC_2|ADC_3|ADC_4|
													ADC_5|ADC_6|ADC_7|ADC_8|ADC_11);   // Movable digital functions enabled

  // Assign the ADC pin trigger functions to pins P0.10 (A seq.) and P0.11 (B seq.)
  //ConfigSWM(ADC_PINTRIG0, P0_10);  // ADC_PINTRIG0 will be for the A sequence
  //ConfigSWM(ADC_PINTRIG1, P0_11);  // ADC_PINTRIG1 will be for the B sequence

  // Step 4. Configure the ADC for the appropriate analog supply voltage using the TRM register
  // For a sampling rate higher than 1 Msamples/s, VDDA must be higher than 2.7 V (on the Max board it is 3.3 V)
  LPC_ADC->TRM &= ~(1<<ADC_VRANGE); // '0' for high voltage

  // Step 4. Choose a CLKDIV divider value
  // A fully accurate conversion requires 25 ADC clocks.
  // A 30 MHz system clock with CLKDIV = 0 results in 1.2 MSPS.
  // We can use the SystemCoreClock variable to calculate for a desired sample rate in Hz.
  #define desired_sample_rate 1200000
  current_clkdiv = (SystemCoreClock / (25 * desired_sample_rate)) - 1;

  // Calibration mode = false, low power mode = false, CLKDIV = appropriate for desired sample rate.
  LPC_ADC->CTRL = ( (0<<ADC_CALMODE) | (0<<ADC_LPWRMODE) | (current_clkdiv<<ADC_CLKDIV) );

  // Step 5. Assign some ADC channels to each sequence
  // Let sequence A = channels 9
  // Let sequence B = channels 10
  current_seqa_ctrl = ((1<<9))<<ADC_CHANNELS;
  current_seqb_ctrl = ((1<<10))<<ADC_CHANNELS;

  // Step 6. Select a trigger source for each of the sequences
  // Let sequence A trigger = ADC pin trigger 0. Connected to an external pin through the switch matrix above.
  // Let sequence B trigger = ADC pin trigger 1. Connected to an external pin through the switch matrix above.

  current_seqa_ctrl |= LOGIC_HIGH<<ADC_TRIGGER;    // Use for software-only triggering
  current_seqb_ctrl |= LOGIC_HIGH<<ADC_TRIGGER;    // Use for software-only triggering

  // Step 7. Select positive (1) or negative (0) edge for the hardware trigger
  // Let sequence A be negative edge triggered
  // Let sequence B be negative edge triggered
  current_seqa_ctrl |= 1<<ADC_TRIGPOL;
  current_seqb_ctrl |= 1<<ADC_TRIGPOL;


  // Write the sequence control word with enable bit set for both sequences
  current_seqa_ctrl |= 1U<<ADC_SEQ_ENA;
  LPC_ADC->SEQA_CTRL = current_seqa_ctrl;

  current_seqb_ctrl |= 1U<<ADC_SEQ_ENA;
  LPC_ADC->SEQB_CTRL = current_seqb_ctrl;

}
/**
  \fn          unsigned int adc_vol_convert (void)
	\brief       to do voltage measurement on ADC_9
  \param[in]   
  \returns
   - \b        the result on ADC_9
   - \b 
*/
unsigned int adc_vol_convert(void) 
{
			uint32_t val9, cnt;
			cnt = 0;
      // Launch sequence A with software
      current_seqa_ctrl |= 1<<ADC_START;
      LPC_ADC->SEQA_CTRL = current_seqa_ctrl;
			while((LPC_ADC->DAT[9] & 0x80000000) == 0) {
				osDelay(10);
				cnt++;
				if(cnt > 10) break;
			}
			val9 = LPC_ADC->DAT[9];
			val9 = (val9 >> 4) & 0x0fff;
	
			return val9;
}
/**
  \fn          unsigned int adc_current_convert (void)
	\brief       to do current measurement on ADC_10
  \param[in]   
  \returns
   - \b        the result on ADC_10
   - \b 
*/
unsigned int  adc_current_convert(void) 
{
			uint32_t  val10, cnt;
			cnt = 0;
      // Launch sequence B with software
      current_seqb_ctrl |= 1<<ADC_START;
      LPC_ADC->SEQB_CTRL = current_seqb_ctrl;
			while((LPC_ADC->DAT[10] & 0x80000000) == 0) {
				osDelay(10);
				cnt++;
				if(cnt > 10) break;
			}
			val10 = LPC_ADC->DAT[10];
			val10 = (val10 >> 4) & 0x0fff;
			return val10;
}
