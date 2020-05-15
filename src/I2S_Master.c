#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "am_mcu_apollo.h"
#include "am_bsp.h"
#include "am_util.h"
#include "I2S_Master.h"
//#include "vogue.h"
//#include "silent.h"


//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
int16_t i16I2SBuf[2][BUF_SIZE] = {{0},{0}};
uint32_t u32I2SPingpong = 0;

static void timer_handler(void);


/*LSB first to MSB first*/
uint16_t reverse_bit16(uint16_t x)
{
	x = ((x & 0x5555) << 1) | ((x & 0xAAAA) >> 1);
	x = ((x & 0x3333) << 2) | ((x & 0xCCCC) >> 2);
	x = ((x & 0x0F0F) << 4) | ((x & 0xF0F0) >> 4);
	return (x << 8) | (x >> 8);
}


// Timer Interrupt Service Routine (ISR)
void am_ctimer_isr(void)
{
    uint32_t ui32Status;
	//am_hal_gpio_output_toggle(6);

    ui32Status = am_hal_ctimer_int_status_get(false);
    am_hal_ctimer_int_clear(ui32Status);

    am_hal_ctimer_int_service(ui32Status);
}

void pwm_out(void)
{


	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(MCLK_TIMER,
	                            MCLK_TIMER_SEG,
	                            MCLK_PIN,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);

	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(MCLK_TIMER,               // ui32TimerNumber
	                            MCLK_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             AM_HAL_CTIMER_HFRC_12MHZ|
	                             AM_HAL_CTIMER_INT_ENABLE) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(MCLK_TIMER,
	                         MCLK_TIMER_SEG, 2, 1); //MCKL 4MHz
	am_hal_ctimer_aux_period_set(MCLK_TIMER,
	                         MCLK_TIMER_SEG, 2, 1);
//////////////////////////////////////////////////////////////////////////////////

//No need BCLK_TIMER output
#if 1
	//
	// Configure the output pin.
	//
	am_hal_ctimer_output_config(REF_TIMER,
	                            REF_TIMER_SEG,
	                            46,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);
#endif
	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(REF_TIMER,               // ui32TimerNumber
	                            REF_TIMER_SEG,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, REF_TIMER_CLOCK) | 
						AM_HAL_CTIMER_INT_ENABLE));
						//AM_HAL_CTIMER_INT_ENABLE | AM_HAL_CTIMER_PIN_INVERT) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(REF_TIMER,
	                         REF_TIMER_SEG, 7, 4);
	am_hal_ctimer_aux_period_set(REF_TIMER,
	                         REF_TIMER_SEG, 7, 4);

	//////////////////////////////////////////////////////////////////////////////////

	//
	// Configure the output pin.
	//	
	am_hal_ctimer_output_config(BCLK_TIMER_INV,
	                            BCLK_TIMER_SEG_INV,
	                            BCLK_PIN_INV,
	                            AM_HAL_CTIMER_OUTPUT_NORMAL,
	                            AM_HAL_GPIO_PIN_DRIVESTRENGTH_2MA);

	//
	// Configure a timer to drive the LED.
	//
	am_hal_ctimer_config_single(BCLK_TIMER_INV,               // ui32TimerNumber
	                            BCLK_TIMER_SEG_INV,           // ui32TimerSegment
	                            (AM_HAL_CTIMER_FN_PWM_REPEAT    |   // ui32ConfigVal
	                             _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, BCLK_TIMER_CLOCK_INV) | 
						//AM_HAL_CTIMER_INT_ENABLE));
						AM_HAL_CTIMER_INT_ENABLE | AM_HAL_CTIMER_PIN_INVERT) );

	//
	// Set up initial timer periods.
	//
	am_hal_ctimer_period_set(BCLK_TIMER_INV,
	                         BCLK_TIMER_SEG_INV, 7, 4);//BCLK INV
	am_hal_ctimer_aux_period_set(BCLK_TIMER_INV,
	                         BCLK_TIMER_SEG_INV, 7, 4);//BCLK INV


	//////////////////////////////////////////////////////////////////////////////////


	//
	// Start the timer.
	//
	am_hal_ctimer_start(MCLK_TIMER, MCLK_TIMER_SEG);
	am_hal_ctimer_start(REF_TIMER, REF_TIMER_SEG);

	am_hal_ctimer_start(BCLK_TIMER_INV, BCLK_TIMER_SEG_INV);


}


void
initialize_pattern128_counter(uint32_t ui32TimerNumber,
                           uint64_t ui64Pattern0,
                           uint64_t ui64Pattern1,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, AM_HAL_CTIMER_BOTH);

    am_hal_ctimer_config_single(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    |
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)(ui64Pattern0 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 0, 
                            (uint32_t)((ui64Pattern0 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, 1, 
                            (uint32_t)((ui64Pattern0 >> 48) & 0xFFFF));

    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)(ui64Pattern1 & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 0, 
                            (uint32_t)((ui64Pattern1 >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, AM_HAL_CTIMER_TIMERB, 1, 
                            (uint32_t)((ui64Pattern1 >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, AM_HAL_CTIMER_BOTH,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, AM_HAL_CTIMER_TIMERA, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);
    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, AM_HAL_CTIMER_BOTH);
}

void
initialize_pattern64_counter(uint32_t ui32TimerNumber,
                           uint32_t ui32TimerSegment,
                           uint64_t ui64Pattern,
                           uint32_t ui32PatternLen,
                           uint32_t ui32Trigger,
                           uint32_t ui32OutputPin,
                           uint32_t ui32PatternClock)
{
    //
    // Set up timer.
    //
    am_hal_ctimer_clear(ui32TimerNumber, ui32TimerSegment);

    am_hal_ctimer_config_single(ui32TimerNumber, ui32TimerSegment,
                              (AM_HAL_CTIMER_FN_PTN_REPEAT    |AM_HAL_CTIMER_INT_ENABLE |
                               ui32PatternClock) );

    //
    // Set the pattern in the CMPR registers.
    //
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)(ui64Pattern & 0xFFFF));
    am_hal_ctimer_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 16) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 0, 
                            (uint32_t)((ui64Pattern >> 32) & 0xFFFF));
    am_hal_ctimer_aux_compare_set(ui32TimerNumber, ui32TimerSegment, 1, 
                            (uint32_t)((ui64Pattern >> 48) & 0xFFFF));
    //
    // Set the timer trigger and pattern length.
    //
    am_hal_ctimer_config_trigger(ui32TimerNumber, ui32TimerSegment,
                               ( (ui32PatternLen << CTIMER_AUX0_TMRA0LMT_Pos) |
                                 ( ui32Trigger << CTIMER_AUX0_TMRA0TRIG_Pos) ) );

    //
    // Configure timer output pin.
    //
    am_hal_ctimer_output_config(ui32TimerNumber, ui32TimerSegment, ui32OutputPin, 
                              AM_HAL_CTIMER_OUTPUT_NORMAL, 
                              AM_HAL_GPIO_PIN_DRIVESTRENGTH_12MA);

    //
    // Start the timer.
    //
    am_hal_ctimer_start(ui32TimerNumber, ui32TimerSegment);
}

void global_disable(uint32_t mask)
{
    CTIMER->GLOBEN &= ~(mask);
}

void global_enable(uint32_t mask)
{
    CTIMER->GLOBEN |= mask;
}

void I2S_init(void)
{
	//
	// Disable all the counters.
	//
	//global_disable();


	initialize_pattern128_counter(SDATA_TIMER, 0x0000000000000000, 0x0000000000000000, 
	               127, CTIMER_AUX0_TMRB0TRIG_DIS, SDATA_PIN,
	               _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, SDATA_TIMER_CLOCK));//Timer1 Clock source is CTIMERA2 OUT
	initialize_pattern64_counter(LRCLK_TIMER, LRCLK_TIMER_SEG, 0xFFFF0000FFFF0000, 
	               63, CTIMER_AUX0_TMRB0TRIG_DIS, LRCLK_PIN,
	               _VAL2FLD(CTIMER_CTRL0_TMRA0CLK, LRCLK_TIMER_CLOCK));//Timer0 Clock source is CTIMERA2 OUT

	pwm_out();	

	//
	// Clear the timer Interrupt
	//
	am_hal_ctimer_int_clear(LRCLK_TIMER_INT);

	//
	// Enable the timer Interrupt.
	//
	am_hal_ctimer_int_register(LRCLK_TIMER_INT,
	               timer_handler);

	am_hal_ctimer_int_enable(LRCLK_TIMER_INT);


	//global_enable();//I2S starts

	//
	// Enable the timer interrupt in the NVIC.
	//
	NVIC_EnableIRQ(CTIMER_IRQn);
}

static void timer_handler(void)
{
	static uint32_t g_bitflag = 0;
	uint16_t ui16Pattern0 = 0;
	uint16_t ui16Pattern1 = 0;
	static uint32_t index = 0;
	static int16_t *pcm_idx = i16I2SBuf[0];

	am_hal_gpio_state_write(6, AM_HAL_GPIO_OUTPUT_CLEAR);

	g_bitflag +=1;

	ui16Pattern0 = reverse_bit16((uint16_t)*(pcm_idx+index));
	ui16Pattern1 = reverse_bit16((uint16_t)*(pcm_idx+index+1));
	//ui16Pattern0 = 0;
	//ui16Pattern1 = 0;

	
	if(g_bitflag%2)
	{
	    am_hal_ctimer_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERA, 0, 
	                            (uint32_t)(ui16Pattern0));
	    am_hal_ctimer_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERA, 1, 
	                            (uint32_t)(ui16Pattern0));
	    am_hal_ctimer_aux_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERA, 0, 
	                            (uint32_t)(ui16Pattern1));
	    am_hal_ctimer_aux_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERA, 1, 
	                            (uint32_t)(ui16Pattern1));
	}
	else
	{
	    am_hal_ctimer_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERB, 0, 
	                            (uint32_t)(ui16Pattern0));
	    am_hal_ctimer_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERB, 1, 
	                            (uint32_t)(ui16Pattern0));
	    am_hal_ctimer_aux_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERB, 0, 
	                            (uint32_t)(ui16Pattern1));
	    am_hal_ctimer_aux_compare_set(SDATA_TIMER, AM_HAL_CTIMER_TIMERB, 1, 
	                            (uint32_t)(ui16Pattern1));
	}
	
	index += 2;
	if(index >= BUF_SIZE)
	{
		index = 0;
		pcm_idx = i16I2SBuf[(++u32I2SPingpong)%2];
		am_hal_gpio_output_toggle(6);
	}
		
	am_hal_gpio_state_write(6, AM_HAL_GPIO_OUTPUT_SET);
	return;
}


#if 0
void main(void)
{
	am_hal_interrupt_master_disable();

	I2S_init();
	
	am_hal_interrupt_master_enable();


	
	//
	// Loop forever while sleeping.
	//
	while (1)
	{

			u32I2Spg = u32I2SPingpong;


			memcpy( i16I2SBuf[(u32I2Spg+1)%2],(int16_t*)(vogue+index),u32FrameSize*2);
			index += u32FrameSize;
			


			if(u32I2Spg != u32I2SPingpong)
			{
				while(1);
			}
	}
}
#endif
