#ifndef I2S_MASTER_H
#define I2S_MASTER_H


#define MCLK_PIN	18
#define MCLK_TIMER	5
#define MCLK_TIMER_SEG AM_HAL_CTIMER_TIMERB

#define BCLK_PIN_INV	13
#define BCLK_TIMER_INV	0
#define BCLK_TIMER_SEG_INV AM_HAL_CTIMER_TIMERB
#define BCLK_TIMER_CLOCK_INV   0x1B//TMRB0CLK is B5

#define REF_TIMER	2
#define REF_TIMER_SEG AM_HAL_CTIMER_TIMERA
#define REF_TIMER_CLOCK   0x1B//TMRA2CLK is B5

#define LRCLK_PIN	25
#define LRCLK_TIMER	0
#define LRCLK_TIMER_SEG AM_HAL_CTIMER_TIMERA
#define LRCLK_TIMER_INT AM_HAL_CTIMER_INT_TIMERA0
#define LRCLK_TIMER_CLOCK   0x17//TMRA0CLK is A2

#define SDATA_PIN   27
#define SDATA_TIMER	1
#define SDATA_TIMER_CLOCK   0x17//TMRA1CLK is A2

/*Table 929: GLOBEN Register Bits*/
//B5 Bit-11
//B0 Bit-1
//A2 Bit-4
//A0 Bit-0
//A1 Bit-2
#define I2S_GLOBEN_MASK ((1<<11)|(1<<1)|(1<<4)|(1<<0)|(1<<2))

#define BUF_SIZE		128
//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
extern int16_t i16I2SBuf[2][BUF_SIZE];
extern uint32_t u32I2SPingpong;

void I2S_init(void);
void global_disable(uint32_t mask);
void global_enable(uint32_t mask);

#endif // I2S_MASTER_H


