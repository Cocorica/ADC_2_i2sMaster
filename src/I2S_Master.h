#ifndef I2S_MASTER_H
#define I2S_MASTER_H


#define BUF_SIZE		128

//*****************************************************************************
//
// Global variables.
//
//*****************************************************************************
extern int16_t i16I2SBuf[2][BUF_SIZE];
extern uint32_t u32I2SPingpong;

void I2S_init(void);
#endif // I2S_MASTER_H


