
#ifndef _BOARD_H_
#define _BOARD_H_

#include "stm32f1xx.h"


void initMCU(void);

extern volatile uint16_t conv;    // ADC temperature sensor convertion value

#endif  // _BOARD_H_
