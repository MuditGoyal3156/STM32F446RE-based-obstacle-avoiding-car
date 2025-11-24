#ifndef TIMING_H_
#define TIMING_H_

#include "stm32f4xx.h"
#include <stdint.h>

void DWT_Delay_Init(void);
uint32_t micros(void);
void delay_us(uint32_t us);

#endif /* TIMING_H_ */
