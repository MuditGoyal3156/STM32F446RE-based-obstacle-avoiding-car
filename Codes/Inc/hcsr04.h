#ifndef HCSR04_H_
#define HCSR04_H_

#include "stm32f4xx.h"
#include <stdint.h>

// Pin definitions
#define HCSR04_TRIG_PIN   (1U << 1)   // PA1
#define HCSR04_ECHO_PIN   (1U << 0)   // PA0
void HCSR04_Init(void);
int32_t HCSR04_ReadDistance_cm_x100(uint32_t timeout_ms);

// Return last measured pulse width in microseconds (0 if none)
uint32_t HCSR04_GetLastPulse_us(void);

#endif /* HCSR04_H_ */
