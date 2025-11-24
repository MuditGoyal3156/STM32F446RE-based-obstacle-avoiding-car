#include "timing.h"

static uint32_t cycles_per_us = 0;

void DWT_Delay_Init(void)
{
    SystemCoreClockUpdate();
    cycles_per_us = SystemCoreClock / 1000000U;
    if (cycles_per_us == 0) cycles_per_us = 1;

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;   // enable trace
    DWT->CYCCNT = 0;                                 // reset cycle counter
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;             // enable cycle counter
}


uint32_t micros(void)
{
    return (uint32_t)(DWT->CYCCNT / cycles_per_us);
}

void delay_us(uint32_t us)
{
    uint32_t start = DWT->CYCCNT;
    uint32_t ticks = us * cycles_per_us;
    while ((uint32_t)(DWT->CYCCNT - start) < ticks) { __NOP(); }
}
