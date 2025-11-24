#include "hcsr04.h"
#include "timing.h"

static volatile uint32_t last_pulse_us = 0;

void HCSR04_Init(void)
{
    DWT_Delay_Init();

    /* Enable GPIOA */
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    /* PA1 -> TRIG output */
    GPIOA->MODER &= ~(3U << 2);
    GPIOA->MODER |=  (1U << 2);   // 01 = output
    GPIOA->BSRR = (HCSR04_TRIG_PIN << 16);

    /* PA0 -> ECHO input */
    GPIOA->MODER &= ~(3U << 0);   // 00 = input
    GPIOA->PUPDR &= ~(3U << 0);
}

/* Internal: trigger pulse 10 us */
static void hcsr04_trigger_pulse(void)
{
    GPIOA->BSRR = HCSR04_TRIG_PIN;           // set PA1
    delay_us(10);
    GPIOA->BSRR = (HCSR04_TRIG_PIN << 16);   // reset PA1
}

/* Blocking read, returns distance*100 or -1 */
int32_t HCSR04_ReadDistance_cm_x100(uint32_t timeout_ms)
{
    uint32_t timeout_us = timeout_ms * 1000U;

    /* Trigger */
    hcsr04_trigger_pulse();

    /* Wait for rising edge */
    uint32_t t_start = micros();
    while (!(GPIOA->IDR & HCSR04_ECHO_PIN))
    {
        if ((uint32_t)(micros() - t_start) >= timeout_us) return -1;
    }

    /* Record rising time */
    uint32_t t_rise = micros();

    /* Wait for falling edge */
    while (GPIOA->IDR & HCSR04_ECHO_PIN)
    {
        if ((uint32_t)(micros() - t_rise) >= timeout_us) return -1;
    }

    uint32_t t_fall = micros();

    /* Pulse duration in microseconds*/
    uint32_t duration_us = (uint32_t)(t_fall - t_rise);
    last_pulse_us = duration_us;

    /* Convert to cm * 100 (with rounding):
       distance_cm = duration_us / 58.0
       So dist_x100 = round(duration_us * 100 / 58)
       integer: (duration_us*100 + 29) / 58  (29 for rounding)
    */
    uint64_t tmp = (uint64_t)duration_us * 100ULL + 29ULL;
    uint32_t dist_x100 = (uint32_t)(tmp / 58ULL);

    return (int32_t)dist_x100;
}

uint32_t HCSR04_GetLastPulse_us(void)
{
    return last_pulse_us;
}
