#include "buzz.h"
#include "stm32f4xx.h"

#define BUZZER_FREQUENCY 2700 // 2.7 kHz

static void buzzer_set_pitch(uint32_t frequency_hz)
{
    if (frequency_hz == 0)
    {
        TIM4->CCR1 = 0;
        return;
    }
    uint32_t arr_value = (1000000U / frequency_hz) - 1;
    TIM4->ARR = arr_value;
    TIM4->CCR1 = arr_value / 2;
}




void buzzer_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    GPIOB->MODER &= ~GPIO_MODER_MODER6_Msk;
    GPIOB->MODER |= GPIO_MODER_MODER6_1;
    GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL6_Msk;
    GPIOB->AFR[0] |= (2U << GPIO_AFRL_AFSEL6_Pos);

    TIM4->PSC = 15; // 16MHz / 16 = 1MHz counter clock
    TIM4->CCMR1 &= ~TIM_CCMR1_OC1M_Msk;
    TIM4->CCMR1 |= TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1;
    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM4->CR1 |= TIM_CR1_ARPE;
    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CR1 |= TIM_CR1_CEN;
}

void buzzer_on(void)
{
    buzzer_set_pitch(BUZZER_FREQUENCY);
}

void buzzer_off(void)
{
    buzzer_set_pitch(0);
}
