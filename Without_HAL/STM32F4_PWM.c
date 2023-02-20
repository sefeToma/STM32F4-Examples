#include "stm32f4xx.h"
#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  //#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

/*
    1 enable GPIOx and TIMx clock in RCC
    2 set the appropriate pin's GPIOx_MODER to AF, and GPIOx_AFR to appropriate value; set also GPIOx_OSPEEDR if there's a need for higher slew rates
    3 set TIMx_CCMRx to set given channel to Output Compare, and one of the PWM modes
    4 set TIMx_CCRx to set duty cycle
    5 enable given channel in TIMx_CCER
      in Advanced timers, set TIMx_BDTR.MOE
    6 set TIMx_PSC/TIMx_ARR to set period
    7 set TIMx_CR1.CEN to start timer's counter
 */

int main(void) {
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

    //AFR4
    //GPIOD->AFR[0]  |= (1 << 17);
    GPIOD->AFR[1]  |= (1 << 17);

    //AFR5
    //GPIOD->AFR[0]  |= (1 << 21);
    GPIOD->AFR[1]  |= (1 << 21);

    //AFR6
    //GPIOD->AFR[0]  |= (1 << 25);
    GPIOD->AFR[1]  |= (1 << 25);

    //AFR7
    //GPIOD->AFR[0]  |= (1 << 29);
    GPIOD->AFR[1]  |= (1 << 29);

    GPIOD->MODER |= (GPIO_MODER_MODER12_1 | GPIO_MODER_MODER13_1 | GPIO_MODER_MODER14_1 | GPIO_MODER_MODER15_1);

    TIM4->PSC = 15999;
    TIM4->ARR = 4999;

    TIM4->CCMR1 |= (TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2);
    TIM4->CCMR2 |= (TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2);

    TIM4->CCMR1 |= TIM_CCMR1_OC1PE;
    TIM4->CR1 |= TIM_CR1_ARPE;

    TIM4->CR1 |= (1 << 4); //downcounter

    TIM4->CCER |= TIM_CCER_CC1E;
    TIM4->CCER |= TIM_CCER_CC2E;
    TIM4->CCER |= TIM_CCER_CC3E;
    TIM4->CCER |= TIM_CCER_CC4E;

    TIM4->CCR1 = 3999; // zelena, gori 4sec
    TIM4->CCR2 = 2999; // oranzna, gori 3sec
    TIM4->CCR3 = 1999; // rdeca, gori 2sec
    TIM4->CCR4 = 999; // modra, gori 1sec

    TIM4->CR1 |= TIM_CR1_CEN;

    for(;;);
}

