#include "stm32f4xx.h"
#include "printf.h"

#include <stdint.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  //#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

int temp = 0;

void setTim(){
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

	TIM2->PSC=15999; //prescaler
	TIM2->ARR=999; //auto reload

	TIM2->DIER |= TIM_DIER_UIE; //enable interrupt
	NVIC_EnableIRQ(TIM2_IRQn);


	TIM2->CNT=0; //counter
	TIM2->CR1=1;

	while(!(TIM2->SR&TIM_SR_UIF));
	TIM2->SR&=~TIM_SR_UIF;
}

void setGPIO(){
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	GPIOD->MODER |= GPIO_MODER_MODER12_0;
}

void TIM2_IRQHandler(void){
	TIM2->SR = 0;

	if(temp == 0) {
		GPIOD->BSRRL = GPIO_BSRR_BS_12;
		temp++;
	}
	else {
		GPIOD->BSRRH = GPIO_BSRR_BS_12;
		temp = 0;
	}
}


int main(void) {
	setTim();
	setGPIO();

    /* Loop forever */
	for(;;);
}
