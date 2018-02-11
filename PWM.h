
#include "stm32f0xx.h"
#define GPIO_AFRH_AFR10_AF2 ((uint32_t)0x00000200)
#define GPIO_AFRH_AFR11_AF2 ((uint32_t)0x00002000)
#define GPIO_AF2 ((uint32_t)0x00000002)

// This projects demonstrates PWM on the RG LED

// Red: PB10, TIM2_CH3

// Green: PB11, TIM2_CH4

// Both run on AF2


// Function prototypes. (make static if possible to limit visibility to only this file)
// Setup

void init_timer(void);
void delay_one(void);
void pivot_left_(void);
void pivot_right_(void);
// Interact


void init_timer(void) {

 RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

 GPIOB->MODER |= GPIO_MODER_MODER10_1; // PB10 = AF

 GPIOB->MODER |= GPIO_MODER_MODER11_1; // PB11 = AF

//PB10_AF = AF2 (ie: map to TIM2_CH3)

//PB11_AF = AF2 (ie: map to TIM2_CH4)

 GPIOB->AFR[1] |= (GPIO_AFRH_AFR10&(GPIO_AF2<<8)); 

 GPIOB->AFR[1] |= (GPIO_AFRH_AFR11&(GPIO_AF2<<12)); 

 TIM2->ARR = 7999; // f = 1 KHz

 TIM2->PSC = 5;

 // specify PWM mode: OCxM bits in CCMRx. We want mode 1

 TIM2->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1); // PWM Mode 1
// enable the OC channels

 TIM2->CCMR2 |= (TIM_CCMR2_OC4M_2 | TIM_CCMR2_OC4M_1); // PWM Mode 1

TIM2->CCER |= TIM_CCER_CC3E;

 
TIM2->CCER |= TIM_CCER_CC4E;


 // set PWM percentages

TIM2->CR1 |= TIM_CR1_CEN; // counter enable



}

void pivot_left_(void){
	//write a green to the LED
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 32*(200); // Green


}

void pivot_right_(void){
	//write a red to the LED
	TIM2->CCR4 = 0;
	TIM2->CCR3 = 32*(200); // Red
}
//Delay for 200ms

void delay_one(void){
        uint32_t delay = 25000;
        while (delay){
            //Decrease until it reaches zero
            delay -= 1;
        }
    }
void fwd_(uint16_t right_pul, uint16_t left_pul){
	//set the respective bits to left and right pulses
	TIM2->CCR3 = right_pul*999;//Red
	TIM2->CCR4 = left_pul*999; // Green
}


