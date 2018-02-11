//********************************************************************
//*                    EEE3017W C template                           *
//*                    LCD test                                      *
//*==================================================================*
//* WRITTEN BY:    	                 		                         *
//* DATE CREATED:                                                    *
//* MODIFIED:                                                        *
//*==================================================================*
//* PROGRAMMED IN: Eclipse Luna Service Release 1 (4.4.1)            *
//* DEV. BOARD:    UCT STM32 Development Board                       *
//*==================================================================*
//* DESCRIPTION:                                                     *
//*                                                                  *
//********************************************************************
// INCLUDE FILES
//====================================================================
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "lcd_stm32f0.h"
#include "stm32f0xx.h"
#include "PWM.h"


//====================================================================
// GLOBAL CONSTANTS
//====================================================================

//====================================================================
// GLOBAL VARIABLES
//====================================================================
//counts the number of bucket tips

char line_one[20];
char line_two[20];
uint16_t new_line;
uint16_t old_line;
uint16_t req_left_pulses =8;
uint16_t req_right_pulses =8;
uint16_t sensor;
int start=1;
int start_0=1;
int stop=0;
int flag =0;
int lost = 0;
int end = 0;
int for_back =0;
int dist_write;
uint32_t claw = 0;
uint32_t arm = 0;
uint16_t counter;
uint16_t delay_1=0;
uint16_t delay_2=0;
int flag_0;
int obj;

//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_ADC(void);
void init_ports(void);

uint16_t libs_ADC5_get(void);

void display(void);
void follow_line(void);
uint16_t check_distance(void);
void fwd(uint16_t , uint16_t);
void NVIC_Enable(void);
void init_EXTI(void);
void EXTI0_1_IRQHandler(void);
void EXTI2_3_IRQHandler(void);
void init_TIM3(void);
void init_TIM16(void);
void TIM16_IRQHandler(void);
void init_TIM15(void);
void TIM15_IRQHandler(void);
void init_TIM17(void);
void TIM17_IRQHandler(void);
void init_TIM14(void);
void EXTI4_15_IRQHandler(void);
//====================================================================
// MAIN FUNCTION
//====================================================================
void main (void)
 {
	//setup
	init_LCD();								// Initialise lcd

	init_ports();
	NVIC_Enable();
	init_EXTI();
	init_timer();
	init_TIM3();
	init_TIM16();
	init_TIM15();
	init_TIM14();
	init_TIM17();
	init_ADC();
	TIM2->CCR3 = 0;
	TIM2->CCR4 = 0;
	GPIOB->ODR &= ~(128);
	claw = 250;
	TIM3->CCR2 = claw;
	delay(100000);
	//stop the claw
	claw = 0;
	TIM3->CCR2 = claw;
	 // soft start the robot
			uint16_t max_speed;
			for(max_speed = 0; max_speed < 255 ; max_speed++){
				delay_one();
				//maps to PB10
				TIM2->CCR3 = 32*(max_speed); // Red

			    TIM2->CCR4 = 32*(max_speed); // Green
			}
	while(1)
	{

		//delay(800000);
		//check whether the light has turned green - PA3
		if (start_0 ==1){


			while (start ==1) {
				//set to move forward
				GPIOB->ODR |= (4);  //for PB2 - right motor
				GPIOB->ODR |= (8); //for PB3 - left motor
				//check if the robot is lost
				follow_line();
				if (delay_1 <= 3 && delay_1 >= 2){
					start = 0;
				}
				sprintf(line_one, "Time is...");
				sprintf(line_two, "%2.2d x25us, %d", delay_1,start);
				display();
			}

			delay(200000);
			//STOP!! Wait a minute...
			TIM2->CCR3 = 0;
			TIM2->CCR4 = 0;
			//align the gripper

			//start closing the gripper around the object

			GPIOB->ODR |= 128;
			claw = 580;
			TIM3->CCR2 = claw;
			//another delay
			delay(800000);
			//stop the claw
			claw = 0;
			TIM3->CCR2 = claw;
			//start retracting the gripper arm
			arm =599;
			TIM3->CCR1 = arm;
			GPIOB->ODR &= ~64;
			//delay
			delay(800000);
			//stop the arm
			arm = 0;
			TIM3->CCR1 = arm;
			//start moving backward

			while (stop <= 10 ){
				GPIOB->ODR &= ~(0b1<<2);  //for PB2 - right motor
				GPIOB->ODR &= ~(0b1<<3); //for PB3 - left motor
				//choose the line following means
				for_back = 1;
				follow_line();

			}//reached the end or lost?

			//stop the robot
			TIM2->CCR3 = 0;
			TIM2->CCR4 = 0;
			//drop the glue stick
			claw = 400;
			TIM3->CCR2 = claw;
			GPIOB->ODR &= ~(0b1<<7);
			start = 0;// stop doing anything

		}

		}	//End of infinite
}// End of main

void display(void)
{
	init_LCD();								// Initialise lcd
	lcd_putstring(line_one);		// Display string on line 1
	lcd_command(LINE_TWO);					// Move cursor to line 2
	lcd_putstring(line_two);

}



void init_ADC(void)
{
	// Enable clock for ADC1
	    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;
	   // Set the resolution and align to res_align in ADC1_CFGR1
	   //right align the data
	    ADC1->CFGR1 &= ~ADC_CFGR1_ALIGN;
	    //8 bit resolution

	    ADC1->CFGR1 |= ADC_CFGR1_RES_1 ;// Set bit 1

	    ADC1->CFGR1 &= ~ADC_CFGR1_RES_0 ;// Clear bit 0
	    //Set to single conversion by default

	    // Set the ADC1 channel. Default = channel 5
	    ADC1->CHSELR |= ADC_CHSELR_CHSEL5;

	    // Activate the ADC1 internally by setting the ADEN bit high in ADC1_CR
	     ADC1->CR |= ADC_CR_ADEN;
	    // Check if the ADC1 is on then continue with the setup by check ADEN bit in ADC1_CR
	    while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) {} // Wait

}

void init_ports(void)
{
	//initialise switches and LEDs
	// Enable clock for GPIOA
	    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	    // Activate pulls-ups for GPIOA in GPIOA_PUPDR
	    GPIOA->PUPDR &= ~(0b111111);
	    GPIOA->PUPDR |= 0b10101010; // Patterns activates pull-downs for PA 0:4
	    // Enable clock for LEDS
	    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
	    // Set PB 0:3,5:7 to outputs in GPIOB_MODER

		GPIOB->MODER|=GPIO_MODER_MODER0_0;
		//configure PB1 to alternative function
		GPIOB->MODER &= ~GPIO_MODER_MODER1_0;
		GPIOB->MODER |= GPIO_MODER_MODER1_1;

		GPIOB->MODER|=GPIO_MODER_MODER2_0;
		GPIOB->MODER|=GPIO_MODER_MODER3_0;
		//configure PB 4:5 to alternative function
		GPIOB->MODER &= ~GPIO_MODER_MODER4_0;
		GPIOB->MODER|=GPIO_MODER_MODER4_1;
		GPIOB->MODER &= ~GPIO_MODER_MODER5_0;
		GPIOB->MODER|=GPIO_MODER_MODER5_1;

		GPIOB->MODER|=GPIO_MODER_MODER6_0;
		GPIOB->MODER|=GPIO_MODER_MODER7_0;
	    // Activate analogue in GPIOA in GPIOA_MODER
	    // Pattern activates analogue for the PA 5:6
	    GPIOA->MODER |= GPIO_MODER_MODER5;
	    GPIOA->MODER |= GPIO_MODER_MODER6;
		//set PA 7 to alternate function mode
		GPIOA->MODER&= ~GPIO_MODER_MODER7_0;
		GPIOA->MODER|= GPIO_MODER_MODER7_1;

}

uint16_t libs_ADC5_get(void){
    ADC1->CHSELR &= ~ADC_CHSELR_CHSEL6;
    ADC1->CHSELR |= ADC_CHSELR_CHSEL5;
    ADC1->CR |= ADC_CR_ADSTART;
    while ((ADC1->ISR & ADC_ISR_EOC) == 0) {} // Wait until EOC bit in ADC1_ISR is set
    uint16_t value = ADC1->DR;
    return value; // Return value in ADC1_DR
}

void follow_line(void){
		//move forward
		fwd(req_right_pulses, req_left_pulses);
		//move backward
}


void fwd(uint16_t req_r_pul, uint16_t req_l_pul){
	//check the rate at which robot should move forward
	sprintf(line_one, "RobotForward");
	sprintf(line_two, "Right %d Left %d", req_r_pul, req_l_pul);
	//display();
	//function to move forward
	fwd_(req_r_pul, req_l_pul);
}



uint16_t check_distance(void)
{
	flag = 0;
	//read the ADC value
	uint16_t distance = libs_ADC5_get();
	//convert to a linear function, no just check three things
	if (distance > 155 && distance < 170){
		dist_write = 5;
	}
	else if (distance > 88 && distance < 93){
		dist_write = 10;
	}

	else if (distance > 75 && distance < 86){
		dist_write = 15;
	}
	else if (distance <= 75 ){

			flag = 1;
	}
	return dist_write;

}

void NVIC_Enable(void){
	//enable external interrupts for these pushbuttons
	NVIC_EnableIRQ(EXTI0_1_IRQn);
	NVIC_EnableIRQ(EXTI2_3_IRQn);
	NVIC_EnableIRQ(EXTI4_15_IRQn);
	//enable interrupts for timer 17
	//NVIC_EnableIRQ(TIM17_IRQn);
	//enable interrupts for timer 16
	NVIC_EnableIRQ(TIM16_IRQn);
	//enable interrupts for timer 15
	NVIC_EnableIRQ(TIM15_IRQn);
	//enable interrupts for timer 14
	NVIC_EnableIRQ(TIM14_IRQn);
	//enable interrupts for timer 3
	NVIC_EnableIRQ(TIM3_IRQn);

}

void init_EXTI(void){


	//enable clock for EXTI
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
	//configure the SYSCFG register
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA;
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI1_PA;


	//Enable rising edge trigger
	EXTI->RTSR|= EXTI_RTSR_TR0;
	EXTI->RTSR|= EXTI_RTSR_TR1;
	EXTI->RTSR|= EXTI_RTSR_TR2;
	EXTI->RTSR|= EXTI_RTSR_TR3;
	//unmask interrupts from the lines
	EXTI->IMR |= EXTI_IMR_MR0;
	EXTI->IMR |= EXTI_IMR_MR1;
	EXTI->IMR |= EXTI_IMR_MR2;
	EXTI->IMR |= EXTI_IMR_MR3;

	//Enable rising edge trigger
	EXTI->RTSR|= EXTI_RTSR_TR4;
	//enable falling edge trigger
	EXTI->FTSR|= EXTI_FTSR_TR4;
	//unmask interrupts from the lines
	EXTI->IMR |= EXTI_IMR_MR4;

}

void EXTI0_1_IRQHandler(void)
{
	//clear the bit in the external interrupt line
	EXTI->PR |= EXTI_PR_PR0;
	EXTI->PR |= EXTI_PR_PR1;
	//set flag
	old_line = 1;
	stop = 0;
	//check which is pulled low
	if(((GPIOA->IDR & GPIO_IDR_0) != 0) && ((GPIOA->IDR & GPIO_IDR_1) == 0)){
		//GPIOB->ODR = 1;
		if (for_back ==0){
		req_right_pulses = 0;
		req_left_pulses = 8;
		}
		else{
			req_right_pulses = 8;
			req_left_pulses = 0;
		}
	}
	//Turn the lights on
	else{
		//GPIOB->ODR = 2;
		sensor = 2;
		lost = 0;
		req_right_pulses = 8;
		req_left_pulses = 8;

	}
}


void EXTI2_3_IRQHandler(void){
	EXTI->PR |= EXTI_PR_PR2;
	EXTI->PR |= EXTI_PR_PR3;
	//clear flag
	old_line = 0;
	//check which one is causing the interrupt
	if(((GPIOA->IDR & GPIO_IDR_2) != 0) && ((GPIOA->IDR & GPIO_IDR_3) == 0)){
		stop = 0;
		//GPIOB->ODR = 4;
			if (for_back ==0){
			req_right_pulses = 8;
			req_left_pulses = 0;
			}
			else{
				req_right_pulses = 0;
				req_left_pulses = 8;
			}
				}
	//Turn the lights on
	else{
		//GPIOB->ODR = 8;
		start_0 =1;
		 // soft start the robot
		uint16_t max_speed;
		for(max_speed = 0; max_speed < 255 ; max_speed++){
			delay_one();
			//maps to PB10
			TIM2->CCR3 = 32*(max_speed); // Red

		    TIM2->CCR4 = 32*(max_speed); // Green
		 }
	}
}

void EXTI4_15_IRQHandler(void){

	//set pending something something
	EXTI->PR|=EXTI_PR_PR4;
	EXTI->PR|=EXTI_PR_PR15;

	//check which is pulled low
		if(((GPIOA->IDR & GPIO_IDR_4) == 0)){
			delay_2 = 0;
		}
		//Turn the lights on
		else if (((GPIOA->IDR & GPIO_IDR_4) != 0)){
			delay_2 = 1;
			delay_1 = counter;
			//set the counter to zero
			counter =0;

		}}


void init_TIM3(void){
	//enable clock to TIM3
	RCC->APB1ENR|= RCC_APB1ENR_TIM3EN;
	//map PB 4 to the TIM3_CH1 using AF0
	GPIOB->AFR[0] |= 0b00010000000000000000;
	GPIOB->AFR[0] &= ~0b00100000000000000000;
	GPIOB->AFR[0] &= ~0b01000000000000000000;
	GPIOB->AFR[0] &= ~0b10000000000000000000;
	//map PB 5 to the TIM3_CH2 using AF1
	GPIOB->AFR[0] |= 0b000100000000000000000000;
	GPIOB->AFR[0] &= ~0b001000000000000000000000;
	GPIOB->AFR[0] &= ~0b010000000000000000000000;
	GPIOB->AFR[0] &= ~0b100000000000000000000000;

	//set to output compare mode
	TIM3->CCMR1 &=~ TIM_CCMR1_CC1S;
	//set to edge triggered mode
	TIM3->CCMR1&=~TIM_CCMR1_OC1M_0;
	TIM3->CCMR1|=TIM_CCMR1_OC1M_1;
	TIM3->CCMR1|=TIM_CCMR1_OC1M_2;

	TIM3->CCMR1&=~TIM_CCMR1_OC2M_0;
	TIM3->CCMR1|=TIM_CCMR1_OC2M_1;
	TIM3->CCMR1|=TIM_CCMR1_OC2M_2;
	//set PSC, ARR and CCR1(used for channel one)
	TIM3->PSC = 79;
	TIM3->ARR = 599;
	TIM3->CCR1 = arm;
	TIM3->CCR2 = claw;
	//set as edge-aligned
	TIM3->CR1 &=~TIM_CR1_CMS;

	//set as active high
	TIM3->CCER &= ~TIM_CCER_CC1P;
	TIM3->CCER &= ~TIM_CCER_CC1NP;

	TIM3->CCER &= ~TIM_CCER_CC2P;
	TIM3->CCER &= ~TIM_CCER_CC2NP;
	//enable capture compare output
	TIM3->CCER |= TIM_CCER_CC1E;
	TIM3->CCER |= TIM_CCER_CC2E;
	//enable the counter
	TIM3->CR1|=TIM_CR1_CEN;

}

void init_TIM16(void)
{
	 // Enable TIM14 clock
	    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	    // Set the PSC in TIM14_PSC
	    TIM16->PSC = 7;
	    // Set the ARR in TIM14_ARR
	    TIM16->ARR = 149;
	    // Enable update interrupt in TIM6_DIER
	    TIM16->DIER |= TIM_DIER_UIE;
	    //enable the counter
		TIM16->CR1 |= TIM_CR1_CEN;

}

void TIM16_IRQHandler(void){
	//clear the interrupt flag
	TIM16->SR &=~TIM_SR_UIF;
	//increment a counter
	if (delay_2==1){
	counter+=1;
	}
}

void init_TIM15(void)
{
	 // Enable TIM14 clock
	    RCC->APB2ENR |= RCC_APB2ENR_TIM16EN;
	    // Set the PSC in TIM14_PSC
	    TIM15->PSC = 7;
	    // Set the ARR in TIM14_ARR
	    TIM15->ARR = 149;
	    // Enable update interrupt in TIM6_DIER
	    TIM15->DIER |= TIM_DIER_UIE;
	    //enable the counter
		TIM15->CR1 |= TIM_CR1_CEN;

}

void TIM15_IRQHandler(void){
	//clear the interrupt flag
	TIM15->SR &=~TIM_SR_UIF;
	//increment a stop flag
	stop++;
}
void init_TIM17(void){
	//enable clock for timer
	RCC->APB2ENR|= RCC_APB2ENR_TIM17EN;
	//define the stuff
	#define GPIO_AFRL_PIN7_AF_0  0b00010000000000000000000000000000;
	#define GPIO_AFRL_PIN7_AF_1  0b00100000000000000000000000000000;
	#define GPIO_AFRL_PIN7_AF_2  0b01000000000000000000000000000000;
	#define GPIO_AFRL_PIN7_AF_3  0b10000000000000000000000000000000;
	//map TIM17_CH1 to PA7 using AF5
	GPIOA->AFR[0] |= GPIO_AFRL_PIN7_AF_0;
	GPIOA->AFR[0] &= ~GPIO_AFRL_PIN7_AF_1;
	GPIOA->AFR[0] |= GPIO_AFRL_PIN7_AF_2;
	GPIOA->AFR[0] &= ~GPIO_AFRL_PIN7_AF_3;
	//configure TIM17 to input capture mode

	//set to input mode
	TIM17->CCMR1|= TIM_CCMR1_CC1S_0;
	TIM17->CCMR1&= ~TIM_CCMR1_CC1S_1;

	//set the filter to 8 sampling periods
	TIM17->CCMR1|= TIM_CCMR1_IC1F_0;
	TIM17->CCMR1|= TIM_CCMR1_IC1F_1;
	TIM17->CCMR1&= ~TIM_CCMR1_IC1F_2;
	TIM17->CCMR1&= ~TIM_CCMR1_IC1F_3;

	//set parameters for input capture prescaler
	TIM17->CCMR1 &= ~TIM_CCMR1_IC1PSC;

	//set it to be triggered by rising and falling edges
	TIM17->CCER|= TIM_CCER_CC1E;
	TIM17->CCER|= TIM_CCER_CC1P;
	TIM17->CCER|= TIM_CCER_CC1NP;
	//set up approximate ARR and PSC values
	TIM17->PSC = 480;
	TIM17->ARR = 1000;

	//enable the interrupt
	TIM17->DIER|=TIM_DIER_CC1IE;

	//enable capture from the counter
	TIM17->CCER|= TIM_CCER_CC1E;

	//start the timer
	TIM17->CR1 |=TIM_CR1_CEN;

}

void TIM17_IRQHandler(void){
	TIM17 -> SR &= ~TIM_SR_CC1IF; // clears CC1IF flag in TIM14_SR
	CCR1_Value = TIM17 -> CCR1; // store value of CCR1 into a variable

	//for storing first and second counter values
	//GPIOB->ODR= 64;
	t_1 = CCR1_Value;

	if (flag_0 ==2){
		//GPIOB->ODR= 128;
		t_2 = CCR1_Value;
		flag_0 =0;
	}
	flag_0+=1;
	//display counter value
	if (t_2 > t_1) {
		uint16_t delay = (t_2 -t_1);
		//check the length of the delay
		sprintf(line_one, "Time is...");
		sprintf(line_two, "This is %d, %d", delay,start);
		display();
		if (delay <= 1000 && delay >= 500){
			//increment some counter
			//obj+=1;
			//if (obj == 2){
				//start = 0;
			//}
			start = 0;
		}
	}

}

void init_TIM14(void){
	//enable clock to TIM14
	RCC->APB1ENR|= RCC_APB1ENR_TIM14EN;
	//map PB1 to the TIM14 using AF0
	GPIOB->AFR[0] &= ~0b11110000;
	//set to output compare mode
	TIM14->CCMR1 &=~ TIM_CCMR1_CC1S;
	//set to edge triggered mode
	TIM14->CCMR1&=~TIM_CCMR1_OC1M_0;
	TIM14->CCMR1|=TIM_CCMR1_OC1M_1;
	TIM14->CCMR1|=TIM_CCMR1_OC1M_2;
	//set PSC, ARR and CCR1(used for channel one)
	TIM14->PSC = 52;
	TIM14->ARR = 59999;
	TIM14->CCR1 = 9;
	//set as edge-aligned
	TIM14->CR1 &=~TIM_CR1_CMS;
	//set as active high
	TIM14->CCER &= ~TIM_CCER_CC1P;
	TIM14->CCER &= ~TIM_CCER_CC1NP;
	//enable capture compare output
	TIM14->CCER |= TIM_CCER_CC1E;
	//enable the counter
	TIM14->CR1|=TIM_CR1_CEN;

}
//********************************************************************
// END OF PROGRAM
//********************************************************************
