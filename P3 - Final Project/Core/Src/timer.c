/*
 * timer.c
 *
 *  Created on: Nov 7, 2024
 *      Author: firaz
 */
#include "main.h"
#include "timer.h"

void TIM2_init(void){
	//configure TIM2 clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM2EN);
	//set TIM2 to count up
	TIM2->CR1 &= ~(TIM_CR1_DIR);
	//set measuring time
	TIM2->ARR = MEASUREMENT_TIME;
	//set time of trigger event
	TIM2->CCR1 = TRIGGER_TIME;
	//configure channel 2 as input
	TIM2->CCMR1 |= TIM_CCMR1_CC2S_0;
	//rising edge polarity
	TIM2->CCER &= ~TIM_CCER_CC2P;
	//Enable capture on Channel 2
	TIM2->CCER |= TIM_CCER_CC2E;
	//enable update event interrupt in TIM2
	TIM2->DIER |= (TIM_DIER_UIE | TIM_DIER_CC1IE | TIM_DIER_CC2IE);
	//clear the flag before starting
	TIM2->SR &= ~(TIM_SR_UIF | TIM_SR_CC1IF | TIM_SR_CC2IF);
	//start timer
	TIM2->CR1 |= TIM_CR1_CEN;
	//enable TIM2 in NVIC
	NVIC_EnableIRQ(TIM2_IRQn);
	//enable interrupts globally
	__enable_irq();
}

void TIM3_init(void){
	//configure TIM3 clock
	RCC->APB1ENR1 |= (RCC_APB1ENR1_TIM3EN);
	// Prescaler of 0 (40MHz clock)
	TIM3->PSC = PRESCALER;
	//set PWM Mode 1 (OC4M = 110)
	TIM3->CCMR2 &= ~TIM_CCMR2_OC4M_Msk;
	TIM3->CCMR2 |= (6 << TIM_CCMR2_OC4M_Pos);
	//enable preload for CCR4
	TIM3->CCMR2 |= TIM_CCMR2_OC4PE;
	//enable timere 3 channel 4
	TIM3->CCER |= TIM_CCER_CC4E;
	//set to 50Hz
	TIM3->ARR = FREQUENCY;
	//set 7.5% duty cycle
	TIM3->CCR4 = DUTY_CYCLE_NEUTRAL;
	//enable ARR preload
	TIM3->CR1 |= TIM_CR1_ARPE;
	//generate an update event to load registers
	TIM3->EGR |= TIM_EGR_UG;
	//start the timer
	TIM3->CR1 |= TIM_CR1_CEN;

}
