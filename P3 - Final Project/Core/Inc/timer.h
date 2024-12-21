/*
 * timer.h
 *
 *  Created on: Nov 7, 2024
 *      Author: firaz
 */

#ifndef INC_TIMER_H_
#define INC_TIMER_H_
#define MEASUREMENT_TIME 2879999 //48MHz x 60ms
#define TRIGGER_TIME 479 //48MHz x 10us
#define PRESCALER 3999
#define DUTY_CYCLE_LOWER 12 //8.5%
#define DUTY_CYCLE_NEUTRAL 18 //7.5%
#define DUTY_CYCLE_RAISE 24 //8.5%
#define FREQUENCY 240 //48MHz/3999 x 1/50Hz

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void TIM2_init(void);
void TIM3_init(void);

#endif /* INC_TIMER_H_ */
