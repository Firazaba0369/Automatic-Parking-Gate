/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "timer.h"
#include "keypad.h"
#include "UART.h"
RNG_HandleTypeDef hrng;
#define RAISE_TIME 3000000
#define LOWER_TIME 3200000
#define WAIT_TIME 8000000
#define FIRST_SAMPLE 0
#define SECOND_SAMPLE 1
#define MAX_ECHO_TIME 24610 //~8.84cm
#define FLAG 1
#define NO_FLAG 0
#define FOUR_DIG_LIMITER 10000


//function prototypes
void sensor_init(void);
void motor_init(void);
void SystemClock_Config(void);
static void MX_RNG_Init(void);

//global vars
uint8_t detection_flag = NO_FLAG;
uint8_t update_passcode = FLAG;
uint8_t sample = FIRST_SAMPLE;
uint32_t time_one = 0;
uint32_t time_two = 0;

int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();

  //initialize peripherals
  TIM2_init();
  TIM3_init();
  UART_init();
  sensor_init();
  motor_init();
  keypad_init();
  LED_init();
  MX_RNG_Init();

  //local vars
  uint8_t down_flag = NO_FLAG;
  uint32_t passcode;

  while (1)
  {
		if(update_passcode && !detection_flag){

			//check flags before reading RNG data
			while(!(RNG->SR & RNG_SR_DRDY));

			// Read random data and limit it to a four-digit passcode
			uint32_t raw_random_number = RNG->DR;
			passcode = raw_random_number % FOUR_DIG_LIMITER;
			char passcode_str[6];

			//reset screen and cursor
			UART_print("\x1B[2J");
			UART_print("\x1B[H");

			//print RNG passcode
			UART_print("Passcode: ");
			sprintf(passcode_str, "%04u", passcode);
			UART_print(passcode_str);

			//set flag low
			update_passcode = NO_FLAG;
		}
			if (detection_flag && !down_flag){
				//reset screen and cursor
				UART_print("\x1B[2J");
				UART_print("\x1B[H");

				//print access status
				UART_print("Enter Passcode: ");

				//get the four digit code entered by the user
				uint16_t code = get_code();

				//check that code is the same as random could provided
				if(code == passcode){
					//reset screen and cursor
					UART_print("\x1B[2J");
					UART_print("\x1B[H");

					//print access status
					UART_print("ACCESS PERMITTED");

					//set pulse time to 2ms to raise gate
					TIM3->ARR = FREQUENCY;
					TIM3->CCR4 = DUTY_CYCLE_RAISE;

					//wait for gate to hit 180 degrees
					for(int i=0; i<RAISE_TIME; i++);

					//set pulse time to 1.5ms for neutral
					TIM3->ARR = FREQUENCY;
					TIM3->CCR4 = DUTY_CYCLE_NEUTRAL;

					//delay to see access status
					for(uint32_t i = 0; i < WAIT_TIME; i++);

					//set flags
					down_flag = FLAG;
					update_passcode = FLAG;
				}
				else if(code == 0){
					//reset screen and cursor
					UART_print("\x1B[2J");
					UART_print("\x1B[H");

					//print access status
					UART_print("DETECTION STOPPED PREMATURELY");

					//delay to see access status
					for(uint32_t i = 0; i < WAIT_TIME; i++);
					//set flag
					update_passcode = FLAG;
				}
				 else{
					//reset screen and cursor
					UART_print("\x1B[2J");
					UART_print("\x1B[H");

					//print access status
					UART_print("ACCESS DENIED");

					//delay to see access status
					for(uint32_t i = 0; i < WAIT_TIME; i++);

					//set flag
					update_passcode = FLAG;
				}
			}
			else if(down_flag && !detection_flag){
				//wait for ~2 second
				for(int i=0; i<WAIT_TIME; i++);

				//set pulse time to 1ms to lower gate
				TIM3->ARR = FREQUENCY;
				TIM3->CCR4 = DUTY_CYCLE_LOWER;

				//wait for gate to lower to 90 degrees
				for(int i=0; i<LOWER_TIME; i++);

				//set pulse time to 1.5ms for neutral
				TIM3->ARR = FREQUENCY;
				TIM3->CCR4 = DUTY_CYCLE_NEUTRAL;
				//set flag
				down_flag = NO_FLAG;
			}
  	  	 }
}

void TIM2_IRQHandler(void){
	// check for CC1 flag
	if (TIM2->SR & TIM_SR_CC1IF){
		//turn off trigger
		GPIOB->ODR &= ~(GPIO_ODR_OD0);
		//clear and update CCR1 flag
		TIM2->SR &= ~(TIM_SR_CC1IF);
	}
	// check for CC2 flag
	else if (TIM2->SR & TIM_SR_CC2IF){
		if(sample == FIRST_SAMPLE){
			//get current time
			time_one = TIM2->CNT;
			//increment sample
			sample++;
			//falling edge polarity
			TIM2->CCER |= TIM_CCER_CC2P;
		}
		else if (sample == SECOND_SAMPLE){
			//get current time
			time_two = TIM2->CNT;
			//increment sample
			sample = FIRST_SAMPLE;
			if(time_two - time_one <= MAX_ECHO_TIME){
				detection_flag = FLAG;
				GPIOA->ODR |= (GPIO_ODR_OD5);
			}
			else{
				detection_flag = NO_FLAG;
				GPIOA->ODR &= ~(GPIO_ODR_OD5);
			}
			//rising edge polarity
			TIM2->CCER &= ~TIM_CCER_CC2P;
		}
		//clear and update CCR1 flag
		TIM2->SR &= ~(TIM_SR_CC2IF);
	}
	// check for update event flag
	else if (TIM2->SR & TIM_SR_UIF){
		//turn on trigger
		GPIOB->ODR |= (GPIO_ODR_OD0);
		// clear update event interrupt flag
		TIM2->SR &= ~(TIM_SR_UIF);
	}
}

void LED_init(void){
	// Enable GPIOA Clock
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	// Set MODER to output
	GPIOA->MODER &= ~(GPIO_MODER_MODE5);
	GPIOA->MODER |= (GPIO_MODER_MODE5_0);
	//set push-pull output type
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT5);
	//no PUPD
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD5);
	//set to high speed
	GPIOA->OSPEEDR |= (GPIO_OSPEEDR_OSPEED5);
}

void sensor_init(void){
	//configure GPIOB clock
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	/*----- Configure PB0 as sensor trigger -----*/
	//setup MODER as output
	GPIOB->MODER &= ~(GPIO_MODER_MODE0);
	GPIOB->MODER |= (GPIO_MODER_MODE0_0);
	//set push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT0);
	//no pull-up/pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD0);
	//set to high speed
	GPIOB->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0);

	/*----- Configure PB3 as sensor echo -----*/
	//set to alternate function
	GPIOB->MODER &= ~(GPIO_MODER_MODER3);
	GPIOB->MODER |= (GPIO_MODER_MODE3_1);
	GPIOB->AFR[0] |= (1 << GPIO_AFRL_AFSEL3_Pos);
	//set push-pull
	GPIOB->OTYPER &= ~(GPIO_OTYPER_OT3);
	//pull-down
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD3);
	GPIOB->PUPDR |= (GPIO_PUPDR_PUPD3_1);

}

void motor_init(void){
	//configure GPIOB clock
	RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN);
	/*---- Configure PB2 as motor signal-----*/
	//setup as alternate function
	GPIOB->MODER &= ~GPIO_MODER_MODE1;
	GPIOB->MODER |= GPIO_MODER_MODE1_1;
	GPIOB->AFR[0] &= ~GPIO_AFRL_AFSEL1;
	GPIOB->AFR[0] |= (2 << GPIO_AFRL_AFSEL1_Pos);
}

/**
  * @brief: Function to read 4 key-presses
  * @retval: int16_t
  */
uint16_t get_code(void){
	//configure variables
	uint16_t dig_total= 0;
	int16_t dig1 = NO_PRESS;
	int16_t dig2 = NO_PRESS;
	int16_t dig3 = NO_PRESS;
	int16_t dig4 = NO_PRESS;
	while(dig1 == NO_PRESS){ //wait for key-press
	  dig1 = keypad_func(); //read key-press of first digit
	  //leave if detection stops
	  if(!detection_flag){
		  return NO_FLAG;
	  }
	}
	while(keypad_func()!= NO_PRESS);//wait for key release
	//print first digit
	char dig1_str[2];
	sprintf(dig1_str, "%u", dig1);
	UART_print(dig1_str);
	while(dig2 == NO_PRESS){ //wait for second key-press
	  dig2 = keypad_func(); //read key-press of second digit
	  //leave if detection stops
	  if(!detection_flag){
		  return NO_FLAG;
	  }
	}
	while(keypad_func()!= NO_PRESS);//wait for key release
	//print second digit
	char dig2_str[2];
	sprintf(dig2_str, "%u", dig2);
	UART_print(dig2_str);
	while(dig3 == NO_PRESS){ //wait for third key-press
	  dig3 = keypad_func(); //read key-press of third digit
	  //leave if detection stops
	  if(!detection_flag){
		  return NO_FLAG;
	  }
	}
	while(keypad_func()!= NO_PRESS); //wait for key release
	//print third digit
	char dig3_str[2];
	sprintf(dig3_str, "%u", dig3);
	UART_print(dig3_str);
	while(dig4 == NO_PRESS){ //wait for fourth key-press
	  dig4 = keypad_func(); //read key-press of fourth digit
	  //leave if detection stops
	  if(!detection_flag){
		  return NO_FLAG;
	  }
	}
	while(keypad_func()!= NO_PRESS); //wait for key release
	//print fourth digit
	char dig4_str[2];
	sprintf(dig4_str, "%u", dig4);
	UART_print(dig4_str);
	//combine digits to get code entered
	dig_total = (dig1 * 1000) + (dig2 * 100) + (dig3 * 10) + dig4;

	return dig_total;
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{

  /* USER CODE BEGIN RNG_Init 0 */

  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */

  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */

  /* USER CODE END RNG_Init 2 */

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
