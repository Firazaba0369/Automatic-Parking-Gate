/*
 * keypad.c
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */
#include "main.h"
#include "keypad.h"
#define SRC_KEYPAD_H_
#define NUM_OF_ROWS 4  // 4-row keypad
#define NUM_OF_COLS 3  // 3-column keypad
#define NO_PRESS (int8_t) -1//signfies no button was pressed
#define ASTERISK (int8_t) 10 //signifies asterisk keypress
#define POUND (int8_t) 11 //signifies pound keypress

/**
  * @brief: function to initialize keypad ports and set columns to 1
  * @retval: None
  */
void keypad_init(void){
	  //set clock for GPIOA, GPIOB, and GPIOC
	  RCC->AHB2ENR   |=  (RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOCEN);
	  /*------------------- Configure PB4-PB6 for GPIOB column input -------------------*/
	  //setup MODER for columns input
	  GPIOB->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6);
	  //setup pull down resistor to avoid floating
	  GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD4 |GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6);
	  GPIOB->PUPDR |= (GPIO_PUPDR_PUPD4_1 |GPIO_PUPDR_PUPD5_1 | GPIO_PUPDR_PUPD6_1);

	  /*------------------- Configure PC0-PC3 for GPIOC row output -------------------*/
	  //setup MODER for row output
	  GPIOC->MODER &= ~(GPIO_MODER_MODE0 | GPIO_MODER_MODE1 | GPIO_MODER_MODE2 | GPIO_MODER_MODE3);
	  GPIOC->MODER |= (GPIO_MODER_MODE0_0 | GPIO_MODER_MODE1_0 | GPIO_MODER_MODE2_0 | GPIO_MODER_MODE3_0);
	  //set push pull output type
	  GPIOC->OTYPER  &=  ~(GPIO_OTYPER_OT0 | GPIO_OTYPER_OT1 | GPIO_OTYPER_OT2 | GPIO_OTYPER_OT3);
	  //no PUPD
	  GPIOC->PUPDR |= (GPIO_PUPDR_PUPD0_1 |GPIO_PUPDR_PUPD1_1 | GPIO_PUPDR_PUPD2_1 | GPIO_PUPDR_PUPD3_1);
	  //set to high speed
	  GPIOC->OSPEEDR |= (GPIO_OSPEEDR_OSPEED0_Msk | GPIO_OSPEEDR_OSPEED1_Msk
			  | GPIO_OSPEEDR_OSPEED2_Msk | GPIO_OSPEEDR_OSPEED3_Msk);

	  /*------------------- Initialize rows-------------------*/
	  //set rows to 1
	  GPIOC->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);
}

/**
  * @brief Helper function to calculate the key for keypad_func
  * @retval int8_t
  */
int8_t calculate_key(int8_t row,int8_t col){
	//2D array to represent keypad
	int8_t keypad[NUM_OF_ROWS][NUM_OF_COLS] = {
	    {1, 2, 3},
	    {4, 5, 6},
	    {7, 8, 9},
	    {ASTERISK, 0, POUND}
	};
		return keypad[row][col];  // Return the character for the pressed key
}

/**
  * @brief: function to check for key-press and return key value if pressed
  * @retval: int8_t
  */
int8_t keypad_func(void){
	int8_t pressed_row = NO_PRESS;
    int8_t pressed_col = NO_PRESS;
    int16_t idr_value = GPIOB->IDR;
    //read cols
    idr_value &= (GPIO_IDR_ID4  | GPIO_IDR_ID5 | GPIO_IDR_ID6);
    if(!((idr_value == 0b10000) || (idr_value == 0b100000) || (idr_value == 0b1000000))){
    	//set rows back to zero for next press
    	GPIOC->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);
    	return NO_PRESS;
    }
    //cycle through rows to determine key-press
    for(int row = 0; row < NUM_OF_ROWS; row++){
        GPIOC->ODR &= ~(GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);
        GPIOC->ODR |= (1<<row);
        idr_value = GPIOB->IDR;
        //checks to see if we get correct idr, stores row, and associated col based on col idr
        if(idr_value & GPIO_IDR_ID4 ){ //check column 0
            pressed_col = 0;
            pressed_row = row;
            break;
        }
        if(idr_value & GPIO_IDR_ID5 ){ //check column 1
            pressed_col = 1;
            pressed_row = row;
            break;
        }
        if(idr_value & GPIO_IDR_ID6){ //check column 3
            pressed_col = 2;
            pressed_row = row;
            break;
        }
    }
    if(pressed_row == NO_PRESS || pressed_col == NO_PRESS){
    	//set rows back to zero for next press
    	GPIOC->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);
    	return NO_PRESS;
    }
    else{
    	//set rows back to zero for next press
    	GPIOC->ODR |= (GPIO_ODR_OD0 | GPIO_ODR_OD1 | GPIO_ODR_OD2 | GPIO_ODR_OD3);
    	return calculate_key(pressed_row, pressed_col);
    }

}

