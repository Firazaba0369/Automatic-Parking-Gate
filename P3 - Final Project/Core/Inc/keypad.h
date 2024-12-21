/*
 * keypad.h
 *
 *  Created on: Oct 23, 2024
 *      Author: firaz
 */

#ifndef INC_KEYPAD_H_
#define INC_KEYPAD_H_
#define SRC_KEYPAD_H_
#define NUM_OF_ROWS 4  // 4-row keypad
#define NUM_OF_COLS 3  // 3-column keypad
#define NO_PRESS (int8_t) -1//signfies no button was pressed
#define ASTERISK (int8_t) 10 //signifies asterisk keypress
#define POUND (int8_t) 11 //signifies pound keypress

void keypad_init(void);
int8_t keypad_func(void);
int8_t calculate_key(int8_t row,int8_t col);
uint16_t get_code(void);

#endif /* INC_KEYPAD_H_ */
