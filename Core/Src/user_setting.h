/*
 * user_setting.h
 *
 *  Created on: Feb 19, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_USER_SETTING_H_
#define SRC_USER_SETTING_H_

#include "main.h"

#define D0_PORT GPIOB
#define D0_PIN GPIO_PIN_13
#define D1_PORT GPIOB
#define D1_PIN GPIO_PIN_12
#define D2_PORT GPIOA
#define D2_PIN GPIO_PIN_11
#define D3_PORT GPIOA
#define D3_PIN GPIO_PIN_10
#define D4_PORT GPIOA
#define D4_PIN GPIO_PIN_9
#define D5_PORT GPIOA
#define D5_PIN GPIO_PIN_8
#define D6_PORT GPIOB
#define D6_PIN GPIO_PIN_15
#define D7_PORT GPIOB
#define D7_PIN GPIO_PIN_14

#define RD_PORT LCD_RD_GPIO_Port
#define RD_PIN  LCD_RD_Pin
#define WR_PORT LCD_WR_GPIO_Port
#define WR_PIN  LCD_WR_Pin
#define CD_PORT LCD_RS_GPIO_Port         // RS PORT
#define CD_PIN  LCD_RS_Pin     // RS PIN
#define CS_PORT LCD_CS_GPIO_Port
#define CS_PIN  LCD_CS_Pin
#define RESET_PORT LCD_RST_GPIO_Port
#define RESET_PIN  LCD_RST_Pin

#define  WIDTH    ((uint16_t)240)
#define  HEIGHT   ((uint16_t)320)


/****************** delay in microseconds ***********************/
extern TIM_HandleTypeDef htim1;
void delay(uint32_t time)
{
	/* change your code here for the delay in microseconds */
	__HAL_TIM_SET_COUNTER(&htim1, 0);
	while ((__HAL_TIM_GET_COUNTER(&htim1))<time);
}




// configure macros for the data pins.

/* First of all clear all the LCD_DATA pins i.e. LCD_D0 to LCD_D7
 * We do that by writing the HIGHER bits in BSRR Register
 *
 * For example :- To clear Pins B3, B4 , B8, B9, we have to write GPIOB->BSRR = 0b0000001100011000 <<16
 *
 *
 *
 * To write the data to the respective Pins, we have to write the lower bits of BSRR :-
 *
 * For example say the PIN LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
 *
 * GPIOB->BSRR = (data & (1<<4)) << 3.  Here first select 4th bit of data (LCD_D4), and than again shift left by 3 (Total 4+3 =7 i.e. PB7)
 *
 * GPIOB->BSRR = (data & (1<<6)) >> 4.  Here first select 6th bit of data (LCD_D6), and than again shift Right by 4 (Total 6-4 =2 i.e. PB2)
 *
 *
 */

void shiftOut(uint8_t data){
	for (int i = 0; i < 8; i++){
		// Shift Out Each Bit
		HAL_GPIO_WritePin(MUL_SCK_GPIO_Port, MUL_SCK_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MUL_MOSI_GPIO_Port, MUL_MOSI_Pin, (data & 0x80) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MUL_SCK_GPIO_Port, MUL_SCK_Pin, GPIO_PIN_SET);

		data <<= 1;
	}

	// Latch Data to Output
	HAL_GPIO_WritePin(MUL_Latch_GPIO_Port, MUL_Latch_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MUL_Latch_GPIO_Port, MUL_Latch_Pin, GPIO_PIN_RESET);
}

void write_8(uint8_t d) {
	shiftOut(0xFF);
    shiftOut(d);
}



  /* To read the data from the Pins, we have to read the IDR Register
   *
   * Take the same example say LCD_D4 is connected to PB7, and LCD_D6 is connected to PB2
   *
   * To read data we have to do the following
   *
   * GPIOB->IDR & (1<<7) >> 3. First read the PIN (1<<7 means we are reading PB7) than shift it to the position, where it is connected to
   * and in this example, that would be 4 (LCD_D4). (i.e. 7-3=4)
   *
   * GPIOB->IDR & (1<<2) << 4. First read the PIN (1<<2 means we are reading PB2) than shift it to the position, where it is connected to
   * and in this case, that would be 6 (LCD_D6). (i.e. 2+4= 6). Shifting in the same direction
   *
   */
	#define read_8() (        (((GPIOB->IDR & (1<<13)) >> 8) \
							   | ((GPIOB->IDR & (1<<12)) >> 1) \
							   | ((GPIOA->IDR & (1<<11)) >> 8) \
							   | ((GPIOA->IDR & (1<<10)) >> 5) \
							   | ((GPIOA->IDR & (1<<9)) >> 3) \
							   | ((GPIOA->IDR & (1<<8)) >> 6) \
							   | ((GPIOB->IDR & (1<<15)) >> 14) \
							   | ((GPIOB->IDR & (1<<14)) >> 11)))



/********************* For 180 MHz *****************************/
//#define WRITE_DELAY { WR_ACTIVE8; }
//#define READ_DELAY  { RD_ACTIVE16;}


/************************** For 72 MHZ ****************************/
//#define WRITE_DELAY { }
//#define READ_DELAY  { RD_ACTIVE;  }


/************************** For 100 MHZ ****************************/
#define WRITE_DELAY { WR_ACTIVE2; }
#define READ_DELAY  { RD_ACTIVE4; }


/************************** For 216 MHZ ****************************/
//#define WRITE_DELAY { WR_ACTIVE8; WR_ACTIVE8; } //216MHz
//#define IDLE_DELAY  { WR_IDLE4;WR_IDLE4; }
//#define READ_DELAY  { RD_ACTIVE16;RD_ACTIVE16;RD_ACTIVE16;}


/************************** For 48 MHZ ****************************/
//#define WRITE_DELAY { }
//#define READ_DELAY  { }


/*****************************  DEFINES FOR DIFFERENT TFTs   ****************************************************/

#define SUPPORT_8347D             //HX8347-D, HX8347-G, HX8347-I, HX8367-A +520 bytes, 0.27s



#endif /* SRC_USER_SETTING_H_ */
