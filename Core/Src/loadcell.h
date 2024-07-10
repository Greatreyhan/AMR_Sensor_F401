/*
 * loadcell.h
 *
 *  Created on: Jul 6, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_LOADCELL_H_
#define SRC_LOADCELL_H_

#include "main.h"

#define DT_PIN GPIO_PIN_1
#define DT_PORT GPIOB
#define SCK_PIN GPIO_PIN_0
#define SCK_PORT GPIOB

void init_loadcell(TIM_HandleTypeDef* htim);
void microDelay(uint16_t delay);
int32_t getHX711(void);
int get_weight_loadcell(void);

#endif /* SRC_LOADCELL_H_ */
