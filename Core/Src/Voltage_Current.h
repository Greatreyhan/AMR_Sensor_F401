/*
 * Voltage_Current.h
 *
 *  Created on: Feb 12, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_VOLTAGE_CURRENT_H_
#define SRC_VOLTAGE_CURRENT_H_

#include "main.h"

typedef struct{
	float voltage;
	float current;
}Voltage_Current_Typedef;

void VoltCurrent_Init(ADC_HandleTypeDef *hadc_config);
void VoltCurrent_Init_DMA(ADC_HandleTypeDef *hadc);
void VoltCurrent_Callback(Voltage_Current_Typedef *config);
void ADC_Select_Voltage(void);
void ADC_Select_Current(void);
void Get_Voltage_Measurement(Voltage_Current_Typedef *config);
void Get_Current_Measurement(Voltage_Current_Typedef *config);

#endif /* SRC_VOLTAGE_CURRENT_H_ */
