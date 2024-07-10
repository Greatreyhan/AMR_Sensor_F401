/*
 * Voltage_Current.c
 *
 *  Created on: Feb 12, 2024
 *      Author: greatreyhan
 */
#include "Voltage_Current.h"
#include <math.h>
static uint32_t value[2];
//static float raw_voltage = 0;
//static float sensitivity = 0.1;
static float const_voltage = 1.1;
static float sensitivity = 0.1; //0.185 for 5A module => datasheet
float Input_votage_range =5; //3.3v stm32
int ADC_range = 4095;
//static float const_voltage = 1.035;

ADC_HandleTypeDef hadc;

void VoltCurrent_Init(ADC_HandleTypeDef *hadc_config){
	hadc = *hadc_config;
}

void ADC_Select_Voltage(void){
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_1;
	  sConfig.Rank = 1;
//	  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void ADC_Select_Current(void){
	  ADC_ChannelConfTypeDef sConfig = {0};
	  sConfig.Channel = ADC_CHANNEL_4;
	  sConfig.Rank = 1;
	  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
}

void Get_Voltage_Measurement(Voltage_Current_Typedef *config){

	  // Reading Voltage Sensor
	  ADC_Select_Voltage();
	  HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc, 1000);
	  value[0] = HAL_ADC_GetValue(&hadc);
	  config->voltage = (float)value[0]/4095*19.5;
	  HAL_ADC_Stop(&hadc);

}

void Get_Current_Measurement(Voltage_Current_Typedef *config){

	  // Reading Current Sensor
	  ADC_Select_Current();
	  HAL_ADC_Start(&hadc);
	  HAL_ADC_PollForConversion(&hadc, 1000);
	  float raw = HAL_ADC_GetValue(&hadc);
	  float volt_measurement = (float) ((raw * Input_votage_range)/ADC_range)*2.5;
//	  float volt_measurement = (float) raw_voltage*3.3*2*const_voltage/4095;
	  config->rawVoltage = volt_measurement;
	  config->current = (volt_measurement-2.5)/sensitivity;
//	  config->current  = fabs((volt_measurement - 1.65))/sensitivity;
	  HAL_Delay(100);
	  HAL_ADC_Stop(&hadc);
}

void VoltCurrent_Init_DMA(ADC_HandleTypeDef *hadc_config){
	hadc = *hadc_config;
	HAL_ADC_Start_DMA(&hadc, value, 2);
}

void VoltCurrent_Callback(Voltage_Current_Typedef *config){
	config->voltage = (float)value[0]/4095*16.5;
	float rawVoltage = (float) value[1]*3.3*2*const_voltage/4095;
	config->current  = (rawVoltage - 2.5)/sensitivity;
	HAL_ADC_Start_DMA(&hadc, value, 2);
}

