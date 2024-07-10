/*
 * loadcell.c
 *
 *  Created on: Jul 6, 2024
 *      Author: greatreyhan
 */
#include "loadcell.h"

static TIM_HandleTypeDef* htim_loadcell;

//static uint32_t tare = 8135833;
//static float knownOriginal = 3400000;  // in milli gram
//static float knownHX711 = 42218;

static uint32_t tare = 1;
static float knownOriginal = 1;  // in milli gram
static float knownHX711 = 1;

void init_loadcell(TIM_HandleTypeDef* htim){
	htim_loadcell = htim;
	HAL_TIM_Base_Start(htim_loadcell);
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
	HAL_Delay(10);
}

void microDelay(uint16_t delay)
{
  __HAL_TIM_SET_COUNTER(htim_loadcell, 0);
  while (__HAL_TIM_GET_COUNTER(htim_loadcell) < delay);
}

int32_t getHX711(void)
{
  uint32_t data = 0;
  uint32_t startTime = HAL_GetTick();
  while(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
  {
    if(HAL_GetTick() - startTime > 200)
      return 0;
  }
  for(int8_t len=0; len<24 ; len++)
  {
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
    microDelay(1);
    data = data << 1;
    HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
    microDelay(1);
    if(HAL_GPIO_ReadPin(DT_PORT, DT_PIN) == GPIO_PIN_SET)
      data ++;
  }
  data = data ^ 0x800000;
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_SET);
  microDelay(1);
  HAL_GPIO_WritePin(SCK_PORT, SCK_PIN, GPIO_PIN_RESET);
  microDelay(1);
  return data;
}

int get_weight_loadcell(void)
{
  int32_t  total = 0;
  int32_t  samples = 50;
  int milligram;
  float coefficient;
  for(uint16_t i=0 ; i<samples ; i++)
  {
      total += getHX711();
  }
  int32_t average = (int32_t)(total / samples);
  coefficient = knownOriginal / knownHX711;
  milligram = (int)(average-tare)*coefficient;
  return milligram;
}
