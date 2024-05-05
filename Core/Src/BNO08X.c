/*
 * BNO08X.c
 *
 *  Created on: Feb 12, 2024
 *      Author: greatreyhan
 */
#include "BNO08X.h"

static uint8_t RX_Data[19];
static UART_HandleTypeDef huart;

void BNO08X_GetData(BNO08X_Typedef *sensorData){
    sensorData->header = RX_Data[0];
    sensorData->index = RX_Data[2];
    sensorData->yaw = (int16_t)((RX_Data[4] << 8) | RX_Data[3]);
    sensorData->pitch = (int16_t)((RX_Data[6] << 8) | RX_Data[5]);
    sensorData->roll = (int16_t)((RX_Data[8] << 8) | RX_Data[7]);
    sensorData->x_acceleration = (int16_t)((RX_Data[10] << 8) | RX_Data[9]);
    sensorData->y_acceleration = (int16_t)((RX_Data[12] << 8) | RX_Data[11]);
    sensorData->z_acceleration = (int16_t)((RX_Data[14] << 8) | RX_Data[13]);

    // MI, MR, Reserved bytes
    for (int i = 0; i < 3; i++) {
        sensorData->mi_mr_reserved[i] = RX_Data[15 + i];
    }

    sensorData->checksum = RX_Data[18];
    HAL_UART_Receive_DMA(&huart, RX_Data, sizeof(RX_Data));
}

void BNO08X_Init(UART_HandleTypeDef *huart_instance){
	huart = *huart_instance;
	HAL_UART_Receive_DMA(&huart, RX_Data, sizeof(RX_Data));
}
