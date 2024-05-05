/*
 * BNO08X.h
 *
 *  Created on: Feb 12, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_BNO08X_H_
#define SRC_BNO08X_H_

#include "main.h"

typedef struct {
    uint8_t header;
    uint8_t index;
    int16_t yaw;
    int16_t pitch;
    int16_t roll;
    int16_t x_acceleration;
    int16_t y_acceleration;
    int16_t z_acceleration;
    uint8_t mi_mr_reserved[3];
    uint8_t checksum;
} BNO08X_Typedef;

void BNO08X_GetData(BNO08X_Typedef *sensorData);
void BNO08X_Init(UART_HandleTypeDef *huart);


#endif /* SRC_BNO08X_H_ */
