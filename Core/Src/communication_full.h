/*
 * communication_pc.h
 *
 *  Created on: Mar 10, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_COMMUNICATION_FULL_H_
#define SRC_COMMUNICATION_FULL_H_


#include "main.h"
#include "BNO08X.h"
#include <stdbool.h>

typedef enum{
	FORWARD_DIR = 0x01U,
	BACKWARD_DIR = 0x02U,
	RIGHT_DIR = 0x03U,
	LEFT_DIR = 0x04U
}move_direction_t;

typedef enum{
	PING = 0x01U,
	STANDBY = 0x02U,
	MOVE = 0x03U,
	ROTATION = 0x04U,
	REQ = 0x05U
}command_type_t;

typedef struct{
	uint16_t temperature;
	uint16_t humidity;
	uint16_t current;
	uint16_t voltage;
	uint16_t loadcell;
}sensor_package_t;

typedef struct{
	uint16_t vertical_distance;
	uint16_t horizontal_distance;
	uint16_t vertical_speed;
	uint16_t horizontal_speed;
}encoder_package_t;

typedef struct{
	bool ping;
	bool standby;
	bool move;
	bool rotation;
	bool req;
}feedback_pc_t;

typedef struct{
	bool ping;
	bool standby;
	bool move;
	bool rotation;
	bool req;
}feedback_ctrl_t;

typedef struct{
	uint16_t x_acceleration;
	uint16_t y_acceleration;
	uint16_t z_acceleration;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	move_direction_t direction;
	uint8_t speed;
	uint16_t distance;
	command_type_t cmd;
}com_pc_get_t;

typedef struct{
	uint16_t x_acceleration;
	uint16_t y_acceleration;
	uint16_t z_acceleration;
	uint16_t roll;
	uint16_t pitch;
	uint16_t yaw;
	move_direction_t direction;
	uint8_t speed;
	uint16_t distance;
	command_type_t cmd;
}com_ctrl_get_t;

void komunikasi_ctrl_init(UART_HandleTypeDef* uart_handler);
uint8_t checksum_ctrl_generator(uint8_t* arr, uint8_t size);
bool tx_ctrl_ping(void);
bool tx_ctrl_send_BNO08X(BNO08X_Typedef BNO08x);
void rx_ctrl_start(void);
void rx_ctrl_start_get(void);
void rx_ctrl_feedback(feedback_ctrl_t* fed);
void rx_ctrl_get(com_ctrl_get_t* get);

void komunikasi_pc_init(UART_HandleTypeDef* uart_handler);
bool tx_pc_ping(void);
uint8_t checksum_pc_generator(uint8_t* arr, uint8_t size);
bool tx_pc_send_BNO08X(BNO08X_Typedef BNO08x);
bool tx_pc_send_Encoder(encoder_package_t Encoder_Package);
bool tx_pc_send_Sensor(sensor_package_t Sensor);
void rx_pc_start(void);
void rx_pc_feedback(feedback_pc_t* fed);
void rx_pc_start_get(void);
void rx_pc_get(com_pc_get_t* get);

#endif /* SRC_COMMUNICATION_PC_H_ */
