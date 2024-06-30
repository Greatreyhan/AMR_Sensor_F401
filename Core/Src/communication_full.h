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
#include "Motor.h"
#include <stdbool.h>

typedef enum{
	PING = 0x01U,
	STANDBY = 0x02U,
	MOVE = 0x03U,
	ROTATION = 0x04U,
	DATA = 0x05U,
}command_type_t;

typedef struct{
	int16_t temperature;
	int16_t humidity;
	int16_t current;
	int16_t voltage;
	int16_t loadcell;
}sensor_package_t;

typedef struct{
	// Astar Algorithm
	int16_t astar_coordinate_x[100];
	int16_t astar_coordinate_y[100];
	uint8_t astar_id;
	uint8_t astar_length;
	uint16_t astar_total_length;
	uint8_t astar_msg_id;

	// Instruction from PC
	uint16_t id_data;
	int16_t x_data;
	int16_t y_data;
	int16_t t_data;
	uint8_t aktuator;

	// Odometry Data
	int16_t x_pos;
	int16_t y_pos;
	int16_t t_pos;
	int16_t x_vel;
	int16_t y_vel;
	int16_t t_vel;
	int16_t orientation;

	// Encoder Data
	int16_t S1;
	int16_t S2;
	int16_t S3;
	int16_t S4;
	int16_t V1;
	int16_t V2;
	int16_t V3;
	int16_t V4;
	int16_t Sx;
	int16_t Sy;
	int16_t St;
	int16_t T;

	// IMU Data
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t x_acceleration;
	int16_t y_acceleration;
	int16_t z_acceleration;

	// Manual Instruction
	int16_t x_speed; // *10 -> max 2000
	int16_t y_speed; // *10 -> max 2000
	int16_t t_speed; // max 360

	// General
	int16_t step;
	command_type_t cmd;
}com_pc_get_t;

typedef struct{
	// Astar Algorithm
	int16_t astar_coordinate_x[100];
	int16_t astar_coordinate_y[100];
	uint8_t astar_id;
	uint8_t astar_length;
	uint16_t astar_total_length;
	uint8_t astar_msg_id;

	// Instruction from PC
	uint16_t id_data;
	int16_t x_data;
	int16_t y_data;
	int16_t t_data;
	uint8_t aktuator;

	// Odometry Data
	int16_t x_pos;
	int16_t y_pos;
	int16_t t_pos;
	int16_t x_vel;
	int16_t y_vel;
	int16_t t_vel;
	int16_t orientation;

	// Encoder Data
	int16_t S1;
	int16_t S2;
	int16_t S3;
	int16_t S4;
	int16_t V1;
	int16_t V2;
	int16_t V3;
	int16_t V4;
	int16_t Sx;
	int16_t Sy;
	int16_t St;
	int16_t T;

	// IMU Data
	int16_t roll;
	int16_t pitch;
	int16_t yaw;
	int16_t x_acceleration;
	int16_t y_acceleration;
	int16_t z_acceleration;

	// Manual Instruction
	int16_t x_speed; // *10 -> max 2000
	int16_t y_speed; // *10 -> max 2000
	int16_t t_speed; // max 360

	// General
	int16_t step;
	command_type_t cmd;
}com_ctrl_get_t;

void komunikasi_ctrl_init(UART_HandleTypeDef* uart_handler);
uint8_t checksum_ctrl_generator(uint8_t* arr, uint8_t size);
bool tx_ctrl_ping(void);
bool tx_ctrl_send_BNO08X(BNO08X_Typedef BNO08x);
bool tx_ctrl_task_done(uint16_t step);
bool tx_ctrl_forwading(uint8_t* msg);
bool tx_ctrl_send_Astar(void);
bool tx_ctrl_send_Command(void);
bool tx_ctrl_send_Odometry(int16_t Sx, int16_t Sy, int16_t St, int16_t Vx, int16_t Vy, int16_t Vt);
bool tx_ctrl_send_Kinematic(uint16_t Sx, uint16_t Sy, uint16_t St, uint16_t T);
bool tx_ctrl_send_Encoder(kinematic_t encoder);
void rx_ctrl_start(void);
void rx_ctrl_start_get(void);
void rx_ctrl_get(com_ctrl_get_t* get);

void komunikasi_pc_init(UART_HandleTypeDef* uart_handler);
bool tx_pc_ping(void);
uint8_t checksum_pc_generator(uint8_t* arr, uint8_t size);
bool tx_pc_send_BNO08X(BNO08X_Typedef BNO08x);
bool tx_pc_task_done(uint16_t step);
bool tx_pc_send_Encoder(kinematic_t encoder);
bool tx_pc_send_Sensor(sensor_package_t Sensor);
bool tx_pc_send_Odometry(int16_t Sx, int16_t Sy, int16_t St, int16_t Vx, int16_t Vy, int16_t Vt);
void rx_pc_start(void);
void rx_pc_start_get(void);
void rx_pc_get(com_pc_get_t* get);

#endif /* SRC_COMMUNICATION_PC_H_ */
