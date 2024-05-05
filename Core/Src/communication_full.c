/*
 * communication_pc.c
 *
 *  Created on: Mar 10, 2024
 *      Author: greatreyhan
 */

#include	"communication_full.h"
#define		USE_FEEDBACK
#define		USE_FORWARDING
#define		TIMEOUT_SEND	100

//-------------------- CONFIG FOR PC COMMUNICATION --------------------------------------//
static UART_HandleTypeDef* huart_pc;
static uint8_t rxbuf_pc[3];
static uint8_t rxbuf_get_pc[16];

//-------------------- CONFIG FOR CONTROL COMMUNICATION --------------------------------------//
static UART_HandleTypeDef* huart_ctrl;
static uint8_t rxbuf_ctrl[3];
static uint8_t rxbuf_get_ctrl[16];

//******************************************** COMMUNICATION TO CONTROL **********************************************//

void komunikasi_ctrl_init(UART_HandleTypeDef* uart_handler){
	huart_ctrl = uart_handler;
}

uint8_t checksum_ctrl_generator(uint8_t* arr, uint8_t size){
	uint8_t chksm = 0;
	for(uint8_t i = 0; i < size; i++) chksm += arr[i];
	return (chksm & 0xFF);
}

bool tx_ctrl_ping(void){
	uint8_t ping[] = {0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ping[15] = checksum_ctrl_generator(ping, 16);

	if(HAL_UART_Transmit(huart_ctrl, ping, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_ctrl_send_BNO08X(BNO08X_Typedef BNO08x){
	uint8_t steady[] = {0xA5, 0x5A, 0x02, ((BNO08x.yaw >> 8) & 0XFF), ((BNO08x.yaw) & 0XFF), ((BNO08x.pitch >> 8) & 0XFF), ((BNO08x.pitch) & 0XFF), ((BNO08x.roll >> 8) & 0XFF), ((BNO08x.roll) & 0XFF), ((BNO08x.x_acceleration >> 8) & 0XFF), ((BNO08x.x_acceleration) & 0XFF), ((BNO08x.y_acceleration >> 8) & 0XFF), ((BNO08x.y_acceleration) & 0XFF), ((BNO08x.z_acceleration >> 8) & 0XFF), ((BNO08x.z_acceleration) & 0XFF), 0x00};
	steady[15] = checksum_ctrl_generator(steady, 16);

	if(HAL_UART_Transmit(huart_ctrl, steady, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

void rx_ctrl_start(void){
	HAL_UART_Receive_DMA(huart_ctrl,rxbuf_ctrl, 3);
}

void rx_ctrl_start_get(void){
	HAL_UART_Receive_DMA(huart_ctrl,rxbuf_get_ctrl, 16);
}

void rx_ctrl_feedback(feedback_ctrl_t* fed){
	if(rxbuf_ctrl[0] == 0xA5 && rxbuf_ctrl[1]  == 0x5A){
		if(rxbuf_pc[2] == 0x01) fed->ping = true;
		else if(rxbuf_pc[2] == 0x02) fed->standby = true;
		else if(rxbuf_pc[2] == 0x03) fed->move = true;
		else if(rxbuf_pc[2] == 0x04) fed->rotation = true;
		else if(rxbuf_pc[2] == 0x05) fed->req = true;
	}
	HAL_UART_Receive_DMA(huart_ctrl,rxbuf_ctrl, 3);
}

void rx_ctrl_get(com_ctrl_get_t* get){
	for(int i = 0; i < 16; i++){
		if((rxbuf_get_ctrl[i] == 0xA5) && (rxbuf_get_ctrl[i+1] == 0x5A)){

			// Check for ping
			if(rxbuf_get_ctrl[i+2] == 0x01){
				get->cmd = 0x01;
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x01};
				HAL_UART_Transmit(huart_ctrl, txbuf, 3, 1);
			}

			// Check for Position
			else if(rxbuf_get_ctrl[i+2] == 0x02){
				get->yaw = (rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4];
				get->pitch = (rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6];
				get->roll = (rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8];
				get->x_acceleration = (rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10];
				get->y_acceleration = (rxbuf_get_ctrl[i+11] << 8) | rxbuf_get_ctrl[i+12];
				get->z_acceleration = (rxbuf_get_ctrl[i+13] << 8) | rxbuf_get_ctrl[i+14];
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x02};
				HAL_UART_Transmit(huart_ctrl, txbuf, 3, 1);
				get->cmd = 0x02;
			}

		}
	}
	HAL_UART_Receive_DMA(huart_ctrl, rxbuf_get_ctrl, 16);
}

//**************************************************** COMMUNICATION TO JETSON NANO *******************************************//

void komunikasi_pc_init(UART_HandleTypeDef* uart_handler){
	huart_pc = uart_handler;
}

uint8_t checksum_pc_generator(uint8_t* arr, uint8_t size){
	uint8_t chksm = 0;
	for(uint8_t i = 0; i < size; i++) chksm += arr[i];
	return (chksm & 0xFF);
}

bool tx_pc_ping(void){
	uint8_t ping[] = {0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ping[15] = checksum_pc_generator(ping, 16);

	if(HAL_UART_Transmit(huart_pc, ping, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send Roll Pitch & Yaw from BNO08X Sensor -------------------------------------------------------------------------//
bool tx_pc_send_BNO08X(BNO08X_Typedef BNO08x){
	uint8_t steady[] = {0xA5, 0x5A, 0x02, ((BNO08x.yaw >> 8) & 0XFF), ((BNO08x.yaw) & 0XFF), ((BNO08x.pitch >> 8) & 0XFF), ((BNO08x.pitch) & 0XFF), ((BNO08x.roll >> 8) & 0XFF), ((BNO08x.roll) & 0XFF), ((BNO08x.x_acceleration >> 8) & 0XFF), ((BNO08x.x_acceleration) & 0XFF), ((BNO08x.y_acceleration >> 8) & 0XFF), ((BNO08x.y_acceleration) & 0XFF), ((BNO08x.z_acceleration >> 8) & 0XFF), ((BNO08x.z_acceleration) & 0XFF), 0x00};
	steady[15] = checksum_pc_generator(steady, 16);

	if(HAL_UART_Transmit(huart_pc, steady, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send Encoder Data ------------------------------------------------------------------------------------------------//
bool tx_pc_send_Encoder(encoder_package_t Encoder_Package){
	uint8_t steady[] = {0xA5, 0x5A, 0x03, ((Encoder_Package.vertical_distance >> 8) & 0XFF), ((Encoder_Package.vertical_distance) & 0XFF), ((Encoder_Package.horizontal_distance >> 8) & 0XFF), ((Encoder_Package.horizontal_distance) & 0XFF), ((Encoder_Package.vertical_speed >> 8) & 0XFF), ((Encoder_Package.vertical_speed) & 0XFF), ((Encoder_Package.horizontal_speed >> 8) & 0XFF), ((Encoder_Package.horizontal_speed) & 0XFF), 0x00, 0x00, 0x00, 0x00};
	steady[15] = checksum_pc_generator(steady, 16);

	if(HAL_UART_Transmit(huart_pc, steady, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send Package Sensor Data -----------------------------------------------------------------------------------------//
bool tx_pc_send_Sensor(sensor_package_t Sensor){
	uint8_t steady[] = {0xA5, 0x5A, 0x04, ((Sensor.temperature >> 8) & 0XFF), ((Sensor.temperature) & 0XFF), ((Sensor.humidity >> 8) & 0XFF), ((Sensor.humidity) & 0XFF), ((Sensor.current >> 8) & 0XFF), ((Sensor.current) & 0XFF), ((Sensor.voltage >> 8) & 0XFF), ((Sensor.voltage) & 0XFF), ((Sensor.loadcell >> 8) & 0XFF), ((Sensor.loadcell) & 0XFF), 0x00, 0x00, 0x00};
	steady[15] = checksum_pc_generator(steady, 16);

	if(HAL_UART_Transmit(huart_pc, steady, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

void rx_pc_start(void){
	HAL_UART_Receive_DMA(huart_pc,rxbuf_pc, 3);
}

void rx_pc_start_get(void){
	HAL_UART_Receive_DMA(huart_pc,rxbuf_get_pc, 16);
}

void rx_pc_feedback(feedback_pc_t* fed){
	if(rxbuf_pc[0] == 0xA5 && rxbuf_pc[1]  == 0x5A){
		if(rxbuf_pc[2] == 0x01) fed->ping = true;
		else if(rxbuf_pc[2] == 0x02) fed->standby = true;
		else if(rxbuf_pc[2] == 0x03) fed->move = true;
		else if(rxbuf_pc[2] == 0x04) fed->rotation = true;
		else if(rxbuf_pc[2] == 0x05) fed->req = true;
	}
	HAL_UART_Receive_DMA(huart_pc,rxbuf_pc, 3);
}

void rx_pc_get(com_pc_get_t* get){
	for(int i = 0; i < 16; i++){
		if((rxbuf_get_pc[i] == 0xA5) && (rxbuf_get_pc[i+1] == 0x5A)){

			// Check for ping
			if(rxbuf_get_pc[i+2] == 0x10){
				get->cmd = PING;

				#ifdef	USE_FORWARDING
				HAL_UART_Transmit(huart_ctrl, rxbuf_get_pc, 16, TIMEOUT_SEND);
				#endif

			}
			// Check for Standby
			else if(rxbuf_get_pc[i+2] == 0x11){
				get->direction = (rxbuf_get_pc[i+3]);
				get->speed = (rxbuf_get_pc[i+4]);
				get->distance = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];
				get->cmd = ROTATION;

				#ifdef	USE_FORWARDING
				HAL_UART_Transmit(huart_ctrl, rxbuf_get_pc, 16, TIMEOUT_SEND);
				#endif
			}

			// Check for "Move" Instruction Given from Jetson Nano
			else if(rxbuf_get_pc[i+2] == 0x12){
				get->direction = (rxbuf_get_pc[i+3]);
				get->speed = (rxbuf_get_pc[i+4]);
				get->distance = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];
				get->cmd = MOVE;

				#ifdef	USE_FORWARDING
				HAL_UART_Transmit(huart_ctrl, rxbuf_get_pc, 16, TIMEOUT_SEND);
				#endif
			}

			// Check for "Rotate" Instruction Given from Jetson Nano
			else if(rxbuf_get_pc[i+2] == 0x13){
				get->direction = (rxbuf_get_pc[i+3]);
				get->speed = (rxbuf_get_pc[i+4]);
				get->distance = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];
				get->cmd = STANDBY;

				#ifdef	USE_FORWARDING
				HAL_UART_Transmit(huart_ctrl, rxbuf_get_pc, 16, TIMEOUT_SEND);
				#endif
			}

		}
	}
	HAL_UART_Receive_DMA(huart_pc, rxbuf_get_pc, 16);
}
