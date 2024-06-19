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
static uint8_t rx_buf_holder[16];

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
//---------------------------------------------------- Send Kinematic Data -----------------------------------------------------------------------------------------//
bool tx_ctrl_send_Kinematic(int16_t Sx, int16_t Sy, int16_t St, int16_t Vx, int16_t Vy, int16_t Vt){
	uint8_t kinematic[] = {0xA5, 0x5A, 0x15, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((Vx >> 8) & 0XFF), ((Vx) & 0XFF), ((Vy >> 8) & 0XFF), ((Vy) & 0XFF), ((Vt >> 8) & 0XFF), ((Vt) & 0XFF), 0x00};
	kinematic[15] = checksum_ctrl_generator(kinematic, 16);

	if(HAL_UART_Transmit(huart_ctrl, kinematic, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
bool tx_ctrl_task_done(uint16_t step){
	uint8_t task_done[] = {0xA5, 0x5A, 0x03, ((step >> 8) & 0XFF), ((step) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	task_done[15] = checksum_ctrl_generator(task_done, 16);

	if(HAL_UART_Transmit(huart_ctrl, task_done, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_ctrl_forwading(uint8_t* msg){
	if(HAL_UART_Transmit(huart_ctrl, msg, 16, TIMEOUT_SEND) == HAL_OK) return true;
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
		if((rxbuf_get_ctrl[0] == 0xA5) && (rxbuf_get_ctrl[1] == 0x5A)){
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			// Check for ping
			if(rxbuf_get_ctrl[i+2] == 0x01){
				get->cmd = 0x01;
				uint8_t txbuf[3] = {0xA5, 0x5A, 0x01};
				HAL_UART_Transmit(huart_ctrl, txbuf, 3, 1);
			}

			// Check for "Move" Instruction Given from Sensor
			else if(rxbuf_get_ctrl[i+2] == 0x15){

				if((rxbuf_get_ctrl[i+3] & 0x80)) get->x_pos = ((rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4])-(65536);
				else get->x_pos = (rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4];

				if((rxbuf_get_ctrl[i+5] & 0x80)) get->y_pos = ((rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6])-(65536);
				else get->y_pos = (rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6];

				if((rxbuf_get_ctrl[i+7] & 0x80)) get->t_pos = ((rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8])-(65536);
				else get->t_pos = (rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8];

				if((rxbuf_get_ctrl[i+9] & 0x80)) get->x_vel = ((rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10])-(65536);
				else get->x_vel = (rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10];

				if((rxbuf_get_ctrl[i+11] & 0x80)) get->y_vel = ((rxbuf_get_ctrl[i+11] << 8) | rxbuf_get_ctrl[i+12])-(65536);
				else get->y_vel = (rxbuf_get_ctrl[i+11] << 8) | rxbuf_get_ctrl[i+12];

				if((rxbuf_get_ctrl[i+13] & 0x80)) get->t_vel = ((rxbuf_get_ctrl[i+13] << 8) | rxbuf_get_ctrl[i+14])-(65536);
				else get->t_vel = (rxbuf_get_ctrl[i+13] << 8) | rxbuf_get_ctrl[i+14];

//				#ifdef	USE_FORWARDING
//				for(int j=0; j<16; j++){
//					rx_buf_holder[j] = rxbuf_get_ctrl[i+j];
//				}
//				HAL_UART_Transmit(huart_pc, rx_buf_holder, 16, TIMEOUT_SEND);
//				#endif

				get->cmd = MOVE;

			}

			// Check for "Move" Instruction Given from Sensor
			else if(rxbuf_get_ctrl[i+2] == 0x12){
				if((rxbuf_get_ctrl[i+3] & 0x80)) get->x_pos = ((rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4])-(65536);
				else get->x_pos = (rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4];

				if((rxbuf_get_ctrl[i+5] & 0x80)) get->y_pos = ((rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6])-(65536);
				else get->y_pos = (rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6];

				if((rxbuf_get_ctrl[i+7] & 0x80)) get->orientation = ((rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8])-(65536);
				else get->orientation = (rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8];

				get->step = rxbuf_get_ctrl[i+10];

				get->cmd = MOVE;

			}

			// Check for Position
			else if(rxbuf_get_ctrl[i+2] == 0x02){

				if((rxbuf_get_ctrl[i+3] & 0x80)) get->yaw = ((rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4])-(65536);
				else get->yaw = (rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4];

				if((rxbuf_get_ctrl[i+5] & 0x80)) get->pitch = ((rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6])-(65536);
				else get->pitch = (rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6];

				if((rxbuf_get_ctrl[i+7] & 0x80)) get->roll = ((rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8])-(65536);
				else get->roll = (rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8];

				if((rxbuf_get_ctrl[i+9] & 0x80)) get->x_acceleration = ((rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10])-(65536);
				else get->x_acceleration = (rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10];

				if((rxbuf_get_ctrl[i+11] & 0x80)) get->y_acceleration = ((rxbuf_get_ctrl[i+11] << 8) | rxbuf_get_ctrl[i+12])-(65536);
				else get->y_acceleration = (rxbuf_get_ctrl[i+11] << 8) | rxbuf_get_ctrl[i+12];

				if((rxbuf_get_ctrl[i+13] & 0x80)) get->z_acceleration = ((rxbuf_get_ctrl[i+13] << 8) | rxbuf_get_ctrl[i+14])-(65536);
				else get->z_acceleration = (rxbuf_get_ctrl[i+13] << 8) | rxbuf_get_ctrl[i+14];

//				uint8_t txbuf[3] = {0xA5, 0x5A, 0x02};
//				HAL_UART_Transmit(huart_ctrl, txbuf, 3, 1);
//				get->cmd = 0x02;
			}

			// Check for Astar Sequence Given from Jetson Nano
			else if(rxbuf_get_ctrl[i+2] == 0x13){
				get->astar_id = (rxbuf_get_ctrl[i+3]);
				get->astar_length = (rxbuf_get_ctrl[i+4]);
				get->astar_coordinate_x[rxbuf_get_ctrl[i+3]*5-4] = (rxbuf_get_ctrl[i+5]);
				get->astar_coordinate_y[rxbuf_get_ctrl[i+3]*5-4] = (rxbuf_get_ctrl[i+6]);
				get->astar_coordinate_x[rxbuf_get_ctrl[i+3]*5-3] = (rxbuf_get_ctrl[i+7]);
				get->astar_coordinate_y[rxbuf_get_ctrl[i+3]*5-3] = (rxbuf_get_ctrl[i+8]);
				get->astar_coordinate_x[rxbuf_get_ctrl[i+3]*5-2] = (rxbuf_get_ctrl[i+9]);
				get->astar_coordinate_y[rxbuf_get_ctrl[i+3]*5-2] = (rxbuf_get_ctrl[i+10]);
				get->astar_coordinate_x[rxbuf_get_ctrl[i+3]*5-1] = (rxbuf_get_ctrl[i+11]);
				get->astar_coordinate_y[rxbuf_get_ctrl[i+3]*5-1] = (rxbuf_get_ctrl[i+12]);
				get->astar_coordinate_x[rxbuf_get_ctrl[i+3]*5-0] = (rxbuf_get_ctrl[i+13]);
				get->astar_coordinate_y[rxbuf_get_ctrl[i+3]*5-0] = (rxbuf_get_ctrl[i+14]);
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

//---------------------------------------------------- Send Kinematic Data -----------------------------------------------------------------------------------------//
bool tx_pc_send_Kinematic(uint16_t Sx, uint16_t Sy, uint16_t St, uint16_t T){
	uint8_t steady[] = {0xA5, 0x5A, 0x05, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((T >> 8) & 0XFF), ((T) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00};
	steady[15] = checksum_pc_generator(steady, 16);

	if(HAL_UART_Transmit(huart_pc, steady, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send DWM1000 Data -----------------------------------------------------------------------------------------//
bool tx_pc_send_DWM(uint16_t Xpos, uint16_t YPos){
	uint8_t steady[] = {0xA5, 0x5A, 0x06, ((Xpos >> 8) & 0XFF), ((Xpos) & 0XFF), ((YPos >> 8) & 0XFF), ((YPos) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	steady[15] = checksum_pc_generator(steady, 16);

	if(HAL_UART_Transmit(huart_pc, steady, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_pc_send_Odometry(int16_t Sx, int16_t Sy, int16_t St, int16_t Vx, int16_t Vy, int16_t Vt){
	uint8_t kinematic[] = {0xA5, 0x5A, 0x15, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((Vx >> 8) & 0XFF), ((Vx) & 0XFF), ((Vy >> 8) & 0XFF), ((Vy) & 0XFF), ((Vt >> 8) & 0XFF), ((Vt) & 0XFF), 0x00};
	kinematic[15] = checksum_pc_generator(kinematic, 16);

	if(HAL_UART_Transmit(huart_pc, kinematic, 16, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
//
//void tx_pc_send_kinematic(void){
//	for(int j=0; j<16; j++){
//		rx_buf_holder[j] = rxbuf_get_pc[i+j];
//	}
//	HAL_UART_Transmit(huart_ctrl, rx_buf_holder, 16, TIMEOUT_SEND);
//	HAL_UART_Receive_DMA(huart_ctrl, rxbuf_get_ctrl, 16);
//}

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

			// Check for Standby
			if(rxbuf_get_pc[i+2] == 0x11){
				get->direction = (rxbuf_get_pc[i+3]);
				get->speed = (rxbuf_get_pc[i+4]);
				get->distance = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];
				get->cmd = ROTATION;

				#ifdef	USE_FORWARDING
				for(int j=0; j<16; j++){
					rx_buf_holder[j] = rxbuf_get_pc[i+j];
				}
				HAL_UART_Transmit(huart_ctrl, rx_buf_holder, 16, TIMEOUT_SEND);
				#endif
			}
			// Check for "Move" Instruction Given from Sensor
			else if(rxbuf_get_pc[i+2] == 0x15){

				if((rxbuf_get_pc[i+3] & 0x80)) get->x_pos = ((rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4])-(65536);
				else get->x_pos = (rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4];

				if((rxbuf_get_pc[i+5] & 0x80)) get->y_pos = ((rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6])-(65536);
				else get->y_pos = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];

				if((rxbuf_get_pc[i+7] & 0x80)) get->t_pos = ((rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8])-(65536);
				else get->t_pos = (rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8];

				if((rxbuf_get_pc[i+9] & 0x80)) get->x_vel = ((rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10])-(65536);
				else get->x_vel = (rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10];

				if((rxbuf_get_pc[i+11] & 0x80)) get->y_vel = ((rxbuf_get_pc[i+11] << 8) | rxbuf_get_pc[i+12])-(65536);
				else get->y_vel = (rxbuf_get_pc[i+11] << 8) | rxbuf_get_pc[i+12];

				if((rxbuf_get_pc[i+13] & 0x80)) get->t_vel = ((rxbuf_get_pc[i+13] << 8) | rxbuf_get_pc[i+14])-(65536);
				else get->t_vel = (rxbuf_get_pc[i+13] << 8) | rxbuf_get_pc[i+14];

//				#ifdef	USE_FORWARDING
//				for(int j=0; j<16; j++){
//					rx_buf_holder[j] = rxbuf_get_pc[i+j];
//				}
//				HAL_UART_Transmit(huart_pc, rx_buf_holder, 16, TIMEOUT_SEND);
//				#endif

				get->cmd = MOVE;

			}

			// Check for "Move" Instruction Given from Jetson Nano
			else if(rxbuf_get_pc[i+2] == 0x12){
				if((rxbuf_get_ctrl[i+3] & 0x80)) get->x_pos = ((rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4])-(65536);
				else get->x_pos = (rxbuf_get_ctrl[i+3] << 8) | rxbuf_get_ctrl[i+4];

				if((rxbuf_get_ctrl[i+5] & 0x80)) get->y_pos = ((rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6])-(65536);
				else get->y_pos = (rxbuf_get_ctrl[i+5] << 8) | rxbuf_get_ctrl[i+6];

				if((rxbuf_get_ctrl[i+7] & 0x80)) get->orientation = ((rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8])-(65536);
				else get->orientation = (rxbuf_get_ctrl[i+7] << 8) | rxbuf_get_ctrl[i+8];

				if((rxbuf_get_ctrl[i+9] & 0x80)) get->orientation = ((rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10])-(65536);
				else get->step = (rxbuf_get_ctrl[i+9] << 8) | rxbuf_get_ctrl[i+10];

				get->cmd = MOVE;

				#ifdef	USE_FORWARDING
				for(int j=0; j<16; j++){
					rx_buf_holder[j] = rxbuf_get_pc[i+j];
				}
				HAL_UART_Transmit(huart_ctrl, rx_buf_holder, 16, TIMEOUT_SEND);
				#endif
			}

			// Check for Astar Sequence Given from Jetson Nano
			else if(rxbuf_get_pc[i+2] == 0x13){
				get->astar_id = (rxbuf_get_pc[i+3]);
				get->astar_length = (rxbuf_get_pc[i+4]);
				get->astar_coordinate_x[rxbuf_get_pc[i+3]*5-4] = (rxbuf_get_pc[i+5]);
				get->astar_coordinate_y[rxbuf_get_pc[i+3]*5-4] = (rxbuf_get_pc[i+6]);
				get->astar_coordinate_x[rxbuf_get_pc[i+3]*5-3] = (rxbuf_get_pc[i+7]);
				get->astar_coordinate_y[rxbuf_get_pc[i+3]*5-3] = (rxbuf_get_pc[i+8]);
				get->astar_coordinate_x[rxbuf_get_pc[i+3]*5-2] = (rxbuf_get_pc[i+9]);
				get->astar_coordinate_y[rxbuf_get_pc[i+3]*5-2] = (rxbuf_get_pc[i+10]);
				get->astar_coordinate_x[rxbuf_get_pc[i+3]*5-1] = (rxbuf_get_pc[i+11]);
				get->astar_coordinate_y[rxbuf_get_pc[i+3]*5-1] = (rxbuf_get_pc[i+12]);
				get->astar_coordinate_x[rxbuf_get_pc[i+3]*5-0] = (rxbuf_get_pc[i+13]);
				get->astar_coordinate_y[rxbuf_get_pc[i+3]*5-0] = (rxbuf_get_pc[i+14]);

				#ifdef	USE_FORWARDING
				for(int j=0; j<16; j++){
					rx_buf_holder[j] = rxbuf_get_pc[i+j];
				}
				HAL_UART_Transmit(huart_ctrl, rx_buf_holder, 16, TIMEOUT_SEND);
				#endif
			}

			// Check for Change Parameter Given from Jetson Nano
			else if(rxbuf_get_pc[i+2] == 0x14){
				get->direction = (rxbuf_get_pc[i+3]);
				get->speed = (rxbuf_get_pc[i+4]);
				get->distance = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];
				get->cmd = STANDBY;

				#ifdef	USE_FORWARDING
				for(int j=0; j<16; j++){
					rx_buf_holder[j] = rxbuf_get_pc[i+j];
				}
				HAL_UART_Transmit(huart_ctrl, rx_buf_holder, 16, TIMEOUT_SEND);
				#endif
			}

		}
	}
	HAL_UART_Receive_DMA(huart_pc, rxbuf_get_pc, 16);
}
