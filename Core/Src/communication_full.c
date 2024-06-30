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
static uint8_t rxbuf_get_pc[19];

//-------------------- CONFIG FOR CONTROL COMMUNICATION --------------------------------------//
static UART_HandleTypeDef* huart_ctrl;
static uint8_t rxbuf_get_ctrl[19];
static uint8_t rx_buf_command[19];
static uint8_t rx_buf_holder[100];
static uint8_t id_holder = 0;
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
	uint8_t ping[] = {0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ping[18] = checksum_ctrl_generator(ping, 19);

	if(HAL_UART_Transmit(huart_ctrl, ping, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_ctrl_send_BNO08X(BNO08X_Typedef BNO08x){
	uint8_t BNO[] = {0xA5, 0x5A, 0x02, ((BNO08x.yaw >> 8) & 0XFF), ((BNO08x.yaw) & 0XFF), ((BNO08x.pitch >> 8) & 0XFF), ((BNO08x.pitch) & 0XFF), ((BNO08x.roll >> 8) & 0XFF), ((BNO08x.roll) & 0XFF), ((BNO08x.x_acceleration >> 8) & 0XFF), ((BNO08x.x_acceleration) & 0XFF), ((BNO08x.y_acceleration >> 8) & 0XFF), ((BNO08x.y_acceleration) & 0XFF), ((BNO08x.z_acceleration >> 8) & 0XFF), ((BNO08x.z_acceleration) & 0XFF), 0x00, 0x00, 0x00, 0x00};
	BNO[18] = checksum_ctrl_generator(BNO, 19);

	if(HAL_UART_Transmit(huart_ctrl, BNO, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_ctrl_send_Astar(void){
	for(int i = 0; i <= id_holder; i++){
		uint8_t tx_data[19];
		for(int j = 0; j < 19;j++){
			tx_data[j] = rx_buf_holder[(i*19)+j];
		}
		HAL_UART_Transmit(huart_ctrl, tx_data, 19, TIMEOUT_SEND);
	}
	return true;
}

bool tx_ctrl_send_Command(void){
	if(HAL_UART_Transmit(huart_ctrl, rx_buf_command, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_ctrl_task_done(uint16_t step){
	uint8_t task_done[] = {0xA5, 0x5A, 0x03, ((step >> 8) & 0XFF), ((step) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	task_done[18] = checksum_ctrl_generator(task_done, 19);

	if(HAL_UART_Transmit(huart_ctrl, task_done, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
//---------------------------------------------------- Send Kinematic Data -----------------------------------------------------------------------------------------//
bool tx_ctrl_send_Kinematic(uint16_t Sx, uint16_t Sy, uint16_t St, uint16_t T){
	uint8_t kinematic[] = {0xA5, 0x5A, 0x05, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((T >> 8) & 0XFF), ((T) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	kinematic[18] = checksum_pc_generator(kinematic, 19);

	if(HAL_UART_Transmit(huart_ctrl, kinematic, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
//---------------------------------------------------- Send Encoder Data ------------------------------------------------------------------------------------------------//
bool tx_ctrl_send_Encoder(kinematic_t encoder){
	uint8_t encoder_data[] = {0xA5, 0x5A, 0x06, (((int16_t)encoder.S3 >> 8) & 0XFF), (((int16_t)encoder.S3) & 0XFF), (((int16_t)encoder.S4 >> 8) & 0XFF), (((int16_t)encoder.S4) & 0XFF), (((int16_t)encoder.V3 >> 8) & 0XFF), (((int16_t)encoder.V3) & 0XFF), (((int16_t)encoder.V4 >> 8) & 0XFF), (((int16_t)encoder.V4) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	encoder_data[18] = checksum_pc_generator(encoder_data, 19);

	if(HAL_UART_Transmit(huart_ctrl, encoder_data, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

bool tx_ctrl_forwading(uint8_t* msg){
	if(HAL_UART_Transmit(huart_ctrl, msg, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
bool tx_ctrl_send_Odometry(int16_t Sx, int16_t Sy, int16_t St, int16_t Vx, int16_t Vy, int16_t Vt){
	uint8_t odom_data[] = {0xA5, 0x5A, 0x15, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((Vx >> 8) & 0XFF), ((Vx) & 0XFF), ((Vy >> 8) & 0XFF), ((Vy) & 0XFF), ((Vt >> 8) & 0XFF), ((Vt) & 0XFF), 0x00, 0x00, 0x00, 0x00};
	odom_data[18] = checksum_ctrl_generator(odom_data, 19);

	if(HAL_UART_Transmit(huart_ctrl, odom_data, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

void rx_ctrl_start_get(void){
	HAL_UART_Receive_DMA(huart_ctrl,rxbuf_get_ctrl, 19);
}

void rx_ctrl_get(com_ctrl_get_t* get){
		if((rxbuf_get_ctrl[0] == 0xA5) && (rxbuf_get_ctrl[1] == 0x5A)){

			// Check for ping
			if(rxbuf_get_ctrl[2] == 0x01){
				get->cmd = PING;
			}
			// Check for BNO08X Sensor
			else if(rxbuf_get_ctrl[2] == 0x02){

				if((rxbuf_get_ctrl[3] & 0x80)) get->yaw = ((rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4])-(65536);
				else get->yaw = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4];

				if((rxbuf_get_ctrl[5] & 0x80)) get->pitch = ((rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6])-(65536);
				else get->pitch = (rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6];

				if((rxbuf_get_ctrl[7] & 0x80)) get->roll = ((rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8])-(65536);
				else get->roll = (rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8];

				if((rxbuf_get_ctrl[9] & 0x80)) get->x_acceleration = ((rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10])-(65536);
				else get->x_acceleration = (rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10];

				if((rxbuf_get_ctrl[11] & 0x80)) get->y_acceleration = ((rxbuf_get_ctrl[11] << 8) | rxbuf_get_ctrl[12])-(65536);
				else get->y_acceleration = (rxbuf_get_ctrl[11] << 8) | rxbuf_get_ctrl[12];

				if((rxbuf_get_ctrl[13] & 0x80)) get->z_acceleration = ((rxbuf_get_ctrl[13] << 8) | rxbuf_get_ctrl[14])-(65536);
				else get->z_acceleration = (rxbuf_get_ctrl[13] << 8) | rxbuf_get_ctrl[14];

				get->cmd = DATA;
			}

			// Check for Task Done
			if(rxbuf_get_ctrl[2] == 0x03){
				if((rxbuf_get_ctrl[3] & 0x80)) get->step = ((rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4])-(65536);
				else get->step = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4];

				for(int i = 0; i <= id_holder; i++){
					for(int j = 0; j < 19;j++){
						rx_buf_holder[(i*19)+j] = 0;
						rx_buf_command[j] = 0;
					}
				}
				id_holder = 0;

				get->cmd = DATA;
			}

			// Check for Kinematic
			else if(rxbuf_get_ctrl[2] == 0x05){

				if((rxbuf_get_ctrl[3] & 0x80)) get->Sx = ((rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4])-(65536);
				else get->Sx = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4];

				if((rxbuf_get_ctrl[5] & 0x80)) get->Sy = ((rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6])-(65536);
				else get->Sy = (rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6];

				if((rxbuf_get_ctrl[7] & 0x80)) get->St = ((rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8])-(65536);
				else get->St = (rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8];

				if((rxbuf_get_ctrl[9] & 0x80)) get->T = ((rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10])-(65536);
				else get->T = (rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10];

				get->cmd = DATA;
			}

			// Check for Encoder
			else if(rxbuf_get_ctrl[2] == 0x06){

				if((rxbuf_get_ctrl[3] & 0x80)) get->S3 = ((rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4])-(65536);
				else get->S3 = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4];

				if((rxbuf_get_ctrl[5] & 0x80)) get->S4 = ((rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6])-(65536);
				else get->S4 = (rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6];

				if((rxbuf_get_ctrl[7] & 0x80)) get->V3 = ((rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8])-(65536);
				else get->V3 = (rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8];

				if((rxbuf_get_ctrl[9] & 0x80)) get->V4 = ((rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10])-(65536);
				else get->V4 = (rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10];

				get->cmd = DATA;
			}

			// Check for Odometry
			else if(rxbuf_get_ctrl[2] == 0x15){
				if((rxbuf_get_ctrl[3] & 0x80)) get->x_pos = ((rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4])-(65536);
				else get->x_pos = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4];

				if((rxbuf_get_ctrl[5] & 0x80)) get->y_pos = ((rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6])-(65536);
				else get->y_pos = (rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6];

				if((rxbuf_get_ctrl[7] & 0x80)) get->t_pos = ((rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8])-(65536);
				else get->t_pos = (rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8];

				if((rxbuf_get_ctrl[9] & 0x80)) get->x_vel = ((rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10])-(65536);
				else get->x_vel = (rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10];

				if((rxbuf_get_ctrl[11] & 0x80)) get->y_vel = ((rxbuf_get_ctrl[11] << 8) | rxbuf_get_ctrl[12])-(65536);
				else get->y_vel = (rxbuf_get_ctrl[11] << 8) | rxbuf_get_ctrl[12];

				if((rxbuf_get_ctrl[13] & 0x80)) get->t_vel = ((rxbuf_get_ctrl[13] << 8) | rxbuf_get_ctrl[14])-(65536);
				else get->t_vel = (rxbuf_get_ctrl[13] << 8) | rxbuf_get_ctrl[14];

				get->cmd = DATA;

			}

			// Check for "Move" Instruction Given from Jetson Nano
			else if(rxbuf_get_ctrl[2] == 0x12){

				get->id_data = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4] ;

				if((rxbuf_get_ctrl[5] & 0x80)) get->x_data = ((rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6])-(65536);
				else get->x_data = (rxbuf_get_ctrl[5] << 8) | rxbuf_get_ctrl[6];

				if((rxbuf_get_ctrl[7] & 0x80)) get->y_data = ((rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8])-(65536);
				else get->y_data = (rxbuf_get_ctrl[7] << 8) | rxbuf_get_ctrl[8];

				if((rxbuf_get_ctrl[9] & 0x80)) get->t_data = ((rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10])-(65536);
				else get->t_data = (rxbuf_get_ctrl[9] << 8) | rxbuf_get_ctrl[10];

				get->aktuator = rxbuf_get_ctrl[11] ;

				get->cmd = MOVE;

			}

			// Check for Astar Sequence Given from Jetson Nano
			else if(rxbuf_get_ctrl[2] == 0x13){
				uint8_t chk = checksum_ctrl_generator(rxbuf_get_ctrl,18);
				if(chk == rxbuf_get_ctrl[18]){
				get->astar_id = (rxbuf_get_ctrl[3]);
				get->astar_length = (rxbuf_get_ctrl[4]);
				get->astar_coordinate_x[rxbuf_get_ctrl[3]*5+0] = (rxbuf_get_ctrl[5]);
				get->astar_coordinate_y[rxbuf_get_ctrl[3]*5+0] = (rxbuf_get_ctrl[6]);
				get->astar_coordinate_x[rxbuf_get_ctrl[3]*5+1] = (rxbuf_get_ctrl[7]);
				get->astar_coordinate_y[rxbuf_get_ctrl[3]*5+1] = (rxbuf_get_ctrl[8]);
				get->astar_coordinate_x[rxbuf_get_ctrl[3]*5+2] = (rxbuf_get_ctrl[9]);
				get->astar_coordinate_y[rxbuf_get_ctrl[3]*5+2] = (rxbuf_get_ctrl[10]);
				get->astar_coordinate_x[rxbuf_get_ctrl[3]*5+3] = (rxbuf_get_ctrl[11]);
				get->astar_coordinate_y[rxbuf_get_ctrl[3]*5+3] = (rxbuf_get_ctrl[12]);
				get->astar_coordinate_x[rxbuf_get_ctrl[3]*5+4] = (rxbuf_get_ctrl[13]);
				get->astar_coordinate_y[rxbuf_get_ctrl[3]*5+4] = (rxbuf_get_ctrl[14]);
				get->astar_total_length = (rxbuf_get_ctrl[15] << 8) | rxbuf_get_ctrl[16];
				get->astar_msg_id = rxbuf_get_ctrl[17];
				get->cmd = MOVE;
				}

			}

		}
	HAL_UART_Receive_DMA(huart_ctrl, rxbuf_get_ctrl, 19);
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
	uint8_t ping[] = {0xA5, 0x5A, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	ping[18] = checksum_pc_generator(ping, 19);

	if(HAL_UART_Transmit(huart_pc, ping, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send Roll Pitch & Yaw from BNO08X Sensor -------------------------------------------------------------------------//
bool tx_pc_send_BNO08X(BNO08X_Typedef BNO08x){
	uint8_t BNO[] = {0xA5, 0x5A, 0x02, ((BNO08x.yaw >> 8) & 0XFF), ((BNO08x.yaw) & 0XFF), ((BNO08x.pitch >> 8) & 0XFF), ((BNO08x.pitch) & 0XFF), ((BNO08x.roll >> 8) & 0XFF), ((BNO08x.roll) & 0XFF), ((BNO08x.x_acceleration >> 8) & 0XFF), ((BNO08x.x_acceleration) & 0XFF), ((BNO08x.y_acceleration >> 8) & 0XFF), ((BNO08x.y_acceleration) & 0XFF), ((BNO08x.z_acceleration >> 8) & 0XFF), ((BNO08x.z_acceleration) & 0XFF), 0x00, 0x00, 0x00, 0x00};
	BNO[18] = checksum_pc_generator(BNO, 19);

	if(HAL_UART_Transmit(huart_pc, BNO, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
//---------------------------------------------------- Send Task Done ------------------------------------------------------------------------------------------------//

bool tx_pc_task_done(uint16_t step){
	uint8_t task_done[] = {0xA5, 0x5A, 0x03, ((step >> 8) & 0XFF), ((step) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	task_done[18] = checksum_ctrl_generator(task_done, 19);

	if(HAL_UART_Transmit(huart_pc, task_done, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send Package Sensor Data -----------------------------------------------------------------------------------------//
bool tx_pc_send_Sensor(sensor_package_t Sensor){
	uint8_t sensor[] = {0xA5, 0x5A, 0x04, ((Sensor.temperature >> 8) & 0XFF), ((Sensor.temperature) & 0XFF), ((Sensor.humidity >> 8) & 0XFF), ((Sensor.humidity) & 0XFF), ((Sensor.current >> 8) & 0XFF), ((Sensor.current) & 0XFF), ((Sensor.voltage >> 8) & 0XFF), ((Sensor.voltage) & 0XFF), ((Sensor.loadcell >> 8) & 0XFF), ((Sensor.loadcell) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	sensor[18] = checksum_pc_generator(sensor, 19);

	if(HAL_UART_Transmit(huart_pc, sensor, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

//---------------------------------------------------- Send Kinematic Data -----------------------------------------------------------------------------------------//
bool tx_pc_send_Kinematic(uint16_t Sx, uint16_t Sy, uint16_t St, uint16_t T){
	uint8_t kinematic[] = {0xA5, 0x5A, 0x05, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((T >> 8) & 0XFF), ((T) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	kinematic[18] = checksum_pc_generator(kinematic, 19);

	if(HAL_UART_Transmit(huart_pc, kinematic, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
//---------------------------------------------------- Send Encoder Data ------------------------------------------------------------------------------------------------//
bool tx_pc_send_Encoder(kinematic_t encoder){
	uint8_t encoder_data[] = {0xA5, 0x5A, 0x06, (((int16_t)encoder.S3 >> 8) & 0XFF), (((int16_t)encoder.S3) & 0XFF), (((int16_t)encoder.S4 >> 8) & 0XFF), (((int16_t)encoder.S4) & 0XFF), (((int16_t)encoder.V3 >> 8) & 0XFF), (((int16_t)encoder.V3) & 0XFF), (((int16_t)encoder.V4 >> 8) & 0XFF), (((int16_t)encoder.V4) & 0XFF), 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	encoder_data[18] = checksum_pc_generator(encoder_data, 19);

	if(HAL_UART_Transmit(huart_pc, encoder_data, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}
bool tx_pc_send_Odometry(int16_t Sx, int16_t Sy, int16_t St, int16_t Vx, int16_t Vy, int16_t Vt){
	uint8_t odometry[] = {0xA5, 0x5A, 0x15, ((Sx >> 8) & 0XFF), ((Sx) & 0XFF), ((Sy >> 8) & 0XFF), ((Sy) & 0XFF), ((St >> 8) & 0XFF), ((St) & 0XFF), ((Vx >> 8) & 0XFF), ((Vx) & 0XFF), ((Vy >> 8) & 0XFF), ((Vy) & 0XFF), ((Vt >> 8) & 0XFF), ((Vt) & 0XFF), 0x00, 0x00, 0x00, 0x00};
	odometry[18] = checksum_pc_generator(odometry, 19);

	if(HAL_UART_Transmit(huart_pc, odometry, 19, TIMEOUT_SEND) == HAL_OK) return true;
	else return false;
}

void rx_pc_start_get(void){
	HAL_UART_Receive_DMA(huart_pc,rxbuf_get_pc, 19);
}

void rx_pc_get(com_pc_get_t* get){
	for(int i = 0; i < 19; i++){
		if((rxbuf_get_pc[i] == 0xA5) && (rxbuf_get_pc[i+1] == 0x5A)){

			// Check for ping
			if(rxbuf_get_pc[2] == 0x01){
				get->cmd = PING;
			}

			// Check for Task Done
			if(rxbuf_get_pc[2] == 0x03){
				if((rxbuf_get_pc[i+3] & 0x80)) get->step = ((rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4])-(65536);
				else get->step = (rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4];

				if((rxbuf_get_ctrl[3] & 0x80)) get->step = ((rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4])-(65536);
					else get->step = (rxbuf_get_ctrl[3] << 8) | rxbuf_get_ctrl[4];

					for(int i = 0; i <= id_holder; i++){
						for(int j = 0; j < 19;j++){
							rx_buf_holder[(i*19)+j] = 0;
							rx_buf_command[j] = 0;
							}
					}
					id_holder = 0;

				get->cmd = DATA;
			}

			// Check for Kinematic
			else if(rxbuf_get_pc[i+2] == 0x05){

				if((rxbuf_get_pc[i+3] & 0x80)) get->Sx = ((rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4])-(65536);
				else get->Sx = (rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4];

				if((rxbuf_get_pc[i+5] & 0x80)) get->Sy = ((rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6])-(65536);
				else get->Sy = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];

				if((rxbuf_get_pc[i+7] & 0x80)) get->St = ((rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8])-(65536);
				else get->St = (rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8];

				if((rxbuf_get_pc[i+9] & 0x80)) get->T = ((rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10])-(65536);
				else get->T = (rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10];

				get->cmd = DATA;

			}

			// Check for Encoder
			else if(rxbuf_get_pc[i+2] == 0x06){

				if((rxbuf_get_pc[i+3] & 0x80)) get->S3 = ((rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4])-(65536);
				else get->S3 = (rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4];

				if((rxbuf_get_pc[i+5] & 0x80)) get->S4 = ((rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6])-(65536);
				else get->S4 = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];

				if((rxbuf_get_pc[i+7] & 0x80)) get->V3 = ((rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8])-(65536);
				else get->V3 = (rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8];

				if((rxbuf_get_pc[i+9] & 0x80)) get->V4 = ((rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10])-(65536);
				else get->V4 = (rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10];

				get->cmd = DATA;
			}

			// Check for Data Odometry
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

				get->cmd = DATA;

			}

			// Check for "Move" Instruction Given from Jetson Nano
			else if(rxbuf_get_pc[i+2] == 0x12){
				for(int j=0; j<19; j++){
					rx_buf_command[j] = rxbuf_get_pc[j];
				}

				get->id_data = (rxbuf_get_pc[i+3] << 8) | rxbuf_get_pc[i+4];

				if((rxbuf_get_pc[i+5] & 0x80)) get->x_data = ((rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6])-(65536);
				else get->x_data = (rxbuf_get_pc[i+5] << 8) | rxbuf_get_pc[i+6];

				if((rxbuf_get_pc[i+7] & 0x80)) get->y_data = ((rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8])-(65536);
				else get->y_data = (rxbuf_get_pc[i+7] << 8) | rxbuf_get_pc[i+8];

				if((rxbuf_get_pc[i+9] & 0x80)) get->t_data = ((rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10])-(65536);
				else get->t_data = (rxbuf_get_pc[i+9] << 8) | rxbuf_get_pc[i+10];

				get->aktuator = rxbuf_get_pc[i+11];

				get->cmd = MOVE;

			}

			// Check for Astar Sequence Given from Jetson Nano
			else if(rxbuf_get_pc[2] == 0x13){
				uint8_t chk = checksum_pc_generator(rxbuf_get_pc,18);
				if(chk == rxbuf_get_pc[18]){
					// Save message to holder
					for(int j=0; j<19; j++){
						rx_buf_holder[((rxbuf_get_pc[i+3])*19)+j] = rxbuf_get_pc[j];
					}
					// get id holder
					if(rxbuf_get_pc[i+3]>id_holder){
						id_holder = rxbuf_get_pc[i+3];
					}
					get->astar_id = (rxbuf_get_pc[i+3]);
					get->astar_length = (rxbuf_get_pc[i+4]);
					get->astar_coordinate_x[rxbuf_get_pc[i+3]*5+0] = (rxbuf_get_pc[i+5]);
					get->astar_coordinate_y[rxbuf_get_pc[i+3]*5+0] = (rxbuf_get_pc[i+6]);
					get->astar_coordinate_x[rxbuf_get_pc[i+3]*5+1] = (rxbuf_get_pc[i+7]);
					get->astar_coordinate_y[rxbuf_get_pc[i+3]*5+1] = (rxbuf_get_pc[i+8]);
					get->astar_coordinate_x[rxbuf_get_pc[i+3]*5+2] = (rxbuf_get_pc[i+9]);
					get->astar_coordinate_y[rxbuf_get_pc[i+3]*5+2] = (rxbuf_get_pc[i+10]);
					get->astar_coordinate_x[rxbuf_get_pc[i+3]*5+3] = (rxbuf_get_pc[i+11]);
					get->astar_coordinate_y[rxbuf_get_pc[i+3]*5+3] = (rxbuf_get_pc[i+12]);
					get->astar_coordinate_x[rxbuf_get_pc[i+3]*5+4] = (rxbuf_get_pc[i+13]);
					get->astar_coordinate_y[rxbuf_get_pc[i+3]*5+4] = (rxbuf_get_pc[i+14]);
					get->astar_total_length = (rxbuf_get_pc[i+15] << 8) | rxbuf_get_pc[i+16];
					get->astar_msg_id = rxbuf_get_pc[i+17];
					get->cmd = MOVE;

				}
			}

		}
	}
	HAL_UART_Receive_DMA(huart_pc, rxbuf_get_pc, 19);
}
