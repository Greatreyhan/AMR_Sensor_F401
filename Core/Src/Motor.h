/*
 * Motor.h
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#ifndef SRC_MOTOR_H_
#define SRC_MOTOR_H_

#include "main.h"
#include <stdbool.h>

#define DEG_TO_RAD(deg) ((deg) * M_PI / 180.0)
#define PULSE_TO_DIST	44.856 // konstanta pulse encoder ke jarak linear u/ hall efect dlm satuan mm

typedef struct{
	TIM_HandleTypeDef* 	tim;
	TIM_TypeDef* 				tim_number;
	uint32_t						counter;
	int16_t							counts;
	int16_t							position;
	int16_t							old_position;
	int									speed;
}encoder_t;

typedef struct{
	TIM_HandleTypeDef* 	tim_R;
	TIM_HandleTypeDef* 	tim_L;
	TIM_TypeDef* 				tim_number_R;
	TIM_TypeDef* 				tim_number_L;
	uint8_t 						channel_R;
	uint8_t 						channel_L;
	int16_t 						speed_R;
	int16_t 						speed_L;
	GPIO_TypeDef*				EN_PORT_R;
	GPIO_TypeDef*				EN_PORT_L;
	uint16_t						EN_PIN_R;
	uint16_t						EN_PIN_L;
	encoder_t						ENC;
}motor_t;


typedef struct{
	double 	Xp;
	double	Yp;
	double	Tp;

	double	Xg;
	double	Yg;
	double	Tg;

	double	V1;
	double	V2;
	double 	V3;
	double 	V4;
	double	Vx;
	double	Vy;
	double	Vt;
	double	S1;
	double	S2;
	double	S3;
	double 	S4;
	double	Sx;
	double	Sy;
	double	St;
}kinematic_t;

/*
	||****** Run Omnidirection Wheel *****||
	-	motor_t motor
	- int16_t speed 		: 0-1000
	- uint8_t direction : 1 -> clockwise
												2 -> counterclockwise
*/
void agv_run_motor(motor_t motor, int16_t speed);
void agv_stop(motor_t motor);
void agv_reset_all(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD);
void agv_stop_all(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD);

// Free move
void agv_play_rotate(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_y(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_x(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_cross_A(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);
void agv_play_cross_B(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed);

// Move Based on Encoder
void agv_move_rotate(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_y(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_x(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_cross_A(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);
void agv_move_cross_B(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD, int16_t speed, int32_t dist);

/*
	||****** Encoder Init *****||
	-	encoder_t encoder
	- TIM_HandleTypeDef* tim 	: timer used
	- TIM_TypeDef*						: TIM Number
*/
void agv_encoder_start(encoder_t encoder, TIM_HandleTypeDef* tim, TIM_TypeDef* tim_number);


// Kinematics
double agv_kinematic_Sx(int pos_A, int pos_B, int pos_C, int pos_D, double yaw);
double agv_kinematic_Sy(int pos_A, int pos_B, int pos_C, int pos_D, double yaw);
double agv_kinematic_St(int pos_A, int pos_B, int pos_C, int pos_D, double yaw);
void agv_forward_kinematic(encoder_t encA, encoder_t encB, encoder_t encC, encoder_t encD, double yaw, kinematic_t kinematic);
void agv_inverse_kinematic(double sx, double sy, double st, double yaw, motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD);
void agv_inverse_kinematic_basic(double sx, double sy, double st, motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD);
double agv_calculate_encoder(encoder_t encoder);
void agv_calculate_distance(kinematic_t agv);
void agv_calculate_rotational_matrix(kinematic_t agv);
void agv_speed_to_pwm(motor_t motor, double speed);

#endif /* SRC_MOTOR_H_ */
