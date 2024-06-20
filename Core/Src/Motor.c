/*
 * Motor.c
 *
 *  Created on: Feb 9, 2024
 *      Author: greatreyhan
 */

#include "Motor.h"
#include <math.h>

// Berdasarkan jarak roda ke titik pusat dalam meter
#define R_AMR	0.35
//#define R_AMR	0.46

void agv_run_motor(motor_t motor, int16_t speed){
	HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_SET);
	HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_SET);
	if(speed > 0){
		if(motor.channel_R == 1){
			motor.tim_number_R->CCR1 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_1);
		}
		else if(motor.channel_R == 2){
			motor.tim_number_R->CCR2 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_2);
		}
		else if(motor.channel_R == 3){
			motor.tim_number_R->CCR3 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_3);
		}
		else if(motor.channel_R == 4){
			motor.tim_number_R->CCR4 = speed;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_4);
		}
		if(motor.channel_L == 1){
			motor.tim_number_L->CCR1 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_1);
		}
		else if(motor.channel_L == 2){
			motor.tim_number_L->CCR2 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_2);
		}
		else if(motor.channel_L == 3){
			motor.tim_number_L->CCR3 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_3);
		}
		else if(motor.channel_L == 4){
			motor.tim_number_L->CCR4 = 0;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_4);
		}
	}
	else if(speed < 0){
		if(motor.channel_R == 1){
			motor.tim_number_R->CCR1 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_1);
		}
		else if(motor.channel_R == 2){
			motor.tim_number_R->CCR2 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_2);
		}
		else if(motor.channel_R == 3){
			motor.tim_number_R->CCR3 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_3);
		}
		else if(motor.channel_R == 4){
			motor.tim_number_R->CCR4 = 0;
			HAL_TIM_PWM_Start(motor.tim_R, TIM_CHANNEL_4);
		}
		if(motor.channel_L == 1){
			motor.tim_number_L->CCR1 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_1);
		}
		else if(motor.channel_L == 2){
			motor.tim_number_L->CCR2 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_2);
		}
		else if(motor.channel_L == 3){
			motor.tim_number_L->CCR3 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_3);
		}
		else if(motor.channel_L == 4){
			motor.tim_number_L->CCR4 = -speed;
			HAL_TIM_PWM_Start(motor.tim_L, TIM_CHANNEL_4);
		}
	}
}

void agv_stop(motor_t motor){
	HAL_GPIO_WritePin(motor.EN_PORT_R, motor.EN_PIN_R, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(motor.EN_PORT_L, motor.EN_PIN_L, GPIO_PIN_RESET);
}

void agv_stop_all(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD){
	agv_stop(motorA);
	agv_stop(motorB);
	agv_stop(motorC);
	agv_stop(motorD);
}

void agv_reset_all(motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD){
	agv_run_motor(motorA,0);
	agv_run_motor(motorB,0);
	agv_run_motor(motorC,0);
	agv_run_motor(motorD,0);
}

void agv_encoder_start(encoder_t encoder, TIM_HandleTypeDef* tim,TIM_TypeDef* tim_number){
	encoder.tim = tim;
	encoder.tim_number = tim_number;
	HAL_TIM_Encoder_Start_IT(tim, TIM_CHANNEL_ALL);
}

///////////////////////////////////////////// FORWARD KINEMATICS ///////////////////////////////////////////////////

void agv_forward_kinematic(encoder_t encA, encoder_t encB, encoder_t encC, encoder_t encD, double yaw, kinematic_t kinematic){
	kinematic.Sx = ((-sin(DEG_TO_RAD(45+yaw))*encA.position) + (-sin(DEG_TO_RAD(135+yaw))*encB.position) + (-sin(DEG_TO_RAD(225+yaw))*encC.position) + (-sin(DEG_TO_RAD(315+yaw))*encD.position))*0.5;
	kinematic.Sy = ((cos(DEG_TO_RAD(45+yaw))*encA.position) + (cos(DEG_TO_RAD(135+yaw))*encB.position) + (cos(DEG_TO_RAD(225+yaw))*encC.position) + (cos(DEG_TO_RAD(315+yaw))*encD.position))*0.5;
	kinematic.St = (((2*encA.position)/R_AMR)+((2*encB.position)/R_AMR)+((2*encC.position)/R_AMR)+((2*encD.position)/R_AMR))*0.5;
}
double agv_kinematic_Sx(int pos_A, int pos_B, int pos_C, int pos_D, double yaw){
	double sx = ((-sin(DEG_TO_RAD(45+yaw))*pos_A) + (-sin(DEG_TO_RAD(135+yaw))*pos_B) + (-sin(DEG_TO_RAD(225+yaw))*pos_C) + (-sin(DEG_TO_RAD(315+yaw))*pos_D))*0.5;
	return sx;
}
double agv_kinematic_Sy(int pos_A, int pos_B, int pos_C, int pos_D, double yaw){
	double sy = ((cos(DEG_TO_RAD(45+yaw))*pos_A) + (cos(DEG_TO_RAD(135+yaw))*pos_B) + (cos(DEG_TO_RAD(225+yaw))*pos_C) + (cos(DEG_TO_RAD(315+yaw))*pos_D))*0.5;
	return sy;
}
double agv_kinematic_St(int pos_A, int pos_B, int pos_C, int pos_D, double yaw){
	double st = (((pos_A)/R_AMR)+((pos_B)/R_AMR)+((pos_C)/R_AMR)+((pos_D)/R_AMR))*0.5;
	return st;
}

///////////////////////////////////////////// INVERSE KINEMATICS ///////////////////////////////////////////////////

void agv_inverse_kinematic(double sx, double sy, double st, double yaw, motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD){
	double V1 = (-sin(DEG_TO_RAD(45+yaw))*sx) + (cos(DEG_TO_RAD(45+yaw))*sy) + (R_AMR*st);
	double V2 = (-sin(DEG_TO_RAD(135+yaw))*sx) + (cos(DEG_TO_RAD(135+yaw))*sy) + (R_AMR*st);
	double V3 = (-sin(DEG_TO_RAD(225+yaw))*sx) + (cos(DEG_TO_RAD(225+yaw))*sy) + (R_AMR*st);
	double V4 = (-sin(DEG_TO_RAD(315+yaw))*sx) + (cos(DEG_TO_RAD(315+yaw))*sy) + (R_AMR*st);
	agv_reset_all(motorA, motorB, motorC, motorD);
	agv_speed_to_pwm(motorA,V1);
	agv_speed_to_pwm(motorB,V2);
	agv_speed_to_pwm(motorC,V3);
	agv_speed_to_pwm(motorD,V4);
}

void agv_inverse_kinematic_basic(double sx, double sy, double st, motor_t motorA, motor_t motorB, motor_t motorC, motor_t motorD){
	double V1 = (-0.707*sx) + (0.707*sy) + (R_AMR*st);
	double V2 = (-0.707*sx) + (-0.707*sy) + (R_AMR*st);
	double V3 = (0.707*sx) + (-0.707*sy) + (R_AMR*st);
	double V4 = (0.707*sx) + (0.707*sy) + (R_AMR*st);
	agv_reset_all(motorA, motorB, motorC, motorD);
	agv_speed_to_pwm(motorA,V1);
	agv_speed_to_pwm(motorB,V2);
	agv_speed_to_pwm(motorC,V3);
	agv_speed_to_pwm(motorD,V4);
}

double agv_calculate_encoder(encoder_t encoder){
	double k_wheel = 3.14*100; // 100 -> diameter
	return encoder.position*(7/k_wheel); // 7->PPR
}

// in mm
void agv_speed_to_pwm(motor_t motor, double speed){
	// Maximum 2,617 m/s -> PWM 1000
	if(speed < 2617){
		agv_run_motor(motor, (speed*(5*M_PI/60)));
	}
	else{
		agv_run_motor(motor, (2617*(5*M_PI/60)));
	}
}
