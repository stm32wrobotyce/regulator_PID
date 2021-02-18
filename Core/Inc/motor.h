/*
 * encoder.h
 *
 *  Created on: Jan 28, 2021
 *      Author: piotr
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "pid_controller.h"

#define MOTOR_A_Kp					1.2
#define MOTOR_A_Ki					0.1
#define MOTOR_A_Kd					0.2
#define MOTOR_A_ANTI_WINDUP			1000

typedef struct
{
	TIM_HandleTypeDef *timer;	//timer obsługujący enkoder silnika

	uint16_t resolution;		//rozdzielczość silnika

	int pulse_count;		//zliczone impulsy
	int measured_speed;		//obliczona prędkość silnika
	int set_speed;			//zadana prędkość silnika

	int actual_PWM;			//wartość PWM

	pid_str pid_controller;
}motor_str;

void motor_str_init(motor_str *, TIM_HandleTypeDef *);
void motor_update_count(motor_str *);
void motor_calculate_speed(motor_str *);

void motor_set_speed(motor_str *, int);

#endif /* INC_MOTOR_H_ */
