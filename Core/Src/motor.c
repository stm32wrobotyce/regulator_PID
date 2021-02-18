/*
 * encoder.c
 *
 *  Created on: Jan 28, 2021
 *      Author: piotr
 */

#include "main.h"

#define ENCODER_RESOLUTION			3
#define TIMER_CONF_BOTH_EDGE_T1T2	4
#define MOTOR_GEAR					150

#define	TIMER_FREQENCY				20
#define	SECOND_IN_MINUTE			60

void motor_str_init(motor_str *m, TIM_HandleTypeDef *tim)
{
	m->timer = tim;
	m->resolution = ENCODER_RESOLUTION * TIMER_CONF_BOTH_EDGE_T1T2 * MOTOR_GEAR;

	m->pulse_count = 0;
	m->measured_speed = 0;
	m->set_speed = 0;
	m->actual_PWM = 0;
}

void motor_calculate_speed(motor_str *m)
{
	motor_update_count(m);

	m->measured_speed = (m->pulse_count * TIMER_FREQENCY * SECOND_IN_MINUTE) / m->resolution;

	int output = pid_calculate(&(m->pid_controller), m->set_speed, m->measured_speed);

	m->actual_PWM += output;

	if(m->actual_PWM >= 0)
	{
		drv8835_set_motorA_direction(CW);
		drv8835_set_motorA_speed(m->actual_PWM);
	}
	else
	{
		drv8835_set_motorA_direction(CCW);
		drv8835_set_motorA_speed(-m->actual_PWM);
	}
}

void motor_update_count(motor_str *m)
{
	m->pulse_count = (int16_t)__HAL_TIM_GET_COUNTER(m->timer);
	__HAL_TIM_SET_COUNTER(m->timer, 0);
}

void motor_set_speed(motor_str *m, int set_speed)
{
	if(set_speed != m->set_speed)
		pid_reset(&(m->pid_controller));

	m->set_speed = set_speed;
}
