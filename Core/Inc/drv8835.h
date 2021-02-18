/*
 * drv8835.h
 *
 *  Created on: 15 lis 2020
 *      Author: piotr
 */

#ifndef INC_DRV8835_H_
#define INC_DRV8835_H_

typedef enum
{
	In_In_Mode = 0,
	Phase_Enable_Mode = 1
}DRV8835_Mode;

typedef enum
{
	CW = 0,
	CCW = 1
}DRV8835_Direction;

void drv8835_init();
void drv8835_mode_control(DRV8835_Mode);
void drv8835_set_motorA_direction(DRV8835_Direction);
void drv8835_set_motorA_speed(uint16_t);

#endif /* INC_DRV8835_H_ */
