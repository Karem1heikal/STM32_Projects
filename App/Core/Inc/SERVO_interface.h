/*
 * SERVO_interface.h
 *
 *  Created on: Jan 22, 2025
 *      Author: user
 */

#ifndef INC_SERVO_INTERFACE_H_
#define INC_SERVO_INTERFACE_H_

#include "main.h"

extern TIM_HandleTypeDef htim2;

void SERVO_voidInit(TIM_HandleTypeDef tim,uint8_t Copy_u8Channel_ID);

void SERVO_voidRotate(uint8_t Copy_u8Channel_ID,uint8_t Copy_u8Angle);

#endif /* INC_SERVO_INTERFACE_H_ */
