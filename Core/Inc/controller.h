/*
 * controller.h
 *
 *  Created on: Jan 11, 2025
 *      Author: akifyavuzsoy
 */

#ifndef INC_CONTROLLER_H_
#define INC_CONTROLLER_H_

#include "ICM20948.h"
#include "main.h"

#define ICM20948_I2C_CH					(&hi2c1)

#define MAJ_VERSİON		01
#define MIN_VERSİON		01
#define REV_VERSİON		01

typedef enum {
    SYS_SUCCESS,
    SYS_FAULT,
    SYS_ERROR
} sys_Status_e;

// IMU_Status, GPS Status gibi devem ettirip en son genel system status yaparız...

typedef struct {

	sys_Status_e sys_Status;
	// TODO: Ana yapıdaki global değişkenleri burada tanımla alt yapıları için ayrı enum ve struct oluşturursun...

} sysController_t;

#define LED_ON(GPIO_PORT, GPIO_PIN)  	HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_SET)
#define LED_OFF(GPIO_PORT, GPIO_PIN) 	HAL_GPIO_WritePin(GPIO_PORT, GPIO_PIN, GPIO_PIN_RESET)
#define LED_TOGGLE(GPIO_PORT, GPIO_PIN) HAL_GPIO_TogglePin(GPIO_PORT, GPIO_PIN)

extern I2C_HandleTypeDef hi2c1;

void check_icm20948(sysController_t *this);
void check_ak09916(sysController_t *this);
void reset_icm20948(sysController_t *this);



void check_System_Status_Action(sysController_t *this);

#endif /* INC_CONTROLLER_H_ */
