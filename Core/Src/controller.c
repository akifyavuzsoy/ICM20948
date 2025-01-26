/*
 * controller.c
 *
 *  Created on: Jan 11, 2025
 *      Author: akifyavuzsoy
 */

#include "controller.h"


// TODO: Event ve Aksiyonlar için fonksiyon oluştur. Örnek aşağıdaki system durumunu kontrol eden bir aksiyon fonksiyonu

// TODO: System Init Fonksiyonu oluştur...
// TODO: System adımlarını oluştur...

void check_System_Status_Action(sysController_t *this)
{
	switch(this->sys_Status){
	case SYS_SUCCESS:
		LED_OFF(LD3_GPIO_Port, LD3_Pin);	// Turn off for Orange Led (PD13)
		LED_OFF(LD5_GPIO_Port, LD5_Pin);	// Turn off for Red Led (PD14)
		LED_ON(LD4_GPIO_Port, LD4_Pin);		// Turn On for Green Led (PD12)
		break;

	case SYS_FAULT:
		LED_ON(LD3_GPIO_Port, LD3_Pin);		// Turn on for Orange Led (PD13)
		LED_OFF(LD5_GPIO_Port, LD5_Pin);	// Turn off for Red Led (PD14)
		LED_OFF(LD4_GPIO_Port, LD4_Pin);	// Turn off for Green Led (PD12)
		break;

	case SYS_ERROR:
		LED_OFF(LD3_GPIO_Port, LD3_Pin);		// Turn off for Orange Led (PD13)
		LED_ON(LD5_GPIO_Port, LD5_Pin);			// Turn on for Red Led (PD14)
		LED_OFF(LD4_GPIO_Port, LD4_Pin);		// Turn off for Green Led (PD12)
		// TODO: Kırmızı led blinkleyebilir...
		break;

	default:
		LED_OFF(LD3_GPIO_Port, LD3_Pin);		// Turn off for Orange Led (PD13)
		LED_OFF(LD5_GPIO_Port, LD5_Pin);		// Turn off for Red Led (PD14)
		LED_OFF(LD4_GPIO_Port, LD4_Pin);		// Turn off for Green Led (PD12)
		break;
	}
}



// IMU.c oluşturup onun içerisine alınailir ileride kod lerleyip büyüdükçe...

/***********************************************************************************************
 * IMU.c
 ***********************************************************************************************/

// TODO: IMU Init Fonksiyonu oluştur...
// TODO: IMU adımlarını oluştur...

/**
* @brief  Checks the status of the ICM20948 sensor by reading its WHO_AM_I register.
*         Updates the system status based on the sensor's response.
* @note   This function communicates with the ICM20948 sensor over the specified I2C channel.
*         The system status is updated as follows:
*         - SYS_SUCCESS: If the device responds with the correct ID.
*         - SYS_FAULT: If the device responds with an incorrect ID.
*         - SYS_ERROR: If there is an error in I2C communication.
* @param  this: Pointer to the sysController_t structure containing the system status.
* @retval None
*/
void check_icm20948(sysController_t *this)
{
    uint8_t device_id = 0;
    HAL_StatusTypeDef result;

    result = icm20948_who_am_i(ICM20948_I2C_CH, &device_id);

    if (result == HAL_OK) {
        if (device_id == ICM20948_ID) {
        	this->sys_Status = SYS_SUCCESS;
        }
        else {
        	this->sys_Status = SYS_FAULT;
        }
    }
    else {
    	this->sys_Status = SYS_ERROR;
    }
}

/**
* @brief  Checks the status of the AK09916 magnetometer by reading its WHO_AM_I register.
*         Updates the system status based on the sensor's response.
* @note   This function communicates with the AK09916 sensor over the specified I2C channel.
*         The system status is updated as follows:
*         - SYS_SUCCESS: If the device responds with the correct ID.
*         - SYS_FAULT: If the device responds with an incorrect ID.
*         - SYS_ERROR: If there is an error in I2C communication.
* @param  this: Pointer to the sysController_t structure containing the system status.
* @retval None
*/
void check_ak09916(sysController_t *this)
{
	uint8_t device_id = 0;
	HAL_StatusTypeDef result;

	result = ak09916_who_am_i(ICM20948_I2C_CH, &device_id);

	if (result == HAL_OK) {
	        if (device_id == AK09916_ID) {
	        	this->sys_Status = SYS_SUCCESS;
	        }
	        else {
	        	this->sys_Status = SYS_FAULT;
	        }
	    }
	    else {
	    	this->sys_Status = SYS_ERROR;
	    }
}

/**
* @brief  Resets the ICM20948 sensor by triggering a device reset.
*         Updates the system status based on the success of the reset operation.
* @note   This function communicates with the ICM20948 sensor over the specified I2C channel.
*         The system status is updated as follows:
*         - SYS_SUCCESS: If the reset operation is successful.
*         - SYS_FAULT: If the reset operation fails.
* @param  this: Pointer to the sysController_t structure containing the system status.
* @retval None
*/
void reset_icm20948(sysController_t *this)
{
	HAL_StatusTypeDef result;

	result = icm20948_device_reset(ICM20948_I2C_CH);
	if (result == HAL_OK) {
		this->sys_Status = SYS_SUCCESS;
	}
	else {
		this->sys_Status = SYS_FAULT;
	}
}
