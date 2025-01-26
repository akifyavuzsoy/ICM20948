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
		this-> ErrorCode = 0x00;
		break;

	case SYS_FAULT:
		LED_ON(LD3_GPIO_Port, LD3_Pin);		// Turn on for Orange Led (PD13)
		LED_OFF(LD5_GPIO_Port, LD5_Pin);	// Turn off for Red Led (PD14)
		LED_OFF(LD4_GPIO_Port, LD4_Pin);	// Turn off for Green Led (PD12)
		this-> ErrorCode = 0x0F;
		break;

	case SYS_ERROR:
		LED_OFF(LD3_GPIO_Port, LD3_Pin);		// Turn off for Orange Led (PD13)
		LED_ON(LD5_GPIO_Port, LD5_Pin);			// Turn on for Red Led (PD14)
		LED_OFF(LD4_GPIO_Port, LD4_Pin);		// Turn off for Green Led (PD12)
		this-> ErrorCode = 0xFF;
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


void system_Init(sysController_t *this)
{
	this->init_Step = IMU_INIT;

	switch(this->init_Step)
	{
		case IMU_INIT:
			status = icm20948_init(ICM20948_I2C_CH);
			if(status == HAL_OK){
				this->sys_Status = SYS_SUCCESS;
				this->sys_Step = IMU_READ;
			}
			else if (status == HAL_ERROR) {
				this->sys_Status = SYS_ERROR;
			}
			else{
				this->sys_Status = SYS_FAULT;
			}
			break;


	}
}

void system_Controller(sysController_t *this)
{

	switch(this->sys_Step)
	{
		case IMU_READ:
			icm20948_gyro_read_dps(ICM20948_I2C_CH, this->gyro_datas);
			icm20948_accel_read_g(ICM20948_I2C_CH, this->accel_datas);
			this->sys_Status = SYS_SUCCESS;
			this->sys_Step = SET_DATASET;
			break;
		case SET_DATASET:

			this->sys_Status = SYS_SUCCESS;
			this->sys_Step = IMU_READ;
			break;

		//case default:
			//this->sys_Step = SYS_WAIT;

	}
}

void set_Datasets(sysController_t *this)
{
	this->DATASET[0] = 0xFA;
	this->DATASET[1] = 0xFB;

	this->DATASET[2] = this->ErrorCode;

	write_float_to_array(this, this->gyro_datas, 8);
	write_float_to_array(this, this->accel_datas, 24);


	this->DATASET[124] = 0xFA;
	this->DATASET[125] = 0xFB;
}


void write_float_to_array(sysController_t *this, axises data_f, uint8_t addr)
{
	uint8_t *x_ptr = (uint8_t *)&data_f->x;
	uint8_t *y_ptr = (uint8_t *)&data_f->y;
	uint8_t *z_ptr = (uint8_t *)&data_f->z;

	this->DATASET[addr++] = x_ptr[0];
	this->DATASET[addr++] = x_ptr[1];
	this->DATASET[addr++] = x_ptr[2];
	this->DATASET[addr++] = x_ptr[3];

	this->DATASET[addr++] = y_ptr[0];
	this->DATASET[addr++] = y_ptr[1];
	this->DATASET[addr++] = y_ptr[2];
	this->DATASET[addr++] = y_ptr[3];

	this->DATASET[addr++] = z_ptr[0];
	this->DATASET[addr++] = z_ptr[1];
	this->DATASET[addr++] = z_ptr[2];
	this->DATASET[addr++] = z_ptr[3];
}


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
