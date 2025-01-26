/*
 * ICM20948.h
 *
 *  Created on: Jan 12, 2025
 *      Author: akifyavuzsoy
 */

#ifndef INC_ICM20948_H_
#define INC_ICM20948_H_

/*
 * Libraries
 *
 * stdbool.h
 * stm32f4xx_hal.h
 * */

#include <stdbool.h>
#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

static float gyro_scale_factor;
static float accel_scale_factor;

/***********************************************************************************************
 * User Configuration
 ***********************************************************************************************/


#define READ							0x80
#define WRITE							0x00


/***********************************************************************************************
 * Typedefs
 ***********************************************************************************************/

typedef enum
{
	ub_0 = 0 << 4,
	ub_1 = 1 << 4,
	ub_2 = 2 << 4,
	ub_3 = 3 << 4
} userbank;

typedef enum
{
	_250dps,
	_500dps,
	_1000dps,
	_2000dps
} gyro_full_scale;

typedef enum
{
	_2g,
	_4g,
	_8g,
	_16g
} accel_full_scale;

typedef struct
{
	float x;
	float y;
	float z;
} axises;

typedef enum
{
	power_down_mode = 0,
	single_measurement_mode = 1,
	continuous_measurement_10hz = 2,
	continuous_measurement_20hz = 4,
	continuous_measurement_50hz = 6,
	continuous_measurement_100hz = 8
} operation_mode;

/***********************************************************************************************
 * Main Functions:
 ***********************************************************************************************/

HAL_StatusTypeDef icm20948_init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ak09916_init(I2C_HandleTypeDef *hi2c);

void icm20948_gyro_read(I2C_HandleTypeDef *hi2c, axises* data);
void icm20948_accel_read(I2C_HandleTypeDef *hi2c, axises* data);
bool ak09916_mag_read(I2C_HandleTypeDef *hi2c, axises* data);
void icm20948_gyro_read_dps(I2C_HandleTypeDef *hi2c, axises* data);
void icm20948_accel_read_g(I2C_HandleTypeDef *hi2c, axises* data);
bool ak09916_mag_read_uT(I2C_HandleTypeDef *hi2c, axises* data);

/***********************************************************************************************
 * Sub Functions:
 ***********************************************************************************************/

HAL_StatusTypeDef icm20948_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *device_id);
HAL_StatusTypeDef ak09916_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *device_id);

HAL_StatusTypeDef icm20948_device_reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef ak09916_soft_reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm20948_wakeup(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm20948_sleep(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef icm20948_spi_slave_enable(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm20948_i2c_master_reset(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm20948_i2c_master_enable(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef icm20948_i2c_master_clk_frq(I2C_HandleTypeDef *hi2c, uint8_t clk_frq);

HAL_StatusTypeDef icm20948_clock_source(I2C_HandleTypeDef *hi2c, uint8_t clk_source);
HAL_StatusTypeDef icm20948_odr_align_enable(I2C_HandleTypeDef *hi2c);

HAL_StatusTypeDef icm20948_gyro_low_pass_filter(I2C_HandleTypeDef *hi2c, uint8_t config);
HAL_StatusTypeDef icm20948_accel_low_pass_filter(I2C_HandleTypeDef *hi2c, uint8_t config);

HAL_StatusTypeDef icm20948_gyro_sample_rate_divider(I2C_HandleTypeDef *hi2c, uint8_t divider);
HAL_StatusTypeDef icm20948_accel_sample_rate_divider(I2C_HandleTypeDef *hi2c, uint16_t divider);

HAL_StatusTypeDef ak09916_operation_mode_setting(I2C_HandleTypeDef *hi2c, operation_mode mode);

void icm20948_gyro_calibration(I2C_HandleTypeDef *hi2c);
void icm20948_accel_calibration(I2C_HandleTypeDef *hi2c);

void icm20948_gyro_full_scale_select(I2C_HandleTypeDef *hi2c, gyro_full_scale full_scale);
void icm20948_accel_full_scale_select(I2C_HandleTypeDef *hi2c, accel_full_scale full_scale);


/***********************************************************************************************
 * ICM-20948 Registers
 ***********************************************************************************************/

#define ICM20948_I2C_ADDR      			0x68
#define ICM20948_ID						0xEA
#define REG_BANK_SEL					0x7F


/***********************************************************************************************
 * USER BANK 0
 ***********************************************************************************************/

#define B0_WHO_AM_I 					0x00
#define B0_USER_CTRL					0x03

#define B0_PWR_MGMT_1					0x06
#define DEVICE_RESET_BIT        		0x01

#define B0_ACCEL_XOUT_H					0x2D
#define B0_ACCEL_XOUT_L					0x2E
#define B0_ACCEL_YOUT_H					0x2F
#define B0_ACCEL_YOUT_L					0x30
#define B0_ACCEL_ZOUT_H					0x31
#define B0_ACCEL_ZOUT_L					0x32
#define B0_GYRO_XOUT_H					0x33
#define B0_GYRO_XOUT_L					0x34
#define B0_GYRO_YOUT_H					0x35
#define B0_GYRO_YOUT_L					0x36
#define B0_GYRO_ZOUT_H					0x37
#define B0_GYRO_ZOUT_L					0x38
#define B0_TEMP_OUT_H					0x39
#define B0_TEMP_OUT_L					0x3A

/***********************************************************************************************
 * USER BANK 1
 ***********************************************************************************************/
#define B1_SELF_TEST_X_GYRO				0x02
#define B1_SELF_TEST_Y_GYRO				0x03
#define B1_SELF_TEST_Z_GYRO				0x04
#define B1_SELF_TEST_X_ACCEL			0x0E
#define B1_SELF_TEST_Y_ACCEL			0x0F
#define B1_SELF_TEST_Z_ACCEL			0x10
#define B1_XA_OFFS_H					0x14
#define B1_XA_OFFS_L					0x15
#define B1_YA_OFFS_H					0x17
#define B1_YA_OFFS_L					0x18
#define B1_ZA_OFFS_H					0x1A
#define B1_ZA_OFFS_L					0x1B
#define B1_TIMEBASE_CORRECTION_PLL		0x28

/***********************************************************************************************
 * USER BANK 2
 ***********************************************************************************************/

#define B2_GYRO_SMPLRT_DIV				0x00
#define B2_GYRO_CONFIG_1				0x01
#define B2_XG_OFFS_USRH					0x03
#define B2_ODR_ALIGN_EN					0x09
#define B2_ACCEL_SMPLRT_DIV_1			0x10
#define B2_ACCEL_SMPLRT_DIV_2			0x11
#define B2_ACCEL_CONFIG					0x14

/***********************************************************************************************
 * AK09916 Registers
 ***********************************************************************************************/
#define AK09916_ID						0x09
#define MAG_SLAVE_ADDR                  0x0C

#define MAG_WIA2						0x01

#define MAG_ST1							0x10
#define MAG_HXL							0x11
#define MAG_ST2							0x18

#define MAG_CNTL2						0x31
#define MAG_CNTL3						0x32


#endif /* INC_ICM20948_H_ */
