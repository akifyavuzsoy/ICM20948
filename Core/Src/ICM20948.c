/*
 * ICM20948.c
 *
 *  Created on: Jan 12, 2025
 *      Author: akifyavuzsoy
 */

#include "ICM20948.h"

/**********************************************************************************************************************************
 * Static Functions:
 **********************************************************************************************************************************/

//static void select_user_bank(userbank ub);
//static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg);

/**********************************************************************************************************************************
 * Main Functions:
 **********************************************************************************************************************************/

/**
* @brief Initializes the ICM-20948 device with default configuration.
* @note This function performs a series of setup operations, including reset, wake-up,
*       clock source configuration, low-pass filter setup, sample rate configuration,
*       and sensor calibration.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_init(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint8_t device_id = 0;
	status = icm20948_who_am_i(hi2c, &device_id);
    // Wait until the device is properly identified
    if(status == HAL_OK)
    {
    	if (device_id == ICM20948_ID)
    	{
    		// Reset and wake up the device
			icm20948_device_reset(hi2c);
			icm20948_wakeup(hi2c);

			// Configure clock source
			icm20948_clock_source(hi2c, 1);

			// Enable Output Data Rate alignment
			icm20948_odr_align_enable(hi2c);

			// Enable SPI slave mode
			icm20948_spi_slave_enable(hi2c);

			// Configure low-pass filters for gyroscope and accelerometer
			icm20948_gyro_low_pass_filter(hi2c, 0);
			icm20948_accel_low_pass_filter(hi2c, 0);

			// Configure sample rate dividers
			icm20948_gyro_sample_rate_divider(hi2c, 0);
			icm20948_accel_sample_rate_divider(hi2c, 0);

			// Calibrate gyroscope and accelerometer
			icm20948_gyro_calibration(hi2c);
			icm20948_accel_calibration(hi2c);

			// Set full-scale ranges for gyroscope and accelerometer
			icm20948_gyro_full_scale_select(hi2c, _2000dps);
			icm20948_accel_full_scale_select(hi2c, _16g);
    	}
    	else
    	{
    		status = HAL_ERROR;
    	}
    }

    return status;
}

/**
* @brief Initializes the AK09916 magnetometer with default configuration.
* @note This function resets the I2C master, configures the clock frequency, and sets
*       the AK09916 magnetometer to the desired operation mode.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef ak09916_init(I2C_HandleTypeDef *hi2c)
{
	uint8_t device_id = 0;
	HAL_StatusTypeDef status;

    // Reset and enable the I2C master
    icm20948_i2c_master_reset(hi2c);
    icm20948_i2c_master_enable(hi2c);

    // Configure I2C master clock frequency
    icm20948_i2c_master_clk_frq(hi2c, 7);

    // Wait until the magnetometer is properly identified
    status = ak09916_who_am_i(hi2c, &device_id);
    if(status == HAL_OK)
    {
    	if (device_id == AK09916_ID)
    	{
    		// Reset the magnetometer
			ak09916_soft_reset(hi2c);

			// Configure the magnetometer to continuous measurement mode at 100Hz
			ak09916_operation_mode_setting(hi2c, continuous_measurement_100hz);
    	}
    	else
		{
			status = HAL_ERROR;
		}
    }

    return status;
}

/**
* @brief Reads gyroscope data (x, y, z) from the ICM-20948 device.
* @note Reads 6 bytes from the GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L,
*       GYRO_ZOUT_H, and GYRO_ZOUT_L registers, and converts them to floating-point values.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param data: Pointer to an axises struct where the gyroscope data will be stored.
* @retval None
*/
void icm20948_gyro_read(I2C_HandleTypeDef *hi2c, axises* data)
{
    uint8_t raw_data[6];

    // Read 6 bytes from the gyroscope data registers
    HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_GYRO_XOUT_H, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY);

    // Convert raw data to signed 16-bit integers and scale to floating-point
    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]);

}

/**
* @brief Reads accelerometer data (x, y, z) from the ICM-20948 device.
* @note Reads 6 bytes from the ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
*       ACCEL_ZOUT_H, and ACCEL_ZOUT_L registers, and converts them to floating-point values.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param data: Pointer to an axises struct where the accelerometer data will be stored.
* @retval None
*/
void icm20948_accel_read(I2C_HandleTypeDef *hi2c, axises* data)
{
    uint8_t raw_data[6];

    // Read 6 bytes from the accelerometer data registers
    HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_ACCEL_XOUT_H, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY);

    // Convert raw data to signed 16-bit integers and scale to floating-point
    data->x = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    data->y = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    data->z = (int16_t)((raw_data[4] << 8) | raw_data[5]) + accel_scale_factor;
    // Add scale factor because calibration function offset gravity acceleration.
}

/**
* @brief Reads magnetic field data (x, y, z) from the AK09916 magnetometer.
* @note Reads 6 bytes from the measurement registers and converts them to floating-point values.
* @param data: Pointer to an axises struct where the magnetic field data will be stored.
* @retval bool: Returns true if data is successfully read, false otherwise.
*/
bool ak09916_mag_read(I2C_HandleTypeDef *hi2c, axises* data)
{
    uint8_t raw_data[6];
    uint8_t status;
    const uint8_t STATUS_DATA_READY = 0x01; // Bit 0: Data Ready status
    const uint8_t STATUS_OVERFLOW = 0x08; // Bit 0: Data Ready status

    // Check if magnetic data is ready
    if (HAL_I2C_Mem_Read(hi2c, (MAG_SLAVE_ADDR << 1), MAG_ST1, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    // If data is not ready, return false
    if ((status & STATUS_DATA_READY) == 0) {
        return false;
    }

    // Read 6 bytes of magnetic field data (XOUT_L to ZOUT_H)
    if (HAL_I2C_Mem_Read(hi2c, (MAG_SLAVE_ADDR << 1), MAG_HXL, I2C_MEMADD_SIZE_8BIT, raw_data, 6, HAL_MAX_DELAY) != HAL_OK) {
        return false;
    }

    // Check if magnetic data is overflow
	if (HAL_I2C_Mem_Read(hi2c, (MAG_SLAVE_ADDR << 1), MAG_ST2, I2C_MEMADD_SIZE_8BIT, &status, 1, HAL_MAX_DELAY) != HAL_OK) {
		return false;
	}

	// If data is not ready, return false
	if ((status & STATUS_OVERFLOW)) {
		return false;
	}

    // Convert raw data to signed 16-bit integers
	data->x = (int16_t)((raw_data[1] << 8) | raw_data[0]);
	data->y = (int16_t)((raw_data[3] << 8) | raw_data[2]);
	data->z = (int16_t)((raw_data[5] << 8) | raw_data[4]);

    return true;
}

/**
* @brief Reads gyroscope data (x, y, z) in dps (degrees per second) from the ICM-20948 device.
* @note Reads 6 bytes from the GYRO_XOUT_H, GYRO_XOUT_L, GYRO_YOUT_H, GYRO_YOUT_L,
*       GYRO_ZOUT_H, and GYRO_ZOUT_L registers, and converts them to floating-point values in dps.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param data: Pointer to an axises struct where the gyroscope data will be stored.
* @retval None
*/
void icm20948_gyro_read_dps(I2C_HandleTypeDef *hi2c, axises* data)
{
    icm20948_gyro_read(hi2c, data);

    // Apply the scale factor to convert to dps
    data->x /= gyro_scale_factor;
    data->y /= gyro_scale_factor;
    data->z /= gyro_scale_factor;
}

/**
* @brief Reads accelerometer data (x, y, z) in g (gravitational units) from the ICM-20948 device.
* @note Reads 6 bytes from the ACCEL_XOUT_H, ACCEL_XOUT_L, ACCEL_YOUT_H, ACCEL_YOUT_L,
*       ACCEL_ZOUT_H, and ACCEL_ZOUT_L registers, and converts them to floating-point values in g.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param data: Pointer to an axises struct where the accelerometer data will be stored.
* @retval None
*/
void icm20948_accel_read_g(I2C_HandleTypeDef *hi2c, axises* data)
{
    icm20948_accel_read(hi2c, data);

    // Apply the scale factor to convert to g
    data->x /= accel_scale_factor;
    data->y /= accel_scale_factor;
    data->z /= accel_scale_factor;
}

/**
* @brief Reads magnetic field data (x, y, z) in µT (microtesla) from the AK09916 magnetometer.
* @note Reads 6 bytes from the magnetic field data registers and converts them to floating-point values in µT.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param data: Pointer to an axises struct where the magnetic field data will be stored.
* @retval bool: Returns true if data is successfully read, false otherwise.
*/
bool ak09916_mag_read_uT(I2C_HandleTypeDef *hi2c, axises* data)
{
    axises temp;
    bool new_data = ak09916_mag_read(hi2c, &temp);
    if(!new_data)	return false;

    // Apply the scale factor to convert to µT
    const float mag_scale_factor = 0.15f; // Magnetic scale factor in µT/LSB (datasheet-dependent)
    data->x = (float)(temp.x * mag_scale_factor);
    data->y = (float)(temp.y * mag_scale_factor);
    data->z = (float)(temp.z * mag_scale_factor);

    return true;
}


/**********************************************************************************************************************************
 * Sub Functions:
 **********************************************************************************************************************************/

/**
* @brief Reads the WHO_AM_I register of the ICM-20948 device to verify its identity.
* @note Expects a specific response (e.g., 0xEA) from the WHO_AM_I register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param device_id: Pointer to a variable where the read device ID will be stored.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *device_id)
{
    HAL_StatusTypeDef status;
    //uint8_t reg = B0_WHO_AM_I;
    uint8_t id;

    // WHO_AM_I register'ını okuma
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        *device_id = id; // Okunan değeri döndür
    }

    return status;
}

/**
* @brief Reads the WHO_AM_I register of the AK09916 magnetometer to verify its identity.
* @note Expects a response of 0x48 from the WHO_AM_I register if the device is present.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param device_id: Pointer to a variable where the read device ID will be stored.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef ak09916_who_am_i(I2C_HandleTypeDef *hi2c, uint8_t *device_id)
{
	HAL_StatusTypeDef status;
	//uint8_t reg = B0_WHO_AM_I;
	uint8_t id;

	status = HAL_I2C_Mem_Read(hi2c, (MAG_SLAVE_ADDR << 1), B0_WHO_AM_I, I2C_MEMADD_SIZE_8BIT, &id, 1, HAL_MAX_DELAY);

	if (status == HAL_OK) {
		*device_id = id; // Okunan değeri döndür
	}

	return status;
}

/**
* @brief Resets the ICM-20948 device and restarts it with default settings.
* @note Writes 0x80 to the PWR_MGMT_1 register to reset and configure the device.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_device_reset(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	//uint8_t reg_value = DEVICE_RESET_BIT;

	status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, (uint8_t*)DEVICE_RESET_BIT, 1, HAL_MAX_DELAY);

	if (status == HAL_OK) {
		HAL_Delay(100); // Reset işlemi sonrası cihazın yeniden başlatılması için kısa bir gecikme
	}

	return status;
}

/**
* @brief Performs a software reset on the AK09916 magnetometer.
* @note Writes 0x01 to the CNTL3 register to reset the device.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef ak09916_soft_reset(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	//uint8_t reg_value = DEVICE_RESET_BIT;

	// Write the reset command to the CNTL3 register
	status = HAL_I2C_Mem_Write(hi2c, (MAG_SLAVE_ADDR << 1), MAG_CNTL3, I2C_MEMADD_SIZE_8BIT, (uint8_t*)DEVICE_RESET_BIT, 1, HAL_MAX_DELAY);

	if (status == HAL_OK) {
		HAL_Delay(10); // Wait for the reset to complete
	}

	return status;
}

/**
* @brief Wakes up the ICM-20948 from sleep mode.
* @note Clears the SLEEP bit in the PWR_MGMT_1 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_wakeup(I2C_HandleTypeDef *hi2c)
{
	HAL_StatusTypeDef status;
	uint8_t reg_value;

	// Read the current value of PWR_MGMT_1 register
	status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

	if (status == HAL_OK) {
		reg_value &= ~0x40; // Clear the SLEEP bit (bit 6)
		// Write the updated value back to PWR_MGMT_1 register
		status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
	}

	return status;
}

/**
* @brief Puts the ICM-20948 into sleep mode.
* @note Sets the SLEEP bit in the PWR_MGMT_1 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_sleep(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Read the current value of PWR_MGMT_1 register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    if (status == HAL_OK) {
        reg_value |= 0x40; // Set the SLEEP bit (bit 6)
        // Write the updated value back to PWR_MGMT_1 register
        status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    }

    return status;
}

/**
* @brief Enables SPI slave mode on the ICM-20948 device.
* @note Sets the SPI_EN bit in the USER_CTRL register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_spi_slave_enable(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Read the current value of USER_CTRL register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Set the SPI_EN bit (bit 5)
    reg_value |= (1 << 5);	// TODO: ????

    // Write the updated value back to the USER_CTRL register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
* @brief Resets the I2C master on the ICM-20948 device.
* @note Sets the I2C_MST_RST bit in the USER_CTRL register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_i2c_master_reset(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Read the current value of USER_CTRL register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Set the I2C_MST_RST bit (bit 1)
    reg_value |= (1 << 1);	// TODO: ????

    // Write the updated value back to the USER_CTRL register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
* @brief Enables I2C master mode on the ICM-20948 device.
* @note Sets the I2C_MST_EN bit in the USER_CTRL register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_i2c_master_enable(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Read the current value of USER_CTRL register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Set the I2C_MST_EN bit (bit 5)
    reg_value |= (1 << 5);	// TODO: ????

    // Write the updated value back to the USER_CTRL register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_USER_CTRL, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
* @brief Configures the I2C master clock frequency on the ICM-20948 device.
* @note Writes to the I2C_MST_CTRL register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param clk_frq: Clock frequency divider value (0-15). Refer to the datasheet for specific frequency mapping.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_i2c_master_clk_frq(I2C_HandleTypeDef *hi2c, uint8_t clk_frq)
{
    const uint8_t I2C_MST_CTRL_REG_ADDR = 0x24;	// TODO: ????

    // Ensure clk_frq is within valid range (0-15)
    if (clk_frq > 15) {
        return HAL_ERROR;
    }

    // Write the clock frequency divider to the I2C_MST_CTRL register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), I2C_MST_CTRL_REG_ADDR, I2C_MEMADD_SIZE_8BIT, &clk_frq, 1, HAL_MAX_DELAY);
}

/**
* @brief Configures the clock source for the ICM-20948 device.
* @note Sets the CLKSEL bits in the PWR_MGMT_1 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param clk_source: Clock source value (0-7). Refer to the datasheet for valid values.
*                    - 0: Internal 20 MHz oscillator.
*                    - 1: Auto selects best available clock source.
*                    - 2-5: PLL with gyroscope as reference.
*                    - 6: Reserved.
*                    - 7: Stops the clock and keeps timing reset.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_clock_source(I2C_HandleTypeDef *hi2c, uint8_t clk_source)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Ensure clk_source is within valid range (0-7)
    if (clk_source > 7) {
        return HAL_ERROR;
    }

    // Read the current value of PWR_MGMT_1 register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Clear the CLKSEL bits (bits 2:0) and set the new clock source
    reg_value &= ~0x07;        // Clear bits 2:0
    reg_value |= (clk_source & 0x07); // Set new clock source

    // Write the updated value back to PWR_MGMT_1 register
    status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B0_PWR_MGMT_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);

    return status;
}

/**
* @brief Enables Output Data Rate (ODR) alignment on the ICM-20948 device.
* @note Sets the ODR_ALIGN_EN bit in the ODR_ALIGN register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_odr_align_enable(I2C_HandleTypeDef *hi2c)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Read the current value of the ODR_ALIGN register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B2_ODR_ALIGN_EN, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Set the ODR_ALIGN_EN bit (bit 0)
    reg_value |= 0x01;

    // Write the updated value back to the ODR_ALIGN register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_ODR_ALIGN_EN, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
* @brief Configures the low-pass filter for the gyroscope on the ICM-20948 device.
* @note Updates the GYRO_DLPFCFG bits in the GYRO_CONFIG_1 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param config: Filter configuration value (0-7). Refer to the datasheet for valid configurations.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_gyro_low_pass_filter(I2C_HandleTypeDef *hi2c, uint8_t config)
{
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Ensure config value is within the valid range (0-7)
    if (config > 7) {
        return HAL_ERROR;
    }

    // Read the current value of the GYRO_CONFIG_1 register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B2_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Clear the GYRO_DLPFCFG bits (bits 2:0) and set the new configuration
    reg_value &= ~0x07;        // Clear bits 2:0
    reg_value |= (config & 0x07); // Set new configuration

    // Write the updated value back to the GYRO_CONFIG_1 register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
* @brief Configures the low-pass filter for the accelerometer on the ICM-20948 device.
* @note Updates the ACCEL_DLPFCFG bits in the ACCEL_CONFIG_2 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param config: Filter configuration value (0-7). Refer to the datasheet for valid configurations.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_accel_low_pass_filter(I2C_HandleTypeDef *hi2c, uint8_t config)
 {
    HAL_StatusTypeDef status;
    uint8_t reg_value;

    // Ensure config value is within the valid range (0-7)
    if (config > 7) {
        return HAL_ERROR;
    }

    // Read the current value of the ACCEL_CONFIG_2 register
    status = HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B2_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Clear the ACCEL_DLPFCFG bits (bits 2:0) and set the new configuration
    reg_value &= ~0x07;        // Clear bits 2:0
    reg_value |= (config & 0x07); // Set new configuration

    // Write the updated value back to the ACCEL_CONFIG_2 register
    return HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
}

/**
* @brief Configures the sample rate divider for the gyroscope on the ICM-20948 device.
* @note Writes the divider value to the GYRO_SMPLRT_DIV register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param divider: Sample rate divider value (0-255). Refer to the datasheet for valid configurations.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_gyro_sample_rate_divider(I2C_HandleTypeDef *hi2c, uint8_t divider)
{
    HAL_StatusTypeDef status;

    // Write the divider value to the GYRO_SMPLRT_DIV register
    status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_GYRO_SMPLRT_DIV, I2C_MEMADD_SIZE_8BIT, &divider, 1, HAL_MAX_DELAY);

    return status;
}

/**
* @brief Configures the sample rate divider for the accelerometer on the ICM-20948 device.
* @note Writes the divider value to the ACCEL_SMPLRT_DIV_1 and ACCEL_SMPLRT_DIV_2 registers.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param divider: Sample rate divider value (0-65535). Refer to the datasheet for valid configurations.
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef icm20948_accel_sample_rate_divider(I2C_HandleTypeDef *hi2c, uint16_t divider)
{
    HAL_StatusTypeDef status;
    uint8_t reg_values[2];

    // Split the 16-bit divider into two 8-bit values
    reg_values[0] = (uint8_t)(divider >> 8); // Higher byte
    reg_values[1] = (uint8_t)(divider & 0xFF); // Lower byte

    // Write the higher byte to the ACCEL_SMPLRT_DIV_1 register
    status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_ACCEL_SMPLRT_DIV_1, I2C_MEMADD_SIZE_8BIT, &reg_values[0], 1, HAL_MAX_DELAY);
    if (status != HAL_OK) {
        return status;
    }

    // Write the lower byte to the ACCEL_SMPLRT_DIV_2 register
    status = HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_ACCEL_SMPLRT_DIV_2, I2C_MEMADD_SIZE_8BIT, &reg_values[1], 1, HAL_MAX_DELAY);

    return status;
}

/**
* @brief Configures the operation mode for the AK09916 magnetometer.
* @note Writes the operation mode value to the CNTL2 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param mode: Operation mode value. Refer to the datasheet for valid modes.
*              Common modes:
*              - 0x00: Power-down mode
*              - 0x01: Single measurement mode
*              - 0x02 to 0x05: Continuous measurement modes (different ODRs)
*              - 0x08: Self-test mode
* @retval HAL_StatusTypeDef: HAL status indicating the result of the operation.
*/
HAL_StatusTypeDef ak09916_operation_mode_setting(I2C_HandleTypeDef *hi2c, operation_mode mode)
{
    HAL_StatusTypeDef status;

    // Write the mode value to the CNTL2 register
    status = HAL_I2C_Mem_Write(hi2c, (MAG_SLAVE_ADDR << 1), MAG_CNTL2, I2C_MEMADD_SIZE_8BIT, &mode, 1, HAL_MAX_DELAY);
    HAL_Delay(100);

    return status;
}

/**
* @brief Calibrates the gyroscope of the ICM-20948 device.
* @note Computes gyroscope biases by averaging multiple samples and writes these biases
*       to the hardware gyroscope offset registers.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval None
*/
void icm20948_gyro_calibration(I2C_HandleTypeDef *hi2c) {
    axises temp;
    int32_t gyro_bias[3] = {0};
    uint8_t gyro_offset[6] = {0};

    // Collect 100 samples to compute the bias
    for (int i = 0; i < 100; i++) {
        icm20948_gyro_read(hi2c, &temp);
        gyro_bias[0] += temp.x;
        gyro_bias[1] += temp.y;
        gyro_bias[2] += temp.z;
        HAL_Delay(10); // Add small delay between samples
    }

    // Compute the average bias
    gyro_bias[0] /= 100;
    gyro_bias[1] /= 100;
    gyro_bias[2] /= 100;

    // Convert biases to the format required by the hardware (LSB)
    gyro_offset[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;
    gyro_offset[1] = (-gyro_bias[0] / 4) & 0xFF;
    gyro_offset[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    gyro_offset[3] = (-gyro_bias[1] / 4) & 0xFF;
    gyro_offset[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    gyro_offset[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Write biases to gyroscope offset registers
    HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_XG_OFFS_USRH, I2C_MEMADD_SIZE_8BIT, gyro_offset, 6, HAL_MAX_DELAY);
}

/**
* @brief Calibrates the accelerometer of the ICM-20948 device.
* @note Computes accelerometer biases by averaging multiple samples and writes these biases
*       to the hardware accelerometer offset registers.
* @param hi2c: Pointer to the I2C handle used for communication.
* @retval None
*/
void icm20948_accel_calibration(I2C_HandleTypeDef *hi2c) {
    axises temp;
    uint8_t* temp2;
    uint8_t* temp3;
    uint8_t* temp4;

    int32_t accel_bias[3] = {0};
    int32_t accel_bias_reg[3] = {0};
    uint8_t accel_offset[6] = {0};

    // Collect 100 samples to compute the bias
    for (int i = 0; i < 100; i++) {
        icm20948_accel_read(hi2c, &temp);
        accel_bias[0] += temp.x;
        accel_bias[1] += temp.y;
        accel_bias[2] += temp.z;
        HAL_Delay(10); // Add small delay between samples
    }

    // Compute the average bias
    accel_bias[0] /= 100;
    accel_bias[1] /= 100;
    accel_bias[2] /= 100;

    uint8_t mask_bit[3] = {0, 0, 0};

    HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B1_XA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &temp2, 2, HAL_MAX_DELAY);
    accel_bias_reg[0] = (int32_t)(temp2[0] << 8 | temp2[1]);
    mask_bit[0] = temp2[1] & 0x01;

    HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B1_YA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &temp3, 2, HAL_MAX_DELAY);
    accel_bias_reg[1] = (int32_t)(temp3[0] << 8 | temp3[1]);
	mask_bit[1] = temp3[1] & 0x01;

	HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B1_ZA_OFFS_H, I2C_MEMADD_SIZE_8BIT, &temp4, 2, HAL_MAX_DELAY);
	accel_bias_reg[2] = (int32_t)(temp4[0] << 8 | temp4[1]);
	mask_bit[2] = temp4[1] & 0x01;

	accel_bias_reg[0] -= (accel_bias[0] / 8);
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);

	accel_offset[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	accel_offset[1] = (accel_bias_reg[0])      & 0xFE;
	accel_offset[1] = accel_offset[1] | mask_bit[0];

	accel_offset[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	accel_offset[3] = (accel_bias_reg[1])      & 0xFE;
	accel_offset[3] = accel_offset[3] | mask_bit[1];

	accel_offset[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	accel_offset[5] = (accel_bias_reg[2])      & 0xFE;
	accel_offset[5] = accel_offset[5] | mask_bit[2];

    // Write biases to accelerometer offset registers
    HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B1_XA_OFFS_H, I2C_MEMADD_SIZE_8BIT, accel_offset[0], 2, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B1_YA_OFFS_H, I2C_MEMADD_SIZE_8BIT, accel_offset[2], 2, HAL_MAX_DELAY);
    HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B1_ZA_OFFS_H, I2C_MEMADD_SIZE_8BIT, accel_offset[4], 2, HAL_MAX_DELAY);
}

/**
* @brief Configures the full-scale range for the gyroscope on the ICM-20948 device.
* @note Updates the GYRO_FS_SEL bits in the GYRO_CONFIG_1 register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param full_scale: Full-scale range selection value.
*                    - _250dps: ±250 degrees per second
*                    - _500dps: ±500 degrees per second
*                    - _1000dps: ±1000 degrees per second
*                    - _2000dps: ±2000 degrees per second
* @retval None
*/
void icm20948_gyro_full_scale_select(I2C_HandleTypeDef *hi2c, gyro_full_scale full_scale)
{
    uint8_t reg_value;

    // Read the current value of the GYRO_CONFIG_1 register
    if (HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B2_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY) == HAL_OK) {
        // Clear the GYRO_FS_SEL bits (bits 3:2) and set the new full-scale range
        //reg_value &= ~0x0C;           // Clear bits 3:2
        //reg_value |= (full_scale << 2); // Set new full-scale range

        switch (full_scale)
        {
        	case _250dps:
        		reg_value |= 0x00;
        		gyro_scale_factor = 131.0;
        		break;
        	case _500dps:
        		reg_value |= 0x02;
				gyro_scale_factor = 65.5;
				break;
        	case _1000dps:
				reg_value |= 0x04;
				gyro_scale_factor = 32.8;
				break;
			case _2000dps:
				reg_value |= 0x06;
				gyro_scale_factor = 16.4;
				break;
        }

        // Write the updated value back to the GYRO_CONFIG_1 register
        HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_GYRO_CONFIG_1, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    }
}

/**
* @brief Configures the full-scale range for the accelerometer on the ICM-20948 device.
* @note Updates the ACCEL_FS_SEL bits in the ACCEL_CONFIG register.
* @param hi2c: Pointer to the I2C handle used for communication.
* @param full_scale: Full-scale range selection value.
*                    - _2g: ±2g
*                    - _4g: ±4g
*                    - _8g: ±8g
*                    - _16g: ±16g
* @retval None
*/
void icm20948_accel_full_scale_select(I2C_HandleTypeDef *hi2c, accel_full_scale full_scale)
{
    uint8_t reg_value;

    // Read the current value of the ACCEL_CONFIG register
    if (HAL_I2C_Mem_Read(hi2c, (ICM20948_I2C_ADDR << 1), B2_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY) == HAL_OK) {
        // Clear the ACCEL_FS_SEL bits (bits 3:2) and set the new full-scale range
        //reg_value &= ~0x0C;           // Clear bits 3:2
        //reg_value |= (full_scale << 2); // Set new full-scale range
    	switch (full_scale)
    	{
			case _2g:
				reg_value |= 0x00;
				accel_scale_factor = 16384;
				break;
			case _4g:
				reg_value |= 0x02;
				accel_scale_factor = 8192;
				break;
			case _8g:
				reg_value |= 0x04;
				accel_scale_factor = 4096;
				break;
			case _16g:
				reg_value |= 0x06;
				accel_scale_factor = 2048;
				break;
    	}

        // Write the updated value back to the ACCEL_CONFIG register
        HAL_I2C_Mem_Write(hi2c, (ICM20948_I2C_ADDR << 1), B2_ACCEL_CONFIG, I2C_MEMADD_SIZE_8BIT, &reg_value, 1, HAL_MAX_DELAY);
    }
}






/**********************************************************************************************************************************
 * Static Functions:
 **********************************************************************************************************************************/

/**
* @brief
* Note
* @param hi2c:
* @param device_id:
* @retval
*/
//static void select_user_bank(userbank ub)
//{
//	uint8_t write_reg[2];
//	write_reg[0] = WRITE | REG_BANK_SEL;
//	write_reg[1] = ub;
//
//	HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_ID, write_reg, 2, 10);
//
//}

/**
* @brief
* Note
* @param hi2c:
* @param device_id:
* @retval
*/
//static uint8_t read_single_icm20948_reg(userbank ub, uint8_t reg)
//{
//	uint8_t read_reg = READ | reg;
//	uint8_t reg_val;
//	select_user_bank(ub);
//
//	HAL_I2C_Master_Transmit(ICM20948_I2C, ICM20948_ID, &read_reg, 1, 1000);
//	HAL_I2C_Master_Receive(ICM20948_I2C, ICM20948_ID, &reg_val, 1, 1000);
//
//	return reg_val;
//
//}

