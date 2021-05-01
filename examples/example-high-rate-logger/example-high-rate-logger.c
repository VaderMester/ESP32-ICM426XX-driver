/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */

#include "example-high-rate-logger.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/Drivers/Icm426xx/Icm426xxExtFunc.h"

/* board driver */
#include "common.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the icm426xx object */
static struct inv_icm426xx icm_driver;

/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx");
	
	rc = inv_icm426xx_init(&icm_driver, icm_serif, NULL);

	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_INFO, "Check Icm426xx whoami value");
	
	rc = inv_icm426xx_get_who_am_i(&icm_driver, &who_am_i);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value.");
		return rc;
	}
	
	if(who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	return rc;
}


int ConfigureInvDevice()
{
	int rc = 0;
	uint8_t data;

	/* Set Pulse to support high rates */
	rc |= inv_icm426xx_read_reg(&icm_driver, MPUREG_INT_CONFIG1, 1, &data);
	data |= ((uint8_t)ICM426XX_INT_TPULSE_DURATION_8_US) | ((uint8_t)ICM426XX_INT_TDEASSERT_DISABLED);
	rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_INT_CONFIG1, 1, &data);

	/* Disable fifo usage, data will be read from sensors registers*/
	rc |= inv_icm426xx_configure_fifo(&icm_driver, INV_ICM426XX_FIFO_DISABLED);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}	

#if defined(ICM42633) && USE_CLK_IN
	/* 
	 * ICM42633 is a triple interface device. To access CLKIN, AUX2 interface needs to be disabled.
	 * Use INV_ICM426XX_DUAL_INTERFACE mode. The following mode are also compatible:
	 *  - INV_ICM426XX_SINGLE_INTERFACE
	 *  - INV_ICM426XX_DUAL_INTERFACE_SPI4
	 */
	rc |= inv_icm426xx_interface_change_procedure(&icm_driver, INV_ICM426XX_DUAL_INTERFACE);
#endif
	rc |= inv_icm426xx_enable_clkin_rtc(&icm_driver, USE_CLK_IN);

	rc |= inv_icm426xx_set_accel_fsr(&icm_driver, ICM426XX_ACCEL_CONFIG0_FS_SEL_4g);
	rc |= inv_icm426xx_set_gyro_fsr(&icm_driver, ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps);

	if (SENSOR_ODR == HIGH_RATE_8KHZ) {
		rc |= inv_icm426xx_set_accel_frequency(&icm_driver, ICM426XX_ACCEL_CONFIG0_ODR_8_KHZ);
		rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, ICM426XX_GYRO_CONFIG0_ODR_8_KHZ);              
	} else if (SENSOR_ODR == HIGH_RATE_16KHZ) {                              
		rc |= inv_icm426xx_set_accel_frequency(&icm_driver, ICM426XX_ACCEL_CONFIG0_ODR_16_KHZ);
		rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, ICM426XX_GYRO_CONFIG0_ODR_16_KHZ);
	} else if (SENSOR_ODR == HIGH_RATE_32KHZ) {                            
		rc |= inv_icm426xx_set_accel_frequency(&icm_driver, ICM426XX_ACCEL_CONFIG0_ODR_32_KHZ);
		rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, ICM426XX_GYRO_CONFIG0_ODR_32_KHZ);
	}
	
	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);
	rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);

	/* Wait the maximum startup time in case gyro is enbaled */
	inv_icm426xx_sleep_us(ICM426XX_GYR_STARTUP_TIME_US);
		
	return rc;
}