/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
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

#include "example-raise-to-wake.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"
#include "timer.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Icm426xx driver object */
static struct inv_icm426xx icm_driver;

/* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
extern  RINGBUFFER(timestamp_buffer, 64, uint64_t);


/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Init device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx");

	rc = inv_icm426xx_init(&icm_driver, icm_serif, NULL);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}
	
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
	
	RINGBUFFER_CLEAR(&timestamp_buffer);
	return rc;
}


int ConfigureInvDevice(ICM426XX_APEX_CONFIG4_R2W_SLEEP_TIME_OUT_t r2w_sleep_time_out,
					   ICM426XX_APEX_CONFIG5_R2W_MOUNTING_MATRIX_t r2w_mounting_matrix,
					   ICM426XX_APEX_CONFIG6_R2W_SLEEP_GEST_DELAY_t r2w_sleep_gest_delay,
					   ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_t power_mode)
{
	int rc = 0;
	inv_icm426xx_apex_parameters_t apex_inputs;
	inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};
	
	/* DMP ODR : 25Hz */
	/* Enable accelerometer to feed the APEX Pedometer algorithm */
	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, ICM426XX_ACCEL_CONFIG0_ODR_25_HZ);

	/* Set 1x averaging, in order to minimize power consumption (16x by default) */
	rc |= inv_icm426xx_set_accel_lp_avg(&icm_driver, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_1);

	rc |= inv_icm426xx_enable_accel_low_power_mode(&icm_driver);

	/* Get the default parameters for the APEX features */
	rc |= inv_icm426xx_init_apex_parameters_struct(&icm_driver, &apex_inputs);
	
	/* Configure raise to wake parameters */
	apex_inputs.r2w_sleep_time_out = r2w_sleep_time_out;
	apex_inputs.r2w_mounting_matrix = r2w_mounting_matrix;
	apex_inputs.r2w_gest_delay = r2w_sleep_gest_delay;

	/* Configure the programmable parameter for Low Power mode (WoM+Pedometer) or Normal mode */
	if (power_mode == ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_EN) {
		/* Configure WOM to compare current sample with the previous sample and to produce signal when all axis exceed 52 mg */
		rc |= inv_icm426xx_configure_smd_wom(&icm_driver, 
			ICM426XX_DEFAULT_WOM_THS_MG, 
			ICM426XX_DEFAULT_WOM_THS_MG, 
			ICM426XX_DEFAULT_WOM_THS_MG, 
			ICM426XX_SMD_CONFIG_WOM_INT_MODE_ANDED,
			ICM426XX_SMD_CONFIG_WOM_MODE_CMP_PREV);

		/* Enable WOM to wake-up the DMP once it goes in power save mode */
		rc |= inv_icm426xx_enable_wom(&icm_driver); /* Enable WOM and disable fifo threshold */

		/* Warning : the ICM426XX_APEX_CONFIG1_DMP_POWER_SAVE_TIME_t value must be superior to the ICM426XX_APEX_CONFIG4_R2W_SLEEP_TIME_OUT_t value */
		apex_inputs.power_save_time = ICM426XX_APEX_CONFIG1_DMP_POWER_SAVE_TIME_SEL_12S;

		apex_inputs.power_save = ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_EN;
	} else
		apex_inputs.power_save = ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_DIS;

	/* Initializes APEX features */
	rc |= inv_icm426xx_configure_apex_parameters(&icm_driver, &apex_inputs);
	rc |= inv_icm426xx_set_apex_frequency(&icm_driver, ICM426XX_APEX_CONFIG0_DMP_ODR_25Hz);

	/* Enable the r2w */
	rc |= inv_icm426xx_enable_apex_r2w(&icm_driver);

	/* Disable fifo threshold, data ready and WOM interrupts INT1 */
	inv_icm426xx_get_config_int1(&icm_driver, &config_int);
	config_int.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_WOM_X = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_WOM_Y = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_WOM_Z = INV_ICM426XX_DISABLE;
	inv_icm426xx_set_config_int1(&icm_driver, &config_int);

	/* Disable fifo threshold, data ready and WOM interrupts IBI */
	inv_icm426xx_get_config_ibi(&icm_driver, &config_int);
	config_int.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_WOM_X = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_WOM_Y = INV_ICM426XX_DISABLE;
	config_int.INV_ICM426XX_WOM_Z = INV_ICM426XX_DISABLE;
	inv_icm426xx_set_config_ibi(&icm_driver, &config_int);
	
	return rc;
}


int GetDataFromInvDevice(void)
{
	uint8_t int_status3;
	uint64_t irq_timestamp = 0;
	int rc;
	
	/*
	 * Extract the timestamp that was buffered when current packet IRQ fired. See 
	 * ext_interrupt_cb() in main.c for more details.
	 * As timestamp buffer is filled in interrupt handler, we should pop it with
	 * interrupts disabled to avoid any concurrent access.
	 */
	inv_disable_irq();
	if (!RINGBUFFER_EMPTY(&timestamp_buffer))
		RINGBUFFER_POP(&timestamp_buffer, &irq_timestamp);
	inv_enable_irq();
	
	/* 
	 * Read r2w interrupt status
	 */
	rc = inv_icm426xx_read_reg(&icm_driver, MPUREG_INT_STATUS3, 1, &int_status3);
	if(rc != INV_ERROR_SUCCESS)
		return rc;
	
	if(int_status3 & (BIT_INT_STATUS3_WAKE_DET))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: WAKE event detected", irq_timestamp);

	if(int_status3 & (BIT_INT_STATUS3_SLEEP_DET))
	INV_MSG(INV_MSG_LEVEL_INFO, "%u: SLEEP event detected", irq_timestamp);
	
	return rc;
}	

