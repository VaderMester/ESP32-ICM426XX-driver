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

#include "example-smd.h"

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


int ConfigureInvDevice(uint8_t is_low_noise_mode, ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g, ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq)
{
	int rc = 0;
	inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};
	
	rc |= inv_icm426xx_set_accel_fsr(&icm_driver, acc_fsr_g);
	
	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, acc_freq);
	
	if (is_low_noise_mode)	{
		rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);
	}else {
		/* Set 1x averaging, in order to minimize power consumption (16x by default) */
		rc |= inv_icm426xx_set_accel_lp_avg(&icm_driver, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_1);
		rc |= inv_icm426xx_enable_accel_low_power_mode(&icm_driver);
	}

	/* Configure WOM to compare current sample with the previous sample and to produce signal when all axis exceed 52 mg */
	rc |= inv_icm426xx_configure_smd_wom(&icm_driver, 
		ICM426XX_DEFAULT_WOM_THS_MG, 
		ICM426XX_DEFAULT_WOM_THS_MG, 
		ICM426XX_DEFAULT_WOM_THS_MG, 
		ICM426XX_SMD_CONFIG_WOM_INT_MODE_ANDED,
		ICM426XX_SMD_CONFIG_WOM_MODE_CMP_PREV);

	rc |= inv_icm426xx_enable_smd(&icm_driver);

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
	uint8_t int_status2;
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
	 *  Read SMD interrupt status
	 */
	rc = inv_icm426xx_read_reg(&icm_driver, MPUREG_INT_STATUS2, 1, &int_status2);
	if(rc != INV_ERROR_SUCCESS)
		return rc;
        
	if(int_status2 & (BIT_INT_STATUS2_SMD_INT)) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: SMD detected, disabling it", (uint32_t)irq_timestamp);
		return inv_icm426xx_disable_smd(&icm_driver);
	}
	
	return rc;
}	

