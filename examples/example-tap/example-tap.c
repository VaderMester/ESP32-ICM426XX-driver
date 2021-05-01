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

#include "example-tap.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the icm426xx object */
static struct inv_icm426xx icm_driver;

/* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
extern  RINGBUFFER(timestamp_buffer, 64, uint64_t);

/* Variable to keep track of the ODR (required to compute the duration between tap for double tap) */
static int icm_odr_ms;

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


int ConfigureInvDevice(ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq, uint8_t min_jerk_thr)
{
	int rc = 0;
	inv_icm426xx_tap_parameters_t tap_inputs;
	inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};
	
	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, acc_freq);
	
	switch (acc_freq) {
		case ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ:  icm_odr_ms = 1; break;
		case ICM426XX_ACCEL_CONFIG0_ODR_500_HZ: icm_odr_ms = 2; break;
		case ICM426XX_ACCEL_CONFIG0_ODR_200_HZ: icm_odr_ms = 5; break;
		default:
			INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : The tap feature requires the accel to run at 200Hz or faster");
			return -1;
	}
	
	rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);

	/* Get the default parameters for the TAP feature */
	rc |= inv_icm426xx_init_tap_parameters_struct(&icm_driver, &tap_inputs);
	
	/* Configure the programmable parameter */
	tap_inputs.min_jerk_thr = min_jerk_thr;
	
	/* Initialize TAP */
	rc |= inv_icm426xx_configure_tap_parameters(&icm_driver, &tap_inputs);
	rc |= inv_icm426xx_enable_tap(&icm_driver);

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

/* std */
#include <stdio.h>
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
	 * Read TAP interrupt status 
	 */
	rc = inv_icm426xx_read_reg(&icm_driver, MPUREG_INT_STATUS3, 1, &int_status3);
	if(rc != INV_ERROR_SUCCESS)
		return rc;
	
	if(int_status3 & (BIT_INT_STATUS3_TAP_DET)) {
		inv_icm426xx_tap_data_t tap_data;
		rc |= inv_icm426xx_get_tap_data(&icm_driver, &tap_data);
		
		char string_axis[7];
		if (tap_data.tap_axis == ICM426XX_APEX_DATA4_TAP_AXIS_X)
			strcpy(string_axis, "X-axis");
		else if (tap_data.tap_axis == ICM426XX_APEX_DATA4_TAP_AXIS_Y)
			strcpy(string_axis, "Y-axis");
		else if (tap_data.tap_axis == ICM426XX_APEX_DATA4_TAP_AXIS_Z)
			strcpy(string_axis, "Z-axis");
		else {
			INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : Unexpected axis");
		}

		char string_dir[9];
		if (tap_data.tap_dir == ICM426XX_APEX_DATA4_TAP_DIR_POSITIVE)
			strcpy(string_dir, "positive");
		else if (tap_data.tap_dir == ICM426XX_APEX_DATA4_TAP_DIR_NEGATIVE)
			strcpy(string_dir, "negative");
		else {
			INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : Unexpected direction");
		}

		if (tap_data.tap_num == ICM426XX_APEX_DATA4_TAP_NUM_SINGLE) {
			INV_MSG(INV_MSG_LEVEL_INFO, "%llu: Single Tap detected on %s (%s direction)", irq_timestamp, &string_axis, &string_dir);
		} else if (tap_data.tap_num == ICM426XX_APEX_DATA4_TAP_NUM_DOUBLE) {
			/* Read double tap timing */
			int double_tap_time = (tap_data.double_tap_timing * 16) * icm_odr_ms; 

			INV_MSG(INV_MSG_LEVEL_INFO, "%llu: Double Tap detected on %s (%s direction) with %d ms between tap", irq_timestamp, &string_axis, &string_dir, double_tap_time);
		}
	}
	
	return rc;
}
