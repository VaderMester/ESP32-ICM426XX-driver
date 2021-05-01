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

#include "example-raw-data-registers.h"
 
/* Clock calibration module */
#include "Invn/Helpers/Icm426xx/helperClockCalib.h"

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

/*
 * ICM mounting matrix
 * Coefficients are coded as Q30 integer
 */
#if defined(ICM_FAMILY_CPLUS)
static int32_t icm_mounting_matrix[9] = {  0,        -(1<<30),      0,
                                          (1<<30),     0,           0,
                                           0,          0,          (1<<30) };
#else
static int32_t icm_mounting_matrix[9] = { (1<<30),     0,           0,
                                           0,         (1<<30),      0,
                                           0,          0,          (1<<30) };
#endif

/* --------------------------------------------------------------------------------------
 *  static function declaration
 * -------------------------------------------------------------------------------------- */

static void apply_mounting_matrix(const int32_t matrix[9], int16_t raw[3]);

/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx");
	
	rc = inv_icm426xx_init(&icm_driver, icm_serif, HandleInvDeviceDataRegisters);
	/* Disable fifo usage, data will be read from sensors registers*/
	rc |= inv_icm426xx_configure_fifo(&icm_driver, INV_ICM426XX_FIFO_DISABLED);
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


int ConfigureInvDevice(uint8_t is_low_noise_mode,
                       ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                       ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
                       uint8_t is_rtc_mode)
{
	int rc = 0;

#if defined(ICM42633) 
	/* 
	 * ICM42633 is a triple interface device. To access CLKIN, AUX2 interface needs to be disabled.
	 * Use INV_ICM426XX_DUAL_INTERFACE mode. The following mode are also compatible:
	 *  - INV_ICM426XX_SINGLE_INTERFACE
	 *  - INV_ICM426XX_DUAL_INTERFACE_SPI4
	 */
	if (is_rtc_mode)
		rc |= inv_icm426xx_interface_change_procedure(&icm_driver, INV_ICM426XX_DUAL_INTERFACE);
#endif
	
	rc |= inv_icm426xx_enable_clkin_rtc(&icm_driver, is_rtc_mode);

	rc |= inv_icm426xx_set_accel_fsr(&icm_driver, acc_fsr_g);
	rc |= inv_icm426xx_set_gyro_fsr(&icm_driver, gyr_fsr_dps);
	
	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, acc_freq);
	rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, gyr_freq);
	
	if (is_low_noise_mode)
		rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);
	else
		rc |= inv_icm426xx_enable_accel_low_power_mode(&icm_driver);
	
	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);

	/* Wait Max of ICM426XX_GYR_STARTUP_TIME_US and ICM426XX_ACC_STARTUP_TIME_US*/
	(ICM426XX_GYR_STARTUP_TIME_US > ICM426XX_ACC_STARTUP_TIME_US) ? inv_icm426xx_sleep_us(ICM426XX_GYR_STARTUP_TIME_US) : inv_icm426xx_sleep_us(ICM426XX_ACC_STARTUP_TIME_US);
		
	return rc;
}

int GetDataFromInvDevice(void)
{
	/*
	 * Read data from registers. Callback defined at init time (i.e. 
	 * HandleInvDeviceDataRegisters) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	return inv_icm426xx_get_data_from_registers(&icm_driver);
}

void HandleInvDeviceDataRegisters(inv_icm426xx_sensor_event_t * event)
{
	uint64_t irq_timestamp = 0;	

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
	
	apply_mounting_matrix(icm_mounting_matrix, event->accel);
	apply_mounting_matrix(icm_mounting_matrix, event->gyro);

	/*
	 * Output data on UART link
	 */
	if((event->accel[0] != INVALID_VALUE_FIFO) && (event->gyro[0] != INVALID_VALUE_FIFO))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, %d, %d, %d", (uint32_t)irq_timestamp,
			event->accel[0], event->accel[1], event->accel[2], 
			event->temperature,
			event->gyro[0], event->gyro[1], event->gyro[2]);
	else if(event->gyro[0] != INVALID_VALUE_FIFO)
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: NA, NA, NA, %d, %d, %d, %d", (uint32_t)irq_timestamp, 
			event->temperature,
			event->gyro[0], event->gyro[1], event->gyro[2]);
	else if (event->accel[0] != INVALID_VALUE_FIFO)
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, NA, NA, NA", (uint32_t)irq_timestamp,
			event->accel[0], event->accel[1], event->accel[2], 
			event->temperature);
}

/* --------------------------------------------------------------------------------------
 *  Static functions definition
 * -------------------------------------------------------------------------------------- */

static void apply_mounting_matrix(const int32_t matrix[9], int16_t raw[3])
{
	unsigned i;
	int64_t data_q30[3];
	
	for(i = 0; i < 3; i++) {
		data_q30[i] =  ((int64_t)matrix[3*i+0] * raw[0]);
		data_q30[i] += ((int64_t)matrix[3*i+1] * raw[1]);
		data_q30[i] += ((int64_t)matrix[3*i+2] * raw[2]);
	}
	raw[0] = (int16_t)(data_q30[0]>>30);
	raw[1] = (int16_t)(data_q30[1]>>30);
	raw[2] = (int16_t)(data_q30[2]>>30);
}