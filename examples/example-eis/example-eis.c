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

 #include "example-eis.h"
 
/* Clock calibration module */
#include "Invn/Helpers/Icm426xx/helperClockCalib.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"
#include "timer.h"
#include "gpio.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Icm426xx driver object */
static struct inv_icm426xx icm_driver;

/* structure allowing to handle clock calibration for ICM device timestamping */
static clk_calib_t clk_calib;
 
 /* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
extern RINGBUFFER_VOLATILE(timestamp_buffer_icm, 64, uint64_t);

/* Contains algorithms input */
static InvnAlgoAGMInput input;

/* Contains algorithms output */
static InvnAlgoAGMOutput output;

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
static uint32_t odr_bitfield_to_us(uint32_t odr_bitfield);
static uint32_t fsr_bitfield_to_dps(uint32_t fsr_bitfield);
static void apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3]);

/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */
 
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;
	
	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize ICM");

#if FSYNC_USE_FIFO
	rc |= inv_icm426xx_init(&icm_driver, icm_serif, HandleInvDeviceFifoPacket);
#else
	rc |= inv_icm426xx_init(&icm_driver, icm_serif, HandleInvDeviceDataRegisters);
#endif
	
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}
	
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_INFO, "Check Icm426xx whoami value");
	
	rc = inv_icm426xx_get_who_am_i(&icm_driver, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value.");
		return rc;
	}
	
	if (who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	RINGBUFFER_VOLATILE_CLEAR(&timestamp_buffer_icm);
	return rc;
}

int ConfigureInvDevice(void)
{
	int rc = 0;
	uint8_t data = 0;

	rc |= clock_calibration_init(&icm_driver, &clk_calib);
	
	rc |= inv_icm426xx_set_gyro_fsr(&icm_driver, GYRO_FSR);
	rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, GYRO_FREQ);
	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);
	
	// Do only for CPLUS family for triple interfaces devices
#if (defined(ICM_FAMILY_CPLUS) && (defined(ICM42631) || defined(ICM42633)))
	/* Update interface for this example, as triple interface is set by default and doesnt allow fsync usage*/
	rc |= inv_icm426xx_interface_change_procedure(&icm_driver, INV_ICM426XX_DUAL_INTERFACE);
#endif

#if !FSYNC_USE_FIFO	
	/* If FIFO is not used then enable fsync flagging in temperature register LSB */
	rc |= inv_icm426xx_read_reg(&icm_driver, MPUREG_FSYNC_CONFIG, 1, &data);
	data &= ~BIT_FSYNC_CONFIG_UI_SEL_MASK;
	data |= ICM426XX_FSYNC_CONFIG_UI_SEL_TEMP;
	rc |= inv_icm426xx_write_reg(&icm_driver, MPUREG_FSYNC_CONFIG, 1, &data);
#else
	(void)data;
#endif
	
	rc |= inv_icm426xx_enable_fsync(&icm_driver);
	
	return rc;
}

int InitInvAGMAlgo(void)
{
	int rc = 0;
	InvnAlgoAGMConfig config;

	memset(&input, 0, sizeof(input));
	memset(&output, 0, sizeof(output));
	memset(&config, 0, sizeof(config));

	config.gyr_fsr = fsr_bitfield_to_dps(GYRO_FSR);
	config.gyr_odr_us = odr_bitfield_to_us(GYRO_FREQ);
	config.temp_sensitivity = 207;
	config.temp_offset = 25;

	/* Initialize algorithm */
	rc |= invn_algo_agm_init(&config);

	return rc;
}

int GetDataFromFIFO(void)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. 
	 * HandleInvDeviceFifoPacket) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	return inv_icm426xx_get_data_from_fifo(&icm_driver);
}


void HandleInvDeviceFifoPacket(inv_icm426xx_sensor_event_t * event)
{
	uint64_t irq_timestamp = 0;
	uint64_t extended_timestamp;
	static uint64_t event_fsync_ts;
	static uint16_t event_fsync_delay_cnt = 0;
	
	/*
	 * Extract the timestamp that was buffered when current packet IRQ fired. See 
	 * ext_interrupt_cb() in main.c for more details.
	 * As timestamp buffer is filled in interrupt handler, we should pop it with
	 * interrupts disabled to avoid any concurrent access.
	 */
	inv_disable_irq();
	if (!RINGBUFFER_VOLATILE_EMPTY(&timestamp_buffer_icm))
		RINGBUFFER_VOLATILE_POP(&timestamp_buffer_icm, &irq_timestamp);
	inv_enable_irq();	
	
	/*
	 * Extend the 16-bit timestamp from the Icm426xx FIFO to a 64 bits timestamp.
	 */
	inv_helper_extend_timestamp_from_fifo(&icm_driver, &clk_calib, 
		event->timestamp_fsync, irq_timestamp, event->sensor_mask, &extended_timestamp);

	if (event->sensor_mask & (1 << INV_ICM426XX_SENSOR_FSYNC_EVENT)) {
		
		event->timestamp_fsync = (uint16_t)(((uint64_t)event->timestamp_fsync 
			* inv_icm426xx_get_reg_timestamp_resolution_us_q24(&icm_driver))>>24);
		event_fsync_ts = extended_timestamp;
		event_fsync_delay_cnt = event->timestamp_fsync;
		/* 
		 * Uncomment to print FSYNC events
		 */
		// INV_MSG(INV_MSG_LEVEL_INFO, "%u: FSYNC %hd", (uint32_t)event_fsync_ts, (uint16_t)event_fsync_delay_cnt);
	}
	
	if ((event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO)) &&
		(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_TEMPERATURE)) ) {
				
		/* Compute calibrated gyroscope */
		input.sRgyr_data[0] = (int32_t)event->gyro[0] << 4;
		input.sRgyr_data[1] = (int32_t)event->gyro[1] << 4;
		input.sRgyr_data[2] = (int32_t)event->gyro[2] << 4;
		
		input.sRtemp_data = event->temperature;
		input.sRimu_time_us = extended_timestamp;
		input.mask = INVN_ALGO_AGM_INPUT_MASK_GYR;
		
		apply_mounting_matrix(icm_mounting_matrix, input.sRgyr_data);
		invn_algo_agm_process(&input,&output);
			
		/* Clear gyr input mask */
		input.mask &= ~INVN_ALGO_AGM_INPUT_MASK_GYR;
			
		/* Compute the EIS data */
		if (extended_timestamp == event_fsync_ts) {
			INV_MSG(INV_MSG_LEVEL_INFO, "%u: %f, %f, %f  %hhd FSYNC event %hd", 
				(uint32_t)extended_timestamp, 
				(float)output.gyr_cal_q16[0] / (1 << 16), 
				(float)output.gyr_cal_q16[1] / (1 << 16), 
				(float)output.gyr_cal_q16[2] / (1 << 16), 
				(int8_t)output.gyr_accuracy_flag, 
				(uint16_t)event_fsync_delay_cnt
				);
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "%u: %f, %f, %f  %hhd", 
				(uint32_t)extended_timestamp, 
				(float)output.gyr_cal_q16[0] / (1 << 16), 
				(float)output.gyr_cal_q16[1] / (1 << 16), 
				(float)output.gyr_cal_q16[2] / (1 << 16),  
				(int8_t)output.gyr_accuracy_flag
				);
		}
	}
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
	static uint16_t event_fsync_delay_cnt = 0;
	
	/*
	 * Extract the timestamp that was buffered when current packet IRQ fired. See 
	 * ext_interrupt_cb() in main.c for more details.
	 * As timestamp buffer is filled in interrupt handler, we should pop it with
	 * interrupts disabled to avoid any concurrent access.
	 */
	inv_disable_irq();
	if (!RINGBUFFER_VOLATILE_EMPTY(&timestamp_buffer_icm))
		RINGBUFFER_VOLATILE_POP(&timestamp_buffer_icm, &irq_timestamp);
	inv_enable_irq();
	
	/* 
	 * Check if fsync flag is set.
	 * event->temperature[0] is configured to expose fsync flag. If fsync flag is set, process fsync counter.
	 */
	if (event->temperature & 1) {
		uint8_t fsync_count[2];      

		/*
		 * Read 16bits fsync counter containing time elapsed between last FSYNC interrupt and last ODR event.
		 * Fsync delta time depends on data endianness as counter is read over 2 registers and timestamp resolution.
		 */
		inv_icm426xx_read_reg(&icm_driver, MPUREG_TMST_FSYNCH, 2, fsync_count);
		
		if (icm_driver.endianess_data == ICM426XX_INTF_CONFIG0_DATA_BIG_ENDIAN)
			event->timestamp_fsync = (uint16_t)(((uint16_t)fsync_count[0] << 8) + (uint16_t)fsync_count[1]);
		else
			event->timestamp_fsync = (uint16_t)(((uint16_t)fsync_count[1] << 8) + (uint16_t)fsync_count[0]);
		
		event->timestamp_fsync = (uint16_t)((((uint64_t)event->timestamp_fsync)
			* inv_icm426xx_get_reg_timestamp_resolution_us_q24(&icm_driver)) >> 24);

		event_fsync_delay_cnt = event->timestamp_fsync;
		/* 
		 * Uncomment to print FSYNC events
		 */
		// INV_MSG(INV_MSG_LEVEL_INFO, "%u: FSYNC %hd", (uint32_t)irq_timestamp, (uint16_t)event_fsync_delay_cnt);
	}


	/*
	 * Output gyro data on UART link
	 */
	if (event->gyro[0] != INVALID_VALUE_FIFO) {
		/*INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d", (uint32_t)irq_timestamp, 
			event->temperature,
			event->gyro[0], event->gyro[1], event->gyro[2]);*/
		
		/* Compute calibrated gyroscope */	
		
		input.sRgyr_data[0] = (int32_t)event->gyro[0] << 4;
		input.sRgyr_data[1] = (int32_t)event->gyro[1] << 4;
		input.sRgyr_data[2] = (int32_t)event->gyro[2] << 4;
		
		input.sRtemp_data = event->temperature;
		input.sRimu_time_us = irq_timestamp;
		input.mask = INVN_ALGO_AGM_INPUT_MASK_GYR;
		
		invn_algo_agm_process(&input,&output);
			
		/* Clear gyr input mask */
		input.mask &= ~INVN_ALGO_AGM_INPUT_MASK_GYR;
		
		/* 
		 * Print CalGyr data and EIS data if relevant
		 */
		if (event->temperature & 1) {
		
			INV_MSG(INV_MSG_LEVEL_INFO, "%u: %f, %f, %f  %hhd FSYNC event %hd", 
				(uint32_t)irq_timestamp, 
				(float)output.gyr_cal_q16[0] / (1 << 16), 
				(float)output.gyr_cal_q16[0] / (1 << 16), 
				(float)output.gyr_cal_q16[0] / (1 << 16), 
				(int8_t)output.gyr_accuracy_flag, 
				(uint16_t)event_fsync_delay_cnt
				);
		} else {
			INV_MSG(INV_MSG_LEVEL_INFO, "%u: %f, %f, %f  %hhd", 
				(uint32_t)irq_timestamp, 
				(float)output.gyr_cal_q16[0] / (1 << 16), 
				(float)output.gyr_cal_q16[1] / (1 << 16), 
				(float)output.gyr_cal_q16[2] / (1 << 16),
				(int8_t)output.gyr_accuracy_flag
				);
		}
	}
}


/* --------------------------------------------------------------------------------------
 *  Static functions definition
 * -------------------------------------------------------------------------------------- */

static uint32_t odr_bitfield_to_us(uint32_t odr_bitfield)
{
	switch ((ICM426XX_GYRO_CONFIG0_ODR_t)odr_bitfield) {
		case ICM426XX_GYRO_CONFIG0_ODR_1_KHZ:      return 1000;
		case ICM426XX_GYRO_CONFIG0_ODR_500_HZ:     return 2000;
		case ICM426XX_GYRO_CONFIG0_ODR_200_HZ:     return 5000;
		case ICM426XX_GYRO_CONFIG0_ODR_100_HZ:     return 10000;
		case ICM426XX_GYRO_CONFIG0_ODR_50_HZ:      return 20000;
		default:                                   return 640000;
	}
}

static uint32_t fsr_bitfield_to_dps(uint32_t fsr_bitfield)
{
	switch ((ICM426XX_GYRO_CONFIG0_FS_SEL_t)fsr_bitfield) {
#if !defined(ICM42686)
	case ICM426XX_GYRO_CONFIG0_FS_SEL_16dps:   return 16;
#endif
	case ICM426XX_GYRO_CONFIG0_FS_SEL_31dps:   return 31;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_62dps:   return 62;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_125dps:  return 125;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_250dps:  return 250;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_500dps:  return 500;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps: return 1000;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps: return 2000;
#if defined(ICM42686)
	case ICM426XX_GYRO_CONFIG0_FS_SEL_4000dps: return 4000;
#endif
	default:                                   return -1;
	}

}

static void apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3])
{
	unsigned i;
	int64_t data_q30[3];
	
	for(i = 0; i < 3; i++) {
		data_q30[i] =  ((int64_t)matrix[3*i+0] * raw[0]);
		data_q30[i] += ((int64_t)matrix[3*i+1] * raw[1]);
		data_q30[i] += ((int64_t)matrix[3*i+2] * raw[2]);
	}
	raw[0] = (int32_t)(data_q30[0]>>30);
	raw[1] = (int32_t)(data_q30[1]>>30);
	raw[2] = (int32_t)(data_q30[2]>>30);
}