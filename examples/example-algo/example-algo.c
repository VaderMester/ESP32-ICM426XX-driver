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

#include "example-algo.h"

#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/RingBuffer.h"

#include "system-interface.h"

/* board drivers */
#include "common.h"
#include "timer.h"
#include "flash_manager.h"

#include <math.h>

#define RAD_TO_DEG(rad) ((float)rad * 57.2957795131)

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Icm426xx driver object */
static struct inv_icm426xx icm_driver;

/* structure allowing to handle clock calibration for ICM device timestamping */
static clk_calib_t clk_calib;

/* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
extern RINGBUFFER_VOLATILE(timestamp_buffer_icm, 64, uint64_t);

/*
 * ICM mounting matrix
 * Coefficients are coded as Q30 integer
 */
#if defined(ICM_FAMILY_CPLUS)
static int32_t icm_mounting_matrix[9] = {  0,        -(1<<30),     0,
                                          (1<<30),     0,          0,
                                           0,          0,         (1<<30) };
#else
static int32_t icm_mounting_matrix[9] = { (1<<30),     0,           0,
                                           0,         (1<<30),      0,
                                           0,          0,          (1<<30) };
#endif

/* Full Scale Range */
#if IS_HIGH_RES_MODE
	#if defined(ICM42686)
		static const int32_t acc_fsr = 32;   /* +/- 32g */
		static const int32_t gyr_fsr = 4000; /* +/- 4000dps */
	#else
		static const int32_t acc_fsr = 16;   /* +/- 16g */
		static const int32_t gyr_fsr = 2000; /* +/- 2000dps */
	#endif
#else
	static const int32_t acc_fsr = 4;        /* +/- 4g */
	static const int32_t gyr_fsr = 2000;     /* +/- 2000dps */
#endif


#if USE_MAG
/* Ak0991x driver object */
static inv_ak0991x_t ak_driver;

/* Buffer to keep track of the timestamp when ak09915 data ready interrupt fires. */
extern RINGBUFFER_VOLATILE(timestamp_buffer_mag, 64, uint64_t);

/*
 * Magnetometer mounting matrix
 * Coefficients are coded as Q30 integer
 */
static int32_t mag_mounting_matrix[9] = {  0,         (1<<30),     0,
                                         -(1<<30),     0,          0,
                                           0,          0,         (1<<30) };

/* Variable to keep track if the mag has initialized successfully */
extern int mag_init_successful;

#endif

static InvnAlgoAGMInput input;
static InvnAlgoAGMOutput output;

/* Counter to keep track of number of sample received */
static int iter_algo = 0;

/* How often do we print the output */
extern int print_period_us;  

/* Accelerometer gyroscope and magnetometer biases */
static int32_t acc_bias[3];
static int32_t gyr_bias[3];
static int32_t mag_bias[3];
/* Accelerometer gyroscope and magnetometer accuracies */
static int32_t acc_accuracy;
static int32_t gyr_accuracy;
static int32_t mag_accuracy;

/* Outptut data to print */
extern uint32_t data_to_print;

/* Bias previously stored in flash */
typedef struct sensor_biases {
	int32_t bias_q16[3];
	uint8_t is_saved;
} sensor_biases_t;

/* --------------------------------------------------------------------------------------
 *  static function declaration
 * -------------------------------------------------------------------------------------- */
static void print_algo_inputs_outputs(void);
static void apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3]);
static void fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift, const uint8_t dim);
static void quaternions_to_angles(const float quat[4], float angles[3]);
static uint32_t odr_bitfield_to_us(uint32_t odr_bitfield);
#if !IS_HIGH_RES_MODE
static int gyro_fsr_dps_to_bitfield(int32_t fsr);
static int accel_fsr_g_to_bitfield(int32_t fsr);
#endif
/* biases storage */
static int retrieve_stored_biases_from_flash(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3]);
static void store_biases_in_flash(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3]);
static void store_biases(void);

/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */

int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Initialize ICM");

	/* Init device */
	rc = inv_icm426xx_init(&icm_driver, icm_serif, HandleInvDeviceFifoPacket);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}
	
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Check ICM whoami value");

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

	rc |= inv_flash_manager_init();
	
#if USE_CLK_IN
	#if defined(ICM42633) 
	/* 
	 * ICM42633 is a triple interface device. To access CLKIN, AUX2 interface needs to be disabled.
	 * Use INV_ICM426XX_DUAL_INTERFACE mode. The following mode are also compatible:
	 *  - INV_ICM426XX_SINGLE_INTERFACE
	 *  - INV_ICM426XX_DUAL_INTERFACE_SPI4
	 */
	rc |= inv_icm426xx_interface_change_procedure(&icm_driver, INV_ICM426XX_DUAL_INTERFACE);
 	#endif
	/* Enable CLKIN */
	if (inv_icm426xx_enable_clkin_rtc(&icm_driver, 1) < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Couldn't enable CLKIN");
		return -1;
		} 
		clock_calibration_reset(&icm_driver, &clk_calib);
		/* Ensure all coefficients are set to 1 as the clock will not drift */
		clk_calib.coef[INV_ICM426XX_PLL] = 1.0f;
		clk_calib.coef[INV_ICM426XX_RC_OSC] = 1.0f;
		clk_calib.coef[INV_ICM426XX_WU_OSC] = 1.0f;
#else
	rc |= clock_calibration_init(&icm_driver, &clk_calib);
#endif
		
#if IS_HIGH_RES_MODE
	rc |= inv_icm426xx_enable_high_resolution_fifo(&icm_driver);
#else
	rc |= inv_icm426xx_set_accel_fsr(&icm_driver, (ICM426XX_ACCEL_CONFIG0_FS_SEL_t)accel_fsr_g_to_bitfield(acc_fsr));
	rc |= inv_icm426xx_set_gyro_fsr(&icm_driver,  (ICM426XX_GYRO_CONFIG0_FS_SEL_t) gyro_fsr_dps_to_bitfield(gyr_fsr));
#endif
	
	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, ACCEL_FREQ);
	rc |= inv_icm426xx_set_gyro_frequency(&icm_driver,  GYRO_FREQ);

#if IS_LOW_NOISE_MODE
	rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);
#else
	rc |= inv_icm426xx_enable_accel_low_power_mode(&icm_driver);
#endif

	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);
	
	return rc;
}


/*
 * This function clears biases and accuracies.
 */
int ResetInvAGMBiases(void)
{
	memset(acc_bias, 0, sizeof(acc_bias));
	memset(gyr_bias, 0, sizeof(gyr_bias));
	memset(mag_bias, 0, sizeof(mag_bias));
	acc_accuracy = 0;
	gyr_accuracy = 0;
	mag_accuracy = 0;

	return 0;
}


/*
 * This function initializes biases and accuracies for accelerometer, gyroscope and magnetometer.
 */
int InitInvAGMBiases(void)
{
	/* Retrieve stored biases */
	if (retrieve_stored_biases_from_flash(acc_bias, gyr_bias, mag_bias) == 0) {
		INV_MSG(INV_MSG_LEVEL_INFO, "   Biases loaded from flash:");
		INV_MSG(INV_MSG_LEVEL_INFO, "    - Accel: [%f %f %f]g",   
			(float)acc_bias[0]/(1<<16), (float)acc_bias[1]/(1<<16), (float)acc_bias[2]/(1<<16));
		INV_MSG(INV_MSG_LEVEL_INFO, "    - Gyro:  [%f %f %f]dps", 
			(float)gyr_bias[0]/(1<<16), (float)gyr_bias[1]/(1<<16), (float)gyr_bias[2]/(1<<16));
		INV_MSG(INV_MSG_LEVEL_INFO, "    - Mag:   [%f %f %f]uT",  
			(float)mag_bias[0]/(1<<16), (float)mag_bias[1]/(1<<16), (float)mag_bias[2]/(1<<16));
		
		acc_accuracy = 3;
		gyr_accuracy = 2;
		mag_accuracy = 1;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "   No bias values retrieved");
		memset(acc_bias,0,sizeof(acc_bias));
		memset(gyr_bias,0,sizeof(gyr_bias));
		memset(mag_bias,0,sizeof(mag_bias));
		acc_accuracy = 0;
		gyr_accuracy = 0;
		mag_accuracy = 0;
	}

	return 0;
}


int InitInvAGMAlgo(void)
{
	int rc = 0;
	InvnAlgoAGMConfig config;

	memset(&input,  0, sizeof(input));
	memset(&output, 0, sizeof(output));
	memset(&config, 0, sizeof(config));

	// config.acc_bias_q16 = output.acc_bias_q16;
	// config.gyr_bias_q16 = output.gyr_bias_q16;
	// config.mag_bias_q16 = output.mag_bias_q16;
	
	/* FSR configurations */
	config.acc_fsr = acc_fsr;
	config.gyr_fsr = gyr_fsr;

	config.acc_odr_us = odr_bitfield_to_us(ACCEL_FREQ);
	config.gyr_odr_us = odr_bitfield_to_us(GYRO_FREQ);

	/* Temoperature sensor configuration */
#if IS_HIGH_RES_MODE
	config.temp_sensitivity = (int32_t)((int64_t)((int64_t)100 << 30)/13248); // high-res;
	config.temp_offset = 25 << 16;
#else
	config.temp_sensitivity = (int32_t)((int64_t)((int64_t)100 << 30)/207);
	config.temp_offset = 25 << 16;
#endif

#if USE_MAG
	config.mag_sc_q16 = 9830;//0.15
	config.mag_odr_us = MAG_ODR_US;
#else
	config.mag_bias_q16 = NULL;
#endif
	
	config.acc_bias_q16 = acc_bias;
	config.gyr_bias_q16 = gyr_bias;
	config.mag_bias_q16 = mag_bias;
	config.acc_accuracy = acc_accuracy;
	config.gyr_accuracy = gyr_accuracy;
	config.mag_accuracy = mag_accuracy;

	/* Initialize algorithm */
	rc |= invn_algo_agm_init(&config);

	return rc;
}

int GetDataFromInvDevice(void)
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

	/*
	 * First, let's process the sensor event timestamp.
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
	 * Extend the 16-bit timestamp from the FIFO to get an accurate timestamping
	 */
	inv_helper_extend_timestamp_from_fifo(&icm_driver, &clk_calib, 
		event->timestamp_fsync, irq_timestamp, event->sensor_mask, &extended_timestamp);

	input.mask = 0;

	/*
	 * Retrieve accel and gyro data
	 */
	if (event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL)) {
#if IS_HIGH_RES_MODE
		input.sRacc_data[0] = (((int32_t)event->accel[0] << 4)) | event->accel_high_res[0];
		input.sRacc_data[1] = (((int32_t)event->accel[1] << 4)) | event->accel_high_res[1];
		input.sRacc_data[2] = (((int32_t)event->accel[2] << 4)) | event->accel_high_res[2];
#else
		input.sRacc_data[0] = (int32_t)event->accel[0] << 4;
		input.sRacc_data[1] = (int32_t)event->accel[1] << 4;
		input.sRacc_data[2] = (int32_t)event->accel[2] << 4;
#endif
		apply_mounting_matrix(icm_mounting_matrix, input.sRacc_data);
		input.mask |= INVN_ALGO_AGM_INPUT_MASK_ACC;
	}

	if (event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO)) {
#if IS_HIGH_RES_MODE	
		input.sRgyr_data[0] = (((int32_t)event->gyro[0] << 4)) | event->gyro_high_res[0];
		input.sRgyr_data[1] = (((int32_t)event->gyro[1] << 4)) | event->gyro_high_res[1];
		input.sRgyr_data[2] = (((int32_t)event->gyro[2] << 4)) | event->gyro_high_res[2];
#else
		input.sRgyr_data[0] = (int32_t)event->gyro[0] << 4;
		input.sRgyr_data[1] = (int32_t)event->gyro[1] << 4;
		input.sRgyr_data[2] = (int32_t)event->gyro[2] << 4;
#endif
		apply_mounting_matrix(icm_mounting_matrix, input.sRgyr_data);
		input.mask |= INVN_ALGO_AGM_INPUT_MASK_GYR;
	}

	input.sRtemp_data = event->temperature;
	input.sRimu_time_us = extended_timestamp;

	/* Process the AgmFusion Algo */
	invn_algo_agm_process(&input, &output);
	
	store_biases();

	/* Print data based on the gyro rate */
	if (output.mask & INVN_ALGO_AGM_OUTPUT_MASK_GYRO_CAL) {
		iter_algo++;

		if (iter_algo % ((int)(print_period_us/odr_bitfield_to_us(GYRO_FREQ))) == 0)
			print_algo_inputs_outputs();
	}
}

#if USE_MAG
int SetupMagDevice(struct inv_ak0991x_serif * akm_serif)
{
	int rc;
	uint8_t whoami;
	
	INV_MSG(INV_MSG_LEVEL_INFO, "Reseting Ak09915");

	/* Reset Ak09915 driver states */
	inv_ak0991x_reset_states(&ak_driver, akm_serif);

	/* Init ak09915 device */
	rc = inv_ak0991x_soft_reset(&ak_driver);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Failed to initialize Ak09915.");
		return rc;
	}

	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Check Ak09915 whoami value");
	
	rc = inv_ak0991x_get_whoami(&ak_driver, &whoami);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Failed to read Ak09915 whoami value.");
		return rc;
	}

	if (whoami != AK0991x_COMPANY_ID) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "Bad WHOAMI value for mag device. Got 0x%02x", whoami);
		return INV_ERROR;
	}
	
	/* register sensitivity adjustment values */
	inv_ak0991x_retrieve_asa_values(&ak_driver);

	RINGBUFFER_VOLATILE_CLEAR(&timestamp_buffer_mag);
	
	return rc;
}


int StartMagDeviceAcquisition(void)
{
	inv_ak0991x_enable_sensor(&ak_driver, 1);
	
	return 0;
}


int GetDataFromMagDevice()
{
	int rc = 0;
	int16_t raw_mag[3];
	uint64_t irq_timestamp = 0;

	inv_disable_irq();
	if (!RINGBUFFER_VOLATILE_EMPTY(&timestamp_buffer_mag))
		RINGBUFFER_VOLATILE_POP(&timestamp_buffer_mag, &irq_timestamp);
	inv_enable_irq();

	/* Read Ak09915 data */
	rc |= inv_ak0991x_poll_data(&ak_driver, raw_mag);

	input.mask = INVN_ALGO_AGM_INPUT_MASK_MAG;
	input.sRmag_data[0] = (int32_t)raw_mag[0];
	input.sRmag_data[1] = (int32_t)raw_mag[1];
	input.sRmag_data[2] = (int32_t)raw_mag[2];
	input.sRmag_time_us = irq_timestamp;
	
	/*
	 * Apply the mounting matrix configuration to the mag data polled
	 */		
	apply_mounting_matrix(mag_mounting_matrix, input.sRmag_data);
	
	invn_algo_agm_process(&input, &output);

	return rc;
}
#endif /* USE_MAG */


/* --------------------------------------------------------------------------------------
 *  Static functions definition
 * -------------------------------------------------------------------------------------- */
static void print_algo_inputs_outputs(void)
{
	float acc_g[3], acc_bias[3];
	float gyr_dps[3], gyr_bias[3];
	float temp;
	float grv_quat[4], angles_deg_grv[3];
	float gravity[3], linear_acc[3];
#if USE_MAG
	float mag_cal[3], mag_bias[3];
	float rv_quat[4], angles_deg_rv[3], rv_heading_accuracy;
#endif
	
	/* Convert data to float before send it to the terminal */
	fixedpoint_to_float( output.acc_cal_q16,            acc_g,       16, 3);
	fixedpoint_to_float( output.acc_bias_q16,           acc_bias,    16, 3);
	fixedpoint_to_float( output.gyr_cal_q16,            gyr_dps,     16, 3);
	fixedpoint_to_float( output.gyr_bias_q16,           gyr_bias,    16, 3);
	fixedpoint_to_float( &output.temp_degC_q16,         &temp,       16, 1);
	fixedpoint_to_float( output.grv_quat_q30,           grv_quat,    30, 4);
	fixedpoint_to_float( output.gravity_q16,            gravity,     16, 3);
	fixedpoint_to_float( output.linear_acc_q16,         linear_acc,  16, 3);
	quaternions_to_angles(grv_quat, angles_deg_grv);

#if USE_MAG
	fixedpoint_to_float( output.mag_cal_q16,            mag_cal,     16, 3);
	fixedpoint_to_float( output.mag_bias_q16,           mag_bias,    16, 3);
	fixedpoint_to_float( output.rv_quat_q30,            rv_quat,     30, 4);
	quaternions_to_angles(rv_quat,     angles_deg_rv);
	rv_heading_accuracy = RAD_TO_DEG((float)output.rv_accuracy_q27/(1 << 27));
#endif


	/* Print intputs */
	if (data_to_print & MASK_PRINT_INPUT_DATA) {
		/* ICM data */
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: INPUT  RAcc=[%d, %d, %d] RGyr=[%d, %d, %d] Rtemp=[%d]",
			input.sRimu_time_us,
#if IS_HIGH_RES_MODE
			input.sRacc_data[0], input.sRacc_data[1], input.sRacc_data[2], 
			input.sRgyr_data[0], input.sRgyr_data[1], input.sRgyr_data[2],
#else
			input.sRacc_data[0] >> 4, input.sRacc_data[1] >> 4, input.sRacc_data[2] >> 4, 
			input.sRgyr_data[0] >> 4, input.sRgyr_data[1] >> 4, input.sRgyr_data[2] >> 4,
#endif
			(int32_t)input.sRtemp_data
			);
#if USE_MAG
		if (mag_init_successful) {
			/* Mag data */
			INV_MSG(INV_MSG_LEVEL_INFO, "%lld: INPUT  RMag=[%d, %d, %d]", 
				input.sRmag_time_us, 
				input.sRmag_data[0], input.sRmag_data[1], input.sRmag_data[2]
				);
		}
#endif
	}

	/* Print outputs */
	if (data_to_print & MASK_PRINT_ACC_DATA) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Acc=[%.3f, %.3f, %.3f]g AccBias=[%.3f, %.3f, %.3f]mg Accuracy=%d",
			input.sRimu_time_us,
			acc_g[0], acc_g[1], acc_g[2],
			acc_bias[0]*1000, acc_bias[1]*1000, acc_bias[2]*1000,
			(int32_t)output.acc_accuracy_flag
			);
	}
	
	if (data_to_print & MASK_PRINT_GYR_DATA) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Gyr=[%.3f, %.3f, %.3f]dps GyrBias=[%.3f, %.3f, %.3f]dps Temp=[%.2f]C Accuracy=%d",
			input.sRimu_time_us,
			gyr_dps[0],  gyr_dps[1],  gyr_dps[2], 
			gyr_bias[0], gyr_bias[1], gyr_bias[2], 
			temp,
			(int32_t)output.gyr_accuracy_flag
			);
	}

#if USE_MAG
	if (mag_init_successful) {
		if (data_to_print & MASK_PRINT_MAG_DATA) {
			INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Mag=[%.3f, %.3f, %.3f]uT MagBias=[%.3f, %.3f, %.3f]uT Accuracy=%d", 
				input.sRmag_time_us,
				mag_cal[0],   mag_cal[1],   mag_cal[2],
				mag_bias[0],  mag_bias[1],  mag_bias[2],
				output.mag_accuracy_flag
				);
		}
		
		if (data_to_print & MASK_PRINT_9AXIS_DATA) {
			INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT 9Axis=[%f, %f, %f, %f] 9AxisAccuracy=[%f]deg",
				input.sRimu_time_us,
				rv_quat[0], rv_quat[1], rv_quat[2], rv_quat[3],
				rv_heading_accuracy
				);

			INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Euler9Axis=[%.2f, %.2f, %.2f]deg 9AxisAccuracy=[%f]deg",
				input.sRimu_time_us,
				angles_deg_rv[0], angles_deg_rv[1], angles_deg_rv[2],
				rv_heading_accuracy
				);
		}
	}
#endif

	if (data_to_print & MASK_PRINT_6AXIS_DATA) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT 6Axis=[%f, %f, %f, %f]",
			input.sRimu_time_us,
			grv_quat[0], grv_quat[1], grv_quat[2], grv_quat[3]
			);
		
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Euler6Axis=[%.2f, %.2f, %.2f]deg",
			input.sRimu_time_us,
			angles_deg_grv[0], angles_deg_grv[1], angles_deg_grv[2]
			);
	}
	
	if (data_to_print & MASK_PRINT_GRAVITY_DATA) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT Gravity=[%f, %f, %f] Accuracy=%d",
			input.sRimu_time_us,
			gravity[0],	gravity[1],	gravity[2],
			output.acc_accuracy_flag);
	}
	if (data_to_print & MASK_PRINT_LINEARACC_DATA) {
		INV_MSG(INV_MSG_LEVEL_INFO, "%lld: OUTPUT LinearAcc=[%f, %f, %f] Accuracy=%d",
			input.sRimu_time_us, 
			linear_acc[0], linear_acc[1], linear_acc[2],
			output.acc_accuracy_flag);
	}
	
	/* Print cariage return to ease reading, only if some data are printed */
	if (data_to_print)
		INV_MSG(INV_MSG_LEVEL_INFO, "");
}


/*
 * \brief Read NV memory and retrieve stored biases
 * \param[out] acc_bias_q16 Previously stored acc bias
 * \param[out] gyr_bias_q16 Previously stored gyr bias
 * \param[out] mag_bias_q16 Previously stored mag bias
 * \return 0 on success, -1 if no bias are in NV, an error otherwise
 */
static int retrieve_stored_biases_from_flash(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3])
{
	uint8_t sensor_bias[84];
	uint8_t idx = 0;
	int rc;
	
	/* Retrieve bias stored in NV memory */
	if ((rc = inv_flash_manager_readData(sensor_bias)) != 0) {
		return -1;
	}
	
	memcpy(acc_bias_q16, &sensor_bias[idx], sizeof(acc_bias_q16[0]) * 3);
	idx += sizeof(acc_bias_q16[0]) * 3;
	
	memcpy(gyr_bias_q16, &sensor_bias[idx], sizeof(gyr_bias_q16[0]) * 3);
	idx += sizeof(gyr_bias_q16[0]) * 3;
	
	memcpy(mag_bias_q16, &sensor_bias[idx], sizeof(mag_bias_q16[0]) * 3);
		
	return rc;
}

/*
 * \brief Write sensor biases into NV memory
 * \param[in] acc_bias_q16 acc bias to be written
 * \param[in] gyr_bias_q16 gyr bias to be written
 * \param[in] mag_bias_q16 mag bias to be written
 */
static void store_biases_in_flash(const int32_t acc_bias_q16[3], const int32_t gyr_bias_q16[3], const int32_t mag_bias_q16[3])
{
	uint8_t sensors_biases[84] = {0};
	uint8_t idx = 0;
	
	memcpy(&sensors_biases[idx], acc_bias_q16, sizeof(acc_bias_q16[0]) * 3);
	idx += sizeof(acc_bias_q16[0]) * 3;
	
	memcpy(&sensors_biases[idx], gyr_bias_q16, sizeof(gyr_bias_q16[0]) * 3);
	idx += sizeof(gyr_bias_q16[0]) * 3;
	
	memcpy(&sensors_biases[idx], mag_bias_q16, sizeof(mag_bias_q16[0]) * 3);
	
	inv_flash_manager_writeData(sensors_biases);
}

/*
 * \brief Evaluate wether biases needs to be written to flash depending on accuracies value
 */
static void store_biases(void)
{		
	static sensor_biases_t acc_bias;
	static sensor_biases_t gyr_bias;
	static sensor_biases_t mag_bias;
	static uint8_t biases_stored = 0;
	
	if (!biases_stored) {
		if (output.acc_accuracy_flag == 3) {
			memcpy(acc_bias.bias_q16, output.acc_bias_q16, sizeof(output.acc_bias_q16));
			acc_bias.is_saved = 1;
		}

		if (output.gyr_accuracy_flag == 3) {
			memcpy(gyr_bias.bias_q16, output.gyr_bias_q16, sizeof(output.gyr_bias_q16));
			gyr_bias.is_saved = 1;
		}

#if USE_MAG
		if (mag_init_successful) {
			if (output.mag_accuracy_flag == 3) {
				memcpy(mag_bias.bias_q16, output.mag_bias_q16, sizeof(output.mag_bias_q16));
				mag_bias.is_saved = 1;
			}
		} else {
			/* Mag is not connected so let's put zeros */
			memset(mag_bias.bias_q16, 0, sizeof(mag_bias.bias_q16));
			mag_bias.is_saved = 1;		
		}
#else
		/* Mag is not connected so let's put zeros */
		memset(mag_bias.bias_q16, 0, sizeof(mag_bias.bias_q16));
		mag_bias.is_saved = 1;
#endif

		if ((acc_bias.is_saved == 1) && (gyr_bias.is_saved == 1) && (mag_bias.is_saved == 1)) {
			/* 
			 * WARNING: With this configuration, the bias can be stored up to 186 iterations in flash before erase sector.
			 * The erase procedure with the first write, can take up to 250ms.
			 * In this example, the erase is done dynamicly. Depending on the context, it could be better to do it 
			 * when the software go to shutdown.
			 */
			store_biases_in_flash(acc_bias.bias_q16, gyr_bias.bias_q16, mag_bias.bias_q16);
			biases_stored = 1;
		}
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

static void fixedpoint_to_float(const int32_t *in, float *out, const uint8_t fxp_shift, const uint8_t dim)
{
	int i;
	float scale = 1.f / (1<<fxp_shift);

	for (i = 0; i < dim; i++)
		out[i] = scale * in[i];
}

static void quaternions_to_angles(const float quat[4], float angles[3])
{
	const float RAD_2_DEG = (180.f/3.14159265358979f);
	float rot_matrix[9];

	{ // quaternion_to_rotation_matrix
		const float dTx  = 2.0f * quat[1];
		const float dTy  = 2.0f * quat[2];
		const float dTz  = 2.0f * quat[3];
		const float dTwx = dTx  * quat[0];
		const float dTwy = dTy  * quat[0];
		const float dTwz = dTz  * quat[0];
		const float dTxx = dTx  * quat[1];
		const float dTxy = dTy  * quat[1];
		const float dTxz = dTz  * quat[1];
		const float dTyy = dTy  * quat[2];
		const float dTyz = dTz  * quat[2];
		const float dTzz = dTz  * quat[3];

		rot_matrix[0] = 1.0f - (dTyy + dTzz);
		rot_matrix[1] = dTxy - dTwz;
		rot_matrix[2] = dTxz + dTwy;
		rot_matrix[3] = dTxy + dTwz;
		rot_matrix[4] = 1.0f - (dTxx + dTzz);
		rot_matrix[5] = dTyz - dTwx;
		rot_matrix[6] = dTxz - dTwy;
		rot_matrix[7] = dTyz + dTwx;
		rot_matrix[8] = 1.0f - (dTxx + dTyy);
	}

	angles[0] = atan2f(-rot_matrix[3], rot_matrix[0])*RAD_2_DEG;
	angles[1] = atan2f(-rot_matrix[7], rot_matrix[8])*RAD_2_DEG;
	angles[2] = asinf(-rot_matrix[6])*RAD_2_DEG;

	if (angles[0] < 0.f)
		angles[0] += 360.f;
}

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
#if !IS_HIGH_RES_MODE
static int gyro_fsr_dps_to_bitfield(int32_t fsr)
{
	switch (fsr) {
#if !defined(ICM42686)
	case 15:
	case 16:   return ICM426XX_GYRO_CONFIG0_FS_SEL_16dps;
#endif
	case 31:
	case 32:   return ICM426XX_GYRO_CONFIG0_FS_SEL_31dps;
	case 62:
	case 63:   return ICM426XX_GYRO_CONFIG0_FS_SEL_62dps;
	case 125:  return ICM426XX_GYRO_CONFIG0_FS_SEL_125dps;
	case 250:  return ICM426XX_GYRO_CONFIG0_FS_SEL_250dps;
	case 500:  return ICM426XX_GYRO_CONFIG0_FS_SEL_500dps;
	case 1000: return ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps;
	case 2000: return ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps;
#if defined(ICM42686)
	case 4000: return ICM426XX_GYRO_CONFIG0_FS_SEL_4000dps;
#endif
	default:   return -1;
	}
}

static int accel_fsr_g_to_bitfield(int32_t fsr)
{
	switch (fsr) {
	case 2:  return ICM426XX_ACCEL_CONFIG0_FS_SEL_2g;
	case 4:  return ICM426XX_ACCEL_CONFIG0_FS_SEL_4g;
	case 8:  return ICM426XX_ACCEL_CONFIG0_FS_SEL_8g;
	case 16: return ICM426XX_ACCEL_CONFIG0_FS_SEL_16g;
#if defined(ICM42686)
	case 32: return ICM426XX_ACCEL_CONFIG0_FS_SEL_32g;
#endif
	default:   return -1;
	}
}
#endif