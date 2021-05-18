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

#include "Message.h"
#include "ErrorHelper.h"
#include "helperClockCalib.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


/* MUX for entering critical section */
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

/* forward declaration */
static int clock_calibration_initial(struct inv_icm426xx * s, struct clk_calib *clk_cal);
static int clock_calibration_update_coef(struct inv_icm426xx * s, struct clk_calib *clk_cal, uint8_t *on_going, uint64_t irq_timestamp, uint16_t fifo_timestamp);
static int get_fastest_running_odr(struct inv_icm426xx * s, uint32_t *odr);
static enum inv_icm426xx_sensor get_fastest_sensor(struct inv_icm426xx * s);

static enum inv_icm426xx_clock_source get_current_clock_source(struct inv_icm426xx * s)
{
	// WU oscillator is active if Accel is enabled alone, in LP, with lp_clk_sel sets to WU_OSC 
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t      gyr_mode;
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t     acc_mode;
	ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_t lp_clk;
	int status = 0;
	uint8_t pwr_mngt_0_reg, intf_cfg_1_reg;

	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &intf_cfg_1_reg);
	
	if (0 > status)
		return INV_ICM426XX_CLOCK_SOURCE_MAX; // error
		
	gyr_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)      (pwr_mngt_0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	acc_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)     (pwr_mngt_0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	lp_clk   = (ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_t) (intf_cfg_1_reg & BIT_ACCEL_LP_CLK_SEL_MASK);

	if (gyr_mode != ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF && gyr_mode != ICM426XX_PWR_MGMT_0_GYRO_MODE_STANDBY)
		return INV_ICM426XX_PLL;
	else if ((acc_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN)
		  || (acc_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP && lp_clk == ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC))
		return INV_ICM426XX_RC_OSC;
	else 
		return INV_ICM426XX_WU_OSC;
}

static char * get_current_clock_source_name(struct inv_icm426xx * s)
{
	switch(get_current_clock_source(s)) {
		case INV_ICM426XX_PLL:    return "PLL";
		case INV_ICM426XX_RC_OSC: return "RC_OSC";
		case INV_ICM426XX_WU_OSC: return "WU_OSC";
		default:                  return "No source";
	}
}

static int64_t apply_time_factor(struct inv_icm426xx * s, struct clk_calib *clk_cal, float dt_fifo)
{
	// Retrieve coef that needs to be applied
	float time_factor = clk_cal->coef[get_current_clock_source(s)];

	float dt_float = ((dt_fifo) * (time_factor)) + clk_cal->dt_error;
	clk_cal->dt_error = dt_float - (int64_t)(dt_float);
	return (int64_t)(dt_float);
}

static float get_fifo_timestamp_resolution(struct inv_icm426xx * s)
{
	return (float)inv_icm426xx_get_fifo_timestamp_resolution_us_q24(s)/(1UL<<24);
}

static float get_reg_timestamp_resolution(struct inv_icm426xx * s)
{
	return (float)inv_icm426xx_get_reg_timestamp_resolution_us_q24(s)/(1UL<<24);
}

int inv_helper_extend_timestamp_from_fifo(struct inv_icm426xx * s, 
	                                      struct clk_calib *clk_cal, 
	                                      uint16_t cur_fifo_timestamp, 
	                                      uint64_t irq_timestamp, 
	                                      int sensor_mask, 
	                                      uint64_t * timestamp)
{
	enum inv_icm426xx_sensor sensor_ref;
	float dt_fifo;
	int64_t dt_us;

	// by default we assume there is nothing in FIFO. timestamp is reset.
	*timestamp = 0;

	// If FSYNC event is received, the FSYNC delay will replace the timestamp field
	if (sensor_mask & (1 << INV_ICM426XX_SENSOR_FSYNC_EVENT)) { 
		// Use timestamp from MCU
		*timestamp = irq_timestamp;
		clk_cal->last_fifo_timestamp[INV_ICM426XX_SENSOR_ACCEL] = 0xDEADBEEF;
		clk_cal->last_fifo_timestamp[INV_ICM426XX_SENSOR_GYRO] = 0xDEADBEEF;
		clk_cal->dt_error = 0; // reset error on deltatime*/
		return 0;
	} else if (!inv_icm426xx_get_clkin_rtc_status(s)) { 
		// Update calibration coefficient unless RTC is enabled
		clock_calibration_update(s, clk_cal, irq_timestamp, cur_fifo_timestamp);	
	}

	// Select which sensor to use for reference (sensor_ref) depending on FIFO content
	if ((sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL)) && (sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))) {
		// ACC and GYR are in FIFO, use the fastest sensor
		sensor_ref = get_fastest_sensor(s);
	} else if (sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL)) {
		// ACC in FIFO
		sensor_ref = INV_ICM426XX_SENSOR_ACCEL; 
	} else if (sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO)) {
		// GYR in FIFO
		sensor_ref = INV_ICM426XX_SENSOR_GYRO; 
	} else {
		// Only temperature is in the FIFO, use MCU timestamp
		*timestamp = irq_timestamp;
		clk_cal->dt_error = 0; // reset error on deltatime
		return 0; // return here, no need to execute the rest
	}

	if(clk_cal->last_fifo_timestamp[sensor_ref] == 0xDEADBEEF) {
		// no last timestamp from FIFO to compute a valid delta, so timestamp from FIFO not used
		*timestamp = irq_timestamp;
		clk_cal->dt_error = 0; // reset error on deltatime
	} else {
		// last timestamp from FIFO is available to compute a valid dt based on read timestamp from FIFO
		dt_fifo = (float)(cur_fifo_timestamp - (uint16_t)clk_cal->last_fifo_timestamp[sensor_ref]);
		
		// Handle rollover
		if (dt_fifo <= 0)
			dt_fifo += (UINT16_MAX+1);

		// Apply proper timestamp resolution
		dt_fifo *= get_fifo_timestamp_resolution(s);

		// Apply calibration coefficient
		dt_us = apply_time_factor(s, clk_cal, dt_fifo);

		// Compute timestamp
		*timestamp = clk_cal->last_timestamp_sent[sensor_ref] + dt_us;
	}
	
	if (sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL)) {
		clk_cal->last_fifo_timestamp[INV_ICM426XX_SENSOR_ACCEL] = cur_fifo_timestamp;
		clk_cal->last_timestamp_sent[INV_ICM426XX_SENSOR_ACCEL] = *timestamp;
	}

	if (sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO)) {
		clk_cal->last_fifo_timestamp[INV_ICM426XX_SENSOR_GYRO] = cur_fifo_timestamp;
		clk_cal->last_timestamp_sent[INV_ICM426XX_SENSOR_GYRO] = *timestamp;
	}

	return 0;
}

void clock_calibration_reset(struct inv_icm426xx * s, struct clk_calib *clk_cal)
{
	/* Let's compute the number of samples before calibration based on the ODR configured at POR */
	clk_cal->initial_recalib_after_n_samples = NUMBER_OF_SAMPLES_FOR_ODR(inv_icm426xx_convert_odr_bitfield_to_us(ICM426XX_ACCEL_CONFIG0_ODR_200_HZ));
	clk_cal->recalib_after_n_samples = clk_cal->initial_recalib_after_n_samples;
	clk_cal->on_going = 0;
	clk_cal->dt_error = 0;

	clock_calibration_reset_sensors_stats(s, clk_cal, INV_ICM426XX_SENSOR_ACCEL);
	clock_calibration_reset_sensors_stats(s, clk_cal, INV_ICM426XX_SENSOR_GYRO);
}

void clock_calibration_reset_sensors_stats(struct inv_icm426xx * s, struct clk_calib *clk_cal, enum inv_icm426xx_sensor sensor)
{
	clk_cal->last_timestamp_sent[sensor] = 0;
	clk_cal->last_fifo_timestamp[sensor] = 0xDEADBEEF;
}


int clock_calibration_restart(struct inv_icm426xx * s, struct clk_calib *clk_cal)
{
	int rc;
	uint32_t smallest_running_odr_us;
		
	if ((rc = get_fastest_running_odr(s, &smallest_running_odr_us)) != 0)
		return rc;
	
	/* in case no valid odr was returned, let's use the default odr value */
	if(smallest_running_odr_us == 0xffffffff)
		smallest_running_odr_us = 10;
	
	clk_cal->recalib_after_n_samples = NUMBER_OF_SAMPLES_FOR_ODR(smallest_running_odr_us);
	clk_cal->on_going = 0;
	
	return rc;
}


int clock_calibration_init(struct inv_icm426xx * s, struct clk_calib *clk_cal)
{
	int rc;
	
	clock_calibration_reset(s, clk_cal);
	rc = clock_calibration_initial(s, clk_cal);
	
	return rc;
}

static float compute_calibration_coefficient(struct inv_icm426xx * s, uint32_t duration_us)
{
	uint32_t tICM_1, tICM_2;
	uint64_t tMCU_1, tMCU_2;
	uint8_t nb_rollover;
	float tICM_rollover;
	float tICM_1f, tICM_2f;

	/* Get the MCU and ICM time */
	//portENTER_CRITICAL(&mux);;
	tMCU_1 = inv_icm426xx_get_time_us();
	inv_icm426xx_get_current_timestamp(s, &tICM_1);
	//portEXIT_CRITICAL(&mux);;

	/* Wait 200ms */
	inv_icm426xx_sleep_us(duration_us);

	/* Get the MCU and ICM time once again */
	//portENTER_CRITICAL(&mux);;
	tMCU_2 = inv_icm426xx_get_time_us();
	inv_icm426xx_get_current_timestamp(s, &tICM_2);
	//portEXIT_CRITICAL(&mux);;
	
	tICM_2f = (float)tICM_2 * get_reg_timestamp_resolution(s);
	tICM_1f = (float)tICM_1 * get_reg_timestamp_resolution(s);
	tICM_rollover = (float) ROLLOVER_20BITS * get_reg_timestamp_resolution(s);

	/* Compute the time factor in order to align the MCU time to the icm426xx timestamp outputted from the FIFO */
	nb_rollover = (uint8_t)((tMCU_2 - tMCU_1) / tICM_rollover);

	/* If tICM_2 is smaller than tICM_1, there was one more rollover than what we estimated */
	if (tICM_2f < tICM_1f)
		nb_rollover += 1;

	return (float) (tMCU_2 - tMCU_1) / (float) (nb_rollover * tICM_rollover + tICM_2f - tICM_1f);
}

static int clock_calibration_initial(struct inv_icm426xx * s, struct clk_calib *clk_cal)
{
	int status = 0;
	uint8_t prev_src0, prev_src8, data;
	
	/*
	 * Enable the ICM time register reading
	 */
	status |= inv_icm426xx_enable_timestamp_to_register(s);

	/*
	 * Power ON the ICM
	 * Enable gyro (the gyro clock is used for combo accel/gyro clocking)
	 * But disable interrupt since we don't want the data here
	 */
	data = 0;
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE0, 1, &prev_src0);
	status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE0, 1, &data);
	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE8_B4, 1, &prev_src8);
	status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE8_B4, 1, &data);
	status |= inv_icm426xx_set_reg_bank(s, 0);

	/* Compute coefficient for PLL */ 
	status |= inv_icm426xx_enable_gyro_low_noise_mode(s);
	/* Wait 40 ms for MEMS start-up */
	inv_icm426xx_sleep_us(40 * 1000);
	clk_cal->coef[INV_ICM426XX_PLL] = compute_calibration_coefficient(s, 200000); // 200 msec
	status |= inv_icm426xx_disable_gyro(s);

	/* Compute coefficient for RC osc*/
	status |= inv_icm426xx_enable_accel_low_noise_mode(s);
	/* Wait 40 ms for MEMS start-up */
	inv_icm426xx_sleep_us(40 * 1000);
	clk_cal->coef[INV_ICM426XX_RC_OSC] = compute_calibration_coefficient(s, 200000); // 200 msec
	status |= inv_icm426xx_disable_accel(s);

	/* Coefficient for WU osc can't be measured as timestamp register is not reliable in this mode */
	clk_cal->coef[INV_ICM426XX_WU_OSC] = clk_cal->coef[INV_ICM426XX_RC_OSC];

	INV_MSG(INV_MSG_LEVEL_DEBUG, "HelperClockCalib: Initial coefficient computed:");
	INV_MSG(INV_MSG_LEVEL_DEBUG, "HelperClockCalib:    - PLL:    %f", clk_cal->coef[INV_ICM426XX_PLL]);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "HelperClockCalib:    - RC_OSC: %f", clk_cal->coef[INV_ICM426XX_RC_OSC]);
	INV_MSG(INV_MSG_LEVEL_DEBUG, "HelperClockCalib:    - WU_OSC: %f", clk_cal->coef[INV_ICM426XX_WU_OSC]);

	/* Disable the 20-bits timestamp register reading */
	status |= inv_icm426xx_disable_timestamp_to_register(s);

	/* Re-enable interrupts */
	status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE0, 1, &prev_src0);
	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE8_B4, 1, &prev_src8);
	status |= inv_icm426xx_set_reg_bank(s, 0);
	
	return status;
}

/*
 * Update the calibration factor after N samples (60 x 20ms at default ODR, every 1200ms)
 */
int clock_calibration_update(struct inv_icm426xx * s, struct clk_calib *clk_cal, uint64_t irq_timestamp, uint16_t fifo_timestamp)
{
	int rc = 0;

	if((--clk_cal->recalib_after_n_samples) == 0) {
		uint32_t smallest_running_odr_us;
		
		rc = clock_calibration_update_coef(s, clk_cal, &clk_cal->on_going, irq_timestamp, fifo_timestamp);
		
		/* now find the smallest running odr to program new calibration */
		if ((rc = get_fastest_running_odr(s, &smallest_running_odr_us)) != 0) {
			/* in case off error we set recalib n sample to the initial one */
			clk_cal->recalib_after_n_samples = clk_cal->initial_recalib_after_n_samples;
			return rc;
		}
		/* in case no valid odr was returned, let's use the default odr value */
		if(smallest_running_odr_us == 0xffffffff)
			smallest_running_odr_us = 10;	
		
		clk_cal->recalib_after_n_samples = NUMBER_OF_SAMPLES_FOR_ODR(smallest_running_odr_us);
	}
	
	return rc;
}

static int clock_calibration_update_coef(struct inv_icm426xx * s, struct clk_calib *clk_cal, uint8_t *on_going, uint64_t irq_timestamp, uint16_t fifo_timestamp)
{
	int status = 0;
	uint64_t tMCU_2;
	uint32_t tICM_2;
	float tICM_1f, tICM_2f;
	float computed_coef_time, tICM_rollover;
	uint8_t nb_rollover;
	/* These variables needs to be static since we assign them in stage 1 and use them in stage 2 */
	static uint64_t tMCU_1;
	static uint32_t tICM_1;
	static enum inv_icm426xx_clock_source cur_source;
	
	
	if(*on_going == 0) {
		/* Stage 1 */ 

		/* Set initial MCU and ICM time */
		tICM_1 = fifo_timestamp;
		tMCU_1 = irq_timestamp;
		*on_going = 1;
		cur_source = get_current_clock_source(s);
	} else if(*on_going == 1) {
		/* Stage 2 */ 

		/* If the clock source has changed between both stages, do not compute new coef */
		if (cur_source != get_current_clock_source(s)) {
			*on_going = 0;
			return status;
		}

		/* Set second MCU and ICM time */
		tICM_2 = fifo_timestamp;
		tMCU_2 = irq_timestamp;

		tICM_1f = (float)tICM_1 * get_fifo_timestamp_resolution(s);
		tICM_2f = (float)tICM_2 * get_fifo_timestamp_resolution(s);
		tICM_rollover = (float) ROLLOVER_16BITS * get_fifo_timestamp_resolution(s);

		/* Estimate number of rollover of the FIFO timestamp */
		nb_rollover = (uint8_t)((tMCU_2 - tMCU_1) / tICM_rollover);

		/* If tICM_2 is smaller than tICM_1, there was one more rollover than what we estimated */
		if (tICM_2f < tICM_1f)
			nb_rollover += 1;

		computed_coef_time = (tMCU_2 - tMCU_1) / (float) (nb_rollover * tICM_rollover + tICM_2f - tICM_1f);

		/* error management : only allow 90-110% variation */
		if ( (computed_coef_time < 1.1) && (computed_coef_time > 0.9) ) {
			clk_cal->coef[get_current_clock_source(s)] = computed_coef_time;
			INV_MSG(INV_MSG_LEVEL_DEBUG, "helperClockCalib: New coefficient computed for %s: %f", 
				get_current_clock_source_name(s), clk_cal->coef[get_current_clock_source(s)]);
		} else
			INV_MSG(INV_MSG_LEVEL_ERROR, "helperClockCalib: Bad coefficient computed for %s: %f, skipping it and keeping %f", 
				get_current_clock_source_name(s), computed_coef_time, clk_cal->coef[get_current_clock_source(s)]);

		*on_going = 0;
	}
	return status;
}

static int get_fastest_running_odr(struct inv_icm426xx * s, uint32_t *odr)
{
	int rc = 0;
	uint8_t accel_cfg_0_reg, gyro_cfg_0_reg, pwr_mngt_0_reg;

	uint32_t smallest_odr = 0xffffffff, accel_odr_us, gyro_odr_us;
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t gyro_mode;
	ICM426XX_ACCEL_CONFIG0_ODR_t accel_odr;
	ICM426XX_GYRO_CONFIG0_ODR_t gyro_odr;

	/* Access current sensors ODR */
	rc |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	rc |= inv_icm426xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &gyro_cfg_0_reg);
	rc |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	accel_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	gyro_mode  = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	gyro_odr   = (ICM426XX_GYRO_CONFIG0_ODR_t)(gyro_cfg_0_reg & BIT_GYRO_CONFIG0_ODR_MASK);
	accel_odr  = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
	accel_odr_us = inv_icm426xx_convert_odr_bitfield_to_us(accel_odr);
	gyro_odr_us = inv_icm426xx_convert_odr_bitfield_to_us(gyro_odr);

	/* now let's find the smallest ODR of running sensors among ACC and GYR */
	if (accel_mode != ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF)
		smallest_odr = accel_odr_us;

	if (gyro_mode != ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF)
		if (gyro_odr_us < smallest_odr)
			smallest_odr = gyro_odr_us;

	*odr = smallest_odr;

	return rc;
}

static enum inv_icm426xx_sensor get_fastest_sensor(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t accel_cfg_0_reg, gyro_cfg_0_reg;
	
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	status |= inv_icm426xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &gyro_cfg_0_reg);
	
	if (0 > status)
		return INV_ICM426XX_SENSOR_MAX; // error
	
	ICM426XX_ACCEL_CONFIG0_ODR_t accel_odr = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
	ICM426XX_GYRO_CONFIG0_ODR_t gyro_odr   = (ICM426XX_GYRO_CONFIG0_ODR_t)(gyro_cfg_0_reg & BIT_GYRO_CONFIG0_ODR_MASK);
	uint32_t accel_odr_us = inv_icm426xx_convert_odr_bitfield_to_us(accel_odr);
	uint32_t gyro_odr_us  = inv_icm426xx_convert_odr_bitfield_to_us(gyro_odr);

	return (accel_odr_us > gyro_odr_us) ? INV_ICM426XX_SENSOR_GYRO : INV_ICM426XX_SENSOR_ACCEL;
}

/* Convert a period in usec in a frequency */
uint32_t period_us_to_frequency(const uint32_t period_us)
{
	uint32_t frequency = (1000000 / period_us);
	
	/* Round up frequency */
	if (period_us != 1000000 / frequency)
		frequency++;

	return frequency;
}
