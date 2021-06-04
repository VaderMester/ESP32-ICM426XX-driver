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
/*
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxExtFunc.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxVersion.h"
*/
#include "Icm426xxDefs.h"
#include "Icm426xxExtFunc.h"
#include "Icm426xxDriver_HL.h"
#include "Icm426xxTransport.h"
#include "Icm426xxVersion.h"


static int inv_icm426xx_configure_serial_interface(struct inv_icm426xx * s);
static int inv_icm426xx_init_hardware_from_ui(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t *config_int);
static int inv_icm426xx_is_wu_osc_active(struct inv_icm426xx * s);

int inv_icm426xx_set_reg_bank(struct inv_icm426xx * s, uint8_t bank)
{
	return inv_icm426xx_write_reg(s, MPUREG_REG_BANK_SEL, 1, &bank);
}

int inv_icm426xx_init(struct inv_icm426xx * s, struct inv_icm426xx_serif * serif, void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event))
{
	int status = 0;

	memset(s, 0, sizeof(*s));
	
	s->transport.serif = *serif;
	
	/* Wait some time for ICM to be properly supplied */
	inv_icm426xx_sleep_us(3000);
	
	if((status |= inv_icm426xx_configure_serial_interface(s)) != 0 )
		return status;

	/* Register the callback to be executed each time inv_icm426xx_get_data_from_fifo extracts 
	 * a packet from fifo or inv_icm426xx_get_data_from_registers read data 
	 */
	s->sensor_event_cb = sensor_event_cb;
	
	/* initialize hardware */
	status |= inv_icm426xx_init_hardware_from_ui(s, NULL);
		
	/* First data are noisy after enabling sensor
	 * This variable keeps track of gyro start time. Set to UINT32_MAX at init 
	 */
	s->gyro_start_time_us = UINT32_MAX;
	/* First data are noisy after enabling sensor
	 * This variable keeps track of accel start time. Set to UINT32_MAX at init 
	 */
	s->accel_start_time_us = UINT32_MAX;

	/* Gyro power-off to power-on transition can cause ring down issue
	 * This variable keeps track of timestamp when gyro is power off. Set to UINT32_MAX at init
	 */
	s->gyro_power_off_tmst = UINT32_MAX;

	return status;
}

int inv_icm426xx_device_reset(struct inv_icm426xx * s)
{
	int status = INV_ERROR_SUCCESS;
	uint8_t data;
	uint8_t intf_cfg4_reg, intf_cfg6_reg;

	/* Set memory bank 1 */
	status |= inv_icm426xx_set_reg_bank(s, 1);
	/* save registers necessary to perform soft reset while still keeping communication link alive */
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &intf_cfg4_reg); // AUX SPI and AP SPI fields
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG6_B1, 1, &intf_cfg6_reg);
	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);
	
	/* Reset the internal registers and restores the default settings.
	 * The bit automatically clears to 0 once the reset is done.
	 * Since soft-reset will trigger OTP reload, SPI mode (bit4) does not need saving
	 */
	data = ICM426XX_DEVICE_CONFIG_RESET_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_DEVICE_CONFIG, 1, &data);
	if(status)
		return status;

	/* Wait 1000us for soft reset to be effective before trying to perform any further read */
	inv_icm426xx_sleep_us(1000);

	status |= inv_icm426xx_set_reg_bank(s, 1);
	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &intf_cfg4_reg);
	status |= inv_icm426xx_set_reg_bank(s, 0);

	/* Check the Int Reset Done bit */
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_STATUS, 1, &data);
	if (0 == (data & BIT_INT_STATUS_RESET_DONE)) {
		return INV_ERROR_UNEXPECTED;
	}

	/* Init transport layer */
	inv_icm426xx_init_transport(s);
	
	status |= inv_icm426xx_set_reg_bank(s, 1);
	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG6_B1, 1, &intf_cfg6_reg);
	/* Configure FSYNC on INT2=pin 9 */
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG5_B1, 1, &data);
	data &= (uint8_t)~BIT_INTF_CONFIG5_GPIO_PAD_SEL_MASK;
	data |= (1 << BIT_INTF_CONFIG5_GPIO_PAD_SEL_POS);
	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG5_B1, 1, &data);
	status |= inv_icm426xx_set_reg_bank(s, 0);

	/* Read and set endianess for further processing */
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &data);
	s->endianess_data = data & BIT_DATA_ENDIAN_MASK;

	if(s->transport.serif.serif_type == ICM426XX_UI_I3C){
		status |= s->transport.serif.configure((struct inv_icm426xx_serif *)s);
	}

	return status;
}

int inv_icm426xx_get_who_am_i(struct inv_icm426xx * s, uint8_t * who_am_i)
{
	return inv_icm426xx_read_reg(s, MPUREG_WHO_AM_I, 1, who_am_i);
}

int inv_icm426xx_force_clock_source(struct inv_icm426xx * s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_t clk_src)
{
	int status = 0;
	uint8_t data;
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
	if(clk_src == ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC)
		data &= (uint8_t)~BIT_ACCEL_LP_CLK_SEL_MASK;
	else
		data |= (uint8_t)ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC;

	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
	
	return status;
}

int inv_icm426xx_enable_accel_low_power_mode(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;
	uint8_t pwr_mgmt0_reg;
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;
	ICM426XX_ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint32_t accel_odr_us = 0;	
	uint8_t accel_cfg_0_reg;

	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt0_reg);
	accel_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mgmt0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	gyro_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)(pwr_mgmt0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);

	/* Subroutines need to be followed when enabling or disabling sensor to ensure ODR regularity
	 * Check if the accelerometer is the only one enabled 
	 */
	if ((accel_mode != ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) && 
	    ((gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) || (gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_STANDBY))){
		/* Get accelerometer's ODR for next required wait */
		status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
		acc_odr_bitfield = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
		accel_odr_us = inv_icm426xx_convert_odr_bitfield_to_us(acc_odr_bitfield);
		// Select the RC OSC as clock source for the accelerometer
		status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC);
	}

	/* FIFO contains Gyro and Accel data if enabled on the OIS path
	 * Dynamically configure the FIFO to publish data only for sensors explicitely enabled on the UI path 
	 */
	if(accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF && gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		uint8_t data_endianess;
		if(s->fifo_is_used) {
			status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_ACCEL_EN;
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_GYRO_EN;
			if(s->fifo_highres_enabled)
				data |= (uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
		}

		/* Read data endianess in order to process correctly data*/
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &data_endianess);
		s->endianess_data = data_endianess & BIT_DATA_ENDIAN_MASK;
	}
#endif

	/* Restore filter averaging settings */
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= s->avg_bw_setting.acc_lp_avg;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	
	/* Enable/Switch the accelerometer in/to low power mode */
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt0_reg);
	pwr_mgmt0_reg &= ~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	pwr_mgmt0_reg |= ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP;
	status |= inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt0_reg);
	inv_icm426xx_sleep_us(200);
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	/* Subroutines need to be followed when enabling or disabling sensor to ensure ODR regularity */
	if ((accel_mode != ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) && 
	    ((gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) || (gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_STANDBY))){
		/* Wait one accelerometer ODR before switching to the WU OSC */
		if (accel_odr_us > 200) /* if ODR is smaller than 200 us, we already waited for one ODR above */
			inv_icm426xx_sleep_us(accel_odr_us - 200);
		status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC);
	}
	
	/* Accel Low Power could report with wrong ODR if internal counter for ODR changed overflowed */
	if (ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP != accel_mode) {
		/* Check whether counter has overflowed */
		if(s->wu_off_acc_odr_changes >= 8) {
			ICM426XX_ACCEL_CONFIG0_ODR_t cur_freq, new_freq;
			/* Dummy transition to ODR valid for both ALN/ALP */
			status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
			cur_freq = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
			new_freq = (cur_freq != ICM426XX_ACCEL_CONFIG0_ODR_1_5625_HZ) ? 
				ICM426XX_ACCEL_CONFIG0_ODR_1_5625_HZ : ICM426XX_ACCEL_CONFIG0_ODR_3_125_HZ;
			accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
			accel_cfg_0_reg |= (uint8_t)new_freq;
			status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
			inv_icm426xx_sleep_us(200);
			accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
			accel_cfg_0_reg |= (uint8_t)cur_freq;
			status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
		}
	}
	/* Even though transition was a dummy and controller received two ODR updates,
	 * it was enabled. Therefore they were not missed, so it is safe to reset counter.
	 */
	s->wu_off_acc_odr_changes = 0;

	if (accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF) {
		/* First data are noisy after enabling sensor 
		 * Keeps track of the start time to discard first sample
		 */
		if(s->fifo_is_used) {
			s->accel_start_time_us = inv_icm426xx_get_time_us();
		}
	}
#endif

	return status;
}

int inv_icm426xx_enable_accel_low_noise_mode(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;
	uint8_t pwr_mgmt0_reg;
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;
	ICM426XX_ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint32_t accel_odr_us;
	uint8_t accel_cfg_0_reg;

	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt0_reg);
	accel_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mgmt0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	gyro_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)(pwr_mgmt0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);

	/* Subroutines need to be followed when enabling or disabling sensor to ensure ODR regularity
	 * Check if the accelerometer is the only one enabled 
	 */
	if ((accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) && 
	    ((gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) || (gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_STANDBY))){
		/* Get accelerometer's ODR for next required wait */
		status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
		acc_odr_bitfield = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
		accel_odr_us = inv_icm426xx_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC);
		/* Wait one accel ODR before switching to low noise mode */
		inv_icm426xx_sleep_us(accel_odr_us);
	}

	/* FIFO contains Gyro and Accel data if enabled on the OIS path
	 * Dynamically configure the FIFO to publish data only for sensors explicitely enabled on the UI path 
	 */
	if(accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF && gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		uint8_t data_endianess;
		
		if(s->fifo_is_used) {
			status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_ACCEL_EN;
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_GYRO_EN;
			if(s->fifo_highres_enabled)
				data |= (uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
		}
		/* Read data endianess in order to process correctly data */
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &data_endianess);
		s->endianess_data = data_endianess & BIT_DATA_ENDIAN_MASK;
	}
#endif
	
	/* Restore filter BW settings */
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= s->avg_bw_setting.acc_ln_bw;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	
	/* Enable/Switch the accelerometer in/to low noise mode */
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt0_reg);
	pwr_mgmt0_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	pwr_mgmt0_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_ACCEL_MODE_LN;
	status |= inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt0_reg);
	inv_icm426xx_sleep_us(200);

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	if (accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF) {
		/* First data are noisy after enabling sensor 
		 * Keeps track of the start time to discard first sample
		 */
		if(s->fifo_is_used) {
			s->accel_start_time_us = inv_icm426xx_get_time_us();
		}
	}
#endif

	return status;
}

int inv_icm426xx_disable_accel(struct inv_icm426xx * s)
{
	int status=0;
	uint8_t pwr_mngt_0_reg;
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t gyro_mode;
	int stop_fifo_usage = 0;
	uint8_t data;
#endif
	
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	gyro_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	if((gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) && s->fifo_is_used) {
		/* First FSYNC event after enable is irrelevant */
		s->fsync_to_be_ignored = 1;
		/* FIFO contains Gyro and Accel data if enabled on the OIS path
		 * Dynamically configure the FIFO to publish data only for sensors explicitely enabled on the UI path 
		 */
		stop_fifo_usage = 1;
	}
#endif
	
	pwr_mngt_0_reg &= (uint8_t)~BIT_PWR_MGMT_0_ACCEL_MODE_MASK;
	pwr_mngt_0_reg |= (uint8_t) ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF;
	status = inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	
	/* Restore POR clock source for the accelerometer */
	status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC);

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	if(stop_fifo_usage && s->fifo_is_used) {
		status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
		data &= (uint8_t)~(BIT_FIFO_CONFIG1_ACCEL_MASK | BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_HIRES_MASK);
		status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);

		/* Reset FIFO explicitely so the new configuration is taken into account */
		status |= inv_icm426xx_reset_fifo(s);
	}
#endif
	
	return status;
}

int inv_icm426xx_enable_gyro_low_noise_mode(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;
	uint8_t pwr_mngt_0_reg;
	uint64_t current_time;

	/* Powering the gyroscope on immediately after powering it off can result in device failure. 
	 * The gyroscope proof mass can continue vibrating after it has been powered off, 
	 * and powering it back on immediately can result in unpredictable proof mass movement.
	 * After powering the gyroscope off, a period of > 150ms should be allowed to elapse before it is powered back on. */
	if (s->gyro_power_off_tmst != UINT32_MAX) {
		current_time = inv_icm426xx_get_time_us();
		/* Handle rollover */
		if (current_time <= s->gyro_power_off_tmst)
			current_time += UINT32_MAX;
		/* If 150 ms are not elapsed since power-off error is returned */
		if ((current_time - s->gyro_power_off_tmst) <= (150 * 1000))
			return INV_ERROR_HW;
	}
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t  gyro_mode;
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	ICM426XX_ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
	uint32_t accel_odr_us;
	uint8_t accel_cfg_0_reg;
	
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	gyro_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	accel_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);

	/* Subroutines need to be followed when enabling or disabling sensor to ensure ODR regularity
	 * Check if the accelerometer is the only one enabled 
	 */	
	if ((accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) &&
	    ((gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) || (gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_STANDBY))) {
		/* Get accelerometer's ODR for next required wait */
		status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
		acc_odr_bitfield = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
		accel_odr_us = inv_icm426xx_convert_odr_bitfield_to_us(acc_odr_bitfield);
		/* Select the RC OSC as clock source for the accelerometer */
		status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC);
		/* Wait one accel ODR before enabling the gyroscope */
		inv_icm426xx_sleep_us(accel_odr_us);
	}

	/* FIFO contains Gyro and Accel data if enabled on the OIS path
	 * Dynamically configure the FIFO to publish data only for sensors explicitely enabled on the UI path 
	 */
	if(accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF && gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		uint8_t data_endianess;

		if(s->fifo_is_used) {
			status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_ACCEL_EN;
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_GYRO_EN;
			if(s->fifo_highres_enabled)
				data |= (uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
		}
		/* Read data endianess in order to process correctly data */
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &data_endianess);
		s->endianess_data = data_endianess & BIT_DATA_ENDIAN_MASK;
	}
#endif

	/* Restore filter BW settings */
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK;
	data |= s->avg_bw_setting.gyr_ln_bw;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);

	/* Enable/Switch the gyroscope in/to low noise mode */
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	pwr_mngt_0_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
	pwr_mngt_0_reg |= (uint8_t)ICM426XX_PWR_MGMT_0_GYRO_MODE_LN;
	status |= inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	inv_icm426xx_sleep_us(200);
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	if (gyro_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) {
		/* First data are noisy after enabling sensor 
		 * Keeps track of the start time to discard first sample
		 */
		if(s->fifo_is_used) {
			s->gyro_start_time_us = inv_icm426xx_get_time_us();
		}
	}
#endif
	
	return status;
}

int inv_icm426xx_disable_gyro(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t pwr_mngt_0_reg;
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_mode;
	int stop_fifo_usage = 0;
	uint8_t data;
	uint8_t accel_cfg_0_reg;
	ICM426XX_ACCEL_CONFIG0_ODR_t acc_odr_bitfield;
#endif

	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	accel_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	if((accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF) && s->fifo_is_used) {
		/* First FSYNC event after enable is irrelevant */
		s->fsync_to_be_ignored = 1;
		/* FIFO contains Gyro and Accel data if enabled on the OIS path
		 * Dynamically configure the FIFO to publish data only for sensors explicitely enabled on the UI path 
		 */
		stop_fifo_usage = 1;
	}
#endif
	
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	/* Check if the accelerometer is enabled in low power mode */
	if (accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) {
		/* Select the RC OSC as clock source for the accelerometer */
		status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_RCOSC);
	}
#endif
	
	pwr_mngt_0_reg &= (uint8_t)~BIT_PWR_MGMT_0_GYRO_MODE_MASK;
	pwr_mngt_0_reg |= ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF;
	status |= inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	/* keep track of gyro power-off time to check if gyro will be power-on after more than 150ms*/
	s->gyro_power_off_tmst = inv_icm426xx_get_time_us();

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	if (accel_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) {
		/* Wait based on accelerometer ODR */
		status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
		acc_odr_bitfield = (ICM426XX_ACCEL_CONFIG0_ODR_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_ODR_MASK);
		inv_icm426xx_sleep_us(2 * inv_icm426xx_convert_odr_bitfield_to_us(acc_odr_bitfield));
		/* Select the WU OSC as clock source for the accelerometer */
		status |= inv_icm426xx_force_clock_source(s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC);
	}

	if(stop_fifo_usage && s->fifo_is_used) {
		status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
		data &= (uint8_t)~(BIT_FIFO_CONFIG1_ACCEL_MASK | BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_HIRES_MASK);
		status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);

		/* Reset FIFO explicitely so the new configuration is taken into account */
		status |= inv_icm426xx_reset_fifo(s);
	}
#endif
	
	return status;
}

int inv_icm426xx_enable_fsync(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t value;

	/* Enable Fsync */
	status |= inv_icm426xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	value &= (uint8_t)~BIT_TMST_CONFIG_TMST_FSYNC_MASK;
	value |= (uint8_t)ICM426XX_TMST_CONFIG_TMST_FSYNC_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);

	return status;
}

int inv_icm426xx_disable_fsync(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t value;

	/* Disable Fsync */
	status |= inv_icm426xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	value &= (uint8_t)~BIT_TMST_CONFIG_TMST_FSYNC_MASK;
	value |= (uint8_t)ICM426XX_TMST_CONFIG_TMST_FSYNC_DIS;
	status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	

	return status;
}

int inv_icm426xx_configure_timestamp_resolution(struct inv_icm426xx * s, ICM426XX_TMST_CONFIG_RESOL_t resol)
{
	int status = 0;
	uint8_t value;

	status |= inv_icm426xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &value);
	value &= (uint8_t)~BIT_TMST_CONFIG_RESOL_MASK;
	value |= (uint8_t)resol;
	status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &value);

	return status;
}

int inv_icm426xx_get_config_ibi(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure)
{
	int status = 0;
	uint8_t data[3] = {0};

	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE8_B4, 3, data); /* burst read int_source8/int_source9/int_source10 */
	status |= inv_icm426xx_set_reg_bank(s, 0);

	/* Handles INT_SOURCE8 bits */
	interrupt_to_configure->INV_ICM426XX_UI_FSYNC  = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE8_UI_FSYNC_IBI_EN)  >> BIT_INT_UI_FSYNC_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_UI_DRDY   = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE8_UI_DRDY_IBI_EN)   >> BIT_INT_UI_DRDY_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_FIFO_THS  = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE8_FIFO_THS_IBI_EN)  >> BIT_INT_FIFO_THS_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_FIFO_FULL = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE8_FIFO_FULL_IBI_EN) >> BIT_INT_FIFO_FULL_IBI_EN_POS);

	/* Handles INT_SOURCE9 bits */
	interrupt_to_configure->INV_ICM426XX_SMD   = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE9_SMD_IBI_EN )   >> BIT_INT_SMD_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WOM_X = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE9_WOM_X_IBI_EN ) >> BIT_INT_WOM_X_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WOM_Y = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE9_WOM_Y_IBI_EN ) >> BIT_INT_WOM_Y_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WOM_Z = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE9_WOM_Z_IBI_EN ) >> BIT_INT_WOM_Z_IBI_EN_POS);

	/* Handles INT_SOURCE10 bits */
	interrupt_to_configure->INV_ICM426XX_STEP_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE10_STEP_DET_IBI_EN)      >> BIT_INT_STEP_DET_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE10_STEP_CNT_OVFL_IBI_EN) >> BIT_INT_STEP_CNT_OVFL_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_TILT_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE10_TILT_DET_IBI_EN)      >> BIT_INT_TILT_DET_IBI_EN_POS);
	interrupt_to_configure->INV_ICM426XX_TAP_DET       = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE10_TAP_DET_IBI_EN)       >> BIT_INT_TAP_DET_IBI_EN_POS);

	return status;
}

int inv_icm426xx_get_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure)
{
	int status = 0;
	uint8_t data[4] = {0};

	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE0, 2, data); /* burst read int_source0/int_source1 */
	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE6_B4, 1, &data[2]); /* int_source6 */
	status |= inv_icm426xx_set_reg_bank(s, 0);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_CONFIG, 1, &data[3]);

	if(!status) {
	/* Handles INT_SOURCE0 bits */
	interrupt_to_configure->INV_ICM426XX_UI_FSYNC  = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_UI_FSYNC_INT1_EN)  >> BIT_INT_UI_FSYNC_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_UI_DRDY   = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_UI_DRDY_INT1_EN)   >> BIT_INT_UI_DRDY_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_FIFO_THS  = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_FIFO_THS_INT1_EN)  >> BIT_INT_FIFO_THS_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_FIFO_FULL = (inv_icm426xx_interrupt_value) ((data[0] & BIT_INT_SOURCE0_FIFO_FULL_INT1_EN) >> BIT_INT_FIFO_FULL_INT_EN_POS);

	/* Handles INT_SOURCE1 bits */
	interrupt_to_configure->INV_ICM426XX_SMD   = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_SMD_INT1_EN )   >> BIT_INT_SMD_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WOM_X = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_WOM_X_INT1_EN ) >> BIT_INT_WOM_X_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WOM_Y = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_WOM_Y_INT1_EN ) >> BIT_INT_WOM_Y_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WOM_Z = (inv_icm426xx_interrupt_value) ((data[1] & BIT_INT_SOURCE1_WOM_Z_INT1_EN ) >> BIT_INT_WOM_Z_INT_EN_POS);

	/* Handles INT_SOURCE6 bits */
	interrupt_to_configure->INV_ICM426XX_STEP_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_STEP_DET_INT1_EN)      >> BIT_INT_STEP_DET_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_STEP_CNT_OVFL_INT1_EN) >> BIT_INT_STEP_CNT_OVFL_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_TILT_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_TILT_DET_INT1_EN)      >> BIT_INT_TILT_DET_INT_EN_POS);
#if defined(ICM_FAMILY_BPLUS)
	interrupt_to_configure->INV_ICM426XX_SLEEP_DET     = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_SLEEP_DET_INT1_EN)     >> BIT_INT_SLEEP_DET_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_WAKE_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_WAKE_DET_INT1_EN)      >> BIT_INT_WAKE_DET_INT_EN_POS);
#elif defined(ICM_FAMILY_CPLUS)
	interrupt_to_configure->INV_ICM426XX_FF_DET        = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_FF_DET_INT1_EN)        >> BIT_INT_FF_DET_INT_EN_POS);
	interrupt_to_configure->INV_ICM426XX_LOWG_DET      = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_LOWG_DET_INT1_EN)      >> BIT_INT_LOWG_DET_INT_EN_POS);
#endif	
	interrupt_to_configure->INV_ICM426XX_TAP_DET       = (inv_icm426xx_interrupt_value) ((data[2] & BIT_INT_SOURCE6_TAP_DET_INT1_EN)       >> BIT_INT_TAP_DET_INT_EN_POS);

	/* Handles INT_COMFIG bits */
	interrupt_to_configure->int_pol = (ICM426XX_INT_CONFIG_INT1_POLARITY_t) ((data[3] & ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH));
	interrupt_to_configure->int_drive = (ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_t)((data[3] & ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP) >> BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_POS);
	interrupt_to_configure->int_mode = (ICM426XX_INT_CONFIG_INT1_MODE_t)((data[3] & ICM426XX_INT_CONFIG_INT1_LATCHED) >> BIT_INT_CONFIG_INT1_MODE_POS);
	}
	return status;
}

int inv_icm426xx_get_config_int2(struct inv_icm426xx *s, inv_icm426xx_interrupt_parameter_t *interrupt_to_configure)
{
	int status = 0;
	uint8_t data[4] = {0};

	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE3, 2, data); /* burst read int_source3/int_source4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE7_B4, 1, &data[2]); /* int_source7 */
	status |= inv_icm426xx_set_reg_bank(s, 0);

	if (!status)
	{
		/* Handles INT_SOURCE3 bits */
		interrupt_to_configure->INV_ICM426XX_UI_FSYNC = (inv_icm426xx_interrupt_value)((data[0] & BIT_INT_SOURCE3_UI_FSYNC_INT2_EN) >> BIT_INT_UI_FSYNC_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_UI_DRDY = (inv_icm426xx_interrupt_value)((data[0] & BIT_INT_SOURCE3_UI_DRDY_INT2_EN) >> BIT_INT_UI_DRDY_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_FIFO_THS = (inv_icm426xx_interrupt_value)((data[0] & BIT_INT_SOURCE3_FIFO_THS_INT2_EN) >> BIT_INT_FIFO_THS_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_FIFO_FULL = (inv_icm426xx_interrupt_value)((data[0] & BIT_INT_SOURCE3_FIFO_FULL_INT2_EN) >> BIT_INT_FIFO_FULL_INT_EN_POS);

		/* Handles INT_SOURCE4 bits */
		interrupt_to_configure->INV_ICM426XX_SMD = (inv_icm426xx_interrupt_value)((data[1] & BIT_INT_SOURCE4_SMD_INT2_EN) >> BIT_INT_SMD_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_WOM_X = (inv_icm426xx_interrupt_value)((data[1] & BIT_INT_SOURCE4_WOM_X_INT2_EN) >> BIT_INT_WOM_X_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_WOM_Y = (inv_icm426xx_interrupt_value)((data[1] & BIT_INT_SOURCE4_WOM_Y_INT2_EN) >> BIT_INT_WOM_Y_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_WOM_Z = (inv_icm426xx_interrupt_value)((data[1] & BIT_INT_SOURCE4_WOM_Z_INT2_EN) >> BIT_INT_WOM_Z_INT_EN_POS);

		/* Handles INT_SOURCE7 bits */
		interrupt_to_configure->INV_ICM426XX_STEP_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_STEP_DET_INT2_EN) >> BIT_INT_STEP_DET_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_STEP_CNT_OVFL_INT2_EN) >> BIT_INT_STEP_CNT_OVFL_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_TILT_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_TILT_DET_INT2_EN) >> BIT_INT_TILT_DET_INT_EN_POS);
#if defined(ICM_FAMILY_BPLUS)
		interrupt_to_configure->INV_ICM426XX_SLEEP_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_SLEEP_DET_INT2_EN) >> BIT_INT_SLEEP_DET_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_WAKE_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_WAKE_DET_INT2_EN) >> BIT_INT_WAKE_DET_INT_EN_POS);
#elif defined(ICM_FAMILY_CPLUS)
		interrupt_to_configure->INV_ICM426XX_FF_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_FF_DET_INT2_EN) >> BIT_INT_FF_DET_INT_EN_POS);
		interrupt_to_configure->INV_ICM426XX_LOWG_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_LOWG_DET_INT2_EN) >> BIT_INT_LOWG_DET_INT_EN_POS);
#endif
		interrupt_to_configure->INV_ICM426XX_TAP_DET = (inv_icm426xx_interrupt_value)((data[2] & BIT_INT_SOURCE7_TAP_DET_INT2_EN) >> BIT_INT_TAP_DET_INT_EN_POS);

		/*Handles INT_COMFIG bits */
		interrupt_to_configure->int_pol = (ICM426XX_INT_CONFIG_INT2_POLARITY_t)((data[3] & ICM426XX_INT_CONFIG_INT2_POLARITY_HIGH)) >> BIT_INT_CONFIG_INT2_POLARITY_POS;
		interrupt_to_configure->int_drive = (ICM426XX_INT_CONFIG_INT2_DRIVE_CIRCUIT_t)((data[3] & ICM426XX_INT_CONFIG_INT2_DRIVE_CIRCUIT_PP) >> BIT_INT_CONFIG_INT2_DRIVE_CIRCUIT_POS);
		interrupt_to_configure->int_mode = (ICM426XX_INT_CONFIG_INT2_MODE_t)((data[3] & ICM426XX_INT_CONFIG_INT2_LATCHED) >> BIT_INT_CONFIG_INT2_MODE_POS);
	}
	return status;
}

int inv_icm426xx_set_config_ibi(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure)
{
	int status = 0;
	uint8_t data[3] = {0};

	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE8_B4, 3, data); /* burst read int_source8/int_source9/int_source10 */

	/* Set INT_SOURCE8 bits */
	data[0] &= (uint8_t)~(BIT_INT_SOURCE8_UI_FSYNC_IBI_EN 
			| BIT_INT_SOURCE8_UI_DRDY_IBI_EN 
			| BIT_INT_SOURCE8_FIFO_THS_IBI_EN 
			| BIT_INT_SOURCE8_FIFO_FULL_IBI_EN);
	data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_FSYNC != 0)  << BIT_INT_UI_FSYNC_IBI_EN_POS);
	data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_DRDY != 0)   << BIT_INT_UI_DRDY_IBI_EN_POS);
	data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_THS != 0)  << BIT_INT_FIFO_THS_IBI_EN_POS);
	data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_FULL != 0) << BIT_INT_FIFO_FULL_IBI_EN_POS);

	/* Set INT_SOURCE9 bits */
	data[1] &= (uint8_t)~(BIT_INT_SOURCE9_SMD_IBI_EN 
			| BIT_INT_SOURCE9_WOM_X_IBI_EN 
			| BIT_INT_SOURCE9_WOM_Y_IBI_EN 
			| BIT_INT_SOURCE9_WOM_Z_IBI_EN);
	data[1] |= ((interrupt_to_configure->INV_ICM426XX_SMD != 0)   << BIT_INT_SMD_IBI_EN_POS);
	data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_X != 0) << BIT_INT_WOM_X_IBI_EN_POS);
	data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Y != 0) << BIT_INT_WOM_Y_IBI_EN_POS);
	data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Z != 0) << BIT_INT_WOM_Z_IBI_EN_POS);

	/* Set INT_SOURCE10 bits */
	data[2] &= (uint8_t)~(BIT_INT_SOURCE10_STEP_DET_IBI_EN
			| BIT_INT_SOURCE10_STEP_CNT_OVFL_IBI_EN
			| BIT_INT_SOURCE10_TILT_DET_IBI_EN
			| BIT_INT_SOURCE10_TAP_DET_IBI_EN);
	data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_DET != 0)      << BIT_INT_STEP_DET_IBI_EN_POS);
	data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL != 0) << BIT_INT_STEP_CNT_OVFL_IBI_EN_POS);
	data[2] |= ((interrupt_to_configure->INV_ICM426XX_TILT_DET != 0)      << BIT_INT_TILT_DET_IBI_EN_POS);
	data[2] |= ((interrupt_to_configure->INV_ICM426XX_TAP_DET != 0)       << BIT_INT_TAP_DET_IBI_EN_POS);

	status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE8_B4, 3, data); /* burst write int_source8/int_source9/int_source10 */
	status |= inv_icm426xx_set_reg_bank(s, 0);
	
	return status;
}

int inv_icm426xx_set_config_int1(struct inv_icm426xx *s, inv_icm426xx_interrupt_parameter_t *interrupt_to_configure)
{
	int status = 0;
	uint8_t data[4] = {0};

	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE0, 2, &data[0]); /* burst read int_source0/int_source1 */
	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE6_B4, 1, &data[2]); /* switch to bank4 for int_source6 */

	if (!status)
	{
		/* Set INT_SOURCE0 bits */
		data[0] &= (uint8_t) ~(BIT_INT_SOURCE0_UI_FSYNC_INT1_EN | BIT_INT_SOURCE0_UI_DRDY_INT1_EN | BIT_INT_SOURCE0_FIFO_THS_INT1_EN | BIT_INT_SOURCE0_FIFO_FULL_INT1_EN);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_FSYNC != 0) << BIT_INT_UI_FSYNC_INT_EN_POS);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_DRDY != 0) << BIT_INT_UI_DRDY_INT_EN_POS);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_THS != 0) << BIT_INT_FIFO_THS_INT_EN_POS);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_FULL != 0) << BIT_INT_FIFO_FULL_INT_EN_POS);

		/* Set INT_SOURCE1 bits */
		data[1] &= (uint8_t) ~(BIT_INT_SOURCE1_SMD_INT1_EN | BIT_INT_SOURCE1_WOM_X_INT1_EN | BIT_INT_SOURCE1_WOM_Y_INT1_EN | BIT_INT_SOURCE1_WOM_Z_INT1_EN);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_SMD != 0) << BIT_INT_SMD_INT_EN_POS);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_X != 0) << BIT_INT_WOM_X_INT_EN_POS);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Y != 0) << BIT_INT_WOM_Y_INT_EN_POS);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Z != 0) << BIT_INT_WOM_Z_INT_EN_POS);

		/* Set INT_SOURCE6 bits */
		data[2] &= (uint8_t) ~(BIT_INT_SOURCE6_STEP_DET_INT1_EN | BIT_INT_SOURCE6_STEP_CNT_OVFL_INT1_EN | BIT_INT_SOURCE6_TILT_DET_INT1_EN
#if defined(ICM_FAMILY_BPLUS)
							   | BIT_INT_SOURCE6_SLEEP_DET_INT1_EN | BIT_INT_SOURCE6_WAKE_DET_INT1_EN
#elif defined(ICM_FAMILY_CPLUS)
							   | BIT_INT_SOURCE6_FF_DET_INT1_EN | BIT_INT_SOURCE6_LOWG_DET_INT1_EN
#endif
							   | BIT_INT_SOURCE6_TAP_DET_INT1_EN);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_DET != 0) << BIT_INT_STEP_DET_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL != 0) << BIT_INT_STEP_CNT_OVFL_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_TILT_DET != 0) << BIT_INT_TILT_DET_INT_EN_POS);
#if defined(ICM_FAMILY_BPLUS)
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_SLEEP_DET != 0) << BIT_INT_SLEEP_DET_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_WAKE_DET != 0) << BIT_INT_WAKE_DET_INT_EN_POS);
#elif defined(ICM_FAMILY_CPLUS)
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_FF_DET != 0) << BIT_INT_FF_DET_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_LOWG_DET != 0) << BIT_INT_LOWG_DET_INT_EN_POS);
#endif
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_TAP_DET != 0) << BIT_INT_TAP_DET_INT_EN_POS);

		/* SET INT_CONFIG bits */
		data[3] |= (interrupt_to_configure->int_pol != 0);
		data[3] |= ((interrupt_to_configure->int_drive != 0) << BIT_INT_CONFIG_INT1_DRIVE_CIRCUIT_POS);
		data[3] |= ((interrupt_to_configure->int_mode != 0) << BIT_INT_CONFIG_INT1_MODE_POS);

		status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE6_B4, 1, &data[2]); /* start with int_source6 since we are still in bank4 */
		status |= inv_icm426xx_set_reg_bank(s, 0);
		status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE0, 2, &data[0]); /* burst write int_source0/int_source1 */
		status |= inv_icm426xx_write_reg(s, MPUREG_INT_CONFIG, 1, &data[3]);  /* burst write int_source0/int_source1 */
	}
	return status;
}

int inv_icm426xx_set_config_int2(struct inv_icm426xx *s, inv_icm426xx_interrupt_parameter_t *interrupt_to_configure)
{
	int status = 0;
	uint8_t data[4] = {0};

	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE3, 2, data); /* burst read int_source3/int_source4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_SOURCE7_B4, 1, &data[2]); /* switch to bank4 for int_source7 */

	if (!status)
	{
		/* Set INT_SOURCE3 bits */
		data[0] &= (uint8_t) ~(BIT_INT_SOURCE3_UI_FSYNC_INT2_EN | BIT_INT_SOURCE3_UI_DRDY_INT2_EN | BIT_INT_SOURCE3_FIFO_THS_INT2_EN | BIT_INT_SOURCE3_FIFO_FULL_INT2_EN);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_FSYNC != 0) << BIT_INT_UI_FSYNC_INT_EN_POS);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_UI_DRDY != 0) << BIT_INT_UI_DRDY_INT_EN_POS);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_THS != 0) << BIT_INT_FIFO_THS_INT_EN_POS);
		data[0] |= ((interrupt_to_configure->INV_ICM426XX_FIFO_FULL != 0) << BIT_INT_FIFO_FULL_INT_EN_POS);

		/* Set INT_SOURCE4 bits */
		data[1] &= (uint8_t) ~(BIT_INT_SOURCE4_SMD_INT2_EN | BIT_INT_SOURCE4_WOM_X_INT2_EN | BIT_INT_SOURCE4_WOM_Y_INT2_EN | BIT_INT_SOURCE4_WOM_Z_INT2_EN);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_SMD != 0) << BIT_INT_SMD_INT_EN_POS);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_X != 0) << BIT_INT_WOM_X_INT_EN_POS);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Y != 0) << BIT_INT_WOM_Y_INT_EN_POS);
		data[1] |= ((interrupt_to_configure->INV_ICM426XX_WOM_Z != 0) << BIT_INT_WOM_Z_INT_EN_POS);

		/* Set INT_SOURCE7 bits */
		data[2] &= (uint8_t) ~(BIT_INT_SOURCE7_STEP_DET_INT2_EN | BIT_INT_SOURCE7_STEP_CNT_OVFL_INT2_EN | BIT_INT_SOURCE7_TILT_DET_INT2_EN
#if defined(ICM_FAMILY_BPLUS)
							   | BIT_INT_SOURCE7_SLEEP_DET_INT2_EN | BIT_INT_SOURCE7_WAKE_DET_INT2_EN
#elif defined(ICM_FAMILY_CPLUS)
							   | BIT_INT_SOURCE7_FF_DET_INT2_EN | BIT_INT_SOURCE7_LOWG_DET_INT2_EN
#endif
							   | BIT_INT_SOURCE7_TAP_DET_INT2_EN);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_DET != 0) << BIT_INT_STEP_DET_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_STEP_CNT_OVFL != 0) << BIT_INT_STEP_CNT_OVFL_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_TILT_DET != 0) << BIT_INT_TILT_DET_INT_EN_POS);
#if defined(ICM_FAMILY_BPLUS)
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_SLEEP_DET != 0) << BIT_INT_SLEEP_DET_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_WAKE_DET != 0) << BIT_INT_WAKE_DET_INT_EN_POS);
#elif defined(ICM_FAMILY_CPLUS)
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_FF_DET != 0) << BIT_INT_FF_DET_INT_EN_POS);
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_LOWG_DET != 0) << BIT_INT_LOWG_DET_INT_EN_POS);
#endif
		data[2] |= ((interrupt_to_configure->INV_ICM426XX_TAP_DET != 0) << BIT_INT_TAP_DET_INT_EN_POS);

		/* SET INT_CONFIG bits */
		data[3] |= ((interrupt_to_configure->int_pol != 0) << BIT_INT_CONFIG_INT2_POLARITY_POS);
		data[3] |= ((interrupt_to_configure->int_drive != 0) << BIT_INT_CONFIG_INT2_DRIVE_CIRCUIT_POS);
		data[3] |= ((interrupt_to_configure->int_mode != 0) << BIT_INT_CONFIG_INT2_MODE_POS);

		status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE7_B4, 1, &data[2]); /* start with int_source7 since we are still in bank4 */
		status |= inv_icm426xx_set_reg_bank(s, 0);
		status |= inv_icm426xx_write_reg(s, MPUREG_INT_SOURCE3, 2, data); /* burst write int_source3/int_source4 */
		status |= inv_icm426xx_write_reg(s, MPUREG_INT_CONFIG, 1, data);  /* burst write int_source0/int_source1 */
	}
	return status;
}

int inv_icm426xx_get_data_from_registers(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t int_status;
	uint8_t temperature[2];
	uint8_t accel[ACCEL_DATA_SIZE];
	uint8_t gyro[GYRO_DATA_SIZE];
	inv_icm426xx_sensor_event_t event;
	
	/* Ensure data ready status bit is set */
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_STATUS, 1, &int_status);
	if(status)
		return status;

	if(int_status & BIT_INT_STATUS_DRDY) {
		
		status = inv_icm426xx_read_reg(s, MPUREG_TEMP_DATA0_UI, TEMP_DATA_SIZE, temperature);
		inv_icm426xx_format_data(s->endianess_data, temperature, (uint16_t *)&event.temperature);
		
		status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_DATA_X0_UI, ACCEL_DATA_SIZE, accel);
		inv_icm426xx_format_data(s->endianess_data, &accel[0], (uint16_t *)&event.accel[0]);
		inv_icm426xx_format_data(s->endianess_data, &accel[2], (uint16_t *)&event.accel[1]);
		inv_icm426xx_format_data(s->endianess_data, &accel[4], (uint16_t *)&event.accel[2]);
		
		status |= inv_icm426xx_read_reg(s, MPUREG_GYRO_DATA_X0_UI, GYRO_DATA_SIZE, gyro);
		inv_icm426xx_format_data(s->endianess_data, &gyro[0], (uint16_t *)&event.gyro[0]);
		inv_icm426xx_format_data(s->endianess_data, &gyro[2], (uint16_t *)&event.gyro[1]);
		inv_icm426xx_format_data(s->endianess_data, &gyro[4], (uint16_t *)&event.gyro[2]);
		
		/* call sensor event callback */
		if(s->sensor_event_cb)
			s->sensor_event_cb(&event);

		/* Device interrupts delayed when communicating with other slaves connected to same bus 
		 * Semi-Write to release interrupt in I2C
		 */
		if((s->transport.serif.serif_type == ICM426XX_UI_I2C) || (s->transport.serif.serif_type == ICM426XX_UI_I3C)) {
			uint8_t data = 0;
			status |= inv_icm426xx_write_reg(s, MPUREG_WHO_AM_I, 1, &data);
		}

	}
	/*else: Data Ready was not set*/
	
	return status;
}

int inv_icm426xx_get_data_from_fifo(struct inv_icm426xx * s)
{
	int status = 0; 
	uint8_t int_status;
	uint8_t data_reg = 0;
	uint8_t data[2];
	uint16_t packet_count_i, packet_count = 0;
	uint16_t packet_size = FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
	fifo_header_t * header;

	/* Ensure data ready status bit is set */
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_STATUS, 1, &int_status);
	if(status)
		return status;

	if((int_status & BIT_INT_STATUS_FIFO_THS) || (int_status & BIT_INT_STATUS_FIFO_FULL)) {
		
		/* FIFO record mode configured at driver init, so we read packet number, not byte count */
		status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_COUNTH, 2, data);
		if(status != INV_ERROR_SUCCESS)
			return status;
		inv_icm426xx_format_data(ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN, data, &packet_count);

		if (packet_count > 0) {
			/* Read FIFO only when data is expected in FIFO */
			/* fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE */
			uint16_t fifo_idx = 0;

			if(s->fifo_highres_enabled)
				packet_size = FIFO_20BYTES_PACKET_SIZE;

			if(s->transport.serif.serif_type == ICM426XX_UI_I3C) {
				/* in case of I3C, need to read packet by packet since INT is embedded on protocol so this can 
				happen that FIFO read is interrupted to handle IBI, and in that case FIFO is partially read.
				To handle this, 2 solution :
				- handle fifo lost packet & partial read
				- read packet by packet
				2nd solution prefered here because less heavy from driver point of view but it is less optimal
				for the timing because we have to initiate N transactions in any case */
				for(packet_count_i = 0 ; packet_count_i < packet_count ; packet_count_i++) {
					status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_DATA, packet_size, &s->fifo_data[packet_count_i*packet_size]);
					if(status) {
						/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
							  reset FIFO and try next chance */
						inv_icm426xx_reset_fifo(s);
						return status;
					}
				}
			} else {
				status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_DATA, packet_size * packet_count, s->fifo_data);
				if(status) {
					/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
						  reset FIFO and try next chance */
					inv_icm426xx_reset_fifo(s);
					return status;
				}
			}
			
			for(packet_count_i = 0; packet_count_i < packet_count; packet_count_i++) {
				inv_icm426xx_sensor_event_t event;
				event.sensor_mask = 0;
				
				header = (fifo_header_t *) &s->fifo_data[fifo_idx];
				fifo_idx += FIFO_HEADER_SIZE;
				
				/* Decode packet */
				if (header->bits.msg_bit) {
					/* MSG BIT set in FIFO header, Resetting FIFO */
					inv_icm426xx_reset_fifo(s);
					return INV_ERROR;
				}

				if(header->bits.accel_bit) {
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[0+fifo_idx], (uint16_t *)&event.accel[0]);
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[2+fifo_idx], (uint16_t *)&event.accel[1]);
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[4+fifo_idx], (uint16_t *)&event.accel[2]);
					fifo_idx += FIFO_ACCEL_DATA_SIZE;
				}

				if (header->bits.gyro_bit) {
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[0+fifo_idx], (uint16_t *)&event.gyro[0]);
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[2+fifo_idx], (uint16_t *)&event.gyro[1]);
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[4+fifo_idx], (uint16_t *)&event.gyro[2]);
					fifo_idx += FIFO_GYRO_DATA_SIZE;
				}

				if ((header->bits.accel_bit) || (header->bits.gyro_bit)) {
					if(header->bits.twentybits_bit) {
						inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[0+fifo_idx], (uint16_t *)&event.temperature);
						fifo_idx += FIFO_TEMP_DATA_SIZE + FIFO_TEMP_HIGH_RES_SIZE;

						/* new temperature data */
						if (event.temperature != INVALID_VALUE_FIFO)
							event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_TEMPERATURE);
					} else {
						event.temperature = (int8_t)s->fifo_data[0+fifo_idx]; /* cast to int8_t since FIFO is in 16 bits mode (temperature on 8 bits) */
						fifo_idx += FIFO_TEMP_DATA_SIZE;

						/* new temperature data */
						if (event.temperature != INVALID_VALUE_FIFO_1B)
							event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_TEMPERATURE);
					}
				}

				if ((header->bits.timestamp_bit) || (header->bits.fsync_bit)) {
					inv_icm426xx_format_data(s->endianess_data, &s->fifo_data[0+fifo_idx], (uint16_t *)&event.timestamp_fsync);
					fifo_idx += FIFO_TS_FSYNC_SIZE;
					
					/* new fsync event */
					/* First FSYNC event after enable is irrelevant
					 * FSYNC tag and FSYNC data should be ignored on the first ODR after restart.
					 */
					if (header->bits.fsync_bit){
						#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
						if(s->fsync_to_be_ignored == 0)
						#endif
							event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_FSYNC_EVENT);
					}
					#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					s->fsync_to_be_ignored = 0;
					#endif
				}
				
				if (header->bits.accel_bit) {
					if( (event.accel[0] != INVALID_VALUE_FIFO) &&
					    (event.accel[1] != INVALID_VALUE_FIFO) &&
					    (event.accel[2] != INVALID_VALUE_FIFO) ) {

						if (header->bits.twentybits_bit) {
							event.accel_high_res[0] = (s->fifo_data[0+fifo_idx] >> 4) & 0xF;
							event.accel_high_res[1] = (s->fifo_data[1+fifo_idx] >> 4) & 0xF;
							event.accel_high_res[2] = (s->fifo_data[2+fifo_idx] >> 4) & 0xF;
						}

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
						if (s->accel_start_time_us == UINT32_MAX) {
							event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
						} else {
							if (!header->bits.fsync_bit) {
								/* First data are noisy after enabling sensor
								 * Compare start time with current time to notify the event
								 */
								if((inv_icm426xx_get_time_us() - s->accel_start_time_us) >= ICM426XX_ACC_STARTUP_TIME_US) {
									s->accel_start_time_us = UINT32_MAX;
									event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
								}
							}
						}
#else
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
#endif
					}
				}
				
				if (header->bits.gyro_bit) {
					if( (event.gyro[0] != INVALID_VALUE_FIFO) &&
					    (event.gyro[1] != INVALID_VALUE_FIFO) &&
					    (event.gyro[2] != INVALID_VALUE_FIFO) ) {

						if (header->bits.twentybits_bit) {
							event.gyro_high_res[0] = (s->fifo_data[0+fifo_idx]) & 0xF;
							event.gyro_high_res[1] = (s->fifo_data[1+fifo_idx]) & 0xF;
							event.gyro_high_res[2] = (s->fifo_data[2+fifo_idx]) & 0xF;
						}

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
						if (s->gyro_start_time_us == UINT32_MAX) {
							event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
						} else {
							if (!header->bits.fsync_bit) {
								/* First data are noisy after enabling sensor
								 * Compare start time with current time to notify the event
								 */
								if((inv_icm426xx_get_time_us() - s->gyro_start_time_us) >= ICM426XX_GYR_STARTUP_TIME_US) {
									s->gyro_start_time_us = UINT32_MAX;
									event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
								}
							}
						}
#else
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
#endif
					}
				}

				if (header->bits.twentybits_bit)
					fifo_idx += FIFO_ACCEL_GYRO_HIGH_RES_SIZE;

				/* call sensor event callback */
				if(s->sensor_event_cb)
					s->sensor_event_cb(&event);

				/* Device interrupts delayed when communicating with other slaves connected to same bus 
				 * Semi-Write to release interrupt in I2C
				 */
				if((s->transport.serif.serif_type == ICM426XX_UI_I2C) || (s->transport.serif.serif_type == ICM426XX_UI_I3C)) {					
					status |= inv_icm426xx_write_reg(s, MPUREG_WHO_AM_I, 1, &data_reg);
				}
				
			} /* end of FIFO read for loop */
		}
		/*else: packet_count was 0*/
	}
	/*else: FIFO threshold was not reached and FIFO was not full*/

	return packet_count;
}

uint32_t inv_icm426xx_convert_odr_bitfield_to_us(uint32_t odr_bitfield)
{
	/*
 odr bitfield - frequency : odr ms
			0 - N/A
			1 - 32K
			2 - 16K
			3 - 8k
			4 - 4k
			5 - 2k
			6 - 1k        : 1ms
  (default) 7 - 200       : 5 ms
			8 - 100       : 10 ms
			9 - 50        : 20 ms
			10 - 25       : 40 ms
			11 - 12.5     : 80 ms
			12 - 6.25     : 160 ms
			13 - 3.125    : 320 ms
			14 - 1.5625   : 640 ms
			15 - 500      : 2 ms
		*/
	
	switch(odr_bitfield ) {
		case ICM426XX_ACCEL_CONFIG0_ODR_32_KHZ:      return 32;
		case ICM426XX_ACCEL_CONFIG0_ODR_16_KHZ:      return 63;
		case ICM426XX_ACCEL_CONFIG0_ODR_8_KHZ:      return 125;
		case ICM426XX_ACCEL_CONFIG0_ODR_4_KHZ:      return 250;
		case ICM426XX_ACCEL_CONFIG0_ODR_2_KHZ:      return 500;
		case ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ:      return 1000;
		case ICM426XX_ACCEL_CONFIG0_ODR_500_HZ:     return 2000;
		case ICM426XX_ACCEL_CONFIG0_ODR_200_HZ:     return 5000;
		case ICM426XX_ACCEL_CONFIG0_ODR_100_HZ:     return 10000;
		case ICM426XX_ACCEL_CONFIG0_ODR_50_HZ:      return 20000;
		case ICM426XX_ACCEL_CONFIG0_ODR_25_HZ:      return 40000;
		case ICM426XX_ACCEL_CONFIG0_ODR_12_5_HZ:    return 80000;
		case ICM426XX_ACCEL_CONFIG0_ODR_6_25_HZ:    return 160000;
		case ICM426XX_ACCEL_CONFIG0_ODR_3_125_HZ:   return 320000;
		case ICM426XX_ACCEL_CONFIG0_ODR_1_5625_HZ:
		default:                                    return 640000;
	}
}

int inv_icm426xx_set_accel_frequency(struct inv_icm426xx * s, const ICM426XX_ACCEL_CONFIG0_ODR_t frequency)
{
	int status = 0;
	uint8_t accel_cfg_0_reg;
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	/* Accel Low Power could report with wrong ODR if internal counter for ODR changed overflowed
	 * Update software counter to handle the overflow
	 */
	uint8_t pwr_mngt_0_reg;
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t accel_pwr_mode;
	
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	accel_pwr_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	if(ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP != accel_pwr_mode)
		s->wu_off_acc_odr_changes++; /* WUOSC is not clock source, this ODR change could be missed */
	else
		s->wu_off_acc_odr_changes = 0; /* WUOSC is on and acc is running, ODR change will be taken into account */
#endif
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_ODR_MASK;
	accel_cfg_0_reg |= (uint8_t)frequency;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	if(!status)	s->accel_odr = frequency;
	return status;
}

int inv_icm426xx_set_gyro_frequency(struct inv_icm426xx * s, const ICM426XX_GYRO_CONFIG0_ODR_t frequency)
{
	int status = 0;
	uint8_t gyro_cfg_0_reg;
	status |= inv_icm426xx_read_reg( s, MPUREG_GYRO_CONFIG0 , 1, &gyro_cfg_0_reg);
	gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_ODR_MASK;
	gyro_cfg_0_reg |= (uint8_t)frequency;
	status |= inv_icm426xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &gyro_cfg_0_reg);
	if(!status)	s->gyro_odr = frequency;
	return status;
}

int inv_icm426xx_set_accel_fsr(struct inv_icm426xx * s, ICM426XX_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g)
{
	int status = 0;
	uint8_t accel_cfg_0_reg;
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
	accel_cfg_0_reg |= (uint8_t)accel_fsr_g;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	if(!status)	s->accel_fsr = accel_fsr_g;
	return status;
}

int inv_icm426xx_set_gyro_fsr(struct inv_icm426xx * s, ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps)
{
	int status = 0;
	uint8_t gyro_cfg_0_reg;
	status |= inv_icm426xx_read_reg( s, MPUREG_GYRO_CONFIG0 , 1, &gyro_cfg_0_reg);
	gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
	gyro_cfg_0_reg |= (uint8_t)gyro_fsr_dps;
	status |= inv_icm426xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &gyro_cfg_0_reg);
	if(!status)	s->gyro_fsr = gyro_fsr_dps;
	return status;
}

int inv_icm426xx_get_accel_fsr(struct inv_icm426xx * s, ICM426XX_ACCEL_CONFIG0_FS_SEL_t * accel_fsr_g)
{
	int status = 0;
	uint8_t accel_cfg_0_reg;
	
	if((s->fifo_highres_enabled) && (s->fifo_is_used == INV_ICM426XX_FIFO_ENABLED))
		*accel_fsr_g = ACCEL_CONFIG0_FS_SEL_MAX;
	else {
		status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
		*accel_fsr_g = (ICM426XX_ACCEL_CONFIG0_FS_SEL_t)(accel_cfg_0_reg & BIT_ACCEL_CONFIG0_FS_SEL_MASK);
	}
	
	return status;
}

int inv_icm426xx_get_gyro_fsr(struct inv_icm426xx * s, ICM426XX_GYRO_CONFIG0_FS_SEL_t * gyro_fsr_dps)
{
	int status = 0;
	uint8_t gyro_cfg_0_reg;

	if((s->fifo_highres_enabled) && (s->fifo_is_used == INV_ICM426XX_FIFO_ENABLED))
		*gyro_fsr_dps = GYRO_CONFIG0_FS_SEL_MAX;
	else {
		status |= inv_icm426xx_read_reg( s, MPUREG_GYRO_CONFIG0 , 1, &gyro_cfg_0_reg);
		*gyro_fsr_dps = (ICM426XX_GYRO_CONFIG0_FS_SEL_t)(gyro_cfg_0_reg & BIT_GYRO_CONFIG0_FS_SEL_MASK);
	}
	
	return status;
}

int inv_icm426xx_set_accel_lp_avg(struct inv_icm426xx * s, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t acc_avg)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= (uint8_t)acc_avg;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);

	if(status == 0)
		/* Update SW mirror to re-apply it when required */
		s->avg_bw_setting.acc_lp_avg = (uint8_t)(acc_avg & BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK);

	return status;
}

int inv_icm426xx_set_accel_ln_bw(struct inv_icm426xx * s, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t acc_bw)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK;
	data |= (uint8_t)acc_bw;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);

	if(status == 0)
		/* Update SW mirror to re-apply it when required */
		s->avg_bw_setting.acc_ln_bw = (uint8_t)(acc_bw & BIT_GYRO_ACCEL_CONFIG0_ACCEL_FILT_MASK);

	return status;
}

int inv_icm426xx_set_gyro_ln_bw(struct inv_icm426xx * s, ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t gyr_bw)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK;
	data |= (uint8_t)gyr_bw;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_GYRO_CONFIG0, 1, &data);

	if(status == 0)
		/* Update SW mirror to re-apply it when required */
		s->avg_bw_setting.gyr_ln_bw = (uint8_t)(gyr_bw & BIT_GYRO_ACCEL_CONFIG0_GYRO_FILT_MASK);

	return status;
}

int inv_icm426xx_reset_fifo(struct inv_icm426xx * s)
{
	uint8_t data;
	uint8_t saved_fifo_config;
	int status = 0;
	uint8_t pwr_mngt_0_reg;
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t acc_mode;
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t gyr_mode;
	
	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt_0_reg);
	gyr_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	acc_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)(pwr_mngt_0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	
	if ( ( (gyr_mode != ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF) || (acc_mode != ICM426XX_PWR_MGMT_0_ACCEL_MODE_OFF) )
		&& (acc_mode != ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) ) {
		data = (uint8_t)ICM426XX_SIGNAL_PATH_RESET_FIFO_FLUSH_EN;
		status |= inv_icm426xx_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data);
	}
	else {
		/* In case no sensor is enabled or in accel low power mode, change the FIFO_MODE to “bypass” (00) mode to force the FIFO reset,
		 * potentials remaining data will be flushed 
		 * Then proceed to a dummy read to released the FIFO reset synchronously with the serial clock
		 */
		status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG, 1, &saved_fifo_config);
		data = (uint8_t)ICM426XX_FIFO_CONFIG_MODE_BYPASS;
		status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG, 1, &data);
		status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG, 1, &saved_fifo_config);
		status |= inv_icm426xx_read_reg(s, MPUREG_WHO_AM_I, 1, &data);
	}
	
	return status;
}

int inv_icm426xx_enable_timestamp_to_register(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t tmst_cfg_reg;
	
	if(!s->tmst_to_reg_en_cnt) {
		/* Enable the 20-bits timestamp register reading
		 * It's needed to wait at least 200us before doing the strobe 
		 */
		status |= inv_icm426xx_read_reg( s, MPUREG_TMST_CONFIG , 1, &tmst_cfg_reg);
		tmst_cfg_reg &= ~(uint8_t)BIT_TMST_CONFIG_TMST_TO_REGS_EN_MASK;
		tmst_cfg_reg |= (uint8_t)ICM426XX_TMST_CONFIG_TMST_TO_REGS_EN;
		status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &tmst_cfg_reg);
		
		inv_icm426xx_sleep_us(200);
	}
	s->tmst_to_reg_en_cnt ++;
	
	return status;
}

int inv_icm426xx_disable_timestamp_to_register(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t tmst_cfg_reg;

	if(!s->tmst_to_reg_en_cnt) {
		/* Disable the 20-bits timestamp register reading */
		status |= inv_icm426xx_read_reg( s, MPUREG_TMST_CONFIG , 1, &tmst_cfg_reg);
		tmst_cfg_reg &= (uint8_t)~BIT_TMST_CONFIG_TMST_TO_REGS_EN_MASK;
		tmst_cfg_reg |=(uint8_t) ICM426XX_TMST_CONFIG_TMST_TO_REGS_DIS;
		status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &tmst_cfg_reg);
	}
	s->tmst_to_reg_en_cnt --;
	
	return status;
}

int inv_icm426xx_get_current_timestamp(struct inv_icm426xx * s, uint32_t * icm_time)
{
	int status = 0;
	uint8_t data[3];
	
	/* Enable timestamp counter to be latched in timestamp register */
	data[0] = (uint8_t)ICM426XX_SIGNAL_PATH_RESET_TMST_STROBE_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data[0]);
	
	/* Get ICM timestamp */
	status |= inv_icm426xx_set_reg_bank(s, 1);
	status |= inv_icm426xx_read_reg(s, MPUREG_TMST_VAL0_B1, 3, data);
	status |= inv_icm426xx_set_reg_bank(s, 0);
	*icm_time = ((uint32_t)(data[2] & 0x0F) << 16) + ((uint32_t)data[1] << 8) + data[0];
	
	return status;
}

int inv_icm426xx_enable_clkin_rtc(struct inv_icm426xx * s, uint8_t enable)
{
	int status = 0;
	uint8_t data;

	if (enable) {

/* CLKIN is not supported for all parts, prevent the CLKIN to be enabled in this case. */
#if (RTC_SUPPORTED)
		/* Configure CLKIN on pin 9 */
		status |= inv_icm426xx_set_reg_bank(s, 1);
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG5_B1, 1, &data);
		data &= (uint8_t)~BIT_INTF_CONFIG5_GPIO_PAD_SEL_MASK;
		data |= (2 << BIT_INTF_CONFIG5_GPIO_PAD_SEL_POS);
		status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG5_B1, 1, &data);
		status |= inv_icm426xx_set_reg_bank(s, 0);
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
		data &= (uint8_t)~BIT_RTC_MODE_MASK;
		data |= (uint8_t)ICM426XX_INTF_CONFIG1_RTC_MODE_EN;
		status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
		status |= inv_icm426xx_configure_timestamp_resolution(s, ICM426XX_TMST_CONFIG_RESOL_16us);
#else
		(void)s;
		(void)enable;
		return INV_ERROR;
#endif

	} else {
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
		data &= (uint8_t)~BIT_RTC_MODE_MASK;
		data |= (uint8_t)ICM426XX_INTF_CONFIG1_RTC_MODE_DIS;
		status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
	}
	
	return status;
}

int inv_icm426xx_get_clkin_rtc_status(struct inv_icm426xx * s)
{
	uint8_t data;
	int status=0;
	
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &data);
	if ((data & BIT_RTC_MODE_MASK) == ICM426XX_INTF_CONFIG1_RTC_MODE_EN)
		return 1;
	else
		return 0;
}

int inv_icm426xx_enable_high_resolution_fifo(struct inv_icm426xx * s)
{
#if (INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	int status = 0;
	uint8_t data;

	status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
	data |= (uint8_t)ICM426XX_FIFO_CONFIG1_HIRES_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
#endif

	/* set FIFO packets to 20bit format (i.e. high res is enabled) */
	s->fifo_highres_enabled = 1;
	
	return 0;
}

int inv_icm426xx_disable_high_resolution_fifo(struct inv_icm426xx * s)
{
#if (INV_ICM426XX_LIGHTWEIGHT_DRIVER)
	int status = 0;
	uint8_t data;

	status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
	data |= (uint8_t)~BIT_FIFO_CONFIG1_HIRES_MASK; /* == ICM426XX_FIFO_CONFIG1_HIRES_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
#endif

	/* set FIFO packets to 16bit format (i.e. high res is disabled) */
	s->fifo_highres_enabled = 0;
	
	return 0;
}

int inv_icm426xx_configure_fifo(struct inv_icm426xx * s, INV_ICM426XX_FIFO_CONFIG_t fifo_config)
{
	int status = 0;
	uint8_t data;
	inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};
	
	s->fifo_is_used = fifo_config;
	
	switch (fifo_config) {
	
		case INV_ICM426XX_FIFO_ENABLED :
			/* Configure:
			 * - FIFO record mode i.e FIFO count unit is packet 
			 * - FIFO snapshot mode i.e drop the data when the FIFO overflows
			 * - Timestamp is logged in FIFO
			 * - Little Endian fifo_count
			*/
			status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG0, 1, &data);
			data |= (uint8_t)ICM426XX_INTF_CONFIG0_FIFO_COUNT_REC_RECORD;
			data &= (uint8_t)~BIT_FIFO_COUNT_ENDIAN_MASK; // little endian
			status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG0, 1, &data);
			data = (uint8_t)ICM426XX_FIFO_CONFIG_MODE_STOP_ON_FULL;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG, 1, &data);
			status |= inv_icm426xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &data);
			data |= ICM426XX_TMST_CONFIG_TMST_EN;
			status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &data);

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
			/* restart and reset FIFO configuration */
			status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			data &= (uint8_t)~(BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK);
			data |= (BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);
#else
			status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			data |= (BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK | BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);
#endif
			/* Set fifo_wm_int_w generating condition : fifo_wm_int_w generated when counter >= threshold */
			data |= (uint8_t)ICM426XX_FIFO_CONFIG1_WM_GT_TH_EN;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			/* Configure FIFO WM so that INT is triggered for each packet */
			data = 0x1;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG2, 1, &data);

			/* Disable Data Ready Interrupt */
			status |= inv_icm426xx_get_config_int1(s, &config_int);
			config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
			status |= inv_icm426xx_set_config_int1(s, &config_int);

#if defined(ICM_FAMILY_CPLUS)
			/* Set FIFO decimation rate to 0 */
			status |= inv_icm426xx_set_fifo_dec_rate(s, 0);
#endif

			break;
		
		case INV_ICM426XX_FIFO_DISABLED :
			/* make sure FIFO is disabled */
			data = ICM426XX_FIFO_CONFIG_MODE_BYPASS;
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG, 1, &data);
			
			/* restart and reset FIFO configuration */
			status |= inv_icm426xx_read_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			data &= (uint8_t)~(BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK);
			data |= (BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);
			status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG1, 1, &data);
			
			/* Enable Data Ready Interrupt */
			status |= inv_icm426xx_get_config_int1(s, &config_int);
			config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_ENABLE;
			status |= inv_icm426xx_set_config_int1(s, &config_int);
			break;

		default :
			status = -1;
	}
	return status;
}

int inv_icm426xx_configure_fifo_wm(struct inv_icm426xx * s, uint16_t wm)
{
	int status = 0;
	inv_icm426xx_interrupt_parameter_t config_int1;
	inv_icm426xx_interrupt_parameter_t config_int2;
	inv_icm426xx_interrupt_parameter_t config_ibi;
	inv_icm426xx_interrupt_value fifo_ths_int1_bk; 
	inv_icm426xx_interrupt_value fifo_ths_int2_bk; 
	inv_icm426xx_interrupt_value fifo_ths_ibi_bk; 

	/* WM is coded on 12 bits so check that 4 MSb are 0 */
	if ((wm & 0xF000) != 0)
		return INV_ERROR_BAD_ARG;

	/* FIFO WM interrupt must be disabled before being configured */
	status |= inv_icm426xx_get_config_int1(s, &config_int1);
	status |= inv_icm426xx_get_config_int2(s, &config_int2);
	status |= inv_icm426xx_get_config_ibi(s, &config_ibi);

	/* backup INT configuration */
	fifo_ths_int1_bk = config_int1.INV_ICM426XX_FIFO_THS; 
	fifo_ths_int2_bk = config_int2.INV_ICM426XX_FIFO_THS; 
	fifo_ths_ibi_bk  = config_ibi.INV_ICM426XX_FIFO_THS; 

	/* Disable FIFO WM */
	if (config_int1.INV_ICM426XX_FIFO_THS == INV_ICM426XX_ENABLE) {
		config_int1.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
		config_int1.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
		status |= inv_icm426xx_set_config_int1(s, &config_int1);	
	}
	if (config_int2.INV_ICM426XX_FIFO_THS == INV_ICM426XX_ENABLE) {
		config_int2.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
		config_int2.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
		status |= inv_icm426xx_set_config_int2(s, &config_int2);	
	}
	if (config_ibi.INV_ICM426XX_FIFO_THS == INV_ICM426XX_ENABLE) {
		config_ibi.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
		config_ibi.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
		status |= inv_icm426xx_set_config_ibi(s, &config_ibi);	
	}
	
	/* Write FIFO WM */
	status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG2, 2, (uint8_t *)&wm);

	/* Restore initial INT config if necessary */
	if (fifo_ths_int1_bk != config_int1.INV_ICM426XX_FIFO_THS) {
		config_int1.INV_ICM426XX_FIFO_THS = fifo_ths_int1_bk;
		status |= inv_icm426xx_set_config_int1(s, &config_int1);
	}
	if (fifo_ths_int2_bk != config_int2.INV_ICM426XX_FIFO_THS) {
		config_int2.INV_ICM426XX_FIFO_THS = fifo_ths_int2_bk;
		status |= inv_icm426xx_set_config_int2(s, &config_int2);
	}
	if (fifo_ths_ibi_bk != config_ibi.INV_ICM426XX_FIFO_THS) {
		config_ibi.INV_ICM426XX_FIFO_THS = fifo_ths_ibi_bk;
		status |= inv_icm426xx_set_config_ibi(s, &config_ibi);
	}

	return status;
}

uint32_t inv_icm426xx_get_fifo_timestamp_resolution_us_q24(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t tmst_cfg_reg;
	ICM426XX_TMST_CONFIG_RESOL_t tmst_resol;
	uint32_t scale_factor_q24 = 1<<24;
	
	status |= inv_icm426xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &tmst_cfg_reg);
	if (status < 0)
		return INV_ERROR;
		
	tmst_resol = (ICM426XX_TMST_CONFIG_RESOL_t)(tmst_cfg_reg & BIT_TMST_CONFIG_RESOL_MASK);

	/* PLL scale factor doesn't apply when WU oscillator or CLKIN is in use */
	if (!inv_icm426xx_is_wu_osc_active(s) && !inv_icm426xx_get_clkin_rtc_status(s))
		scale_factor_q24 = PLL_SCALE_FACTOR_Q24;

	if (tmst_resol == ICM426XX_TMST_CONFIG_RESOL_1us)
		return 1 * scale_factor_q24;
	else if (inv_icm426xx_get_clkin_rtc_status(s))
		/* RTC is enabled, the resolution of the timestamp is one tick of RTC
		 * RTC runs at 32768 Hz, so resolution is 1/32768 s, or 1000000/32768 us	
		 */
		return ((1000000UL<<12)/32768UL)<<(24-12);
	else if (tmst_resol == ICM426XX_TMST_CONFIG_RESOL_16us)
		return 16 * scale_factor_q24;
	else 
	/* Should not happen, return 0 */		
		return 0; 
}

uint32_t inv_icm426xx_get_reg_timestamp_resolution_us_q24(struct inv_icm426xx * s)
{
	uint32_t scale_factor_q24 = 1<<24;

	/* RTC is enabled, the resolution of the timestamp is one tick of RTC
	 * Our RTC runs at 32768 Hz, so resolution is 1/32768 s, or 1000000/32768 us	
	 */
	if (inv_icm426xx_get_clkin_rtc_status(s))
		return ((1000000UL<<12)/32768UL)<<(24-12);

	/* PLL scale factor doesn't apply when WU oscillator is in use */
	if (!inv_icm426xx_is_wu_osc_active(s))
		scale_factor_q24 = PLL_SCALE_FACTOR_Q24;
	
	return 1 * scale_factor_q24;
}

#if defined(ICM_FAMILY_CPLUS)
int inv_icm426xx_set_fifo_dec_rate(struct inv_icm426xx * s, uint8_t dec_rate)
{
	int status = 0;
	uint8_t data;
	
	status |= inv_icm426xx_set_reg_bank(s, 4);

	status |= inv_icm426xx_read_reg(s, MPUREG_FDR_CONFIG_B4, 1, &data);
	data &= (uint8_t)~BIT_FDR_CONFIG_FDR_SEL_MASK;
	data |= dec_rate << BIT_FDR_CONFIG_FDR_SEL_POS;
	status |= inv_icm426xx_write_reg(s, MPUREG_FDR_CONFIG_B4, 1, &data);

	status |= inv_icm426xx_set_reg_bank(s, 0);

	return status;
}

int inv_icm426xx_interface_change_procedure(struct inv_icm426xx * s, inv_icm426xx_interface_mode_t interface_mode)
{

	int status = 0;
	uint8_t data;

	/* 1. Disable weak pull-up/down */
	status |= inv_icm426xx_set_reg_bank(s, 3);

	data = 0x00;
	status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG1_B3, 1, &data);  //OSC_RC_TRIM3
	status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG2_B3, 1, &data); //MB_CORE_TRIM6

	/* 2. Disable pad int1 */
	status |= inv_icm426xx_set_reg_bank(s, 2);
	data = 0x01;
	status |= inv_icm426xx_write_reg(s, MPUREG_TMD4_B2, 1, &data);  //PAD_INT1
	
	/* 3. Disable pads aux_si, aux_sclk and aux_cs (only for going to single interface) */
	if (interface_mode & INV_ICM426XX_SINGLE_INTERFACE) {
		data = 0x01;
		status |= inv_icm426xx_write_reg(s, MPUREG_TMD5_B2, 1, &data);  //AUX_SDI
		status |= inv_icm426xx_write_reg(s, MPUREG_TMD6_B2, 1, &data);  //AUX_SCLK
		status |= inv_icm426xx_write_reg(s, MPUREG_TMD7_B2, 1, &data);  //AUX_CS
	}

	/* 4. Disable pads int2, fsync, clkin and aux_sdo */
	status |= inv_icm426xx_set_reg_bank(s, 1);
	data = 0xF0;
	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG3_B1, 1, &data);

	/* 5. Set default driver configuration FSYNC on INT2=pin 9 */
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG5_B1, 1, &data);
	data &= (uint8_t)~BIT_INTF_CONFIG5_GPIO_PAD_SEL_MASK;
	data |= (1 << BIT_INTF_CONFIG5_GPIO_PAD_SEL_POS);
	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG5_B1, 1, &data);

	/* 6. Select proper pad scenario */
	status |= inv_icm426xx_read_reg(s, MPUREG_SENSOR_CONFIG1_B1, 1, &data);
	data &= 0x0F;

	if (interface_mode & INV_ICM426XX_SINGLE_INTERFACE)
		data |= 4 << 4;
	else if (interface_mode & INV_ICM426XX_DUAL_INTERFACE)
		data |= 3 << 4;
	else if (interface_mode & INV_ICM426XX_TRIPLE_INTERFACE)
		data |= 2 << 4;
	
	status |= inv_icm426xx_write_reg(s, MPUREG_SENSOR_CONFIG1_B1, 1, &data);

	/* 7. Select SPI4W/SPI3W configuration for AUX1, requested only for dual interface mode*/
	/* 8. enable pad aux_sdo, requested only for dual interface mode*/
	if (interface_mode & INV_ICM426XX_DUAL_INTERFACE) {
		/* 7 */
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &data);
		data &= ~BIT_INTF_CONFIG4_AUX1_SPI_MASK;
		if (interface_mode == INV_ICM426XX_DUAL_INTERFACE_SPI4)
			data |= ICM426XX_INTF_CONFIG4_AUX1_SPI4W;
		else
			data |= ICM426XX_INTF_CONFIG4_AUX1_SPI3W;
		status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &data);
		/* 8 */
		if (interface_mode == INV_ICM426XX_DUAL_INTERFACE_SPI4) {
			status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG3_B1, 1, &data);
		  	data &= ~(1<<4); // pads_aux_sdo enable for AUX_SPI4
		  	status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG3_B1, 1, &data);
		}	
	}

	/* 9. enable pad int1 */
	status |= inv_icm426xx_set_reg_bank(s, 2);
	data = 0x00;
	status |= inv_icm426xx_write_reg(s, MPUREG_TMD4_B2, 1, &data);  //PAD_INT1

	/* 10. enable pads aux cs,aux_sclk and aux sdi only of triple or dual interface */
	if (!(interface_mode & INV_ICM426XX_SINGLE_INTERFACE)) {
		data = 0x00;
		status |= inv_icm426xx_write_reg(s, MPUREG_TMD7_B2, 1, &data);  //AUX_CS
		status |= inv_icm426xx_write_reg(s, MPUREG_TMD6_B2, 1, &data);  //AUX_SCLK
		status |= inv_icm426xx_write_reg(s, MPUREG_TMD5_B2, 1, &data);  //AUX_SDI
	}

	/* 11. enable pads fsync, clkin and aux sdo only for going to triple interface*/
	/* 12. enable pad int2 only for single or dual interface */
	status |= inv_icm426xx_set_reg_bank(s, 1);
	if (interface_mode & INV_ICM426XX_TRIPLE_INTERFACE) {
		/* 11 */
		data = 0x80;
		status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG3_B1, 1, &data);
	} else {
		/* 12, we do a read before in case AUX1_SPI4 is set in 8. */
		status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG3_B1, 1, &data);
		data &= ~0x80;
		status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG3_B1, 1, &data);
	}

	/* 13. enable weak pull-up/down */
	status |= inv_icm426xx_set_reg_bank(s, 3);
	if (interface_mode & INV_ICM426XX_SINGLE_INTERFACE) {
		data = 0x9F;
		status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG1_B3, 1, &data); //OSC_RC_TRIM3
		data = 0xAA;
		status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG2_B3, 1, &data); //MB_CORE_TRIM6
	} else if (interface_mode & INV_ICM426XX_DUAL_INTERFACE) {
		data = 0x9F;
		status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG1_B3, 1, &data); //OSC_RC_TRIM3
		data = 0xAA;
		status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG2_B3, 1, &data); //MB_CORE_TRIM6
	} else if (interface_mode & INV_ICM426XX_TRIPLE_INTERFACE) {
		data = 0xAF;
		status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG1_B3, 1, &data); //OSC_RC_TRIM3
		data = 0xAA;
		status |= inv_icm426xx_write_reg(s, MPUREG_PU_PD_CONFIG2_B3, 1, &data); //MB_CORE_TRIM6
	}
	status |= inv_icm426xx_set_reg_bank(s, 0);

	return status;
}
#endif

const char * inv_icm426xx_get_version(void)
{
	return INV_ICM426XX_VERSION_STRING;
}

/*
 * Static functions definition
 */
static int inv_icm426xx_configure_serial_interface(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t value;

	/* Set memory bank 1 */
	status |= inv_icm426xx_set_reg_bank(s, 1);

	switch(s->transport.serif.serif_type) {
	
		case ICM426XX_UI_I2C:
			/* Enable I2C 50ns spike filtering */
			status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG6_B1, 1, &value);
			value &= (uint8_t)~(BIT_INTF_CONFIG6_I3C_SDR_EN_MASK | BIT_INTF_CONFIG6_I3C_DDR_EN_MASK);
			status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG6_B1, 1, &value);
			break;
			
		case ICM426XX_UI_I3C:
			/* Enable In Band Interrupt for I3C UI interface and associated payload byte and assign dynamic address */
			status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG6_B1, 1, &value);
			value |= (BIT_INTF_CONFIG6_I3C_IBI_BYTE_EN_MASK | BIT_INTF_CONFIG6_I3C_IBI_EN_MASK);
			status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG6_B1, 1, &value);
			status |= s->transport.serif.configure((struct inv_icm426xx_serif *)s);
			break;
			
		case ICM426XX_UI_SPI4:
			value = ICM426XX_INTF_CONFIG4_AP_SPI4W;
			status |= inv_icm426xx_write_reg(s, MPUREG_INTF_CONFIG4_B1, 1, &value);
			break;
			
		default:
			status |= INV_ERROR_BAD_ARG;
	}

	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);
	
	return status;
}

static int inv_icm426xx_init_hardware_from_ui(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t *config_int)
{
	uint8_t data;
	int status = 0;
	uint8_t wom_threshold[3];
	uint8_t gyro_cfg_0_reg, accel_cfg_0_reg, tmst_cfg_reg;
	if (config_int == NULL)
	{
		inv_icm426xx_interrupt_parameter_t intcfg = {
			.INV_ICM426XX_UI_FSYNC = INV_ICM426XX_DISABLE,
			.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE,
			.INV_ICM426XX_FIFO_THS = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_FIFO_FULL = INV_ICM426XX_DISABLE,
			.INV_ICM426XX_SMD = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_WOM_X = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_WOM_Y = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_WOM_Z = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_STEP_DET = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_STEP_CNT_OVFL = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_TILT_DET = INV_ICM426XX_ENABLE,
#if defined(ICM_FAMILY_BPLUS)
			.INV_ICM426XX_SLEEP_DET = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_WAKE_DET = INV_ICM426XX_ENABLE,
#elif defined(ICM_FAMILY_CPLUS)
			.INV_ICM426XX_FF_DET = INV_ICM426XX_ENABLE,
			.INV_ICM426XX_LOWG_DET = INV_ICM426XX_ENABLE,
#endif
			.INV_ICM426XX_TAP_DET = INV_ICM426XX_ENABLE,
		};
		config_int = &intcfg;
	}
	status |= inv_icm426xx_device_reset(s);
	if(status) {
		return status;
	}

	inv_icm426xx_sleep_us(1000);

	/* Setup MEMs properties */
	status |= inv_icm426xx_read_reg(s, MPUREG_GYRO_CONFIG0, 1, &gyro_cfg_0_reg);
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	gyro_cfg_0_reg &= (uint8_t)~BIT_GYRO_CONFIG0_FS_SEL_MASK;
	gyro_cfg_0_reg |= (uint8_t)ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps;
	accel_cfg_0_reg &= (uint8_t)~BIT_ACCEL_CONFIG0_FS_SEL_MASK;
	accel_cfg_0_reg |= (uint8_t)ICM426XX_ACCEL_CONFIG0_FS_SEL_4g;
	status |= inv_icm426xx_write_reg(s, MPUREG_GYRO_CONFIG0, 1, &gyro_cfg_0_reg);
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_CONFIG0, 1, &accel_cfg_0_reg);
	
	/* make sure FIFO is disabled */
	data = ICM426XX_FIFO_CONFIG_MODE_BYPASS;
	status |= inv_icm426xx_write_reg(s, MPUREG_FIFO_CONFIG, 1, &data);
	
	/* Deactivate FSYNC by default */
	status |= inv_icm426xx_read_reg(s, MPUREG_FSYNC_CONFIG, 1, &data);
	data &= (uint8_t)~BIT_FSYNC_CONFIG_UI_SEL_MASK;
	status |= inv_icm426xx_write_reg(s, MPUREG_FSYNC_CONFIG, 1, &data);
	
	status |= inv_icm426xx_read_reg(s, MPUREG_TMST_CONFIG, 1, &tmst_cfg_reg);
	tmst_cfg_reg &= (uint8_t)~BIT_TMST_CONFIG_TMST_FSYNC_MASK; // == ICM426XX_FSYNC_CONFIG_UI_SEL_NO
	status |= inv_icm426xx_write_reg(s, MPUREG_TMST_CONFIG, 1, &tmst_cfg_reg);
	
	/* Set default timestamp resolution 16us (Mobile use cases) */
	status |= inv_icm426xx_configure_timestamp_resolution(s, ICM426XX_TMST_CONFIG_RESOL_16us);
	
	status |= inv_icm426xx_configure_fifo(s, INV_ICM426XX_FIFO_ENABLED);
	
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_CONFIG, 1, &data);
	/* Enable push pull on INT1 to avoid moving in Test Mode after a soft reset */
	data |= (uint8_t)config_int->int_drive;
	/* Configure the INT1 interrupt pulse as active high */
	data |= (uint8_t)config_int->int_pol;
	status |= inv_icm426xx_write_reg(s, MPUREG_INT_CONFIG, 1, &data);
	
	/* Set interrupt config */
	status |= inv_icm426xx_set_config_int1(s,config_int);
	config_int->INV_ICM426XX_UI_DRDY  = INV_ICM426XX_ENABLE;
	config_int->INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
	status |= inv_icm426xx_set_config_ibi(s,config_int);
	
	/* Set the ASY_RESET_DISABLE bit to 0 (async enabled) in order to chop Tpulse as soon as interrupt status is read
	 * Guideline is to set the ASY_RESET_DISABLE bit to 0 in pulse mode
	 * No effect in latch mode */
	status |= inv_icm426xx_read_reg(s, MPUREG_INT_CONFIG1, 1, &data);
	data &= (uint8_t)~BIT_INT_CONFIG1_ASY_RST_MASK;
	data |= (uint8_t)ICM426XX_INT_CONFIG1_ASY_RST_ENABLED;
	status |= inv_icm426xx_write_reg(s, MPUREG_INT_CONFIG1, 1, &data);

	/* Set the UI filter order to 2 for both gyro and accel */
	status |= inv_icm426xx_read_reg(s, MPUREG_GYRO_CONFIG1, 1, &data);
	data &= (uint8_t)~BIT_GYRO_CONFIG1_GYRO_UI_FILT_ORD_MASK;
	data |= (uint8_t)ICM426XX_GYRO_CONFIG_GYRO_UI_FILT_ORD_2ND_ORDER;
	status |= inv_icm426xx_write_reg(s, MPUREG_GYRO_CONFIG1, 1, &data);
	status |= inv_icm426xx_read_reg(s, MPUREG_ACCEL_CONFIG1, 1, &data);
	data &= (uint8_t)~BIT_ACCEL_CONFIG1_ACCEL_UI_FILT_ORD_MASK;
	data |= (uint8_t)ICM426XX_ACCEL_CONFIG_ACCEL_UI_FILT_ORD_2ND_ORDER;
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_CONFIG1, 1, &data);
	
	/* FIFO packets are 16bit format by default (i.e. high res is disabled) */
	status |= inv_icm426xx_disable_high_resolution_fifo(s);
	
	/* For retro-compatibility, configure WOM to compare current sample with the previous sample and to produce signal when all axis exceed 52 mg */
	status |= inv_icm426xx_set_reg_bank(s, 4); /* Set memory bank 4 */
	wom_threshold[0] = ICM426XX_DEFAULT_WOM_THS_MG; /* Set X threshold */
	wom_threshold[1] = ICM426XX_DEFAULT_WOM_THS_MG; /* Set Y threshold */
	wom_threshold[2] = ICM426XX_DEFAULT_WOM_THS_MG; /* Set Z threshold */
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_WOM_X_THR_B4, sizeof(wom_threshold), &wom_threshold[0]);
	
	status |= inv_icm426xx_set_reg_bank(s, 0); /* Set memory bank 0 */
	data = ((uint8_t)ICM426XX_SMD_CONFIG_WOM_INT_MODE_ANDED) | ((uint8_t)ICM426XX_SMD_CONFIG_WOM_MODE_CMP_PREV);
	status |= inv_icm426xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &data);

	/* by default, set IIR filter BW to ODR/4 for LN, 16x averaging for GLP, 16x averaging for ALP */
	s->avg_bw_setting.acc_ln_bw = (uint8_t)ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_4;
	s->avg_bw_setting.gyr_ln_bw = (uint8_t)ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_4;
	s->avg_bw_setting.acc_lp_avg = (uint8_t)ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_16;

	/* Reset self-test result variable*/
	s->st_result = 0;

#if defined(ICM_FAMILY_BPLUS)
	/* Configure Gyro in Periodic reset */
	status |= inv_icm426xx_set_reg_bank(s, 3); /* Set memory bank 3 */

	/* Reg AMP_GSXYZ_TRIM0 */
	status |= inv_icm426xx_read_reg(s, 0x2E, 1, &data); 
	data &= 0xfd; 
	status |= inv_icm426xx_write_reg(s, 0x2E, 1, &data);
	
	/* Reg AMP_GX_TRIM2 */
	status |= inv_icm426xx_read_reg(s, 0x32, 1, &data); 
	data &= 0x9f; 
	status |= inv_icm426xx_write_reg(s, 0x32, 1, &data);
	
	/* Reg AMP_GY_TRIM2 */
	status |= inv_icm426xx_read_reg(s, 0x37, 1, &data);	
	data &= 0x9f; 
	status |= inv_icm426xx_write_reg(s, 0x37, 1, &data);
	
	/* Reg AMP_GZ_TRIM2 */
	status |= inv_icm426xx_read_reg(s, 0x3C, 1, &data);	
	data &= 0x9f; 
	status |= inv_icm426xx_write_reg(s, 0x3C, 1, &data);

	status |= inv_icm426xx_set_reg_bank(s, 0); /* Set memory bank 0 */
#endif
	
	return status;
}

static int inv_icm426xx_is_wu_osc_active(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t pwr_mngt0_reg;
	uint8_t intf_cfg_1_reg;
	
	/* WU oscillator is active is Accel is enabled alone, in LP, with lp_clk_sel sets to WU_OSC */
	ICM426XX_PWR_MGMT_0_GYRO_MODE_t      gyr_mode;
	ICM426XX_PWR_MGMT_0_ACCEL_MODE_t     acc_mode;
	ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_t lp_clk;

	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mngt0_reg);
	status |= inv_icm426xx_read_reg(s, MPUREG_INTF_CONFIG1, 1, &intf_cfg_1_reg);
	gyr_mode = (ICM426XX_PWR_MGMT_0_GYRO_MODE_t)      (pwr_mngt0_reg & BIT_PWR_MGMT_0_GYRO_MODE_MASK);
	acc_mode = (ICM426XX_PWR_MGMT_0_ACCEL_MODE_t)     (pwr_mngt0_reg & BIT_PWR_MGMT_0_ACCEL_MODE_MASK);
	lp_clk   = (ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_t) (intf_cfg_1_reg & BIT_ACCEL_LP_CLK_SEL_MASK);


	return (  (gyr_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_OFF || gyr_mode == ICM426XX_PWR_MGMT_0_GYRO_MODE_STANDBY) /* Gyro is off or standby */
	       && (acc_mode == ICM426XX_PWR_MGMT_0_ACCEL_MODE_LP) /* Accel is enabled in LP */
	       && (lp_clk == ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_WUOSC)); /* WU oscillator is selected */
}

/* Default implementation converts ICM endian to little endian */
void inv_icm426xx_format_data(const uint8_t endian, const uint8_t *in, uint16_t *out)
{
	if(endian == ICM426XX_INTF_CONFIG0_DATA_BIG_ENDIAN)
		*out = (in[0] << 8) | in[1];
	else
		*out = (in[1] << 8) | in[0];
}
