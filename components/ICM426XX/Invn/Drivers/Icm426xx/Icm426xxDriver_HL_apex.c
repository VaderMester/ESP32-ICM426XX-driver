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
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL_apex.h"
*/
#include "Icm426xxDefs.h"
#include "Icm426xxExtFunc.h"
#include "Icm426xxDriver_HL.h"
#include "Icm426xxDriver_HL_apex.h"

static int inv_icm426xx_resume_dmp(struct inv_icm426xx * s);

int inv_icm426xx_configure_smd_wom(struct inv_icm426xx * s, const uint8_t x_th, const uint8_t y_th, const uint8_t z_th, ICM426XX_SMD_CONFIG_WOM_INT_MODE_t wom_int, ICM426XX_SMD_CONFIG_WOM_MODE_t wom_mode)
{
	int status = 0;
	uint8_t data[3];
	
	/* Set memory bank 4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);
	
	data[0] = x_th; /* Set X threshold */
	data[1] = y_th; /* Set Y threshold */
	data[2] = z_th; /* Set Z threshold */
	status |= inv_icm426xx_write_reg(s, MPUREG_ACCEL_WOM_X_THR_B4, sizeof(data), &data[0]);
	
	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);
	
	data[0] = ((uint8_t)wom_int) | ((uint8_t)wom_mode);
	status |= inv_icm426xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &data[0]);
	
	return status;
}

int inv_icm426xx_enable_wom(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;
	inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};
	
	/* Enable WOM only if SMD is not enabled
	 * If SMD is enabled, WOM event will be generated
	 */
	if (s->wom_smd_mask == ICM426XX_SMD_CONFIG_SMD_MODE_DISABLED) {
		/* Set SMD mode to WOM */
		status |= inv_icm426xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &data);
		data &= (uint8_t)~BIT_SMD_CONFIG_SMD_MODE_MASK;
		data |= (uint8_t)ICM426XX_SMD_CONFIG_SMD_MODE_WOM;
		status |= inv_icm426xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &data);
		/* Update mask to inform wom is enabled */
		s->wom_smd_mask = ICM426XX_SMD_CONFIG_SMD_MODE_WOM;
	}

	/* Disable fifo threshold int1 */
	status |= inv_icm426xx_get_config_int1(s, &config_int);	
	config_int.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
	status |= inv_icm426xx_set_config_int1(s, &config_int);

	/* Disable data ready ibi (used instead of FIFO THS for I3C) */
	status |= inv_icm426xx_get_config_ibi(s, &config_int);
	config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_DISABLE;
	status |= inv_icm426xx_set_config_ibi(s, &config_int);

	s->wom_enable = 1;

	return status;
}

int inv_icm426xx_disable_wom(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;
	inv_icm426xx_interrupt_parameter_t config_int = {(inv_icm426xx_interrupt_value)0};

	/* Enable fifo threshold int1 */
	status |= inv_icm426xx_get_config_int1(s, &config_int);
	config_int.INV_ICM426XX_FIFO_THS = INV_ICM426XX_ENABLE;
	status |= inv_icm426xx_set_config_int1(s, &config_int);

	/* Enable data ready ibi (used instead of FIFO THS for I3C) */
	status |= inv_icm426xx_get_config_ibi(s, &config_int);
	config_int.INV_ICM426XX_UI_DRDY = INV_ICM426XX_ENABLE;
	status |= inv_icm426xx_set_config_ibi(s, &config_int);
	
	if(s->wom_smd_mask == ICM426XX_SMD_CONFIG_SMD_MODE_WOM) {
		/* Set SMD mode to disabled */
		status |= inv_icm426xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &data);
		data &= (uint8_t)~BIT_SMD_CONFIG_SMD_MODE_MASK; /* == ICM426XX_SMD_CONFIG_SMD_MODE_DISABLED */
		status |= inv_icm426xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &data);
		/* Update mask to inform wom is disabled */
		s->wom_smd_mask = ICM426XX_SMD_CONFIG_SMD_MODE_DISABLED;
	}
	s->wom_enable = 0;

	return status;
}

int inv_icm426xx_enable_smd(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;
	
	/*
	 * SMD engine is based on two occurences of WoM events within a defined timeframe.
	 * POR value for WoM mode is to compare current sample against first sample,
	 * which may happen during accel engine stabilisation and therefore could
	 * be significantly different compared to the normalised value. Comparing
	 * the two has a higher chance of leading to wrong SMD events signalling,
	 * therefore it is required to configure WoM to compare the current sample
	 * against the previous sample instead.
	 */

	/* Set SMD mode to SMD */
	status |= inv_icm426xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &data);
	data &= (uint8_t)~BIT_SMD_CONFIG_SMD_MODE_MASK;
	data |= (uint8_t)ICM426XX_SMD_CONFIG_SMD_MODE_LONG;
	status |= inv_icm426xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &data);
	/* Update mask to inform smd is enabled */
	s->wom_smd_mask = ICM426XX_SMD_CONFIG_SMD_MODE_LONG;

	return status;
}

int inv_icm426xx_disable_smd(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_read_reg(s, MPUREG_SMD_CONFIG, 1, &data);
	data &= (uint8_t)~BIT_SMD_CONFIG_SMD_MODE_MASK;

	if (s->wom_smd_mask == ICM426XX_SMD_CONFIG_SMD_MODE_SHORT || 
	    s->wom_smd_mask == ICM426XX_SMD_CONFIG_SMD_MODE_LONG) {
		if (s->wom_enable == 1) {
			data |= (uint8_t)ICM426XX_SMD_CONFIG_SMD_MODE_WOM;
			s->wom_smd_mask = ICM426XX_SMD_CONFIG_SMD_MODE_WOM;
		} else {
			s->wom_smd_mask = ICM426XX_SMD_CONFIG_SMD_MODE_DISABLED;
		}
	}

	status |= inv_icm426xx_write_reg(s, MPUREG_SMD_CONFIG, 1, &data);

	return status;
}

int inv_icm426xx_init_tap_parameters_struct(struct inv_icm426xx *s, inv_icm426xx_tap_parameters_t *tap_inputs)
{
	/* Default parameters at POR */
	tap_inputs->min_jerk_thr = ICM426XX_APEX_CONFIG7_TAP_MIN_JERK_THR_281MG_DEFAULT;
	tap_inputs->max_peak_tol = ICM426XX_APEX_CONFIG7_TAP_MAX_PEAK_TOL_25;
	tap_inputs->tmax         = ICM426XX_APEX_CONFIG8_TAP_TMAX_500MS;
	tap_inputs->tmin         = ICM426XX_APEX_CONFIG8_TAP_TMIN_171MS;
	tap_inputs->tavg         = ICM426XX_APEX_CONFIG8_TAP_TAVG_8SAMPLE;
	
	return 0;
}

int inv_icm426xx_configure_tap_parameters(struct inv_icm426xx * s, const inv_icm426xx_tap_parameters_t *tap_inputs)
{
	int status = 0;
	uint8_t data[2];

	data[0] = ((tap_inputs->min_jerk_thr << 2) | tap_inputs->max_peak_tol);
	data[1] = ((uint8_t)tap_inputs->tmax | (uint8_t)tap_inputs->tmin | (uint8_t)tap_inputs->tavg);

	/* Set memory bank 4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);

	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG7_B4, sizeof(data), &data[0]);

	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);

	return status;
}

int inv_icm426xx_get_tap_parameters(struct inv_icm426xx *s, inv_icm426xx_tap_parameters_t *tap_params)
{
	int status = 0;
	uint8_t data[2];
	
	/*  Set memory bank 4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);

	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG7_B4, sizeof(data), &data[0]);

	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);
	
	tap_params->min_jerk_thr = (data[0] & BIT_APEX_CONFIG7_TAP_MIN_JERK_THR_MASK) >> BIT_APEX_CONFIG7_TAP_MIN_JERK_THR_POS;
	tap_params->max_peak_tol = (ICM426XX_APEX_CONFIG7_TAP_MAX_PEAK_TOL_t)(data[0] & BIT_APEX_CONFIG7_TAP_MAX_PEAK_TOL_MASK);
	tap_params->tmax = (ICM426XX_APEX_CONFIG8_TAP_TMAX_t)(data[1] & BIT_APEX_CONFIG8_TAP_TMAX_MASK);
	tap_params->tmin = (ICM426XX_APEX_CONFIG8_TAP_TMIN_t)(data[1] & BIT_APEX_CONFIG8_TAP_TMIN_MASK);
	tap_params->tavg = (ICM426XX_APEX_CONFIG8_TAP_TAVG_t)(data[1] & BIT_APEX_CONFIG8_TAP_TAVG_MASK);

	return status;
}

int inv_icm426xx_enable_tap(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	/* Enable TAP */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data |= (uint8_t)ICM426XX_APEX_CONFIG0_TAP_ENABLE_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

int inv_icm426xx_disable_tap(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	/* Disable TAP */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_TAP_ENABLE_MASK; /* == ICM426XX_APEX_CONFIG0_TAP_ENABLE_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

int inv_icm426xx_init_apex_parameters_struct(struct inv_icm426xx *s, inv_icm426xx_apex_parameters_t *apex_inputs)
{
	int status = 0;
	
	/* Default parameters at POR */
	apex_inputs->pedo_amp_th         = ICM426XX_APEX_CONFIG2_PEDO_AMP_TH_62MG;
	apex_inputs->pedo_step_cnt_th    = 0x5;
	apex_inputs->pedo_step_det_th    = 0x2;
	apex_inputs->pedo_sb_timer_th    = ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_150_SAMPLES;
	apex_inputs->pedo_hi_enrgy_th    = ICM426XX_APEX_CONFIG3_PEDO_HI_ENRGY_TH_107;
	apex_inputs->tilt_wait_time      = ICM426XX_APEX_CONFIG4_TILT_WAIT_TIME_4S;
#if defined(ICM_FAMILY_BPLUS)
	apex_inputs->r2w_sleep_time_out  = ICM426XX_APEX_CONFIG4_R2W_SLEEP_TIME_OUT_6_4S;
	apex_inputs->r2w_mounting_matrix = ICM426XX_APEX_CONFIG5_R2W_MOUNTING_MATRIX_0;
	apex_inputs->r2w_gest_delay      = ICM426XX_APEX_CONFIG6_R2W_SLEEP_GEST_DELAY_0_96S;
#elif defined(ICM_FAMILY_CPLUS)
	apex_inputs->ff_debounce_duration = ICM426XX_APEX_CONFIG10_FF_DEBOUNCE_DURATION_2000_MS;
	apex_inputs->ff_max_duration_cm   = ICM426XX_APEX_CONFIG10_FF_MAX_DURATION_201_CM;
	apex_inputs->ff_min_duration_cm   = ICM426XX_APEX_CONFIG10_FF_MIN_DURATION_13_CM;
	apex_inputs->lowg_peak_th         = ICM426XX_APEX_CONFIG5_LOWG_PEAK_TH_563MG;
	apex_inputs->lowg_peak_hyst       = ICM426XX_APEX_CONFIG4_LOWG_PEAK_TH_HYST_156MG;
	apex_inputs->lowg_samples_th      = ICM426XX_APEX_CONFIG5_LOWG_TIME_TH_1_SAMPLE;
	apex_inputs->highg_peak_th        = ICM426XX_APEX_CONFIG6_HIGHG_PEAK_TH_2500MG;
	apex_inputs->highg_peak_hyst      = ICM426XX_APEX_CONFIG4_HIGHG_PEAK_TH_HYST_156MG;
	apex_inputs->highg_samples_th     = ICM426XX_APEX_CONFIG6_HIGHG_TIME_TH_1_SAMPLE;
#endif
	apex_inputs->power_save_time     = ICM426XX_APEX_CONFIG1_DMP_POWER_SAVE_TIME_SEL_8S;
	apex_inputs->power_save          = ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_EN;
	apex_inputs->sensitivity_mode    = ICM426XX_APEX_CONFIG9_SENSITIVITY_MODE_NORMAL;
	apex_inputs->low_energy_amp_th   = ICM426XX_APEX_CONFIG1_LOW_ENERGY_AMP_TH_SEL_80MG;

	return status;
}

int inv_icm426xx_configure_apex_parameters(struct inv_icm426xx *s, const inv_icm426xx_apex_parameters_t *apex_inputs)
{
	int status = 0;
	uint8_t data, data2[7];

	/* DMP cannot be configured if it is running, hence make sure all APEX algorithms are off */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	if(data & BIT_APEX_CONFIG0_PEDO_EN_MASK)
		return INV_ERROR;
	if(data & BIT_APEX_CONFIG0_TILT_EN_MASK)
		return INV_ERROR;
#if defined(ICM_FAMILY_BPLUS)
	if(data & BIT_APEX_CONFIG0_R2W_EN_MASK)
		return INV_ERROR;
#elif defined(ICM_FAMILY_CPLUS)
	if(data & BIT_APEX_CONFIG0_FF_EN_MASK)
		return INV_ERROR;
	if(data & BIT_APEX_CONFIG0_LOWG_EN_MASK)
		return INV_ERROR;
#endif

	/* Pedometer parameters (APEX_CONFIG2, APEX_CONFIG3) */
	data2[1] = (apex_inputs->pedo_amp_th | apex_inputs->pedo_step_cnt_th);
	data2[2] = (apex_inputs->pedo_step_det_th | apex_inputs->pedo_sb_timer_th | apex_inputs->pedo_hi_enrgy_th);

	/* Tilt parameter (APEX_CONFIG4) */
	data2[3] = apex_inputs->tilt_wait_time;

#if defined(ICM_FAMILY_BPLUS)

	/* Raise to wake parameter (APEX_CONFIG4, APEX_CONFIG5, APEX_CONFIG6) */
	data2[3] |= apex_inputs->r2w_sleep_time_out;
	data2[4] = apex_inputs->r2w_mounting_matrix;
	data2[5] = apex_inputs->r2w_gest_delay;

#elif defined(ICM_FAMILY_CPLUS)

	/* LowG and HighG parameters (APEX_CONFIG4, APEX_CONFIG5, APEX_CONFIG6) */
	data2[3] |= ((uint8_t)apex_inputs->lowg_peak_hyst | (uint8_t)apex_inputs->highg_peak_hyst);
	data2[4] = ((uint8_t)apex_inputs->lowg_peak_th | (uint8_t)apex_inputs->lowg_samples_th);
	data2[5] = ((uint8_t)apex_inputs->highg_peak_th | (uint8_t)apex_inputs->highg_samples_th);

#endif

	/* Power Save mode parameters (APEX_CONFIG0, APEX_CONFIG1) */
	data &= (uint8_t)~BIT_APEX_CONFIG0_DMP_POWER_SAVE_MASK;
	data |= (uint8_t)apex_inputs->power_save;
	data2[0] = apex_inputs->power_save_time;

	/* Additionnal parameters for Pedometer in Slow Walk mode (APEX_CONFIG1, APEX_CONFIG9) */
	data2[0] |= (uint8_t)apex_inputs->low_energy_amp_th;
	data2[6] = apex_inputs->sensitivity_mode;

	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	/* Set memory bank 4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);

	/* Access continuous config registers (CONFIG1-CONFIG6) */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG1_B4, 6, &data2[0]);

#if defined(ICM_FAMILY_CPLUS)
	/* freefall parameters (APEX_CONFIG10) */
	data2[0] = ((uint8_t)apex_inputs->ff_debounce_duration | (uint8_t)apex_inputs->ff_max_duration_cm | (uint8_t)apex_inputs->ff_min_duration_cm);
	/* Access continuous config registers (CONFIG1-CONFIG6) */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG10_B4, 1, &data2[0]);
#endif

	/* Spare bank sel transaction since CONFIG9 is also in bank 4 (but not contiguous) */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG9_B4, 1, &data2[6]);

	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);

	return status;
}

int inv_icm426xx_get_apex_parameters(struct inv_icm426xx *s, inv_icm426xx_apex_parameters_t *apex_params)
{
	int status = 0;
	uint8_t data[6] /*in B4*/, data2[2] /*also in B4*/;

	/* Set memory bank 4 */
	status |= inv_icm426xx_set_reg_bank(s, 4);

	/* Access continuous config registers (CONFIG1-CONFIG6) */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG1_B4, sizeof(data), &data[0]);
	/* Spare bank sel transaction since CONFIG9-10 is also in bank 4 (but not contiguous) */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG9_B4, sizeof(data2), &data2[0]);

	/* Set memory bank 0 */
	status |= inv_icm426xx_set_reg_bank(s, 0);

	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, (uint8_t *)&apex_params->power_save);
	apex_params->power_save &= BIT_APEX_CONFIG0_DMP_POWER_SAVE_MASK;

	apex_params->pedo_amp_th = (ICM426XX_APEX_CONFIG2_PEDO_AMP_TH_t)
		(data[1] & BIT_APEX_CONFIG2_PEDO_AMP_TH_MASK);
	apex_params->pedo_step_cnt_th =
		(data[1] & BIT_APEX_CONFIG2_PEDO_STEP_CNT_TH_MASK) >> BIT_APEX_CONFIG2_PEDO_STEP_CNT_TH_POS;
	apex_params->pedo_step_det_th =
		(data[2] & BIT_APEX_CONFIG3_PEDO_STEP_DET_TH_MASK) >> BIT_APEX_CONFIG3_PEDO_STEP_DET_TH_POS;
	apex_params->pedo_sb_timer_th = (ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_t)
		(data[2] & BIT_APEX_CONFIG3_PEDO_SB_TIMER_TH_MASK);
	apex_params->pedo_hi_enrgy_th = (ICM426XX_APEX_CONFIG3_PEDO_HI_ENRGY_TH_t)
		(data[2] & BIT_APEX_CONFIG3_PEDO_HI_ENRGY_TH_MASK);
	apex_params->tilt_wait_time = (ICM426XX_APEX_CONFIG4_TILT_WAIT_TIME_t)
		(data[3] & BIT_APEX_CONFIG4_TILT_WAIT_TIME_MASK);
	apex_params->power_save_time = (ICM426XX_APEX_CONFIG1_DMP_POWER_SAVE_TIME_t)
		(data[0] & BIT_APEX_CONFIG1_DMP_POWER_SAVE_TIME_SEL_MASK);
	apex_params->sensitivity_mode = (ICM426XX_APEX_CONFIG9_SENSITIVITY_MODE_t)
		(data2[0] & BIT_APEX_CONFIG9_SENSITIVITY_MODE_MASK);
	apex_params->low_energy_amp_th = (ICM426XX_APEX_CONFIG1_LOW_ENERGY_AMP_TH_t)
		(data[0] & BIT_APEX_CONFIG1_LOW_ENERGY_AMP_TH_SEL_MASK);
#if defined(ICM_FAMILY_BPLUS)
	apex_params->r2w_sleep_time_out = (ICM426XX_APEX_CONFIG4_R2W_SLEEP_TIME_OUT_t)
		(data[3] & BIT_APEX_CONFIG4_R2W_SLEEP_TIME_OUT_MASK);
	apex_params->r2w_mounting_matrix = (ICM426XX_APEX_CONFIG5_R2W_MOUNTING_MATRIX_t)
		(data[4] & BIT_APEX_CONFIG5_R2W_MOUNTING_MATRIX_MASK);
	apex_params->r2w_gest_delay = (ICM426XX_APEX_CONFIG6_R2W_SLEEP_GEST_DELAY_t)
		(data[5] & BIT_APEX_CONFIG6_R2W_SLEEP_GEST_DELAY_MASK);
#elif defined(ICM_FAMILY_CPLUS)
	apex_params->lowg_peak_hyst = (ICM426XX_APEX_CONFIG4_LOWG_PEAK_TH_HYST_t)
		(data[3] & BIT_APEX_CONFIG4_LOWG_PEAK_TH_HYST_MASK);
	apex_params->highg_peak_hyst = (ICM426XX_APEX_CONFIG4_HIGHG_PEAK_TH_HYST_t)
		(data[3] & BIT_APEX_CONFIG4_HIGHG_PEAK_TH_HYST_MASK);
	apex_params->lowg_peak_th = (ICM426XX_APEX_CONFIG5_LOWG_PEAK_TH_t)
		(data[4] & BIT_APEX_CONFIG5_LOWG_PEAK_TH_MASK);
	apex_params->lowg_samples_th = (ICM426XX_APEX_CONFIG5_LOWG_TIME_TH_SAMPLES_t)
		(data[4] & BIT_APEX_CONFIG5_LOWG_TIME_TH_MASK);
	apex_params->highg_peak_th = (ICM426XX_APEX_CONFIG6_HIGHG_PEAK_TH_t)
		(data[5] & BIT_APEX_CONFIG6_HIGHG_PEAK_TH_MASK);
	apex_params->highg_samples_th = (ICM426XX_APEX_CONFIG6_HIGHG_TIME_TH_SAMPLES_t)
		(data[5] & BIT_APEX_CONFIG6_HIGHG_TIME_TH_MASK);
	apex_params->ff_debounce_duration = (ICM426XX_APEX_CONFIG10_FF_DEBOUNCE_DURATION_t)
		(data2[1] & BIT_APEX_CONFIG10_FF_DEBOUNCE_DURATION_MASK);
	apex_params->ff_max_duration_cm = (ICM426XX_APEX_CONFIG10_FF_MAX_DURATION_t)
		(data2[1] & BIT_APEX_CONFIG10_FF_MAX_DURATION_MASK);
	apex_params->ff_min_duration_cm = (ICM426XX_APEX_CONFIG10_FF_MIN_DURATION_t)
		(data2[1] & BIT_APEX_CONFIG10_FF_MIN_DURATION_MASK);
#endif


	return status;
}

int inv_icm426xx_set_apex_frequency(struct inv_icm426xx * s, const ICM426XX_APEX_CONFIG0_DMP_ODR_t frequency)
{
	uint8_t data;
	int status = 0;
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_DMP_ODR_MASK;
	data |= (uint8_t)frequency;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	return status;
}

int inv_icm426xx_start_dmp(struct inv_icm426xx * s)
{
	int status = 0;

	/* On first enabling of DMP, reset internal state */
	if (!s->dmp_is_on) {
		if (!s->dmp_from_sram) {
			// Reset SRAM to 0's if execution from ROM
			// otherwise, it's user responsibility to do it.
		status |= inv_icm426xx_reset_dmp(s);
		}
		s->dmp_is_on = 1;
	}

	/* Initialize DMP */
	status |= inv_icm426xx_resume_dmp(s);

	return status;
}

int inv_icm426xx_reset_dmp(struct inv_icm426xx * s)
{
	int status = 0;
	const int ref_timeout = 5000; /*50 ms*/
	int timeout = ref_timeout;
	uint8_t data;

	/* Reset DMP internal memories */
	data = ICM426XX_SIGNAL_PATH_RESET_DMP_MEM_RESET_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data);
	inv_icm426xx_sleep_us(1000U);

	/* Make sure reset procedure has finished by reading back mem_reset_en bit */
	do {
		status |= inv_icm426xx_read_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data);
		inv_icm426xx_sleep_us(10U);
	} while ((data & BIT_SIGNAL_PATH_RESET_DMP_MEM_RESET_MASK) && timeout--);

	if (timeout <= 0)
		return INV_ERROR_TIMEOUT;

	return status;
}

int inv_icm426xx_enable_apex_pedometer(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_start_dmp(s);

	/* Enable Pedometer */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data |= (uint8_t)ICM426XX_APEX_CONFIG0_PEDO_EN_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

int inv_icm426xx_disable_apex_pedometer(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	/* Disable Pedometer */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_PEDO_EN_MASK; /* == ICM426XX_APEX_CONFIG0_PEDO_EN_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

#if defined(ICM_FAMILY_BPLUS)

int inv_icm426xx_enable_apex_r2w(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_start_dmp(s);

	/* Enable r2w */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data |= (uint8_t)ICM426XX_APEX_CONFIG0_R2W_EN_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

int inv_icm426xx_disable_apex_r2w(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	/* Disable r2w */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_R2W_EN_MASK; /* == ICM426XX_APEX_CONFIG0_R2W_EN_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

#elif defined(ICM_FAMILY_CPLUS)

int inv_icm426xx_enable_apex_ff(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;
	
	status |= inv_icm426xx_start_dmp(s);
	
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data |= (uint8_t)ICM426XX_APEX_CONFIG0_FF_EN_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	
	return status;
}

int inv_icm426xx_disable_apex_ff(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;

	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_FF_EN_MASK; /* == ICM426XX_APEX_CONFIG0_FF_EN_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	
	return status;
}

int inv_icm426xx_enable_apex_lowg(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;
	
	status |= inv_icm426xx_start_dmp(s);
	
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data |= (uint8_t)ICM426XX_APEX_CONFIG0_LOWG_EN_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	
	return status;
}

int inv_icm426xx_disable_apex_lowg(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;

	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_LOWG_EN_MASK; /* == ICM426XX_APEX_CONFIG0_LOWG_EN_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	
	return status;
}
#endif

int inv_icm426xx_enable_apex_tilt(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	status |= inv_icm426xx_start_dmp(s);

	/* Enable Tilt */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data |= (uint8_t)ICM426XX_APEX_CONFIG0_TILT_EN_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

int inv_icm426xx_disable_apex_tilt(struct inv_icm426xx * s)
{
	uint8_t data;
	int status = 0;

	/* Disable Tilt */
	status |= inv_icm426xx_read_reg(s, MPUREG_APEX_CONFIG0, 1, &data);
	data &= (uint8_t)~BIT_APEX_CONFIG0_TILT_EN_MASK; /* == ICM426XX_APEX_CONFIG0_TILT_EN_DIS */
	status |= inv_icm426xx_write_reg(s, MPUREG_APEX_CONFIG0, 1, &data);

	return status;
}

int inv_icm426xx_get_apex_data_activity(struct inv_icm426xx * s, inv_icm426xx_apex_step_activity_t * apex_activity)
{
	uint8_t data[4];
	int status = inv_icm426xx_read_reg(s, MPUREG_APEX_DATA0, 4, data);
	
	apex_activity->step_cnt = (((uint16_t)data[1]) << 8) | data[0];
	apex_activity->step_cadence = data[2];
	apex_activity->activity_class = data[3] & BIT_APEX_DATA3_ACTIVITY_CLASS_MASK;
	
	return status;
}


int inv_icm426xx_get_tap_data(struct inv_icm426xx * s, inv_icm426xx_tap_data_t * tap_data)
{
	uint8_t data[2];
	int status = inv_icm426xx_read_reg(s, MPUREG_APEX_DATA4, 2, data);

	tap_data->tap_num = (ICM426XX_APEX_DATA4_TAP_NUM_t) (data[0] & BIT_APEX_DATA4_TAP_NUM_MASK);
	tap_data->tap_axis = (ICM426XX_APEX_DATA4_TAP_AXIS_t)(data[0] & BIT_APEX_DATA4_TAP_AXIS_MASK);
	tap_data->tap_dir = (ICM426XX_APEX_DATA4_TAP_DIR_t) (data[0] & BIT_APEX_DATA4_TAP_DIR_MASK);
	tap_data->double_tap_timing = (data[1] & BIT_APEX_DATA5_DOUBLE_TAP_TIMING_MASK);

	return status;
}

int inv_icm426xx_load_dmp_sram_code(struct inv_icm426xx *s, const uint8_t *dmp_prog, const uint32_t start_offset, const uint32_t size)
{
	uint8_t pwr_mgmt;
	uint32_t i, cur_idx;
	uint8_t dmp_sram_offset = 0x40, data;
	int status = 0;

	/* DMP SRAM is updated by DMP DMA every time accel ODR triggers, hence 
	 * first 7B of SRAM are not usable.
	 * Since start addresses are aligned on 32B binary, there is some loss
	 * here (minor however).
	 */
	if (size + start_offset > 1024U)
		return INV_ERROR_SIZE;

	status |= inv_icm426xx_read_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt);
	/* Writing DMP SRAM not supported when accel is not IDLE */
	if (pwr_mgmt & BIT_PWR_MGMT_0_ACCEL_MODE_MASK)
		return INV_ERROR;

	data = (pwr_mgmt | ((uint8_t)ICM426XX_PWR_MGMT_0_IDLE_DIS));
	status |= inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &data);
	inv_icm426xx_sleep_us(200);

	status |= inv_icm426xx_reset_dmp(s);

	data = BIT_DMP_MEM_ACCESS_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_SCAN0, 1, &data);
	inv_icm426xx_sleep_us(100);

	/* Write new DMP program into SRAM */
	data = dmp_sram_offset;
	status |= inv_icm426xx_write_reg(s, MPUREG_MEM_BANK_SEL, 1, &data);
	cur_idx = 0;
	for (i = start_offset; i < (size + start_offset); i++) {
		if (0 == (i % 256)) {
			/* Start filling new DMP SRAM bank */
			data = dmp_sram_offset + (i / 256);
			status |= inv_icm426xx_write_reg(s, MPUREG_MEM_BANK_SEL, 1, &data);
		}

		data = i % 256;
		status |= inv_icm426xx_write_reg(s, MPUREG_MEM_START_ADDR, 1, &data);
		data = dmp_prog[cur_idx];
		status |= inv_icm426xx_write_reg(s, MPUREG_MEM_R_W, 1, &data);
		cur_idx++;
	}

	if (cur_idx != size) {
		status = INV_ERROR_UNEXPECTED;
		goto end_load_dmp_sram;
	}

	/* Verify SRAM content is matching loaded DMP program */
	cur_idx = 0;
	i = start_offset;
	data = dmp_sram_offset;
	status |= inv_icm426xx_write_reg(s, MPUREG_MEM_BANK_SEL, 1, &data);
	while (i < (size + start_offset)) {
		/* Handle MEM_BANK_SEL switch on 256 multiples */
		if (0 == (i % 256)) {
			data = dmp_sram_offset + (i / 256);
			status |= inv_icm426xx_write_reg(s, MPUREG_MEM_BANK_SEL, 1, &data);
		}
		/* Read bytes [0-255] of each bank which was written */
		data = i % 256;
		status |= inv_icm426xx_write_reg(s, MPUREG_MEM_START_ADDR, 1, &data);
		status |= inv_icm426xx_read_reg(s, MPUREG_MEM_R_W, 1, &data);
		/* Check SRAM content match byte-to-byte */
		if (data != dmp_prog[cur_idx]) {
			status = INV_ERROR_MEM;
			goto end_load_dmp_sram;
		}

		i++;
		cur_idx++;
	}

	data = 0;
	status |= inv_icm426xx_write_reg(s, MPUREG_SCAN0, 1, &data);
	inv_icm426xx_sleep_us(100);
	if (!status) {
		// Succesful SRAM load, execution is possible
		s->dmp_from_sram = 1;
	}

	/* Update DMP program counter to start from SRAM */
	status |= inv_icm426xx_set_reg_bank(s, 3);
	data = 0x8;
	status |= inv_icm426xx_write_reg(s, 0x71, 1, &data);
	status |= inv_icm426xx_set_reg_bank(s, 0);

end_load_dmp_sram:
	status |= inv_icm426xx_write_reg(s, MPUREG_PWR_MGMT_0, 1, &pwr_mgmt);

	return status;
}

/*
 * Static functions definition
 */

static int inv_icm426xx_resume_dmp(struct inv_icm426xx * s)
{
	int status = 0;
	uint8_t data;
	const int ref_timeout = 5000; /*50 ms*/
	int timeout = ref_timeout;

	data = ICM426XX_SIGNAL_PATH_RESET_DMP_INIT_EN;
	status |= inv_icm426xx_write_reg(s, MPUREG_SIGNAL_PATH_RESET, 1, &data);

	/* Max accel ODR is 100Hz, and DMP INIT is updated every accel ODR */
	inv_icm426xx_sleep_us(10000U);

	/* Wait for DMP idle */
	do {
		status |= inv_icm426xx_read_reg(s, MPUREG_APEX_DATA3, 1, &data);
		inv_icm426xx_sleep_us(10U);
	} while ((0 == (data & BIT_APEX_DATA3_DMP_IDLE_MASK)) && timeout--);

	if (timeout <= 0)
		return INV_ERROR_TIMEOUT;

	return status;
}
