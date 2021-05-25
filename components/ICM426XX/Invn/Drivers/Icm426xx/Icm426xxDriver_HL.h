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

/** @defgroup DriverIcm426xxDriver_HL Icm426xx driver high level functions
 *  @brief High-level function to setup an Icm426xx device
 *  @ingroup  DriverIcm426xx
 *  @{
 */

/** @file Icm426xxDriver_HL.h
 * High-level function to setup an Icm426xx device
 */

#ifndef _INV_ICM426xx_DRIVER_HL_H_
#define _INV_ICM426xx_DRIVER_HL_H_

#ifdef __cplusplus
extern "C" {
#endif
/*
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"

#include "Invn/InvError.h"
*/
#include "Icm426xxDefs.h"
#include "Icm426xxTransport.h"

#include "InvError.h"

#include <stdint.h>
#include <string.h>

/** @brief Ligthen driver logic by stripping out procedures on transitions
 *  @details
 *  In the nominal case, ie. when sensors are enabled and their output has
 *  settled, ICM-426XX will not need the logic to handle each transition. They
 *  are part of each API function so this code will be linked in regardless.
 *  This might not be desirable for the most size-constrained platforms and
 *  it can be avoided by setting this define to 1.
 */
#ifndef INV_ICM426XX_LIGHTWEIGHT_DRIVER
	#define INV_ICM426XX_LIGHTWEIGHT_DRIVER 0
#endif

/** @brief Scale factor and max ODR 
 *  Dependant of chip
 */
#if (defined(ICM42600) || defined(ICM42622) || defined(ICM42633) || defined(ICM42686) || defined(ICM42688))
	/* 
	 * For parts with maximum ODR of 32 KHz,
	 * PLL is running at 19.2 MHz instead of the nominal 20.48 MHz
	 * Time resolution has to be scaled by 20.48/19.2=1.06666667
	 * To prevent floating point usage, one can use 32/30
	 *
	 * The decimator also need to know the maximum ODR.
	 */
	#define PLL_SCALE_FACTOR_Q24 ((32UL<<24)/30)
	#define ICM_PART_DEFAULT_OIS_MODE ICM426XX_SENSOR_CONFIG2_OIS_MODE_32KHZ
#else
	#define PLL_SCALE_FACTOR_Q24 (1UL<<24)
	#define ICM_PART_DEFAULT_OIS_MODE ICM426XX_SENSOR_CONFIG2_OIS_MODE_8KHZ
#endif

/** @brief Max FSR values for accel and gyro
 *  Dependant of chip
 */
#if defined(ICM42686)
	#define ACCEL_CONFIG0_FS_SEL_MAX ICM426XX_ACCEL_CONFIG0_FS_SEL_32g
	#define GYRO_CONFIG0_FS_SEL_MAX  ICM426XX_GYRO_CONFIG0_FS_SEL_4000dps

	#define ACCEL_OFFUSER_MAX_MG 2000 
	#define GYRO_OFFUSER_MAX_DPS 128 
#else
	#define ACCEL_CONFIG0_FS_SEL_MAX ICM426XX_ACCEL_CONFIG0_FS_SEL_16g
	#define GYRO_CONFIG0_FS_SEL_MAX  ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps

	#define ACCEL_OFFUSER_MAX_MG 1000 
	#define GYRO_OFFUSER_MAX_DPS 64
#endif

/** @brief RTC Support flag
 *  Define whether the RTC mode is supported
 *  Dependant of chip
 */	
#if (defined(ICM42600) || defined(ICM42622) || defined(ICM42633) || defined(ICM42686) || defined(ICM42688))
	#define RTC_SUPPORTED 1
#else
	#define RTC_SUPPORTED 0
#endif

/** @brief Icm426xx maximum buffer size mirrored from FIFO at polling time
 *  @warning fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE
 */
#define ICM426XX_FIFO_MIRRORING_SIZE 16 * 129 // packet size * max_count = 2064

/** @brief Default value for the WOM threshold
 *  Resolution of the threshold is ~= 4mg
 */
#define ICM426XX_DEFAULT_WOM_THS_MG 52>>2 /* = 52mg/4 */

/** @brief Icm426xx Accelerometer start-up time before having correct data
 */
#define ICM426XX_ACC_STARTUP_TIME_US 20000U

/** @brief Icm426xx Gyroscope start-up time before having correct data
 */
#define ICM426XX_GYR_STARTUP_TIME_US 60000U

/** @brief Sensor identifier for UI control and OIS function
 */
enum inv_icm426xx_sensor {
	INV_ICM426XX_SENSOR_ACCEL,               /**< Accelerometer (UI control path) */
	INV_ICM426XX_SENSOR_GYRO,                /**< Gyroscope (UI control path) */
	INV_ICM426XX_SENSOR_FSYNC_EVENT,         /**< Used by OIS and UI control layers */
	INV_ICM426XX_SENSOR_OIS,                 /**< Only used by OIS layer */
	INV_ICM426XX_SENSOR_TEMPERATURE,         /**< Chip temperature, enabled by default. However, it will be reported only if Accel and/or Gyro are also enabled. 
	                                              The Temperature's ODR (Output Data Rate) will match the ODR of Accel or Gyro, or the fastest if both are enabled*/
	INV_ICM426XX_SENSOR_TAP,                 /**< Tap and Double tap */
	INV_ICM426XX_SENSOR_DMP_PEDOMETER_EVENT, /**< Pedometer: step is detected */
	INV_ICM426XX_SENSOR_DMP_PEDOMETER_COUNT, /**< Pedometer: step counter */
	INV_ICM426XX_SENSOR_DMP_TILT,            /**< Tilt */
#if defined(ICM_FAMILY_BPLUS)
	INV_ICM426XX_SENSOR_DMP_R2W,             /**< Raise to wake */
#elif defined(ICM_FAMILY_CPLUS)
	INV_ICM426XX_SENSOR_DMP_FF,              /**< Free Fall */
	INV_ICM426XX_SENSOR_DMP_LOWG,            /**< Low G */
#endif
	INV_ICM426XX_SENSOR_MAX
};

/** @brief Configure Fifo usage
 */
typedef enum {
	INV_ICM426XX_FIFO_DISABLED = 0,              /**< Fifo is disabled and data source is sensors registers */
	INV_ICM426XX_FIFO_ENABLED  = 1,              /**< Fifo is used as data source */
}INV_ICM426XX_FIFO_CONFIG_t;

/** @brief Sensor event structure definition
 */
typedef struct {
	int sensor_mask;
	uint16_t timestamp_fsync;
	int16_t accel[3]; 
	int16_t gyro[3]; 
	int16_t temperature;
	int8_t accel_high_res[3];
	int8_t gyro_high_res[3];
} inv_icm426xx_sensor_event_t;

/** @brief Icm426xx driver states definition
 */
struct inv_icm426xx {
	struct inv_icm426xx_transport transport;                              /**< Warning : this field MUST be the first one of 
	                                                                   struct icm426xx */

	void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event); /**< callback executed by inv_icm426xx_get_data_from_fifo function
	                                                                   for each data packet extracted from fifo or inv_icm426xx_get_data_from_registers read data from register
	                                                                   This field may be NULL if inv_icm426xx_get_data_from_fifo/inv_icm426xx_get_data_from_registers
	                                                                   is not used by application. */

	int gyro_st_bias[3];                                          /**< collected bias values (lsb) during self test */
	int accel_st_bias[3];
	int st_result;                                                   /**< Flag to keep track if self-test has been already run by storing acc and gyr results */

	uint8_t fifo_data[ICM426XX_FIFO_MIRRORING_SIZE];              /**<  FIFO mirroring memory area */

	uint8_t tmst_to_reg_en_cnt;                                   /**< internal counter to keep track of the timestamp to register access availability */
	
	uint8_t dmp_is_on;                                            /**< DMP started status */
	uint8_t dmp_from_sram;                                        /**< DMP executes from SRAM */

	uint64_t gyro_start_time_us;                                  /**< internal state needed to discard first gyro samples */
	uint64_t accel_start_time_us;                                 /**< internal state needed to discard first accel samples */
	uint8_t endianess_data;                                       /**< internal status of data endianess mode to report correctly data */
	uint8_t fifo_highres_enabled;                                 /**< FIFO packets are 20 bytes long */
	INV_ICM426XX_FIFO_CONFIG_t fifo_is_used;                      /**< Data are get from FIFO or from sensor registers. By default Fifo is used*/
	
	#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
		/* First FSYNC event after enable is irrelevant 
		 * When the sensors are switched off and then on again, an FSYNC tag with incorrect FSYNC value can be generated 
		 * on the next first ODR. 
		 * Solution: FSYNC tag and FSYNC data should be ignored on this first ODR.
		 */
		uint8_t fsync_to_be_ignored;
	#endif
	
	/* Accel Low Power could report with wrong ODR if internal counter for ODR changed overflowed
	 * WUOSC clock domain is informed through an 3bit counter that ODR
	 * has changed in RC/PLL mode. Every 8 event, this counter overflows
	 * and goes back to 0, same as 'no ODR changes', therefore previous ALP
	 * ODR is re-used.
	 * Solution: Keep track of ODR changes when WUOSC is disabled and perform dummy ODR changes when re-enabling WU after 8*n ODR changes.
	 */
	uint32_t wu_off_acc_odr_changes;

	uint8_t wom_smd_mask;   /**< This variable keeps track if wom or smd is enabled */

	uint8_t wom_enable;   /**< This variable keeps track if wom is enabled */
	
	/* Software mirror of BW and AVG settings in hardware, to be re-applied on each enabling of sensor */
	struct {
		uint8_t acc_lp_avg; /**< Low-power averaging setting for accelerometer */
		uint8_t reserved;   /**< reserved field */
		uint8_t acc_ln_bw;  /**< Low-noise filter bandwidth setting for accelerometer */
		uint8_t gyr_ln_bw;  /**< Low-noise filter bandwidth setting for gyroscope */
	} avg_bw_setting;

	uint64_t gyro_power_off_tmst;   /**< This variable keeps track of timestamp when gyro is power off */
};

/* Interrupt enum state for INT1, INT2, and IBI */
typedef enum{
	INV_ICM426XX_DISABLE = 0,
	INV_ICM426XX_ENABLE
}inv_icm426xx_interrupt_value;

/** @brief Icm426xx set of interrupt enable flag
 */
typedef struct {
	inv_icm426xx_interrupt_value INV_ICM426XX_UI_FSYNC;
	inv_icm426xx_interrupt_value INV_ICM426XX_UI_DRDY;
	inv_icm426xx_interrupt_value INV_ICM426XX_FIFO_THS;
	inv_icm426xx_interrupt_value INV_ICM426XX_FIFO_FULL;
	inv_icm426xx_interrupt_value INV_ICM426XX_SMD;
	inv_icm426xx_interrupt_value INV_ICM426XX_WOM_X;
	inv_icm426xx_interrupt_value INV_ICM426XX_WOM_Y;
	inv_icm426xx_interrupt_value INV_ICM426XX_WOM_Z;
	inv_icm426xx_interrupt_value INV_ICM426XX_STEP_DET;
	inv_icm426xx_interrupt_value INV_ICM426XX_STEP_CNT_OVFL;
	inv_icm426xx_interrupt_value INV_ICM426XX_TILT_DET;
#if defined(ICM_FAMILY_BPLUS) 
	inv_icm426xx_interrupt_value INV_ICM426XX_SLEEP_DET;
	inv_icm426xx_interrupt_value INV_ICM426XX_WAKE_DET;
#elif defined(ICM_FAMILY_CPLUS)
	inv_icm426xx_interrupt_value INV_ICM426XX_FF_DET;
	inv_icm426xx_interrupt_value INV_ICM426XX_LOWG_DET;
#endif
	inv_icm426xx_interrupt_value INV_ICM426XX_TAP_DET;

	/* INT config */
	ICM426XX_INT_CONFIG_INT1_POLARITY_t int_pol;
	ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_t int_drive;
	ICM426XX_INT_CONFIG_INT1_MODE_t int_mode;
}inv_icm426xx_interrupt_parameter_t;

#if defined(ICM_FAMILY_CPLUS)
/* Possible interface configurations mode */
typedef enum {
	INV_ICM426XX_SINGLE_INTERFACE    = 0x01,
	INV_ICM426XX_DUAL_INTERFACE      = 0x02,
	INV_ICM426XX_DUAL_INTERFACE_SPI4 = 0x06,
	INV_ICM426XX_TRIPLE_INTERFACE    = 0x08,
}inv_icm426xx_interface_mode_t;
#endif

/** @brief Set register bank index
 *  @param bank new bank to be set
 *  @return 0 on success, negative value otherwise
 */
int inv_icm426xx_set_reg_bank(struct inv_icm426xx * s, uint8_t bank);

/** @brief Configure the serial interface used to access the device and execute hardware initialization.
 *
 *  This functions first configures serial interface passed in parameter to make sure device 
 *  is accessible both in read and write. Thus no serial access should be done before 
 *  succesfully executing the present function.
 *
 *  Then if requested serial interface is a primary interface (aka UI interface or AP 
 *  interface), this function initializes the device using the following hardware settings:
 *    - gyroscope fsr = 2000dps
 *    - accelerometer fsr = 4g
 *    - set timestamp resolution to 16us
 *    - enable FIFO mechanism with the following configuration:
 *        - FIFO record mode i.e FIFO count unit is packet 
 *        - FIFO snapshot mode i.e drop the data when the FIFO overflows
 *        - Timestamp is logged in FIFO
 *        - Little Endian fifo_count and fifo_data
 *        - generate FIFO threshold interrupt when packet count reaches FIFO watermark
 *        - set FIFO watermark to 1 packet
 *        - enable temperature and timestamp data to go to FIFO
 *
 *  In case requested serial interface is an auxliary interface (i.e. AUX1 or AUX2) this 
 *  function returns an error.
 *
 *
 *  @param[in] s driver structure. Note that first field of this structure MUST be a struct 
 *  inv_icm426xx_serif.
 *
 *  @param[in] serif pointer on serial interface structure to be used to access icm426xx.
 *
 *  @param[in] sensor_event_cb callback executed by inv_icm426xx_get_data_from_fifo function
 *  each time it extracts some valid data from fifo. Or inv_icm426xx_get_data_from_registers read data
 *  from register. Thus this parameter is optional as long
 *  as inv_icm426xx_get_data_from_fifo/inv_icm426xx_get_data_from_registers function is not used.
 *
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_init(struct inv_icm426xx * s, struct inv_icm426xx_serif * serif, void (*sensor_event_cb)(inv_icm426xx_sensor_event_t * event));

/** @brief Perform a soft reset of the device
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_device_reset(struct inv_icm426xx * s);

/** @brief return WHOAMI value
 *  @param[out] who_am_i WHOAMI for device
 *  @return     0 on success, negative value on error
 */
int inv_icm426xx_get_who_am_i(struct inv_icm426xx * s, uint8_t * who_am_i);

/** @brief Configure Accel clock source
 *  @param[in] new clock source to use
 *  @return 0 on success, negative value on error
 *  @note Transitions when enabling/disabling sensors are already handled by
 *        the driver. This function is here only to force a specific clock
 *        source and shall be used with care.
 */
int inv_icm426xx_force_clock_source(struct inv_icm426xx * s, ICM426XX_INTF_CONFIG1_ACCEL_LP_CLK_t clk_src);

/** @brief Enable/put accel in low power mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables accel and gyro data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 *  @warning inv_icm426xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_icm426xx_enable_accel_low_power_mode(struct inv_icm426xx * s);

/** @brief Enable/put accel in low noise mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables accel and gyro data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 *  @warning inv_icm426xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_icm426xx_enable_accel_low_noise_mode(struct inv_icm426xx * s);

/** @brief Disable all 3 axes of accel
 *  @return 0 on success, negative value on error.
 *  @details
 *  If both accel and gyro are turned off as a result of this
 *  function, they will also be removed from the FIFO and a
 *  FIFO reset will be performed (to guarantee no side effects
 *  until the next enable sensor call)
 *  @warning inv_icm426xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_icm426xx_disable_accel(struct inv_icm426xx * s);

/** @brief Enable/put gyro in low noise mode
 *  @return 0 on success, negative value on error.
 *  @details
 *  It enables gyro and accel data in the FIFO (so
 *  the packet format is 16 bytes). If called first,
 *  the configuration will be applied, otherwise it
 *  will be ignored if the FIFO is not empty (but since
 *  the new configuration is identical it is not a issue).
 *  @warning inv_icm426xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_icm426xx_enable_gyro_low_noise_mode(struct inv_icm426xx * s);

/** @brief Disable all 3 axes of gyro
 *  @return 0 on success, negative value on error.
 *  @details
 *  If both accel and gyro are turned off as a result of this
 *  function, they will also be removed from the FIFO and a
 *  FIFO reset will be performed (to guarantee no side effects
 *  until the next enable sensor call)
 *  @warning inv_icm426xx::register_cache::pwr_mngt_0_reg is modified by this function
 */
int inv_icm426xx_disable_gyro(struct inv_icm426xx * s);

/** @brief Enable fsync tagging functionnality.
 *  In details it:
 *     - enables fsync
 *     - enables timestamp to registers. Once fysnc is enabled fsync counter is pushed to 
 *       fifo instead of timestamp. So timestamp is made available in registers. Note that 
 *       this increase power consumption.
 *     - enables fsync related interrupt
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_enable_fsync(struct inv_icm426xx * s);

/** @brief Disable fsync tagging functionnality.
 *  In details it:
 *     - disables fsync
 *     - disables timestamp to registers. Once fysnc is disabled  timestamp is pushed to fifo 
 *        instead of fsync counter. So in order to decrease power consumption, timestamp is no 
 *        more available in registers.
 *     - disables fsync related interrupt
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_disable_fsync(struct inv_icm426xx * s);

/** @brief  Configure timestamp resolution from FIFO.
 *  @param[in] resol The expected resolution of the timestamp. See enum ICM426XX_TMST_CONFIG_RESOL_t.
 *  @return 0 on success, negative value on error.
 *  @warning The resolution will have no effect if RTC is enabled
 */
int inv_icm426xx_configure_timestamp_resolution(struct inv_icm426xx * s, ICM426XX_TMST_CONFIG_RESOL_t resol);

/** @brief  Configure which interrupt source can trigger ibi interruptions.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage ibi interruptions.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_set_config_ibi(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage IBI interruptions.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_get_config_ibi(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Configure which interrupt source can trigger INT1.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage INT1.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_set_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage INT1.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_get_config_int1(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Configure which interrupt source can trigger INT2.
 *  @param[in] interrupt_to_configure structure with the corresponding state to INT2.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_set_config_int2(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief  Retrieve interrupts configuration.
 *  @param[in] interrupt_to_configure structure with the corresponding state to manage INT2.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_get_config_int2(struct inv_icm426xx * s, inv_icm426xx_interrupt_parameter_t * interrupt_to_configure);

/** @brief Read all registers containing data (tempereature, accelerometer and gyroscope). Then it calls 
 *  sensor_event_cb funtion passed in parameter of inv_icm426xx_init function for each packet 
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_get_data_from_registers(struct inv_icm426xx * s);

/** @brief Read all available packets from the FIFO. For each packet function builds a
 *  sensor event containing packet data and validity information. Then it calls 
 *  sensor_event_cb funtion passed in parameter of inv_icm426xx_init function for each 
 *  packet.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_get_data_from_fifo(struct inv_icm426xx * s);

/** @brief Converts ICM426XX_ACCEL_CONFIG0_ODR_t or ICM426XX_GYRO_CONFIG0_ODR_t enums to period expressed in us
 *  @param[in] odr_bitfield An ICM426XX_ACCEL_CONFIG0_ODR_t or ICM426XX_GYRO_CONFIG0_ODR_t enum
 *  @return The corresponding period expressed in us
 */
uint32_t inv_icm426xx_convert_odr_bitfield_to_us(uint32_t odr_bitfield);

/** @brief Configure accel Output Data Rate
 *  @param[in] frequency The requested frequency.
 *  @sa ICM426XX_ACCEL_CONFIG0_ODR_t
 *  @return 0 on success, negative value on error.
 *  @warning inv_icm426xx::register_cache::accel_cfg_0_reg is modified by this function
 */
int inv_icm426xx_set_accel_frequency(struct inv_icm426xx * s, const ICM426XX_ACCEL_CONFIG0_ODR_t frequency);

/** @brief Configure gyro Output Data Rate
 *  @param[in] frequency The requested frequency.
 *  @sa ICM426XX_GYRO_CONFIG0_ODR_t
 *  @return 0 on success, negative value on error.
 *  @warning inv_icm426xx::register_cache::gyro_cfg_0_reg is modified by this function
 */
int inv_icm426xx_set_gyro_frequency(struct inv_icm426xx * s, const ICM426XX_GYRO_CONFIG0_ODR_t frequency);

/** @brief Set accel full scale range
 *  @param[in] accel_fsr_g requested full scale range.
 *  @sa ICM426XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 *  @warning inv_icm426xx::register_cache::accel_cfg_0_reg is modified by this function
 */
int inv_icm426xx_set_accel_fsr(struct inv_icm426xx * s, ICM426XX_ACCEL_CONFIG0_FS_SEL_t accel_fsr_g);

/** @brief Access accel full scale range
 *  @param[out] accel_fsr_g current full scale range.
 *  @sa ICM426XX_ACCEL_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 *  @warning inv_icm426xx::register_cache::accel_cfg_0_reg is relied upon by this function
 */
int inv_icm426xx_get_accel_fsr(struct inv_icm426xx * s, ICM426XX_ACCEL_CONFIG0_FS_SEL_t * accel_fsr_g);

/** @brief Set gyro full scale range
 *  @param[in] gyro_fsr_dps requested full scale range.
 *  @sa ICM426XX_GYRO_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 *  @warning inv_icm426xx::register_cache::gyro_cfg_0_reg is modified by this function
 */
int inv_icm426xx_set_gyro_fsr(struct inv_icm426xx * s, ICM426XX_GYRO_CONFIG0_FS_SEL_t gyro_fsr_dps);

/** @brief Access gyro full scale range
 *  @param[out] gyro_fsr_dps current full scale range.
 *  @sa ICM426XX_GYRO_CONFIG0_FS_SEL_t.
 *  @return 0 on success, negative value on error.
 *  @warning inv_icm426xx::register_cache::gyro_cfg_0_reg is relied upon by this function
 */
int inv_icm426xx_get_gyro_fsr(struct inv_icm426xx * s, ICM426XX_GYRO_CONFIG0_FS_SEL_t * gyro_fsr_dps);

/** @brief Set accel Low-Power averaging value
 *  @param[in] acc_avg requested averaging value
 *  @sa ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_set_accel_lp_avg(struct inv_icm426xx * s, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_AVG_t acc_avg);

/** @brief Set accel Low-Noise bandwidth value
 *  @param[in] acc_bw requested averaging value
 *  @sa ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_set_accel_ln_bw(struct inv_icm426xx * s, ICM426XX_GYRO_ACCEL_CONFIG0_ACCEL_FILT_BW_t acc_bw);

/** @brief Set gyro Low-Noise bandwidth value
 *  @param[in] gyr_bw requested averaging value
 *  @sa ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_set_gyro_ln_bw(struct inv_icm426xx * s, ICM426XX_GYRO_ACCEL_CONFIG0_GYRO_FILT_BW_t gyr_bw);

/** @brief reset ICM426XX fifo
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_reset_fifo(struct inv_icm426xx * s);

/** @brief Enable the 20bits-timestamp register access to read in a reliable way the strobed timestamp. To do that, the fine clock is forced enabled at some power cost.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_enable_timestamp_to_register(struct inv_icm426xx * s);

/** @brief Disable the 20bits-timestamp register access. Register read always return 0's.
 *  @return 0 on success, negative value on error.
 */
int inv_icm426xx_disable_timestamp_to_register(struct inv_icm426xx * s);

/** @brief Get the timestamp value of icm426xx from register
 *  @param[in] icm_time timestamp read from register
 *  @return 0 on success, negative value on error.
 *  @warning Prior to call this API, the read access to timestamp register must be enabled (see @ref inv_icm426xx_enable_timestamp_to_register() function)
 */
int inv_icm426xx_get_current_timestamp(struct inv_icm426xx * s, uint32_t * icm_time);

/** @brief Enable or disable CLKIN/RTC capability
 *  @param[in] enable 1 if external 32kHz is provided to ICM, 0 otherwise
 *  @return 0 on success, negative value on error.
 *  @warning In case CLKIN is disabled, it is recommended to call inv_icm426xx_configure_timestamp_resolution()
 *  just afterwards so that timestamp resolution is in line with system request
 *  @warning inv_icm426xx::register_cache::intf_cfg_1_reg is modified by this function
 */
int inv_icm426xx_enable_clkin_rtc(struct inv_icm426xx * s, uint8_t enable);

/** @brief Get CLKIN/RTC feature status
 *  @return 0 if CLKIN is disabled, 1 if enabled.
 *  @warning In case CLKIN is disabled, it is recommended to call inv_icm426xx_configure_timestamp_resolution()
 *  just afterwards so that timestamp resolution is in line with system request
 *  @warning inv_icm426xx::register_cache::intf_cfg_1_reg is relied upon by this function
 */
int inv_icm426xx_get_clkin_rtc_status(struct inv_icm426xx * s);

/** @brief Enable 20 bits raw acc and raw gyr data in fifo.
 *  @return 0 on success, negative return code otherwise
 */
int inv_icm426xx_enable_high_resolution_fifo(struct inv_icm426xx * s);

/** @brief Disable 20 bits raw acc and raw gyr data in fifo.
 *  @return 0 on success, negative return code otherwise
 */
int inv_icm426xx_disable_high_resolution_fifo(struct inv_icm426xx * s);

 /** @brief Configure Fifo to select the way data are gathered
 *  @param[in] fifo_config Fifo configuration method : 
 *  if enabled data are comming from fifo and Interrupt is configured on Fifo Watermark
 *  if disabled data are comming from sensor registers and Interrupt is configured on Data ready
 *  @sa INV_ICM426XX_FIFO_CONFIG_t
 */
int inv_icm426xx_configure_fifo(struct inv_icm426xx * s, INV_ICM426XX_FIFO_CONFIG_t fifo_config);

 /** @brief Configure Fifo watermark (also refered to as fifo threshold)
 *  @param[in] wm Watermark value
 */
int inv_icm426xx_configure_fifo_wm(struct inv_icm426xx * s, uint16_t wm);

/** @brief Get FIFO timestamp resolution
 *  @return the timestamp resolution in us as a q24 or 0 in case of error
 */
uint32_t inv_icm426xx_get_fifo_timestamp_resolution_us_q24(struct inv_icm426xx * s);

/** @brief Get register timestamp resolution
 *  @return the timestamp resolution in us as a q24 or 0 in case of error
 */
uint32_t inv_icm426xx_get_reg_timestamp_resolution_us_q24(struct inv_icm426xx * s);


#if defined(ICM_FAMILY_CPLUS)
 /** @brief Configure Fifo decimation rate
 *  @param[in] dec_rate decimation valmue from 0 to 127
 *  if 0, no sample is decimated, 1 means 1 sample over 2 are skipped.
 */
int inv_icm426xx_set_fifo_dec_rate(struct inv_icm426xx * s, uint8_t dec_rate);

/** @brief Configure interface mode
 *  @param[in] interface_mode value can be single, dual (AUX1-SPI3/AUX1-SPI4) or triple interface according to inv_icm426xx_interface_mode_t
 *  @return 0 on success, negative return code otherwise
 */
int inv_icm426xx_interface_change_procedure(struct inv_icm426xx * s, inv_icm426xx_interface_mode_t interface_mode);
#endif

/** @brief Return driver version x.y.z-suffix as a char array
 *  @retval driver version a char array "x.y.z-suffix"
 */
const char * inv_icm426xx_get_version(void);

void inv_icm426xx_format_data(const uint8_t endian, const uint8_t *in, uint16_t *out);



#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM426xx_DRIVER_HL_H_ */

/** @} */
