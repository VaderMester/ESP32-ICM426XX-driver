/*
$License:
	Copyright (C) 2018 InvenSense Corporation, All Rights Reserved.
$
*/
 

#ifndef _INVN_ALGO_AGM_H_
#define _INVN_ALGO_AGM_H_

#include "invn/common/invn_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/** \defgroup AGM AGM
 *  \brief Algorithm that provides device orientation. Algorithm inputs are raw Accelerometer, Gyroscope and Magnetometer data. 
 *         Algorithm outputs: calibrated sensor and 9-axis sensor fusion. 
 *  \warning supported sampling frequency [50 Hz-1000 Hz]
 *  \warning supported gyroscope FSR [250 dps, 500 dps, 1000 dps, 2000 dps, 4000 dps]
 *  \warning supported accelerometer FSR [1 g, 2 g, 4 g, 8 g, 16 g]
 */

#define INVN_ALGO_AGM_INPUT_MASK_ACC        1  ///< Raw Accel update mask
#define INVN_ALGO_AGM_INPUT_MASK_GYR        2  ///< Raw Gyro update mask
#define INVN_ALGO_AGM_INPUT_MASK_MAG        4  ///< Raw Mag update mask

#define INVN_ALGO_AGM_OUTPUT_MASK_ACCEL_CAL 1  ///< Accel cal output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_GYRO_CAL  2  ///< Gyro cal output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_MAG_CAL   4  ///< Mag cal output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AG   8  ///< Game Rotation Vector (Accel and Gyro Fusion) output update mask 
#define INVN_ALGO_AGM_OUTPUT_MASK_QUAT_AGM  16 ///< Rotation Vector (Accel, Gyro and Magnetometer Fusion) output update mask 
#define INVN_ALGO_AGM_OUTPUT_MASK_GRAVITY   32 ///< Gravity vector output update mask
#define INVN_ALGO_AGM_OUTPUT_MASK_LINEARACC 64 ///< Linear acceleration vector output update mask


/* Forward declarations */
struct inv_icm426xx;

/*! \struct InvnAlgoAGMInput
 * AGM input structure (raw data) \ingroup AGM
 */
typedef struct 
{
	int32_t mask;            /*!<  mask to specify updated inputs. */
	int64_t sRimu_time_us;   /*!<  timestamp \f$ [\mu s]\f$ of raw accel and gyro */
	int32_t sRacc_data[3];   /*!<  raw accelerometer in high resolution mode. Expect Full Scale Value coded on 20 bit (i.e. +/- FSR g = 1<<19 LSB) */
	int32_t sRgyr_data[3];   /*!<  raw gyroscope in high resolution mode. Expect Full Scale Value coded on 20 bit (i.e. +/- FSR dps = 1<<19 LSB) */
	int16_t sRtemp_data;     /*!<  raw temperature */
	int64_t sRmag_time_us;   /*!<  timestamp of raw mag */
	int32_t sRmag_data[3];   /*!<  raw mag, (1uT = 1<<16) */
} InvnAlgoAGMInput; 

	
/*! \struct InvnAlgoAGMOutput
 * AGM output structure (calibrated sensors and fusion output)  \ingroup AGM
 */
typedef struct 
{
	int32_t mask;                /*!< mask to specify updated outputs */
	int32_t acc_uncal_q16[3];    /*!< uncalibrated accelerometer (1 g = 0x1000) */
	int32_t acc_cal_q16[3];      /*!< calibrated accelerometer (1 g = 0x1000) */
	int32_t acc_bias_q16[3];     /*!< accelerometer bias (1 g = 0x1000)*/
	int8_t  acc_accuracy_flag;   /*!< accelerometer accuracy from 0(non calibrated) to 3(well calibrated) */

	int32_t gyr_uncal_q16[3];    /*!< uncalibrated gyroscope (1 dps = 0x1000) */
	int32_t gyr_cal_q16[3];      /*!< calibrated gyroscope (1 dps = 0x1000) */
	int32_t gyr_bias_q16[3];     /*!< gyro bias (1 dps = 0x1000)*/
	int8_t  gyr_accuracy_flag;   /*!< gyro accuracy, from 0(non calibrated) to 3(well calibrated) */

	int32_t mag_uncal_q16[3];    /*!< uncalibrated magnetometer (1uT = 1<<16) */
	int32_t mag_cal_q16[3];      /*!< calibrated magnetometer (1uT = 1<<16) */
	int32_t mag_bias_q16[3];     /*!< magnetometer bias (1uT = 1<<16) */
	int8_t  mag_accuracy_flag;   /*!< magnetometer accuracy, from 0(non calibrated) to 3(well calibrated) */

	int32_t grv_quat_q30[4];     /*!< 6-axis (accel and gyro fusion) quaternion */	
	int32_t rv_quat_q30[4];      /*!< 9-axis (accel, gyro and magnetometer fusion) quaternion */
	int32_t rv_accuracy_q27;     /*!< 9-axis (accel, gyro and magnetometer fusion) 3\sigma accuracy in rad */
	int32_t gravity_q16[3];      /*!< gravity estimation in sensor frame */
	int32_t linear_acc_q16[3];   /*!< linear acceleration estimation in sensor frame */
	
	int32_t temp_degC_q16;       /*!< temperature (1 \f$ [^{\circ}C]\f$ = 0x1000)*/
} InvnAlgoAGMOutput; 


/*! \struct InvnAlgoAGMConfig
 * AGM configuration structure (sensor related settings) \ingroup AGM
 */
typedef struct 
{
	int32_t * acc_bias_q16;     /*!<  Previously stored accel bias pointer. If pointer is NULL or 0, offset will be set to { 0, 0, 0} */
	int32_t * gyr_bias_q16;     /*!<  Previously stored gyro bias pointer. If pointer is NULL or 0, offset will be set to { 0, 0, 0} */
	int32_t * mag_bias_q16;     /*!<  mag_bias_q16 Previously stored mag bias pointer If pointer is NULL or 0, offset will be set to { 0, 0, 0} */
	int8_t  acc_accuracy;       /*!<  Previously stored accelerometer bias accuracy (0 to 3) */
	int8_t  gyr_accuracy;       /*!<  Previously stored gyroscope bias accuracy (0 to 3)  */
	int8_t  mag_accuracy;       /*!<  Previously stored magnetometer bias accuracy (0 to 3)  */

	int32_t acc_fsr;            /*!<  accelerometer full scale range [g] */
	int32_t gyr_fsr;            /*!<  gyroscope full scale range [dps] */

	uint32_t acc_odr_us;        /*!<  accelerometer output data rate in \f$ [\mu s]\f$ */
	uint32_t gyr_odr_us;        /*!<  gyroscope output data rate \f$ [\mu s]\f$ */

	int32_t   mag_sc_q16;       /*!<  magnetometer sensitivity (uT/LSB, e.g. mag_uT = (mag_sc_q16 * raw_mag_LSB)/65536) */
	uint32_t  mag_odr_us;       /*!<  magnetometer output data rate \f$ [\mu s]\f$ */

	int32_t temp_sensitivity;   /*!<  temperature sensitivity in q30 (if temperature(\f$ ^{\circ}C \f$) = LSB * k + z, then temp_sensitivity = k) */
	int32_t temp_offset;        /*!<  temperature offset in q16 (if temperature(\f$ ^{\circ}C \f$) = LSB * k + z, then temp_offset = z) */
} InvnAlgoAGMConfig; 


/*!
 * \brief Return library version x.y.z-suffix as a char array
 * \retval libray version a char array "x.y.z-suffix"
 * \ingroup AGM
 */
const char * invn_algo_agm_version(void);

/*!
 * \brief Initializes algorithms with default parameters and reset states.
 * (\icm_device[in] Invensense ICM426XX device pointer. Only when gyro assisted is enabled.)
 * \config[in] algo init parameters structure.
 * \return initialization success indicator.
 * \retval 0 Success
 * \retval 1 Fail
 * \ingroup AGM
 */
#ifdef WITH_GYRO_ASSIST
uint8_t invn_algo_agm_init_a(struct inv_icm426xx * icm_device, const InvnAlgoAGMConfig * config);
#else
uint8_t invn_algo_agm_init(const InvnAlgoAGMConfig * config);
#endif

/*!
 * \brief Sets algo config structure.
 * \config[in] config structure of the algo.
  * \ingroup AGM
 */
void invn_algo_agm_set_config(const InvnAlgoAGMConfig * config);


/*!
 * \brief Performs algorithm computation.
 * \in inputs algorithm input. Input mask (inputs->mask) should be set with respect to new sensor data in InvnAlgoAGMInput.
 * \out outputs algorithm output. Output mask (outputs->mask) reports updated outputs in InvnAlgoAGMOutput.
 * \ingroup AGM
 */
void invn_algo_agm_process(const InvnAlgoAGMInput *inputs, InvnAlgoAGMOutput *outputs);

#ifdef __cplusplus
}
#endif


#endif
