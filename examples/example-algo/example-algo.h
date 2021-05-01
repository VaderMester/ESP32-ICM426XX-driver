/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
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
#ifndef _EXAMPLE_ALGO_H_
#define _EXAMPLE_ALGO_H_

#include <stdint.h>
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"

#include "Invn/Drivers/Ak0991x/Ak0991x.h"

#include "Invn/Helpers/Icm426xx/helperClockCalib.h"

#include "invn_algo_agm.h"

/*
 * Clock In feature management
 * Enable Clock In feature only for ICM42622 and ICM42688
 * Please set a hardware bridge between PA17 (from MCU) and CLKIN pins (to ICM).
 */
#if defined(ICM42622) || defined(ICM42688)
#define USE_CLK_IN           1
#else
#define USE_CLK_IN           0
#endif


/*
 * Set this define to 0 to disable mag support
 * Recommended value: 1
 */
#define USE_MAG              1

/*
 * Set power mode flag
 * Set this flag to run example in low-noise mode.
 * Reset this flag to run example in low-power mode.
 * Note : low-noise mode is not available with sensor data frequencies less than 12.5Hz.
 */
#define IS_LOW_NOISE_MODE    1

/*
 * Accelerometer and gyroscope frequencies.
 * Recommended value: ICM426XX_GYRO_CONFIG0_ODR_1_KHZ and ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ (1000 Hz)
 * Possible values (same for accel, replace "GYRO" with "ACCEL"): 
 * - ICM426XX_GYRO_CONFIG0_ODR_1_KHZ  (1000 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_500_HZ (500 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_200_HZ (200 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_100_HZ (100 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_50_HZ (50 Hz)
 */
#define GYRO_FREQ            ICM426XX_GYRO_CONFIG0_ODR_1_KHZ
#define ACCEL_FREQ           ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ

/*
 * Magnetometer output data rate in us
 * Only supported value: 10000 us (100 Hz)
 * Possible values:
 * - 5000  (200 Hz)
 * - 10000 (100 Hz)
 * - 20000 (50 Hz)
 * Note: Not use if `USE_MAG` is set to 0
 */
#define MAG_ODR_US           10000

/*
 * Select Fifo resolution Mode (default is low resolution mode)
 * Low resolution mode : 16 bits data format
 * High resolution mode : 20 bits data format
 * Warning: Enabling High Res mode will force FSR to 16g and 2000dps
 */
#define IS_HIGH_RES_MODE 0

/* Mask to print data throught UART */
#define MASK_PRINT_INPUT_DATA          0x01  /** algorithm inputs */
#define MASK_PRINT_ACC_DATA            0x02  /** accel data */
#define MASK_PRINT_GYR_DATA            0x04  /** gyro data */
#define MASK_PRINT_MAG_DATA            0x08  /** magnetometer data */
#define MASK_PRINT_6AXIS_DATA          0x10  /** 6-axis data */
#define MASK_PRINT_9AXIS_DATA          0x20  /** 9-axis data */
#define MASK_PRINT_GRAVITY_DATA        0x40  /** Gravity vector */
#define MASK_PRINT_LINEARACC_DATA      0x80  /** Linear acceleration vector */

/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be succesfully executed before any access to Icm426xx device.
 * 
 * \param[in] icm_serif : Serial interface object.
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 *
 * It initialyses clock calibration module (this will allow to extend the 16 bits 
 * timestamp produced by Icm426xx to a 64 bits timestamp).
 * Then function sets full scale range and frequency for both accel and gyro and it 
 * starts them in the requested power mode. (note that low-power mode is not available 
 * for gyroscope in Icm426xx).
 *
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice(void);

/**
 * \This function clears biases and accuracies.
 *
 * \return 0 on success, negative value on error.
 */
int ResetInvAGMBiases(void);

/**
 * \brief This function initializes biases and accuracies for accelerometer, gyroscope and magnetometer.
 *
 * \return 0 on success, negative value on error.
 */
int InitInvAGMBiases(void);

/**
 * \brief This function initializes the AGM algorithm.
 *
 * \return 0 on success, negative value on error.
 */
int InitInvAGMAlgo(void);

/**
 * \brief This function extracts data from the Icm426xx FIFO.
 *
 * The function just calls Icm426xx driver function inv_icm426xx_get_data_from_fifo.
 * But note that for each packet extracted from FIFO, a user defined function is called to 
 * allow custom handling of each packet. In this example custom packet handling function
 * is HandleInvDeviceFifoPacket.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromInvDevice(void);

/**
 * \brief This function is the custom handling packet function.
 *
 * It is passed in parameter at driver init time and it is called by 
 * inv_icm426xx_get_data_from_fifo function each time a new valid packet is extracted 
 * from FIFO.
 * In this implementation, function extends packet timestamp from 16 to 64 bits and then
 * process data from packet and print them on UART.
 *
 * \param[in] event structure containing sensor data from one packet
 */
void HandleInvDeviceFifoPacket(inv_icm426xx_sensor_event_t * event);

#if USE_MAG
/*!
 * \brief Set up magnetometer AKM09915
 * \param[in] akm_serif : Serial interface object.
 * \return 0 in case of success, negative value in case 
 */
int SetupMagDevice(struct inv_ak0991x_serif * akm_serif);

/*!
 * \brief Start a new data acquisition on magnetometer.
 * 
 * Magnetometer only works in one shot mode. Thus this function only initiates one data
 * acquisition.
 *
 * \return 0 in case of success, negative value in case 
 */
int StartMagDeviceAcquisition(void);

/*!
 * \brief This function reads data from the Akm09915 and pipe them to the algorithms.
 * Finnally it prints alogrithms output on log uart.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromMagDevice(void);
#endif /* USE_MAG */

#endif /* !_EXAMPLE_ALGO_H_ */
