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
#ifndef _EXAMPLE_EIS_H_
#define _EXAMPLE_EIS_H_

#include <stdint.h>
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"

/* InvenSense algorithm */
#include "invn_algo_agm.h"


/*
 * If set, FSYNC flag and counter are read from FIFO
 * else they are read from ICM registers 
*/
#define FSYNC_USE_FIFO 1

/*
 * Gyroscope fequency
 * Use type ICM426XX_GYRO_CONFIG0_ODR_t to define gyro frequency.
 * This type is defined in Icm426xxDefs.h.
 */
#define GYRO_FREQ   ICM426XX_GYRO_CONFIG0_ODR_200_HZ

/*
 * Gyroscope Full Scale Range
 * Use type ICM426XX_GYRO_CONFIG0_FS_SEL_t defined in Icm426xxDefs.h
 */
#define GYRO_FSR    ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps



/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be successfully executed before any access to Icm426xx device.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 *
 * It initialyses clock calibration module (this will allow to extend the 16 bits 
 * timestamp produced by Icm426xx to a 64 bits timestamp).
 * Then function sets full scale range and frequency for gyro and it 
 * starts it in low-noise mode. (note that low-power mode is not available 
 * for gyroscope in Icm426xx).
 *
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice();

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
int GetDataFromFIFO(void);

/**
 * \brief This function provides custom handling for FIFO data packets.
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


/**
 * \brief This function extracts data from the Icm426xx registers.
 *
 * The function just calls Icm426xx driver function inv_icm426xx_get_data_from_registers.
 * But note that for each packet extracted from FIFO, a user defined function is called to 
 * allow custom handling of each packet. In this example custom packet handling function
 * is HandleInvDeviceFifoPacket.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromInvDevice(void);

/**
 * \brief This function provides custom handling for data registers.
 *
 * It is passed in parameter at driver init time and it is called by 
 * inv_icm426xx_get_data_from_registers function each time new data are read from registers.
 * In this implementation, function process data and print them on UART.
 *
 * \param[in] event structure containing sensor data from one packet
 */
void HandleInvDeviceDataRegisters(inv_icm426xx_sensor_event_t * event);

/*!
 * \brief Initialize algorithm to compute the calibrated gyroscope
 *
 * \param[in] odr_us  icm426xx output gyroscope data rate in microseconds
 */
void ConfigureAlgorithm(uint32_t gyro_odr_us);

#endif /* !_EXAMPLE_EIS_H_ */
