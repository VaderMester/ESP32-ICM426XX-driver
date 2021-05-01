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
#ifndef _EXAMPLE_RAW_DATA_REGISTER_H_
#define _EXAMPLE_RAW_DATA_REGISTER_H_

#include <stdint.h>
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"


/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be succesfully executed before any access to Icm426xx device.
 *
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 *
 * It initialyses clock calibration module (this will allow to extend the 16 bits 
 * timestamp produced by Icm426xx to a 64 bits timestamp).
 * Then function sets full scale range and frequency for both accel and gyro and it 
 * starts them in the requested power mode. 
 *
 * \param[in] is_low_noise_mode : if true sensors are started in low-noise mode else in 
 *                                low-power mode.
 * \param[in] acc_fsr_g :   full scale range for accelerometer. See ICM426XX_ACCEL_CONFIG0_FS_SEL_t in Icm426xxDefs.h
 *                         for possible values.
 * \param[in] gyr_fsr_dps : full scale range for gyroscope. See ICM426XX_GYRO_CONFIG0_FS_SEL_t in Icm426xxDefs.h
 *                         for possible values.
 * \param[in] acc_freq :    accelerometer frequency. See ICM426XX_ACCEL_CONFIG0_ODR_t in Icm426xxDefs.h
 *                         for possible values.
 * \param[in] gyr_freq :    gyroscope frequency. See ICM426XX_GYRO_CONFIG0_ODR_t in Icm426xxDefs.h
 *                         for possible values.
 * \param[in] is_rtc_mode :    requested status for RTC/CLKIN feature
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice(uint8_t is_low_noise_mode,
					   ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
					   ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
					   ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
					   ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
					   uint8_t is_rtc_mode);

/**
 * \brief This function extracts data from the Icm426xx FIFO.
 *
 * The function just calls Icm426xx driver function inv_icm426xx_get_data_from_registers.
 * But note that for each data extracted from FIFO, a user defined function is called to 
 * allow custom handling of each data. In this example custom packet handling function
 * is HandleInvDeviceDataRegisters.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromInvDevice(void);

/**
 * \brief This function is the custom handling packet function.
 *
 * It is passed in parameter at driver init time and it is called by 
 * inv_icm426xx_get_data_from_registers function each time new data are read from registers.
 * In this implementation, function process data and print them on UART.
 *
 * \param[in] event structure containing sensor data from one packet
 */
void HandleInvDeviceDataRegisters(inv_icm426xx_sensor_event_t * event);

#endif /* !_EXAMPLE_RAW_DATA_REGISTER_H_ */
