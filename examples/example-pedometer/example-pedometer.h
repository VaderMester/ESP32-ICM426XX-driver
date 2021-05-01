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
#ifndef _EXAMPLE_PEDOMETER_H_
#define _EXAMPLE_PEDOMETER_H_

#include <stdint.h>
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL_apex.h"

/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be succesfully executed before any access to Icm426xx device.
 *
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output accelerometer.
 *
 * Then function sets frequency for accel and pedometer and it 
 * starts them in the requested power mode with the requested configuration.
 *
 * \param[in] pedometer_freq :   pedometer frequency. See ICM426XX_APEX_CONFIG0_DMP_ODR_t in Icm426xxDefs.h 
 *                         for possible values.
 * \param[in] power_mode :       pedometer power save mode. See ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_t in Icm426xxDefs.h 
 *                         for possible values.
 * \param[in] pedo_amp_th : pedometer peak threshold to consider a valid step. See ICM426XX_APEX_CONFIG2_PEDO_AMP_TH_t 
 *                         in Icm426xxDefs.h for possible values.
 * \param[in] pedo_step_cnt_th : number of steps that must be detected before the pedometer step count begins incrementing.
 * \param[in] pedo_sb_timer_th : pedometer duration of non-walk to exit the curent walk mode in number of samples. See 
 *                         ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_t in Icm426xxDefs.h for possible values.
 * \param[in] pedo_step_det_th : number of low latency steps that must be detected before the pedometer step count begins incrementing.
 * \param[in] sensitivity_mode : pedometer sensitivity mode. See ICM426XX_APEX_CONFIG9_SENSITIVITY_MODE_t in Icm426xxDefs.h 
 *                         for possible values.
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice(ICM426XX_APEX_CONFIG0_DMP_ODR_t pedometer_freq,
					   ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_t power_mode,
					   ICM426XX_APEX_CONFIG2_PEDO_AMP_TH_t pedo_amp_th,
					   uint8_t pedo_step_cnt_th,
					   ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_t pedo_sb_timer_th,
					   uint8_t pedo_step_det_th,
					   ICM426XX_APEX_CONFIG9_SENSITIVITY_MODE_t sensitivity_mode);

/**
 * \brief This function extracts data from the Icm426xx FIFO.
 *
 * The function just calls Icm426xx driver function inv_icm426xx_get_data_from_fifo.
 * But note that for each packet extracted from FIFO, a user defined function is called to 
 * allow custom handling of each packet.
 *
 * \return 0 on success, negative value on error.
 */
int GetDataFromInvDevice(void);


#endif /* !_EXAMPLE_PEDOMETER_H_ */
