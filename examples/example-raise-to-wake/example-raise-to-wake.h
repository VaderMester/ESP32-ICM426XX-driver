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
#ifndef _EXAMPLE_RAISE_TO_WAKE_H_
#define _EXAMPLE_RAISE_TO_WAKE_H_

#include <stdint.h>
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL_apex.h"


/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be succesfully executed before any access to Icm426xx device.
 * 
 * \param[in] icm_serif : Serial interface object.
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output raise to wake.
 *
 * Then function sets frequency for accel and raise to wake and it 
 * starts them in the requested power mode and with requested configuration.
 *
 * \param[in] r2w_sleep_time_out :  Time out for a sleep detection.
 * \param[in] r2w_mounting_matrix : Mounting matrices on a device board.
 * \param[in] r2w_sleep_gest_delay : Detection window for a sleep gesture detection.
 * \param[in] power_mode : Define raise to wake power save mode.
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice(ICM426XX_APEX_CONFIG4_R2W_SLEEP_TIME_OUT_t r2w_sleep_time_out,
					   ICM426XX_APEX_CONFIG5_R2W_MOUNTING_MATRIX_t r2w_mounting_matrix,
					   ICM426XX_APEX_CONFIG6_R2W_SLEEP_GEST_DELAY_t r2w_sleep_gest_delay,
					   ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_t power_mode);

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


#endif /* !_EXAMPLE_RAISE_TO_WAKE_H_ */
