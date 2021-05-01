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
#ifndef _EXAMPLE_HIGH_RATE_LOGGER_H_
#define _EXAMPLE_HIGH_RATE_LOGGER_H_

#include <stdint.h>
#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"


/* 
 * Set this to 0 if you want to test timestamping mechanism without CLKIN 32k capability.
 * Please set a hardware bridge between PA17 (from MCU) and CLKIN pins (to ICM).
 * Warning: This option is not available for all ICM426XX. Please check the datasheet.
 */
#define USE_CLK_IN   0

typedef enum {
	HIGH_RATE_32KHZ = 0, /* Warning: at 32 KHz, only one axis at a time can be logged */
	HIGH_RATE_16KHZ = 1,
	HIGH_RATE_8KHZ  = 2,
} SENSOR_ODR_t;
/* 
 * Select sensor ODR (32, 16 or 8 KHz)
 * Please refer to the datasheet of your device to know the maximum ODR supported
 */
#define SENSOR_ODR   HIGH_RATE_32KHZ


typedef enum {
	ACCEL_X_AXIS = 0, 
	ACCEL_Y_AXIS = 1,
	ACCEL_Z_AXIS = 2,
	GYRO_X_AXIS  = 3,
	GYRO_Y_AXIS  = 4,
	GYRO_Z_AXIS  = 5,
} AXIS_TO_LOG_t;
/*
 * /!\ Only apply if HIGH_RATE_32KHZ is selected as SENSOR_ODR
 * Select which axis to log when ODR is 32 KHz
 */
#define AXIS_TO_LOG  ACCEL_X_AXIS



/* 
 * Uart frame Indexes
 */
#define HEADER_LSB_IDX 0
#define HEADER_MSB_IDX 1
#define FRAME_CNT_IDX  2
#define RACC_X_IDX     3
#define RACC_Y_IDX     5
#define RACC_Z_IDX     7
#define RGYR_X_IDX     9
#define RGYR_Y_IDX     11
#define RGYR_Z_IDX     13

/* 
 * Uart frame Size in byte
 */
#define HEADER_SIZE         2
#define FRAME_CNT_SIZE      1
#define SENSOR_1AXIS_SIZE   2
#define SENSOR_6AXIS_SIZE   12

#define FRAME_SIZE_MAX HEADER_SIZE + FRAME_CNT_SIZE + SENSOR_6AXIS_SIZE


/* 
 * Uart frame headers
 */
#define HEADER_LSB 0x55
#define HEADER_MSB 0xA0



/**
 * \brief This function is in charge of reseting and initializing Icm426xx device. It should
 * be succesfully executed before any access to Icm426xx device.
 *
 * \return 0 on success, negative value on error.
 */
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif);

/**
 * \brief This function configures the device in order to output gyro and accelerometer.
 * \return 0 on success, negative value on error.
 */
int ConfigureInvDevice();

#endif /* !_EXAMPLE_HIGH_RATE_LOGGER_H_ */
