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

#include "example-selftest.h"

/* InvenSense drivers and utils */
#include "Invn/EmbUtils/Message.h"

/* board driver */
#include "common.h"

/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Just a handy variable to handle the icm426xx object */
static struct inv_icm426xx icm_driver;


/* --------------------------------------------------------------------------------------
 *  Functions definition
 * -------------------------------------------------------------------------------------- */
 
int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Initialize device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initialize Icm426xx");
	
	rc = inv_icm426xx_init(&icm_driver, icm_serif, NULL);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}
	
	
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_INFO, "Check Icm426xx whoami value");
	
	rc = inv_icm426xx_get_who_am_i(&icm_driver, &who_am_i);
	if(rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value.");
		return rc;
	}
	
	if(who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR :  bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	return rc;
}

void RunSelfTest(void)
{
	int rc = 0, st_result = 0;

	rc = inv_icm426xx_run_selftest(&icm_driver, &st_result);

	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while running selftest");
	} else {
		/* Check for GYR success (1 << 0) and ACC success (1 << 1) */
		if (st_result & 0x1)
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest PASS");
		else
			INV_MSG(INV_MSG_LEVEL_INFO, "Gyro Selftest FAIL");
		
		if (st_result & 0x2)
			INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest PASS");
		else
			INV_MSG(INV_MSG_LEVEL_INFO, "Accel Selftest FAIL");
	} 
}

void GetBias(void)
{
	int rc = 0;
	int raw_bias[6];

	/* Get Low Noise / Low Power bias computed by self-tests scaled by 2^16 */
	INV_MSG(INV_MSG_LEVEL_INFO, "Getting ST bias");
	rc |= inv_icm426xx_get_st_bias(&icm_driver, raw_bias);
	if (rc < 0)
		INV_MSG(INV_MSG_LEVEL_ERROR, "An error occured while getting ST bias");
	
	INV_MSG(INV_MSG_LEVEL_INFO, "GYR LN bias (dps): x=%f, y=%f, z=%f",
			(float)(raw_bias[0]) / (float)(1 << 16), (float)(raw_bias[1]) / (float)(1 << 16), (float)(raw_bias[2]) / (float)(1 << 16));
	INV_MSG(INV_MSG_LEVEL_INFO, "ACC LN bias (g): x=%f, y=%f, z=%f",
			(float)(raw_bias[0 + 3] / (float)(1 << 16)), (float)(raw_bias[1 + 3] / (float)(1 << 16)), (float)(raw_bias[2 + 3] / (float)(1 << 16)));
}
