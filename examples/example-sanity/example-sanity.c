/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2018-2019 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively "Software") is subject
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

#include "example-sanity.h"

/* InvenSense utils */
#include "Invn/EmbUtils/Message.h"


#define ACC_FSR                 32       // gee
#define GYR_FSR                 4000     // dps
#define MAG_SENSITIVITY_Q16     9830     // 0.15 uT/LSB
#define ACC_ODR_US              10000    // us
#define GYR_ODR_US              10000    // us
#define MAG_ODR_US              10000    // us
#define RTEMP_OFFSET            1638400  // (25 deg << 16)
#define RTEMP_SENSITIVITY       8104935  // (100 << 30)/13248); // high-res

#define TIME_INDEX              0
#define RACC_INDEX              (TIME_INDEX + 1)
#define RGYR_INDEX              (RACC_INDEX + 3)
#define RTEMP_INDEX             (RGYR_INDEX + 3)
#define RMAG_INDEX              (RTEMP_INDEX + 1)

#define RV_QUAT_INDEX           (RMAG_INDEX + 3)
#define ACC_BIAS_INDEX          (RV_QUAT_INDEX + 4)
#define GYR_BIAS_INDEX          (ACC_BIAS_INDEX + 3)
#define MAG_BIAS_INDEX          (GYR_BIAS_INDEX + 3)

#define TOTAL_NBR_COLUMN        (MAG_BIAS_INDEX + 3)



/* --------------------------------------------------------------------------------------
 *  Static and extern variables
 * -------------------------------------------------------------------------------------- */

/* Icm426xx driver object */
static struct inv_icm426xx icm_driver;


int check_output(int32_t expected_output, int32_t output, const char * str, int line);

int SetupInvDevice(struct inv_icm426xx_serif * icm_serif)
{
	int rc = 0;
	uint8_t who_am_i;

	/* Init device */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Initialize Icm426xx");

	rc = inv_icm426xx_init(&icm_driver, icm_serif, NULL);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to initialize Icm426xx.");
		return rc;
	}
	
	/* Check WHOAMI */
	INV_MSG(INV_MSG_LEVEL_VERBOSE, "Check Icm426xx whoami value");

	rc = inv_icm426xx_get_who_am_i(&icm_driver, &who_am_i);
	if (rc != INV_ERROR_SUCCESS) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : failed to read Icm426xx whoami value.");
		return rc;
	}
	
	if (who_am_i != ICM_WHOAMI) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "!!! ERROR : bad WHOAMI value. Got 0x%02x (expected: 0x%02x)", who_am_i, ICM_WHOAMI);
		return INV_ERROR;
	}

	return rc;
}

int InitInvAgmFusionAlgo()
{
	int rc = 0;
	InvnAlgoAGMConfig config;

	memset(&config, 0, sizeof(config));

	config.acc_odr_us = ACC_ODR_US;
	config.gyr_odr_us = GYR_ODR_US;
	config.mag_odr_us = MAG_ODR_US;
	config.acc_fsr = ACC_FSR;
	config.gyr_fsr = GYR_FSR;
	config.mag_sc_q16 = MAG_SENSITIVITY_Q16;
	/* Temperature configuration */
	config.temp_offset = 25 << 16;
	config.temp_sensitivity = (int32_t)((int64_t)((int64_t)100 << 30)/13248); // high-res

	/* Initialize agm fusion algorithms */
	rc |= invn_algo_agm_init(&config);

	return rc;
}

int RunTestVector(const int32_t testVector[][NUM_COLUMN], uint32_t numRows, uint32_t numCol, const char * testName)
{
	uint32_t i = 0;
	uint32_t j = 0;
	int rc = 0;

	InvnAlgoAGMInput  input;
	InvnAlgoAGMOutput output;
	InvnAlgoAGMOutput expected_output;

	memset(&input          , 0, sizeof(input));
	memset(&output         , 0, sizeof(output));
	memset(&expected_output, 0, sizeof(expected_output));

	INV_MSG(INV_MSG_LEVEL_INFO, "");
	INV_MSG(INV_MSG_LEVEL_INFO, "   > Test case: %s", testName);

	// Ensure algo is intialized
	rc |= InitInvAgmFusionAlgo();
	if (rc) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "   > Error while initializing AgmFusion algorithm");
		return rc;
	}


	for (i = 0; i < numRows; i++) {
		input.mask = 7;
		input.sRimu_time_us = testVector[i][TIME_INDEX];
		input.sRtemp_data = testVector[i][RTEMP_INDEX];
		for (j = 0; j < 3; j++) {
			/* Fill input data for acc, gyr and mag*/
			input.sRacc_data[j] = testVector[i][j+RACC_INDEX];
			input.sRgyr_data[j] = testVector[i][j+RGYR_INDEX];
			input.sRmag_data[j] = testVector[i][j+RMAG_INDEX];

			/* Fill expected output for acc bias, gyr bias and mag bias */
			expected_output.acc_bias_q16[j] = testVector[i][j+ACC_BIAS_INDEX];
			expected_output.gyr_bias_q16[j] = testVector[i][j+GYR_BIAS_INDEX];
			expected_output.mag_bias_q16[j] = testVector[i][j+MAG_BIAS_INDEX];
		} 

		/* Fill expected output for rv quaternion */
		for (j = 0; j < 4; j++) {
			expected_output.rv_quat_q30[j] = testVector[i][j+RV_QUAT_INDEX];
		}

		/* run algo */
		invn_algo_agm_process(&input, &output);

		/* Check output */
		for (j = 0; j < 4; j++) {
			rc |= check_output(expected_output.rv_quat_q30[j], output.rv_quat_q30[j], "rv_quat_q30", i);	
		}

		for (j = 0; j < 3; j++) {
			rc |= check_output(expected_output.acc_bias_q16[j], output.acc_bias_q16[j], "acc_bias_q16", i);
			rc |= check_output(expected_output.gyr_bias_q16[j], output.gyr_bias_q16[j], "gyr_bias_q16", i);
			rc |= check_output(expected_output.mag_bias_q16[j], output.mag_bias_q16[j], "mag_bias_q16", i);

		}

		/* Break loop if an error is detected */
		if (rc != 0)
			break;
	}

	if (rc == 0)
		INV_MSG(INV_MSG_LEVEL_INFO, "   > SUCCESS");
	else
		INV_MSG(INV_MSG_LEVEL_ERROR, "   > FAILURE");
	
	return rc;
}

int check_output(int32_t expected_output, int32_t output, const char * str, int line)
{
	if (expected_output != output) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "   > /!\\ Error on `%s` at line %d: got %d instead of %d", 
			str, line, output, expected_output);

		// Sleep 100 ms to ensure trace is properly printed
		inv_icm426xx_sleep_us(100000); 

		return -1;
	}

	return 0;
}

