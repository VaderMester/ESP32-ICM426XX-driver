#include "esp_log.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sdkconfig.h"
#include "duk_module_duktape.h"

#include "duktape.h"
#include "jsfileio.h"
#include <sys/time.h>

#include "duktape_utils.h"
#include "dukf_utils.h"

#include "module_motion.h"
#include <MPU6050_lite.h>

#define LOGNAME "cubeOS"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)   ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

int16_t accelgyrosamples[50][6];

static duk_ret_t js_cubeos_readAccelGyro(duk_context *ctx) {
	int16_t ax, ay, az, gx, gy, gz;
	getAccelGyro(&ax, &ay, &az, &gx, &gy, &gz);
	//WARNING("ax: %i, ax: %i, ay: %i, gx: %i, gy: %i, gz: %i", ax, ay, az, gx, gy, gz);
	duk_idx_t arr_idx;
	arr_idx = duk_push_array(ctx);
	duk_push_int(ctx, ax);
	duk_put_prop_index(ctx, arr_idx, 0);
	duk_push_int(ctx, ay);
	duk_put_prop_index(ctx, arr_idx, 1);
	duk_push_int(ctx, az);
	duk_put_prop_index(ctx, arr_idx, 2);

	duk_push_int(ctx, gx);
	duk_put_prop_index(ctx, arr_idx, 3);
	duk_push_int(ctx, gy);
	duk_put_prop_index(ctx, arr_idx, 4);
	duk_push_int(ctx, gz);
	duk_put_prop_index(ctx, arr_idx, 5);
	//duk_pop(ctx);
	return 1;
} // js_cubeos_sleep

/*
 * Get Gyro Direction checked against a treshold
 * [0] - Preferred Axis (0 - any, 1: X, 2: Y, 3: Z)
 * [1] - Treshold+
 * [2] - Treshold- (Number is positive)
 * If any of the Thresholds are negative, they will be ignored
 * 
 * If 0 is given for Axis, the program will go from X->Z, and return the first it detects it's over treshold
 * RETURNS:
 * 		0		Nothing is detected
 * 		+/- 1	X (sign shows direction)
 * 		+/- 2	Y (sign shows direction)
 * 		+/- 3	Z (sign shows direction)
 */
static duk_ret_t js_cubeos_getGyroDir(duk_context *ctx) {
	int16_t axis  = duk_get_int(ctx, 0);
	int16_t thresholdP  = duk_get_int(ctx, 1);
	int16_t thresholdN  = duk_get_int(ctx, 2);
	int16_t accel[3] = {0, 0, 0};
	int16_t gyro[3] = {0, 0, 0};
	int16_t retval = 0;
	getAccelGyro(&accel[0], &accel[1], &accel[2], &gyro[0], &gyro[1], &gyro[2]);

	//WARNING("ax: %i, ax: %i, ay: %i, gx: %i, gy: %i, gz: %i", accel[0], accel[1], accel[2], gyro[0], gyro[1], gyro[2]);
	if(axis > 0) {
		if((gyro[axis-1]> thresholdP) && (thresholdP >=0 )) {
			retval = axis;
		}
		if((gyro[axis-1]< -thresholdN) && (thresholdN >=0 )){
			retval = -axis;
		}
	} else {
		for (int i = 0; i < 3; i++)
		{
			if (retval == 0)
			{
				if ((gyro[i] > thresholdP) && (thresholdP >=0 ))
				{
					retval = i + 1;
				}
				if ((gyro[i] < -thresholdN) && (thresholdN >= 0))
				{
					retval = -(i + 1);
				}
			}
		}
	}
	duk_push_int(ctx, retval);
	//duk_pop(ctx);
	return 1;
} // js_cubeos_sleep


/************* SAMPLING API **************/


//static duk_ret_t js_cubeos_sampleStart(duk_context *ctx) {



/********** END OF SAMPLING API **********/

static duk_function_list_entry motion_funcs[] = {
	{"getAccelGyro", js_cubeos_readAccelGyro, 0},
	{"getGyroDir", js_cubeos_getGyroDir, 3},
	{ NULL, NULL, 0 }
};

void motion_module_register(duk_context *ctx) {
	/* Set global 'cubeOS'. */
	duk_push_global_object(ctx);
	duk_push_object(ctx);
	duk_put_function_list(ctx, -1, motion_funcs);
	duk_put_prop_string(ctx, -2, "Motion");
	duk_pop(ctx);
}