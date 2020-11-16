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

#include "module_cubeos.h"
#include <MPU6050_lite.h>

#define LOGNAME "cubeOS"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)   ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

/**
 * Attach the debugger.
 */
/*
static duk_ret_t js_cubeos_debug(duk_context *ctx) {
	ESP_LOGD(TAG, ">> js_cubeos_debug");
	duk_trans_socket_init();
	duk_trans_socket_waitconn();
	ESP_LOGD(TAG, "Debugger reconnected, call duk_debugger_attach()");

	duk_debugger_attach(ctx,
		duk_trans_socket_read_cb,
		duk_trans_socket_write_cb,
		duk_trans_socket_peek_cb,
		duk_trans_socket_read_flush_cb,
		duk_trans_socket_write_flush_cb,
		NULL,
		NULL,
		NULL);
	ESP_LOGD(TAG, "<< js_cubeos_debug");
	return 0;
} // js_esp32_debug
*/

/*
duk_double_t js_cubeos_get_now() {
   struct timeval tv;
   gettimeofday(&tv, NULL);
   duk_double_t ret = floor(tv.tv_sec * 1000 + tv.tv_usec/1000);
   return ret;
} 
*/

// Ask JS to perform a gabrage collection.
static duk_ret_t js_cubeos_gc(duk_context *ctx) {
	duk_gc(ctx, 0);
	return 0;
} // js_cubeos_gc


// Return the global object.
static duk_ret_t js_cubeos_global(duk_context *ctx) {
	duk_push_global_object(ctx);
	return 1;
} // js_cubeos_global


/*
 * Log the heap size with a tag.
 */
static duk_ret_t js_cubeos_getFreeMem(duk_context *ctx) {
	duk_push_uint(ctx, (duk_uint_t) dukf_get_free_heap_size());
	return 1;
} // js_cubeos_logHeap

/*
 * Set the named file as the start file.  In our logic, after initialization, we
 * check the value of the NVS esp32duktape->start for existence and for a string
 * file name.  If this key has a value then we try and run that script.
 *
 * [0] - string - fileName to try and run at startup.
 */
/*
static duk_ret_t js_cubeos_setStartFile(duk_context *ctx) {
	const char *fileName = duk_get_string(ctx, -1);
	nvs_handle handle;
	nvs_open("esp32duktape", NVS_READWRITE, &handle);
	nvs_set_str(handle, "start", fileName);
	nvs_commit(handle);
	nvs_close(handle);
	return 0;
} // js_cubeos_setStartFile
*/

/*
 * Sleep for the specified number of milliseconds.
 * [0] - int milliseconds
 */
static duk_ret_t js_cubeos_sleep(duk_context *ctx) {
	uint32_t ms = duk_get_uint(ctx, 0);
	osSleep(ms);
	return 0;
} // js_cubeos_sleep

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

static duk_function_list_entry cubeos_funcs[] = {
	{"gc",          js_cubeos_gc,         1},
	{"global",      js_cubeos_global,     0},
	{"getFreeMem",  js_cubeos_getFreeMem, 0},
	{"delay",       js_cubeos_sleep,      1},
	{"writeFile",	cubeos_write_file,	2},
	{"readFile",	cubeos_read_file,	2},
	{"getAccelGyro", js_cubeos_readAccelGyro, 0},
	{ NULL, NULL, 0 }
};

void cubeos_module_register(duk_context *ctx) {
	/* Set global 'cubeOS'. */
	duk_push_global_object(ctx);
	duk_push_object(ctx);
	duk_put_function_list(ctx, -1, cubeos_funcs);
	duk_put_prop_string(ctx, -2, "cubeOS");
	duk_pop(ctx);
}