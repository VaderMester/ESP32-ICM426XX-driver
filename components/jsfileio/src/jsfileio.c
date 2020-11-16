#include <stdio.h>
#include <dirent.h> 
#include <errno.h>
#include <esp_system.h>         //<-- configuration
#include "esp_log.h"            //<-- log

#include "jsfileio.h"


static const char *TAG = "JS File IO";

duk_ret_t cubeos_read_file(duk_context *ctx) {
	// FIXME: parameter check shall be implemented
	char filepath[256];
	sprintf(filepath, "/spiffs/%s", duk_to_string(ctx, 0));
	
	FILE* f = fopen(filepath, "r");

	if(NULL != f) {
		duk_size_t size;
		char *buf = duk_require_buffer_data(ctx, 1, &size);
		int read = fread(buf, 1, size, f);
		ESP_LOGI(TAG, "%d bytes were read", read);
		fclose(f);
	}

	return 0;
}

static duk_ret_t cubeos_read_file_string(duk_context *ctx) {
	// FIXME: parameter check shall be implemented
	char filepath[256];
	sprintf(filepath, "/spiffs/%s", duk_to_string(ctx, 0));
	FILE* f = fopen(filepath, "r");
	if(NULL != f) {
		fseek(f, 0L, SEEK_END);
    	duk_size_t size = ftell(f);
    	rewind(f);
		const char *string = duk_require_string(ctx, 1);
		int read = fread(string, 1, size, f);
		ESP_LOGI(TAG, "%d bytes were read", read);
		fclose(f);
	}
	return 0;
}
duk_ret_t cubeos_write_file(duk_context *ctx) {
	// FIXME: parameter check shall be implemented
	char filepath[256];
	sprintf(filepath, "/spiffs/%s", duk_to_string(ctx, 0));

	FILE* f = fopen(filepath, "wb");

	if(NULL != f) {
		ESP_LOGI(TAG, "File opened");
		duk_size_t size;
		void *buf = duk_to_fixed_buffer(ctx, 1, &size);

		if(NULL == buf) {
			ESP_LOGI(TAG, "Failed to get buffer pointer");
		}

        int written = fwrite((const void *)buf, 1, size, f);
		ESP_LOGI(TAG, "%d bytes were written", written);
		fclose(f);
    } else {
		ESP_LOGI(TAG, "File cannot be written");
	}

	return 0;
}

static duk_ret_t cubeos_get_filesize(duk_context *ctx) {
	// FIXME: parameter check shall be implemented
	int fs = 0;
	char filepath[256];
	sprintf(filepath, "/spiffs/%s", duk_to_string(ctx, 0));
	
	FILE* f = fopen(filepath, "r");

	if(NULL != f) {
		fseek(f, 0 , SEEK_END);
		fs = ftell(f);
		fclose(f);
	}

	duk_push_int(ctx, fs);

	return 1;
}

void jsfileio_init(duk_context *ctx) {
    ESP_LOGI(TAG, "Init begin");

    duk_push_c_function(ctx, cubeos_read_file, 2);
    duk_put_global_string(ctx, "cubeOS.readFile");

	duk_push_c_function(ctx, cubeos_read_file_string, 1);
    duk_put_global_string(ctx, "cubeOS.readFileToString");

    duk_push_c_function(ctx, cubeos_write_file, 2);
    duk_put_global_string(ctx, "cubeOS.writeFile");

	duk_push_c_function(ctx, cubeos_get_filesize, 1);
    duk_put_global_string(ctx, "cubeOS.getFileSize");

    ESP_LOGI(TAG, "Init end");
}
