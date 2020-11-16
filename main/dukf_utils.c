/**
 * Common utilities for the ESP32-Duktape framework.
 */
#include <esp_system.h>
#include <esp_spiffs.h>
#include <esp_log.h>

#include <duktape.h>
#include <errno.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "dukf_utils.h"
#include "duktape_utils.h"
#include <jsfileio.h>

#include <server.h>
#include <wifilog.h>

#define MAX_RUN_AT_START (5)

#define LOGNAME "JS ENV>"
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)   ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)                      

#define JSPRINT(fmt, ...) wifilog_va(LOGNAME, fmt, ## __VA_ARGS__)
//#define JSPRINT(char *logmsg) wifilog(LOGNAME, logmsg)

/*
#define JSPRINT(fmt, ...) { \
    wifilog(LOGNAME, fmt, ## __VA_ARGS__); \
    ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__); \
	}
*/

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));


//----------STATIC DECLARATIONS-----------
static duk_ret_t js_mod_search(duk_context *ctx);
static duk_ret_t dukf_native_print(duk_context *ctx);

static const char *TAG = "JS Utils";
// Number of scripts registered to run at the start.
static int g_runAtStartCount = 0;

// Array of scripts to run at start.
static char *g_runAtStartfileNames[MAX_RUN_AT_START];

/**
 * Log the heap value to the console.
 */


void dukf_log_heap(const char *localTag) {
	if (localTag == NULL) {
		localTag = "<no tag>";
	}
	ESP_LOGD(TAG, "%s: heapSize=%d", localTag, dukf_get_free_heap_size());
} // dukf_log_heap

/**
 * Run the files that were registered to be run at startup.
 */
void dukf_runAtStart(duk_context *ctx) {
	ESP_LOGD(TAG, "Running scripts registered to auto start:");
	int i;
	for (i=0; i<g_runAtStartCount; i++) {
		ESP_LOGD(TAG, " autostart: %s", g_runAtStartfileNames[i]);
		dukf_runFile(ctx, g_runAtStartfileNames[i]);
	}
} // dukf_runAtStart

/*
static duk_ret_t dukf_handle_print(duk_context *ctx) {
    INFO("%s", duk_to_string(ctx, 0));
    return 0;  // no return value (= undefined)
}
*/

duk_ret_t dukf_handle_assert(duk_context *ctx) {
	if (duk_to_boolean(ctx, 0)) {
		return 0;
	}
	(void) duk_generic_error(ctx, "assertion failed: %s", duk_safe_to_string(ctx, 1));
	return 0;
}

//int dukf_init_ctx(duk_context *ctx, xSemaphoreHandle xContextSem, StaticSemaphore_t xContextSemBuf) {
	/*
int dukf_init_ctx(duk_context *ctx) {
    //xContextSem = xSemaphoreCreateBinaryStatic(&xContextSemBuf);
    ctx = duk_create_heap_default();
	//xSemaphoreGive(xContextSem);
	if(ctx) {
		INFO("JS Context created");
	}
	return 1;
}
*/
void dukf_destruct_ctx(duk_context *ctx) {
    duk_destroy_heap(ctx);
}

void dukf_fs_init(void) {
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = false
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
}

void dukf_fs_info(void) {
    size_t total = 0, used = 0;
    esp_err_t ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
}
/*
int dukf_modSearch_register(duk_context *ctx) {
	if (ctx == NULL)
	{
		INFO("ModSearch Register failed: JS Context not initialized!");
		return 0;
	}
	const char *modSearchfunc = "Duktape.modSearch = function(id, require, exports, module){DUKF.gc();var name = id;if(!StringUtils.endsWith(id, \".js\")){ name += \".js\";}module.filename = name;var size = cubeOS.getFileSize('modules/' + name);var fdata = new Buffer(size); cubeOS.readFile(('modules/' + name),fdata); return fdata.toString;};";
	int len = strlen(modSearchfunc);
	ESP_LOGE(TAG, "Adding ModSearch Function for require()");
	if (duk_pcompile_lstring(ctx, 0, modSearchfunc, len) != 0)
	{
		INFO("Modsearch failed to compile");
		esp32_duktape_log_error(ctx);
		return 1;
	}
	int rc = duk_pcall(ctx, 0);
	if (rc != 0)
	{
		ESP_LOGE(TAG, "FAILED to add Modsearch Function!");
		esp32_duktape_log_error(ctx);
		duk_pop(ctx);
		return 1;
	}
	INFO("ModSearch Function registered");
	return 0;
}*/

static duk_ret_t dukf_native_print(duk_context *ctx) {
    //ESP_LOGI(TAG, "%s", duk_to_string(ctx, 0));
	duk_push_string(ctx, " ");
	duk_insert(ctx, 0);
	duk_join(ctx, duk_get_top(ctx) - 1);
	//JSPRINT("%s", duk_safe_to_string(ctx, -1));
	wifilog(LOGNAME, duk_safe_to_string(ctx, -1));
	return 0;
}

void dukf_print_register(duk_context *ctx) {
	duk_push_c_function(ctx, dukf_native_print, DUK_VARARGS);
	duk_put_global_string(ctx, "print");
}

void dukf_modSearch_register(duk_context *ctx) {
    duk_get_global_string(ctx, "Duktape");
    duk_push_c_function(ctx, js_mod_search, 4 /*nargs*/);
    duk_put_prop_string(ctx, -2, "modSearch");
    duk_pop(ctx);
}

static duk_ret_t js_mod_search(duk_context *ctx)
{
	/* Nargs was given as 4 and we get the following stack arguments:
     *   index 0: id
     *   index 1: require
     *   index 2: exports
     *   index 3: module
     */
	char *path = "/spiffs/modules/";

	// Pull Arguments
	const char *fileName = duk_require_string(ctx, 0);
	if (fileName != NULL)
	{
		JSPRINT("ID => %s \n", fileName);
		int fnamelen = (int)strlen(fileName);
		bool jstag = false;		//we check if ID ends with .js
		if ((fileName[fnamelen - 3] == '.') && (fileName[fnamelen - 2] == 'j') && (fileName[fnamelen - 1] == 's'))
		{
			jstag = true;
		}
		if (!jstag)
		{
			strcat(fileName, ".js");
		}
		strcat(path, fileName);
		// Read File and calculate its size (as DUKtape examples)
		FILE *f = fopen(path, "rb");
		if (f == NULL)
		{
			JSPRINT("Failed to load module. File '%s' not found in the 'modules' folder", fileName);
			return -1;
		}
		JSPRINT("Module	%s found, loading... \n", fileName);
		fseek(f, 0L, SEEK_END);
		int fsize = ftell(f);
		rewind(f);
		// Rewind
		fseek(f, 0, SEEK_SET);

		char *buf = heap_caps_malloc(fsize, MALLOC_CAP_SPIRAM);
		fread(buf, 1, fsize, f);
		fclose(f);
		duk_push_lstring(ctx, buf, fsize);
		free(buf);
		return 1;
	}
	// Error
	return -1;
}

/*
static void dukf_fatal_handle(void *udata, const char *msg){
    const char *fatal_msg = (msg); //
    (void) udata;
    ESP_LOGE(TAG, "*** FATAL ERROR: %s\n", fatal_msg ? fatal_msg : "no message");
    if(xJShandle !=NULL){
	vTaskDelete(xJShandle);
    }
    //we try to restart from a Fatal error
    xTaskCreatePinnedToCore(duktape_fs_test, "duktape", 64000, NULL, 5, &xJShandle, 0);
}
*/

/**
 * Load the named file and run it in a JS environment.  We try and load the file first
 * from the SPIFFS file system and if that fails, we try and load it from the SPIFFS
 * file system.
 */
int dukf_runFile(duk_context *ctx, const char *fileName) {
	char *fileFunc = "runFile";
	if(!dukf_loadFile_ctx(ctx, fileName)) {
		ESP_LOGE(TAG, "FAILED to LOAD '%s' with '%s'", fileName, fileFunc);
		return 0;
	}
	INFO("Running file '%s' with '%s'", fileName, fileFunc);
	int rc = duk_pcall(ctx, 0);
	if (rc != 0) {
		ESP_LOGE(TAG, "FAILED to RUN '%s' with '%s'", fileName, fileFunc);
		esp32_duktape_log_error(ctx);
		duk_pop(ctx);
		return 0;
	}
	duk_pop(ctx);
	return 1;
} // dukf_runFile

int dukf_loadFile_ctx(duk_context *ctx, const char *fileName) {
	char *fileFunc = "loadFile_stack";
	INFO("Loading '%s' with '%s'", fileName, fileFunc);
	char *spiffstag = "/spiffs/";
	int pathlen = strlen(fileName) + strlen(spiffstag) + 1;
	char filePath[pathlen];
	strcpy(filePath, spiffstag);
	strcat(filePath, fileName);
	INFO("FilePath: %s", filePath);
	FILE* f = fopen(filePath, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s with %s", fileName, fileFunc);
        return 0;
    }
    fseek(f, 0L, SEEK_END);
    int filesize = ftell(f);
    rewind(f);

    char *buf = heap_caps_malloc(filesize, MALLOC_CAP_SPIRAM);
	if(!buf) {
		ESP_LOGE(TAG, "Failed to allocate buffer for '%s', with '%s'", fileName, fileFunc);
	return 0;
	}

	INFO("Compiling %s with %s", fileName, fileFunc);
    fread(buf, 1, filesize, f);
	fclose(f);
    duk_push_lstring(ctx, fileName, filesize);
    int rc = duk_pcompile_lstring_filename(ctx, 0, buf, filesize);
	free(buf);
	if (rc != 0) {
		esp32_duktape_log_error(ctx);
		return 0;		
    }
	INFO("File '%s' loaded with '%s'", fileName, fileFunc);
	return 1;
}

duk_ret_t dukf_loadFile_buff(duk_context *ctx, const char *fileName) {
	char *fileFunc = "loadFile_buff";
	INFO("Loading '%s' with '%s'", fileName, fileFunc);
	char *spiffstag = "/spiffs/";
	int pathlen = strlen(fileName) + strlen(spiffstag);
	char filePath [pathlen];
	strcpy(filePath, spiffstag);
	strcat(filePath, fileName);
	INFO("FilePath: %s", filePath);
	FILE* f = fopen(filePath, "rb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open %s with %s", fileName, fileFunc);
        return 0;
    }
    fseek(f, 0L, SEEK_END);
    int filesize = ftell(f);
    rewind(f);

    char buf [filesize];
    fread(buf, 1, filesize, f);
	fclose(f);
	return *buf;
} // dukf_loadFile
/**
 * Record the name of a file we wish to run at the start.
 */

void dukf_addRunAtStart(const char *fileName) {
	// Add the fileName to the list of scripts to run at the start after first
	// checking that we haven't tried to remember too many.
	if (g_runAtStartCount < MAX_RUN_AT_START) {
		g_runAtStartfileNames[g_runAtStartCount] = (char *)fileName;
		g_runAtStartCount++;
	} else {
		ESP_LOGE(TAG, "Can't run program %s at start ... too many already", fileName);
	}
} // dukf_addRunAtStart

uint32_t dukf_get_free_heap_size() {
	return  heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
}

//#if defined(ESP_PLATFORM)

/*
 * Initialize the default NVS values that are used by ESP32-Duktape.  These
 * include:
 * * useSerial - u32 - 1 = using serial processor
 *
 */
/*
void dukf_init_nvs_values() {
	nvs_handle handle;
	uint32_t u32Val;

	esp_err_t errRc = nvs_open("esp32duktape", NVS_READWRITE, &handle);
	if (errRc != ESP_OK) {
		ESP_LOGE(TAG, LOG_TAG, "nvs_open: %s", esp32_errToString(errRc));
		return;
	}
	// See if we have a "useSerial" entry.
	if (nvs_get_u32(handle, "useSerial", &u32Val) == ESP_ERR_NVS_NOT_FOUND) {
		nvs_set_u32(handle, "useSerial", 0);
	}
	nvs_commit(handle); // Commit any changes we may have made.
	nvs_close(handle);  // Close the NVS handle.
} // dukf_init_nvs_values
*/



/**
 * Load the named file and return the data and the size of it.
 * We load the file from the DUKF file system based on ESPFS.
 * Since this is flash mapped data, the return data does not need to be released
 * and tests conducted do in fact show that it doesn't reduce the heap size.
 *
 * * path - The path to the file to be opened.
 * * fileSize - The size of the data loaded.
 */
/*
const char *dukf_loadFileFromESPFS(const char *path, size_t *fileSize) {
	ESP_LOGD(TAG, ">> dukf_loadFile: (ESPFS) %s, heapSize=%d", path, dukf_get_free_heap_size());

	assert(fileSize != NULL);
	assert(path != NULL);

	EspFsFile *fh = espFsOpen((char *)path);
	if (fh == NULL) {
		ESP_LOGD(TAG, " Failed to open file %s", path);
		return NULL;
	}

  char *fileData;
  espFsAccess(fh, (void **)&fileData, fileSize);
  espFsClose(fh);
  // Note ... because data is mapped in memory from flash ... it will be good
  // past the file close.
  ESP_LOGD(TAG, "<< duk_loadFile: Read file %s for size %d", path, *fileSize);
  return fileData;
} // dukf_loadFile
*/

//#else // ESP_PLATFORM
/**
 * Load the named file and return the data and the size of it.
 * We load the file from the DUKF file system.
 */
/*
#define DUKF_BASE_DIR "/home/kolban/esp32/esptest/apps/workspace/duktape/filesystem"
const char *dukf_loadFileFromESPFS(const char *path, size_t *fileSize) {

	char fileName[256];
	sprintf(fileName, "%s/%s", DUKF_BASE_DIR, path);
	int fd = open(fileName, O_RDONLY);
	if (fd < 0) {
		ESP_LOGE(TAG, "open: %s - %d %s", path, errno, strerror(errno));
		return NULL;
	}
	struct stat statBuf;
	int rc = fstat(fd, &statBuf);
	if (rc < 0) {
		ESP_LOGE(TAG, "open: %d %s", errno, strerror(errno));
		close(fd);
		return NULL;
	}

	char *data = mmap(NULL, statBuf.st_size, PROT_READ, MAP_SHARED, fd, 0);
	if (data == MAP_FAILED) {
		ESP_LOGE(TAG, "mmap: %d %s", errno, strerror(errno));
		close(fd);
		return NULL;
	}
	close(fd);
  ESP_LOGD(TAG, "duk_loadFile: Read file %s for size %d", path, (int)statBuf.st_size);
  *fileSize = statBuf.st_size;
	return data;
} // dukf_loadFile

#endif // ESP_PLATFORM
*/

/*
 * Load the named file from the Posix file system.
 * * path - the name of the file to load.
 * * fileSize - the size of the file we loaded.
 *
 * Return:
 * A pointer to the data of the file or NULL if we failed to load the file.
 * The data for the file was malloced and it is the responsibility of the
 * caller to release the storage when done.
 */
/*
char *dukf_loadFileFromPosix(const char *path, size_t *fileSize) {
	struct stat statBuf;
	char *data;
	int fd = open(path, O_RDONLY);
	if (fd == -1) {
		ESP_LOGE(TAG, "Failed to open file %s - %s", path, strerror(errno));
		return NULL;
	}
	fstat(fd, &statBuf);
	*fileSize = statBuf.st_size;
	data = malloc(*fileSize);
	read(fd, data, *fileSize);
	close(fd);
	return data;
} // dukf_loadFileFromPosix
*/