#include <stdio.h>
#include "esp_system.h"         //<-- configuration
#include "assert.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"            //<-- log
#include "esp_spiffs.h"         //<-- filesystem
#include "duktape.h"
//#include "c_eventloop.h"      //TODO to make setTimeout and setInterval work
#include "module_cubeos.h"
#include "module_GFX.h"
#include "dukf_utils.h"
#include <jsfileio.h>
#include "soc/lldesc.h"

#include <mdns.h>
#include <wifilog.h>
#include <duk_console.h>
#include <configwrapper.h>
#include <server.h>
#include "wifi_sta.h"
#include "sntp_component.h"

#include <MPU6050_lite.h>

#define LOGNAME "cubeOS"
#define HOSTNAME "cubeos.local"

#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)   ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)

#define OS_CPU 0
#define JS_CPU 1

#define WIFI_ALLOW_SCAN true

//#define JSPRINT(fmt, ...)    btlog(fmt, ## __VA_ARGS__);                           

//#define JSPRINT(fmt, ...) ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)

//#define JSPRINT(fmt, ...) wifilog_va(LOGNAME, fmt, ## __VA_ARGS__)
//#define JSPRINT(char *logmsg) wifilog(LOGNAME, logmsg);

/*
#define JSPRINT(fmt, ...) { \
    wifilog(LOGNAME, fmt, ## __VA_ARGS__); \
    ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__); \
	}
*/
#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));


//static const char *TAG = "CubeOS";

//Context protection semaphore
xSemaphoreHandle ctxsem = NULL;
StaticSemaphore_t ctxsembuf;

//Module loading buffer pointer.
//Will be used in malloc for load function in Duktape.modSearch function
SemaphoreHandle_t accelGyroInitialized = NULL;

SemaphoreHandle_t xWifiConnected = NULL;
StaticSemaphore_t xWifiConnectedBuf;

char ssid[32] = "UPC7D6AC2F";
char wifipw[64] = "XZ7crkcfura8";

static duk_context *ctx;

static TaskHandle_t xJShandle = NULL;
static TaskHandle_t xSuspend = NULL;
static TaskHandle_t xSNTPtask = NULL;

//int32_t AccelGyro_OK = false;

//static void duktape_fatal_handle(void *udata, const char *msg);
static void sDumpMemInfo(void);
static void sSuspendJS(void *pParam);
static void sServerInit(void *pParam);
//static void sBTinit(void *pParam);
static void sJSTask(void *pParam);
static void sJSinit(void *pParam);
static void sStartmDNS(void *pParam);
static void sInitGFXTask (void *pParam);
static void sWifiConnectTask(void *pParam);
static void js_restart(void);

duk_alloc_function jsmalloc(void *udata, duk_size_t size)
{
    return heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
}

duk_realloc_function jsrealloc(void *udata, void *ptr, duk_size_t size)
{
    return heap_caps_realloc(ptr, size, MALLOC_CAP_SPIRAM);
}

duk_free_function jsfree(void *udata, void *ptr)
{
    heap_caps_free(ptr);
    return;
}

static void sJSRestartTask(void *pParam)
{
    js_restart();
    vTaskDelete(NULL);
}

static void js_restart(void) {
    vTaskDelete(xSuspend);
    vTaskDelete(xJShandle);
    cube_gfx_ClearBuf();
    duk_destroy_heap(ctx);
    wifilog("JS ENV>", "****STOPPING JS EXECUTION*****");
    osSleep(3000);
    wifilog("JS ENV>", "****RESTARTING*****");
    if(ctx != NULL){
        ESP_LOGE(LOGNAME, "JS Context still exists, DELETING!");
        ctx = NULL;
    }
    xTaskCreatePinnedToCore(sJSinit, "JS init", 8000, NULL, 6, NULL, JS_CPU);
}

static void js_fatal_handler(void *udata, const char *msg) {
    (void) udata;  /* ignored in this case, silence warning */

    /* Note that 'msg' may be NULL. */
    char *errmsg = malloc(snprintf(NULL, NULL, "*** FATAL ERROR: %s\n", (msg ? msg : "no message")));
    sprintf(errmsg, "*** FATAL ERROR: %s\n", (msg ? msg : "no message"));
    wifilog("JS ENV>", errmsg);
    fflush(stderr);
    free(errmsg);
    js_restart();
}


static void sWifiInit(void *pParam)
{
    INFO("Wifi?");
    if(wifi_init_sta() != 0) 
    {
        //TODO
        xSemaphoreTake(accelGyroInitialized, portMAX_DELAY);
        wifi_set_sta_creds(ssid, wifipw);
        xSemaphoreGive(xWifiConnected);
        xTaskCreatePinnedToCore(sWifiConnectTask, "SNTP task", 6000, NULL, 8, &xSNTPtask, OS_CPU);
    } else {
        esp_restart();
    }
    INFO("*******  Wifi Initialized!  *******");
    xSemaphoreGive(accelGyroInitialized);
        sDumpMemInfo();
    vTaskDelete(NULL);
}

//This should be called time to-time
static void sWifiConnectTask(void *pParam)
{
    xSemaphoreTake(xWifiConnected, portMAX_DELAY);
    bool connectstatus = false;
    while((connectstatus = wifi_connect(false)) == 0) {
        ESP_LOGW(LOGNAME, "WiFi: %s", connectstatus ? "Connected" : "Not connected");
        osSleep(2000);
    }
    //ESP_LOGW(LOGNAME, "WiFi: %s", connectstatus ? "Connected" : "Not connected");
    if(connectstatus) {
        xTaskCreatePinnedToCore(sSTNPtask, "SNTP task", 5000, NULL, 8, &xSNTPtask, OS_CPU);
    }
    xSemaphoreGive(xWifiConnected);
    vTaskDelete(NULL);
}

static void sServerInit(void *pParam)
{
    if(server_init() == ESP_OK) {
        INFO("*******  Server Up!  *******");
        sDumpMemInfo();
    }
    vTaskDelete(NULL);
}

static void sStartmDNS(void *pParam)
{
    //initialize mDNS service
    xSemaphoreTake(xWifiConnected, portMAX_DELAY);
    xSemaphoreGive(xWifiConnected);
    esp_err_t err = mdns_init();
    if (err) {
        WARNING("MDNS Init failed: %d\n", err);
        vTaskDelete(NULL);
    }
    //set hostname
    mdns_hostname_set(HOSTNAME);
    //set default instance
    mdns_instance_name_set("cubeOS Terminal Server");
    WARNING("mDNS is 'hopefully' up :) ");
    vTaskDelete(NULL);
}

static void sInitGFXTask (void *pParam)
{
    gfx_module_init();
    //sDumpMemInfo();
    osSleep(2000);
    sDumpMemInfo();
    //cube_gfx_ClearAll();
    vTaskDelete(NULL);
}

static void sJSinit(void *pParam)
{
    xSemaphoreTake(accelGyroInitialized, portMAX_DELAY);
    xSemaphoreGive(accelGyroInitialized);
    //ctx = duk_create_heap_default();
    ctx = duk_create_heap(jsmalloc, jsrealloc, jsfree, NULL, js_fatal_handler);
	//xSemaphoreGive(xContextSem);
    if(ctx)
    {
        INFO("JS Context created");
        dukf_print_register(ctx);
        //duk_console_init(ctx, DUK_CONSOLE_PROXY_WRAPPER);
        //Init SPIFFS and registering module
        dukf_fs_init();
        dukf_fs_info();
        jsfileio_init(ctx);
        dukf_modSearch_register(ctx);
        cubeos_module_register(ctx);
        INFO("Starting JS task");
        xTaskCreatePinnedToCore(sJSTask, "JS main", 12000, NULL, 5, &xJShandle, JS_CPU);
        sDumpMemInfo();
    }
    vTaskDelete(NULL);
}


static void sJSTask(void *pParam)
{
    //osSleep(2000);
    INFO("JS context %s", ctx ? "Succeffully created" : "Failed to create");
    if (ctx)
    {
        //dukf_init_ctx(ctx, ctxsem, ctxsembuf);
        //jsfileio must be inited before modsearch is registered.
        gfx_module_register(ctx);
        //sDumpMemInfo();
        xTaskCreatePinnedToCore(sSuspendJS, "JS suspend task", 2000, NULL, 5, &xSuspend, OS_CPU);
        const char *startfile = getStartJSfile(); //load config
        if(startfile)
        {
            //JSPRINT("Running file: %s", startfile);
            //wifilog_va(LOGNAME, "Running file: %s", startfile);
            dukf_runFile(ctx, startfile);
        } else {
            wifilog("JS ENV> ", "Failed to load startfile from config");
            //wifilog("JS ENV> ", "Running default file: fibonacci.js");
            //dukf_runFile(ctx, "fibonacci.js");
            //wifilog("JS ENV> ", "Running default file: snake.js");
            //dukf_runFile(ctx, "snake.js");
            wifilog("JS ENV> ", "Running default file: accelgyro_test.js");
            dukf_runFile(ctx, "accelgyro_test.js");
            xTaskCreatePinnedToCore(sJSRestartTask, "Restart JS", 2000, NULL, 1, NULL, OS_CPU);
            osSleep(3000);
            sDumpMemInfo();
            vTaskDelete(NULL);
        }
        //vTaskDelete(xSuspend);
        //wifilog("cubeOS>", "ERROR - Stopping JS environment");
    }
    INFO("Viu viu!!!");
    vTaskDelete(NULL);
}

static void sSuspendJS(void *pParam) {
    while(1) {
        //uint32_t prevTick = xTaskGetTickCount();
        vTaskSuspend(xJShandle);
        ESP_LOGW(LOGNAME, "JS TASK suspended!");
        //vTaskDelayUntil(&prevTick, MS2TICKS(10));
        osSleep(20);
        //prevTick = xTaskGetTickCount();
        vTaskResume(xJShandle);
        //vTaskDelayUntil(&prevTick, MS2TICKS());
        osSleep(980);
        continue;
    }
    vTaskDelete(NULL);
}

static void sAccelGyroInitTask(void *pvParam)
{
    MPU6050_initialize();
    MPU6050_calibrate();
    //AccelGyro_SemaphoreCreate();
    ESP_LOGW("ACCELGYRO--", "**** AccelGyro - MPU6050 is up ****");
    //xTaskCreatePinnedToCore(sAccelGyroReadTask, "accelgyro task", 2000, NULL, 4, NULL, OS_CPU);
    xSemaphoreGive(accelGyroInitialized);
    ESP_LOGW("ACCELGYRO--", "**** AccelGyro Init Complete! *****");
    vTaskDelete(NULL);
}

static void sDumpMemInfo(void)
{
    WARNING("Heap Integrity: %s", heap_caps_check_integrity_all(true) ? "OK" : "CURRUPT");
    WARNING("heap: EXEC free=%u (min=%u, largest=%u), 32BIT free=%u (min=%u, largest=%u), 8BIT free=%u (min=%u, largest=%u), DMA free=%u (min=%u, largest=%u)",
        heap_caps_get_free_size(MALLOC_CAP_EXEC), heap_caps_get_minimum_free_size(MALLOC_CAP_EXEC), heap_caps_get_largest_free_block(MALLOC_CAP_EXEC),
        heap_caps_get_free_size(MALLOC_CAP_32BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_32BIT), heap_caps_get_largest_free_block(MALLOC_CAP_32BIT),
        heap_caps_get_free_size(MALLOC_CAP_8BIT), heap_caps_get_minimum_free_size(MALLOC_CAP_8BIT), heap_caps_get_largest_free_block(MALLOC_CAP_8BIT),
        heap_caps_get_free_size(MALLOC_CAP_DMA), heap_caps_get_minimum_free_size(MALLOC_CAP_DMA), heap_caps_get_largest_free_block(MALLOC_CAP_DMA));
}


/*
static void duktape_fs_test(void * parameter) {
    if(duktape_init_ctx() != 0) {
    showLogo();
    duk_idx_t top = duk_get_top(ctx);
    moduledukf(ctx);
    assert(top == duk_get_top(ctx));
        //print_register(ctx);
        //eventloop_register(ctx);
        //fileio_register(ctx);
    
        //poll_register(ctx);
        //fileio_push_file_string(ctx, "/spiffs/modules/init.js");
        //duktape_run_file("/spiffs/modules/init.js");
        dukf_runFile(ctx, "modules/init.js");
        dukf_runFile(ctx, "fibonacci.js");
	    //duk_eval(ctx);
        //duktape_run_file("/spiffs/timer-test.js");
        for(int i = 0; i < 10; i++) {
            ESP_LOGI(TAG, "Repeats: %d", i);
            //duktape_run_file("/spiffs/fibonacci.js");
            dukf_runFile(ctx, "fibonacci.js");
            vTaskDelay(10/portTICK_PERIOD_MS);
        }
    }
    ESP_LOGE(TAG, "Failed to Init JS Context");
    size_t totalFreeHeap = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    size_t freeDMAHeap = heap_caps_get_free_size(MALLOC_CAP_DMA);
    size_t largestfreedmablk = heap_caps_get_largest_free_block(MALLOC_CAP_DMA);
    ESP_LOGI(TAG, "Total Free internal HEAP: %d kBytes", totalFreeHeap/1024);
    ESP_LOGI(TAG, "Free DMA HEAP: %d kBytes", freeDMAHeap/1024);
    ESP_LOGI(TAG, "Largest Free DMA block: %d kBytes", largestfreedmablk/1024);
    //duktape_destruct_ctx();

    vTaskDelete(xJShandle);
}
*/

void app_main(void) {
    //sDumpMemInfo();
    accelGyroInitialized = xSemaphoreCreateBinary();
    xWifiConnected = xSemaphoreCreateBinaryStatic(&xWifiConnectedBuf);
    xTaskCreatePinnedToCore(sAccelGyroInitTask, "AccelGyro Init", 5000, NULL, 5, NULL, JS_CPU);
    xTaskCreatePinnedToCore(sWifiInit, "Wifi init", 12000, NULL, 5, NULL, OS_CPU);
    set_time_env_var("CET-1CEST-2,M3.4.0/01:00:00,M10.4.0/02:00:00");   //Settimezone Variable
    xSemaphoreTake(accelGyroInitialized, portMAX_DELAY);
    xTaskCreatePinnedToCore(sStartmDNS, "mDNS init", 8000, NULL, 5, NULL, OS_CPU);
    xTaskCreatePinnedToCore(sServerInit, "Terminal Server init", 12000, NULL, 5, NULL, OS_CPU);
    xTaskCreatePinnedToCore(sInitGFXTask, "GFX Init", 8000, NULL, 5, NULL, JS_CPU);
    osSleep(1000);
    xSemaphoreGive(accelGyroInitialized);       //sJSinit task is goint to take this
    xTaskCreatePinnedToCore(sJSinit, "JS init", 8000, NULL, 6, NULL, JS_CPU);
    sDumpMemInfo();
}
