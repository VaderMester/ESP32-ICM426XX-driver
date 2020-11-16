/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <esp_system.h>
#include <esp_event.h>
#include <esp_log.h>
#include <esp_attr.h>
#include <esp_sleep.h>
#include <nvs_flash.h>
#include <esp_sntp.h>
#include <nvs.h>
#include <sntp_component.h>

static const char *TAG = "sntp";
static const char *nvstimetag = "stored_time";
static nvs_handle_t timehandle;
static char env[50];

/* Variable holding number of times ESP32 restarted since first boot.
 * It is placed into RTC memory using RTC_DATA_ATTR and
 * maintains its value when ESP32 wakes from deep sleep.
 */
//RTC_DATA_ATTR static int boot_count = 0;  //not used but nice to know :)

static void obtain_time(void);
static void initialize_sntp(void);
void sntp_sync_time(struct timeval *tv);


void sntp_sync_time(struct timeval *tv)
{
   settimeofday(tv, NULL);
   ESP_LOGI(TAG, "Time is synchronized from custom code");
   sntp_set_sync_status(SNTP_SYNC_STATUS_COMPLETED);
   time_t storedtime = tv->tv_sec;
   write_stored_time(&storedtime);
}

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

void sSTNPtask(void *pvParam)
{
    initialize_sntp();
    time_t now;
    struct tm timeinfo;
    time(&now);
    localtime_r(&now, &timeinfo);
    // Is time set? If not, tm_year will be (1970 - 1900).
    if (timeinfo.tm_year < (2016 - 1900)) {
        ESP_LOGI(TAG, "Time is not set yet. Connecting to WiFi and getting time over NTP.");
        obtain_time();
        // update 'now' variable with current time
        time(&now);
    }

    char strftime_buf[64];

    // Set timezone to Eastern Standard Time and print local time
    setenv("TZ", env, 1);
    tzset();
    localtime_r(&now, &timeinfo);
    strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
    while(1) {
        vTaskDelay((3600*1000)/portTICK_PERIOD_MS);
        obtain_time();
    }
}

static void obtain_time(void)
{
    // wait for time to be set
    int retry = 0;
    time_t now = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    write_stored_time(&now);
}

//NVS must have been inited before this
esp_err_t read_stored_time(time_t *timeval)
{
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &timehandle);
    uint32_t *nvstime = NULL;
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error (%s) opening NVS storage for time value!\n", esp_err_to_name(err));
    } else {
        if(nvs_get_u32(timehandle, nvstimetag, nvstime) != ESP_OK) {
            ESP_LOGE(TAG, "Error (%s) reading Time value from Flash!\n", esp_err_to_name(err));
            time(timeval);
            write_stored_time(timeval);
            *timeval = (time_t)*nvstime;
        }
        nvs_close(timehandle);
        return ESP_OK;
    }
    return err;
}

esp_err_t write_stored_time(time_t *timeval)
{
    uint32_t nvstime = (uint32_t)*timeval;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &timehandle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Error (%s) opening NVS storage for time value!\n", esp_err_to_name(err));
    }
    else
    {
        err = nvs_set_u32(timehandle, nvstimetag, nvstime);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "Error (%s) writing Time value to Flash!\n", esp_err_to_name(err));
        }
        nvs_close(timehandle);
    }
    return err;
}

void set_time_env_var(char *environment)
{
    sprintf(env, "%s", environment);
}

int get_time_env_var(char *environment)
{
    return sprintf(environment, "%s", env);
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}
