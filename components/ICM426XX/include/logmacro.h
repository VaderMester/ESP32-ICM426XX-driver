/**
*
*  @brief: MACROS for augmented logging, and useful stuff
*  @note:  LOG_LOCAL_LEVEL and LOGNAME must be defined before including this header.
*/

#ifndef __LOGMACRO_H__
#define __LOGMACRO_H__

#include <stdint.h>
#include <sdkconfig.h>
#include "esp_err.h"

//#define LOG_LOCAL_LEVEL CONFIG_ICM426XX_LOCAL_LOG_LEVEL

#include "esp_log.h"

#define LOGNAME "ICM426XX"

/**
 * This below is normally not needed, for using local logging.
 * However ESP_LOG_LEVEL_LOCAL is called always for checking the log level
 * To avoid it, we replace it with empty functions.
 * */

#if CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 0
#define ERROR(fmt, ...) {}
#define WARNING(fmt, ...) {}
#define DEBUG(fmt, ...) {}
#define TRACE(fmt, ...) {}
#define INFO(fmt, ...) {}
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 1
#define ERROR(fmt, ...) ESP_LOGE(LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) {}
#define DEBUG(fmt, ...) {}
#define INFO(fmt, ...) {}
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 2
#define ERROR(fmt, ...) ESP_LOGE(LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) {}
#define INFO(fmt, ...) {}
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 3
#define ERROR(fmt, ...) ESP_LOGE(LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) {}
#define TRACE(fmt, ...) {}
#define INFO(fmt, ...) ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 4
#define ERROR(fmt, ...) ESP_LOGE(LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...) {}
#define INFO(fmt, ...) ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 5
#define ERROR(fmt, ...) ESP_LOGE(LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...) ESP_LOGV( LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...) ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#endif //#if CONFIG_ICM426XX_LOCAL_LOG_LEVEL == ESP_LOG_NONE

/* Helper macros to help better error checking */
#define ERR_RET(val) do {if(val) return val;}while(0)
#define ERR_LOG_FUNC(val) do {if(val) ERROR("Error in %s: %d", __func__, val);}while(0)
#define ERR_LOG_FUNC_RET(val) do {if(val) ERROR("Error in %s: %d", __func__, val); return val;}while(0)

#define ERR_LOG_MSG_RET(val, fmt, ...) do {if(val) ERROR(fmt, ## __VA_ARGS__); return val;}while(0)


#define MS2TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms) vTaskDelay(MS2TICKS(ms));


#endif //#ifndef __ICM426XX_H__
