/**
*
*  @brief: MACROS for augmented logging, and useful stuff
*  @note:  LOCAL_LOG_LEVEL and LOGNAME must be defined before including this header.
*/

#ifndef __LOGMACRO_H__
#define __LOGMACRO_H__

#include <stdint.h>
#include <sdkconfig.h>
#include "esp_log.h"
#include "esp_err.h"

/**
 * This below is normally not needed, for using local logging.
 * However ESP_LOG_LEVEL_LOCAL is called always for checking the log level
 * To avoid it, we replace it with empty functions.
 * */
#if LOCAL_LOG_LEVEL == ESP_LOG_NONE
#define ERROR(fmt, ...) {};
#define WARNING(fmt, ...) {};
#define DEBUG(fmt, ...) {};
#define TRACE(fmt, ...) {};
#define INFO(fmt, ...) {};
#elif LOCAL_LOG_LEVEL == ESP_LOG_ERROR
#define ERROR(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) {};
#define DEBUG(fmt, ...) {};
#define INFO(fmt, ...) {};
#elif LOCAL_LOG_LEVEL == ESP_LOG_WARN
#define ERROR(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) {};
#define INFO(fmt, ...) {};
#elif LOCAL_LOG_LEVEL == ESP_LOG_INFO
#define ERROR(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) {};
#define TRACE(fmt, ...) {};
#define INFO(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, LOGNAME, fmt, ## __VA_ARGS__)
#elif LOCAL_LOG_LEVEL == ESP_LOG_DEBUG
#define ERROR(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...) {};
#define INFO(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, LOGNAME, fmt, ## __VA_ARGS__)
#elif LOCAL_LOG_LEVEL == ESP_LOG_VERBOSE
#define ERROR(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, fmt, ##__VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_WARN, LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...) ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, LOGNAME, fmt, ## __VA_ARGS__)
#endif //#if LOCAL_LOG_LEVEL == ESP_LOG_NONE

/* Helper macros to help better error checking */
#define ERR_RET(val) do {if(val) return val;}while(0)
#define ERR_LOG_FUNC(val) do {if(val) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME,  "Error in %s: %d", __func__, val);}while(0)
#define ERR_LOG_FUNC_RET(val) do {if(val) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, "Error in %s: %d", __func__, val); return val;}while(0)

#define ERR_LOG_MSG_RET(val, fmt, ...) do {if(val) ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, LOGNAME, fmt, ## __VA_ARGS__); return val;}while(0)


#define MS2TICKS(ms) ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms) vTaskDelay(MS2TICKS(ms));


#endif //#ifndef __ICM426XX_H__
