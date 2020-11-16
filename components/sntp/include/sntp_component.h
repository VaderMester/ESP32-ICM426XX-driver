#ifndef __SNTP_COMPONENT_H__
#define __SNTP_COMPONENT_H__

#include <time.h>
#include <sys/time.h>

extern void sSTNPtask(void *pvParam);
extern void sntp_sync_time(struct timeval *tv);
extern void time_sync_notification_cb(struct timeval *tv);
extern esp_err_t read_stored_time(time_t *time);
extern esp_err_t write_stored_time(time_t *time);
extern void set_time_env_var(char *environment);
extern int get_time_env_var(char *environment);

#endif /* __SNTP_COMPONENT_H__ */