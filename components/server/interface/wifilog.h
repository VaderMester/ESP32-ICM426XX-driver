#ifndef __WIFILOG_H__
#define __WIFILOG_H__

#include <stdio.h>

extern void wifilog_va(const char *TAG, char *format, ...);
extern void wifilog(const char *TAG, char *logmsg);
extern void wifilog_from_cb(const char *TAG, char *logmsg);

#endif /* __WIFILOG_H__ */
