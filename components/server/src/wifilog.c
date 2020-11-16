#include <string.h>

#include "esp_log.h"                //<-- log

#include "server.h"
#include "websocket_server.h"
#include <stdarg.h>
#include <string.h>

#define MESSAGE_MAX_BYTES 1000

void wifilog_va(const char *TAG, char *format, ...)
{
	va_list args;
	va_start(args, format);
	size_t fmtlen = vsnprintf(NULL, 0, format, args)+1;
	//printf(format, args);
	//char *msg;
	size_t taglen = strlen(TAG);
	size_t bufsize = taglen+fmtlen;
	char *msg = malloc(bufsize);
	if (msg)
	{
		snprintf(msg, taglen, "%s", TAG);
		sprintf(msg+taglen, format, args);
		va_end(args);
		if (ws_server_len_all())
		{
			msg[bufsize - 1] = '\n';
			ws_server_send_text_all(msg, (uint32_t)bufsize);
		}
		free(msg);
		return;
	}
	printf("wifilog MALLOC ERR");
}

void wifilog(const char *TAG, char *logmsg)
{
	ESP_LOGI(TAG, "%s", logmsg);
		if (ws_server_len_all())
		{
			size_t loglen = snprintf(NULL, 0, "%s%s", TAG, logmsg);
			if(loglen > 200)
			{
			char *msg = malloc(loglen);
			sprintf(msg, "%s%s", TAG, logmsg);
			//ws_server_send_text_all(TAG, (uint32_t)taglen);
			//ws_server_send_text_all(logmsg, (uint32_t)msglen);
			ws_server_send_text_all(msg, (uint32_t)loglen);
			free(msg);
			} else {
				char msg[loglen];
				sprintf(msg, "%s%s", TAG, logmsg);
				ws_server_send_text_all(msg, (uint32_t)loglen);
			}
		}
	return;
}

void wifilog_from_cb(const char *TAG, char *logmsg)
{
	ESP_LOGI(TAG, "%s", logmsg);
		if (ws_server_len_all())
		{
			size_t loglen = snprintf(NULL, 0, "%s%s", TAG, logmsg);
			if(loglen > 200)
			{
			char *msg = malloc(loglen);
			sprintf(msg, "%s%s", TAG, logmsg);
			//ws_server_send_text_all(TAG, (uint32_t)taglen);
			//ws_server_send_text_all(logmsg, (uint32_t)msglen);
			ws_server_send_text_all_from_callback(msg, (uint32_t)loglen);
			free(msg);
			} else {
				char msg[loglen];
				sprintf(msg, "%s%s", TAG, logmsg);
				ws_server_send_text_all_from_callback(msg, (uint32_t)loglen);
			}
		}
	return;
}