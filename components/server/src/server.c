#include <string.h>

#include "esp_log.h"                //<-- log


#include "server.h"
#include "websocket_server.h"
#include "cubeoshello.h"


/******************************************************************************
 * Typedefs
 *****************************************************************************/


/******************************************************************************
 * Function prototypes
 *****************************************************************************/
static void server_task(void *pvParameters);
static void server_handle_task(void *pvParameters);
static void server_http_serve(struct netconn *conn);
static void server_websocket_callback(uint8_t num, WEBSOCKET_TYPE_t type, char *msg, uint64_t len);


/******************************************************************************
 * Global variables
 *****************************************************************************/
      static const char *TAG = "Server";
      static QueueHandle_t server_client_queue;
const static int server_client_queue_size = 3;


/******************************************************************************
 * Static functions
 *****************************************************************************/
// handles clients when they first connect. passes to a queue
static void server_task(void *pvParameters) {
	struct netconn *conn, *newconn;
	static err_t err;
	server_client_queue = xQueueCreate(server_client_queue_size, sizeof(struct netconn *));

	conn = netconn_new(NETCONN_TCP);
	netconn_bind(conn, NULL, 80);
	netconn_listen(conn);
	ESP_LOGI(TAG, "server listening");
	do {
		err = netconn_accept(conn, &newconn);
		ESP_LOGI(TAG, "new client");
		if (err == ERR_OK) {
			xQueueSendToBack(server_client_queue, &newconn, portMAX_DELAY);
		}
	} while (err == ERR_OK);
	netconn_close(conn);
	netconn_delete(conn);
	ESP_LOGE(TAG, "task ending, rebooting board");
	esp_restart();
}

// receives clients from queue, handles them
static void server_handle_task(void *pvParameters) {
	struct netconn *conn;
	ESP_LOGI(TAG, "task starting");
	for (;;)
	{
		xQueueReceive(server_client_queue, &conn, portMAX_DELAY);
		if (!conn)
			continue;
		server_http_serve(conn);
	}
	vTaskDelete(NULL);
}

// serves any clients
static void server_http_serve(struct netconn *conn) {
	const static char HTML_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/html\n\n";
	const static char ERROR_HEADER[] = "HTTP/1.1 404 Not Found\nContent-type: text/html\n\n";
	const static char JS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/javascript\n\n";
	const static char CSS_HEADER[] = "HTTP/1.1 200 OK\nContent-type: text/css\n\n";
	//const static char PNG_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/png\n\n";
	//const static char ICO_HEADER[] = "HTTP/1.1 200 OK\nContent-type: image/x-icon\n\n";
	//const static char PDF_HEADER[] = "HTTP/1.1 200 OK\nContent-type: application/pdf\n\n";
	//const static char EVENT_HEADER[] = "HTTP/1.1 200 OK\nContent-Type: text/event-stream\nCache-Control: no-cache\nretry: 3000\n\n";
	//const static char OCTET_STREAM_HEADER[] = "HTTP/1.1 200 OK\nContent-type: application/octet-stream\n\n";
	struct netbuf *inbuf;
	static char *buf;
	static uint16_t buflen;
	static err_t err;

	// default page
	/*
  extern const uint8_t root_html_start[] asm("_binary_root_html_start");
  extern const uint8_t root_html_end[] asm("_binary_root_html_end");
  const uint32_t root_html_len = root_html_end - root_html_start;
  */

	// test.js
	extern const uint8_t test_js_start[] asm("_binary_log_js_start");
	extern const uint8_t test_js_end[] asm("_binary_log_js_end");
	const uint32_t test_js_len = test_js_end - test_js_start;

	// test.css
	extern const uint8_t test_css_start[] asm("_binary_log_css_start");
	extern const uint8_t test_css_end[] asm("_binary_log_css_end");
	const uint32_t test_css_len = test_css_end - test_css_start;

	// favicon.ico
//	extern const uint8_t favicon_ico_start[] asm("_binary_favicon_ico_start");
//	extern const uint8_t favicon_ico_end[] asm("_binary_favicon_ico_end");
//	const uint32_t favicon_ico_len = favicon_ico_end - favicon_ico_start;

	// error page
	extern const uint8_t error_html_start[] asm("_binary_error_html_start");
	extern const uint8_t error_html_end[] asm("_binary_error_html_end");
	const uint32_t error_html_len = error_html_end - error_html_start;

	// data page
	extern const uint8_t data_html_start[] asm("_binary_data_html_start");
	extern const uint8_t data_html_end[] asm("_binary_data_html_end");
	const uint32_t data_html_len = data_html_end - data_html_start;

	netconn_set_recvtimeout(conn, 10000); // allow a connection timeout of 10 second
	ESP_LOGI(TAG, "reading from client...");
	err = netconn_recv(conn, &inbuf);
	ESP_LOGI(TAG, "read from client");
	if (err == ERR_OK)
	{
		netbuf_data(inbuf, (void **)&buf, &buflen);
		if (buf)
		{
			if (strstr(buf, "GET / ") && !strstr(buf, "Upgrade: websocket"))
			{
				ESP_LOGI(TAG, "Sending /");
				netconn_write(conn, HTML_HEADER, sizeof(HTML_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, data_html_start, data_html_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET / ") && strstr(buf, "Upgrade: websocket"))
			{
				ESP_LOGI(TAG, "Requesting websocket on /");
				ws_server_add_client(conn, buf, buflen, "/", server_websocket_callback);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET /favicon.ico "))
			{
//				ESP_LOGI(TAG, "Sending favicon.ico");
//				netconn_write(conn, ICO_HEADER, sizeof(ICO_HEADER) - 1, NETCONN_NOCOPY);
//				netconn_write(conn, favicon_ico_start, favicon_ico_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET /log.js "))
			{
				ESP_LOGI(TAG, "Sending /log.js");
				netconn_write(conn, JS_HEADER, sizeof(JS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, test_js_start, test_js_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET /log.css "))
			{
				ESP_LOGI(TAG, "Sending /log.css");
				netconn_write(conn, CSS_HEADER, sizeof(CSS_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, test_css_start, test_css_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else if (strstr(buf, "GET /"))
			{
				ESP_LOGI(TAG, "Unknown request, sending error page: %s", buf);
				netconn_write(conn, ERROR_HEADER, sizeof(ERROR_HEADER) - 1, NETCONN_NOCOPY);
				netconn_write(conn, error_html_start, error_html_len, NETCONN_NOCOPY);
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}
			else
			{
				ESP_LOGI(TAG, "Unknown request");
				netconn_close(conn);
				netconn_delete(conn);
				netbuf_delete(inbuf);
			}


		}
		else
		{
			ESP_LOGI(TAG, "Unknown request (empty?...)");
			netconn_close(conn);
			netconn_delete(conn);
			netbuf_delete(inbuf);
		}
	}
	else
	{ // if err==ERR_OK
		ESP_LOGI(TAG, "error on read, closing connection");
		netconn_close(conn);
		netconn_delete(conn);
		netbuf_delete(inbuf);
	}
}

// handles websocket events
static void server_websocket_callback(uint8_t num, WEBSOCKET_TYPE_t type, char *msg, uint64_t len) {
	switch (type) {
        case WEBSOCKET_CONNECT:
            ESP_LOGI(TAG, "client %i connected!", num);
			ws_server_send_text_client_from_callback(num, cubeoshello, hellolen);
            break;
        case WEBSOCKET_DISCONNECT_EXTERNAL:
            ESP_LOGI(TAG, "client %i sent a disconnect message", num);
            break;
        case WEBSOCKET_DISCONNECT_INTERNAL:
            ESP_LOGI(TAG, "client %i was disconnected", num);
            break;
        case WEBSOCKET_DISCONNECT_ERROR:
            ESP_LOGI(TAG, "client %i was disconnected due to an error", num);
            break;
        case WEBSOCKET_TEXT:
            ESP_LOGI(TAG, "client %i sent text message of size %i", num, (uint32_t)len);
			ESP_LOGI(TAG, "%s", msg);
			//ws_server_send_text_all_from_callback(msg, len);
            break;
        case WEBSOCKET_BIN:
            ESP_LOGI(TAG, "client %i sent binary message of size %i", num, (uint32_t)len);
            break;
        case WEBSOCKET_PING:
            ESP_LOGI(TAG, "client %i pinged us with message of size %i", num, (uint32_t)len);
            break;
        case WEBSOCKET_PONG:
            ESP_LOGI(TAG, "client %i responded to the ping", num);
            break;
        default:
            ESP_LOGI(TAG, "unknown websocket event from client %i", num);
            break;
	}
}


/******************************************************************************
 * Global functions
 *****************************************************************************/
esp_err_t server_init(void) {
    ESP_LOGI(TAG, "init");

    // start websocket server
    ws_server_start();

    // create tasks
    xTaskCreatePinnedToCore(&server_task, "server_task", 5000, NULL, 9, NULL, 0);
	xTaskCreatePinnedToCore(&server_handle_task, "server_handle_task", 6000, NULL, 6, NULL, 0);

    return ESP_OK;
}
