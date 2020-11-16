#ifndef __WIFI_STA_H__
#define __WIFI_STA_H__

/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
/* The examples use WiFi configuration that you can set via project configuration menu

   If you'd rather not, just change the below entries to strings with
   the config you want - ie #define EXAMPLE_WIFI_SSID "mywifissid"
*/

#include <stdio.h>

#define SSIDLEN 32
#define PWLEN 64
#define DEFAULT_SCAN_LIST_SIZE 10
#define NUM_NVS_KEYS    5
#define ESP_MAXIMUM_RETRY  3

uint8_t glob_ssid[SSIDLEN];
uint8_t glob_password[PWLEN];

extern int wifi_init_sta(void);
extern int wifi_connect(bool allow_scan);
extern esp_err_t wifi_set_sta_creds(char *stassid, char *stapw);

#endif