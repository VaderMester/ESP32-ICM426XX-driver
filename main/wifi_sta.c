/* WiFi station Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "wifi_sta.h"

#include "lwip/err.h"
#include "lwip/sys.h"

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

#define snprintf_nowarn(...) (snprintf(__VA_ARGS__) < 0 ? abort() : (void)0)

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

static const char *TAG = "wifi station";

static const char *nvsSSID = "ssid-";
static const char *nvsPW = "pwrd-";


static int s_retry_num = 0;
static int s_nvs_index = 0;
static int wifi_scan(void);

static wifi_ap_record_t selected;

static void event_handler(void* arg, esp_event_base_t event_base,
                                int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
            ESP_LOGI(TAG,"connect to the AP fail");
        }  
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:%s",
                 ip4addr_ntoa(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

//Reads NVS values stored in keyindex and stores results in glob_ssid and glob_password
static esp_err_t wifi_sta_get_nvs_creds(uint8_t keyindex)
{
    esp_err_t err = ESP_FAIL;
    if(keyindex >= NUM_NVS_KEYS) {
        return err;
    }
    nvs_handle_t wifiCreds;
    err = nvs_open("WIFICREDS", NVS_READWRITE, &wifiCreds);
    uint8_t keylen = 0;
    if (err == ESP_OK) {
        //ESP_LOGI(TAG, "Opening Non-Volatile Storage (NVS) handle... ");
        if(NUM_NVS_KEYS < 10) {
            keylen = 6+1;
        } else {
            keylen = 6+2;
        }
        char ssid_from_nvs[SSIDLEN];
        char pwrd_from_nvs[PWLEN];
        char ssidkey[keylen];
        char pwrdkey[keylen];
        size_t ssid_len = SSIDLEN;
        size_t pwrd_len = PWLEN;
        snprintf_nowarn(ssidkey, keylen, "%s%d", nvsSSID, keyindex);
        snprintf_nowarn(pwrdkey, keylen, "%s%d", nvsPW, keyindex);
        err = nvs_get_str(wifiCreds, ssidkey, ssid_from_nvs, &ssid_len);
        if(err == ESP_OK) {
            err = nvs_get_str(wifiCreds, pwrdkey, pwrd_from_nvs, &pwrd_len);
            if(err == ESP_OK) {
                memcpy((void*)glob_ssid, (void*)ssid_from_nvs, SSIDLEN);
                memcpy((void*)glob_password, (void*)pwrd_from_nvs, PWLEN);
            }
            nvs_close(wifiCreds);
            //ESP_LOGE(TAG, "wifi_sta_get_nvs_creds: 0x%04X", err);
            return err;
        }
        nvs_close(wifiCreds);
    }
    //ESP_LOGE(TAG, "wifi_sta_get_nvs_creds: 0x%04X", err);
    return err;
}


//Sets NVS values stored at keyindex from glob_ssid and glob_password
static esp_err_t wifi_sta_set_nvs_creds(uint8_t keyindex)
{
    esp_err_t err = ESP_FAIL;
    if (keyindex >= NUM_NVS_KEYS)
    {
        return err;
    }
    nvs_handle_t wifiCreds;
    err = nvs_open("WIFICREDS", NVS_READWRITE, &wifiCreds);
    uint8_t keylen = 0;
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "Successfully openeded NVS with handle: WIFICREDS");
        if (NUM_NVS_KEYS < 10)
        {
            keylen = 6 + 1;
        }
        else
        {
            keylen = 6 + 2;
        }
        char ssid_to_nvs[SSIDLEN];
        char pwrd_to_nvs[PWLEN];
        char ssidkey[keylen];
        char pwrdkey[keylen];

        snprintf_nowarn(ssidkey, keylen, "%s%d", nvsSSID, keyindex);
        snprintf_nowarn(pwrdkey, keylen, "%s%d", nvsPW, keyindex);
        //ESP_LOGI(TAG, "%s %s", ssidkey, pwrdkey);
        memcpy((void *)ssid_to_nvs, (void *)glob_ssid, SSIDLEN);
        memcpy((void *)pwrd_to_nvs, (void *)glob_password, PWLEN);
        //ESP_LOGI(TAG, "%s %s", ssid_to_nvs, glob_ssid);
        //ESP_LOGI(TAG, "%s %s", pwrd_to_nvs, glob_password);
        err = nvs_set_str(wifiCreds, ssidkey, ssid_to_nvs);
        if (err == ESP_OK)
        {
            err = nvs_set_str(wifiCreds, pwrdkey, pwrd_to_nvs);
            if (err == ESP_OK)
            {
               err = nvs_commit(wifiCreds);
            }
        }
        nvs_close(wifiCreds);
    }
    //ESP_LOGE(TAG, "wifi_sta_set_nvs_creds: 0x%04X", err);
    return err;
}

esp_err_t wifi_set_sta_creds(char *stassid, char *stapw)
{
    uint8_t foundIndex = NUM_NVS_KEYS +1;
    esp_err_t err = ESP_FAIL;
    if(stassid == NULL || stapw == NULL)
    {
        //ESP_LOGE(TAG, "wifi_sta_set_creds: 0x%04X", err);
        return err;
    }
    for(int i = 0; (i < NUM_NVS_KEYS) && (foundIndex > NUM_NVS_KEYS); i++) {
        memset(glob_ssid, NULL, SSIDLEN);
        memset(glob_password, NULL, PWLEN);
        if((err = wifi_sta_get_nvs_creds(i)) == ESP_OK) 
        {
            //ESP_LOGI(TAG, "glob_ssid: %s, argument: %s", glob_ssid, stassid);
            //ESP_LOGI(TAG, "glob_password: %s, argument: %s", glob_password, stapw);
            if(strncmp((char*)glob_ssid, (char*)stassid, strlen(stassid)) == 0)
            {
                if(strncmp((char*)glob_password, (char*)stapw, strlen(stapw)) == 0)
                {
                    foundIndex = i;
                    err = ESP_OK;
                }
            }
        }
        else if (err == ESP_ERR_NVS_NOT_FOUND)
        {
            memcpy((void*)glob_ssid, (void*)stassid, strlen(stassid));
            memcpy((void*)glob_password, (void*)stapw, strlen(stapw));
            err = wifi_sta_set_nvs_creds(i);
            ESP_LOGW(TAG, "WIFI Credentials are set");
        }
        if(foundIndex < NUM_NVS_KEYS) {
            ESP_LOGI(TAG, "Found given credentials already set at %d", foundIndex);
        }
    }
    //ESP_LOGE(TAG, "wifi_sta_set_creds: 0x%04X", err);
    return err;
}

int wifi_init_sta(void)
{
    //Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    s_wifi_event_group = xEventGroupCreate();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    return 1;
}

int wifi_connect(bool allow_scan) {

    int retval = 0;

    if(allow_scan)
    {
        int foundSSID = 0;
        uint8_t i = 0;
        //TODO
        while(foundSSID == 0 && i < 3) {
            memset(glob_ssid, NULL, SSIDLEN);
            memset(glob_password, NULL, PWLEN);
            foundSSID = wifi_scan();
            i++;
            osSleep(20);
        }
        if(foundSSID == 0) {
            return 0;
        }
    }
    else
    {
        memset(glob_ssid, NULL, SSIDLEN);
        memset(glob_password, NULL, PWLEN);
        if(wifi_sta_get_nvs_creds(s_nvs_index) != ESP_OK) { //This modifies global_ssid and global_password;
            s_nvs_index++;
            return 0;
        }    
    }

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, NULL));

    wifi_config_t wifi_config;
    memcpy((void*)wifi_config.sta.ssid, (void*)glob_ssid, 32);
    memcpy((void*)wifi_config.sta.password, (void*)glob_password, 64);
    wifi_config.sta.pmf_cfg.capable = true;
    wifi_config.sta.pmf_cfg.required = false;
    //memcpy((void*)wifi_config.sta.bssid, (void*)selected.bssid, 6);
    //wifi_config.sta.channel = selected.primary;
    /*
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = ssid,
            .password = password,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };
    */
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_start() );

    ESP_LOGI(TAG, "Wifi STA Started.");

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
        retval = 1;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 wifi_config.sta.ssid, wifi_config.sta.password);
        retval = 0;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        retval = 0;
    }

    ESP_ERROR_CHECK(esp_event_handler_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler));
    ESP_ERROR_CHECK(esp_event_handler_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler));
    vEventGroupDelete(s_wifi_event_group);
    if(retval == 0)
    {
        s_nvs_index++;
    } else {
        s_nvs_index = 0;
    }
    return retval;
}

static void print_auth_mode(int authmode)
{
    switch (authmode) {
    case WIFI_AUTH_OPEN:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_OPEN");
        break;
    case WIFI_AUTH_WEP:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WEP");
        break;
    case WIFI_AUTH_WPA_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_PSK");
        break;
    case WIFI_AUTH_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA_WPA2_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA_WPA2_PSK");
        break;
    case WIFI_AUTH_WPA2_ENTERPRISE:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_ENTERPRISE");
        break;
    case WIFI_AUTH_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA3_PSK");
        break;
    case WIFI_AUTH_WPA2_WPA3_PSK:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_WPA2_WPA3_PSK");
        break;
    default:
        ESP_LOGI(TAG, "Authmode \tWIFI_AUTH_UNKNOWN");
        break;
    }
}

static void print_cipher_type(int pairwise_cipher, int group_cipher)
{
    switch (pairwise_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Pairwise Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }

    switch (group_cipher) {
    case WIFI_CIPHER_TYPE_NONE:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_NONE");
        break;
    case WIFI_CIPHER_TYPE_WEP40:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP40");
        break;
    case WIFI_CIPHER_TYPE_WEP104:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_WEP104");
        break;
    case WIFI_CIPHER_TYPE_TKIP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP");
        break;
    case WIFI_CIPHER_TYPE_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_CCMP");
        break;
    case WIFI_CIPHER_TYPE_TKIP_CCMP:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_TKIP_CCMP");
        break;
    default:
        ESP_LOGI(TAG, "Group Cipher \tWIFI_CIPHER_TYPE_UNKNOWN");
        break;
    }
}

/* Initialize Wi-Fi as sta and set scan method */
static int wifi_scan(void)
{
    int ret = 0;
    //ESP_ERROR_CHECK(esp_netif_init());
    //ESP_ERROR_CHECK(esp_event_loop_create_default());
    //esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    //assert(sta_netif);

    //wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    //ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    uint16_t number = DEFAULT_SCAN_LIST_SIZE;
    wifi_ap_record_t ap_info[DEFAULT_SCAN_LIST_SIZE];
    uint16_t ap_count = 0;
    memset(ap_info, 0, sizeof(ap_info));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_scan_start(NULL, true));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_records(&number, ap_info));
    ESP_ERROR_CHECK(esp_wifi_scan_get_ap_num(&ap_count));
    ESP_LOGI(TAG, "Total APs scanned = %u", ap_count);
    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        ESP_LOGI(TAG, "SSID \t\t%s", ap_info[i].ssid);
        ESP_LOGI(TAG, "RSSI \t\t%d", ap_info[i].rssi);
        print_auth_mode(ap_info[i].authmode);
        if (ap_info[i].authmode != WIFI_AUTH_WEP) {
            print_cipher_type(ap_info[i].pairwise_cipher, ap_info[i].group_cipher);
        }
        ESP_LOGI(TAG, "Channel \t\t%d\n", ap_info[i].primary);
    }
    uint apNum = DEFAULT_SCAN_LIST_SIZE+1;
    uint8_t nvsindex = NUM_NVS_KEYS+1;
    //uint8_t nvstries = 0;
    for (int i = 0; (i < DEFAULT_SCAN_LIST_SIZE) && (i < ap_count); i++) {
        for (int j = 0; j < NUM_NVS_KEYS; j++)
        {
            if (wifi_sta_get_nvs_creds(j) == ESP_OK)
            {
                if (strncmp((char *)ap_info[i].ssid, (char *)glob_ssid, strlen((char *)ap_info[i].ssid)) == 0)
                {
                    //nvstries++;
                    if (apNum > DEFAULT_SCAN_LIST_SIZE)
                    {
                        apNum = i;
                        nvsindex = j;
                    }
                    else
                    {
                        if (ap_info[apNum].rssi < ap_info[i].rssi)
                        {
                            apNum = i;
                            nvsindex = j;
                        }
                    }
                    ret = 1;
                }
            }
            else
            {
            ESP_LOGI(TAG, "Could not get valid Wifi Creds from NVS");
            }
        }
    }
    if(apNum < DEFAULT_SCAN_LIST_SIZE) {
        ESP_LOGW(TAG, "SSID found! List item: %d, Stored SSID index: %d", apNum, nvsindex);
    }
    else
    {
        ESP_LOGW(TAG, "No Scanned Wifi networks match stored credentials");
    }
    if (ret == 1) {
        selected = ap_info[apNum];
        ESP_LOGI(TAG, "Selected SSID: %s, AP MAC: %02X:%02X:%02X:%02X:%02X:%02X, Channel: %d, RSSI: %d", selected.ssid, selected.bssid[0], selected.bssid[1], selected.bssid[2], selected.bssid[3], selected.bssid[4], selected.bssid[5], selected.primary, selected.rssi);
    }
    return ret;
}