#include <stdio.h>
#include <dirent.h>
#include <string.h>
#include <esp_system.h>         //<-- configuration
#include "esp_log.h"            //<-- log
#include "cJSON.h"              //<-- JSON parser

#include "configwrapper.h"

static const char *TAG = "Configuration";


const char *json_configuration      = "configuration";
const char *json_wifi               = "wifi";
const char *json_accesspoint        = "accesspoint";
const char *json_station            = "station";
const char *json_enable             = "enable";
const char *json_ssid               = "ssid";
const char *json_password           = "password";
const char *json_channel            = "channel";
const char *json_retries            = "retries";
const char *json_CubeOS             = "cubeos";
const char *json_startfile          = "startfile";
char startfilename[25];

cJSON *configuration;

/*
{
	"configuration": {
		"wifi": {
			"accesspoint": {
				"enable": true,
				"ssid": "CubeOS",
				"password": "",
				"channel": 11
			},
			"station": {
				"enable": false,
				"ssid": "Valami_SSID",
				"password": "hozza_a_pasword",
				"retries": 10
			}
		}
	}
}
*/

static bool configwrapper_fileexists(const char *filepath) {
    // Try to open file
    FILE *f = fopen(filepath, "r");

    // If file does not exists 
    if (f == NULL) {
        return false;
    } else {
        // OK
    }

    // Close file and return true.
    fclose(f);

    return true;
}

static void configwrapper_parsewifiap(cJSON *ap) {
    cJSON *enable = cJSON_GetObjectItem(ap, json_enable);
    cJSON *ssid = cJSON_GetObjectItem(ap, json_ssid);
    cJSON *password = cJSON_GetObjectItem(ap, json_password);
    cJSON *channel = cJSON_GetObjectItem(ap, json_channel);

    if(enable && ssid && password && channel) {
        ESP_LOGI(TAG, "Wifi access point settings: %s %s %s %d", (enable->valueint)?("true"):("false"), ssid->valuestring, password->valuestring, channel->valueint);
    } else {
        // Missing dO_ob
    }
}

static void configwrapper_parsewifist(cJSON *st) {
    cJSON *enable = cJSON_GetObjectItem(st, json_enable);
    cJSON *ssid = cJSON_GetObjectItem(st, json_ssid);
    cJSON *password = cJSON_GetObjectItem(st, json_password);
    cJSON *retries = cJSON_GetObjectItem(st, json_retries);

    if(enable && ssid && password && retries) {
        ESP_LOGI(TAG, "Wifi station settings: %s %s %s %d", (enable->valueint)?("true"):("false"), ssid->valuestring, password->valuestring, retries->valueint);
    } else {
        // Missing dO_ob
    }
}

static void configwrapper_parseconf(cJSON *conf) {
    cJSON *wifi = cJSON_GetObjectItem(conf, json_wifi);
    if(wifi) {
        cJSON *ap = cJSON_GetObjectItem(wifi, json_accesspoint);
        cJSON *st = cJSON_GetObjectItem(wifi, json_station);

        if(ap) {
            configwrapper_parsewifiap(ap);
        } else {
            // Missing dO_ob
        }

        if(st) {
            configwrapper_parsewifist(st);
        } else {
            // Missing dO_ob
        }
        cJSON *cubeOS = cJSON_GetObjectItem(conf, json_CubeOS);
        cJSON *startfile = NULL;
        if (cubeOS)
        {
            startfile = cJSON_GetObjectItem(cubeOS, json_startfile);
        }
        if (startfile != NULL)
        {
                strcpy(startfilename, cJSON_GetStringValue(startfile));
                ESP_LOGI(TAG, "Startfile: %s", startfilename);
        }
    } else {
        // Missing dO_ob
    }

}

const char *getStartJSfile(void) {
    char ret[strlen(startfilename) + 1];
    strcpy(ret, startfilename);
    return ret;
}

configwrapper_status_t configwrapper_loadconfig(const char *filepath) {
    ESP_LOGI(TAG, "This is the path to us to load configuration: %s", filepath);

    if (configwrapper_fileexists(filepath)) {
        ESP_LOGI(TAG, "File exists");
    } else {
        return STATUS_FILE_MISSING;
    }

    FILE* f = fopen(filepath, "r");

    if(NULL != f) {
        ESP_LOGI(TAG, "File opened");
    } else {
        return STATUS_FILE_IO_ERROR;
    }

    // Get length of configuration file
    fseek(f, 0L, SEEK_END);
    int numbytes = ftell(f);

    // Allocate buffer
    char *buffer = malloc(numbytes);

    if(buffer) {
        ESP_LOGI(TAG, "File read");
    } else {
        fclose(f);
        return STATUS_MEM_ALLOC_ERROR;
    }

    // set file position to the begining
    fseek(f, 0L, SEEK_SET);

    // Read file
    fread(buffer, sizeof(char), numbytes, f);
    
    // Cloase file
    fclose(f);

    // Parse buffer
    cJSON *root = cJSON_Parse(buffer);

    if (root) {
        ESP_LOGI(TAG, "File parsed");
        
        // Parse json structure
        configuration = cJSON_GetObjectItem(root, json_configuration);
        if(configuration) {
            configwrapper_parseconf(configuration);
        } else {
            // Missing dO_ob
        }


        // Free memory
        cJSON_Delete(root);
    } else {
        free(buffer);
        return STATUS_JSON_PARSE_ERROR;
    }

    free(buffer);
    return STATUS_OK;
}
