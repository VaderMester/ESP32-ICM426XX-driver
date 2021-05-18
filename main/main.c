#include <stdio.h>
#include <stdint.h>
#include "ICM426XX.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "ICM426XX-Example"

#define SERIF_TYPE ICM426XX_UI_I2C

#ifdef COMFIG_ICM426XX_USE_SPI4
#define SERIF_TYPE ICM426XX_UI_SPI4
#endif

static TaskHandle_t sICMtask = NULL;

static void ICM426XX_rawDataFifoTask(void *pvParams)
{
	uint32_t prevTicks;
	agData_t recv;
	while(1)
	{
		prevTicks = xTaskGetTickCount();
		if(xSemaphoreTake(irqSem, MS2TICKS(2000)) == pdFALSE) {
			ESP_LOGI(TAG, "Waiting for an interrupt to happen!");
			xSemaphoreTake(irqSem, portMAX_DELAY);
		}
		ESP_LOGI(TAG, "Data waiting on Queue: %d", uxQueueMessagesWaiting(xICMeventQ));
		xQueueReceive(xICMeventQ, &recv, portMAX_DELAY);
		ESP_LOGI(TAG, "RAW AG DATA: <%lld>\t[x]-%d\t[y]-%d\t[z]-%d\t-\t[Rx]-%d\t[Ry]-%d\t[Rz]-%d", recv.timestamp, recv.acc[0], recv.acc[1], recv.acc[2], recv.gyr[0], recv.gyr[1], recv.gyr[2]);
		vTaskDelayUntil(&prevTicks, 1); //Delay for at least 1 tick
	}
	vTaskDelete(NULL);
}

void app_main(void)
{
	{
		esp_chip_info_t chip_info;
		esp_chip_info(&chip_info);
		printf("This is ESP32 chip with %d CPU cores, WiFi%s%s, ",
			   chip_info.cores,
			   (chip_info.features & CHIP_FEATURE_BT) ? "/BT" : "",
			   (chip_info.features & CHIP_FEATURE_BLE) ? "/BLE" : "");
		printf("silicon revision %d\n", chip_info.revision);
	}
	esp_err_t devFound = ESP_FAIL;
	struct inv_icm426xx_serif serif = {0};
	serif.context   = 0;        /* no need */
	serif.read_reg  = ESP32_HAL_read_reg;
	serif.write_reg = ESP32_HAL_write_reg;
	serif.max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	serif.max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	serif.serif_type = SERIF_TYPE;

	ESP_ERROR_CHECK(ESP32_icm_serif_init(&serif));
	ESP_LOGI(TAG, "Scanning I2C bus for 0x%.2x", CONFIG_ICM426XX_DEV_ADDRESS);
	uint8_t found[10];
	I2Cdev_scan(&found[0], 10);
	
	for(int i = 0; i < 10; i++)
	{
		if(found[i] == CONFIG_ICM426XX_DEV_ADDRESS)
		{
			ESP_LOGI(TAG, "Found ICM device at 0x%.2x", found[i]);
			devFound = ESP_OK;
		}
	}
	if(devFound != ESP_OK) ESP_LOGE(TAG, "Device not found on I2C bus");

	inv_icm426xx_interrupt_parameter_t intconf = {0};
	char icmName[9] = {0};
	if (ICM426XX_driver_init(&serif, &intconf, ICM426XX_rawDataFifoTask, &sICMtask, icmName) == ESP_OK)
	{
		ESP_LOGI(TAG, "************** %s CURRENT INTERRUPT CONFIGURATION *************", icmName);
		ESP_LOGI(TAG, "\tFSYNC: \t%s", intconf.INV_ICM426XX_UI_FSYNC ? "Y" : "N");
		ESP_LOGI(TAG, "\tDATA RDY: \t%s", intconf.INV_ICM426XX_UI_DRDY ? "Y" : "N");
		ESP_LOGI(TAG, "\tFIFO THS: \t%s", intconf.INV_ICM426XX_FIFO_THS ? "Y" : "N");
		ESP_LOGI(TAG, "\tFIFO FULL: \t%s", intconf.INV_ICM426XX_FIFO_FULL ? "Y" : "N");
		ESP_LOGI(TAG, "\tSMD: \t%s", intconf.INV_ICM426XX_SMD ? "Y" : "N");
		ESP_LOGI(TAG, "\tWOM_X: \t%s", intconf.INV_ICM426XX_WOM_X ? "Y" : "N");
		ESP_LOGI(TAG, "\tWOM_Y: \t%s", intconf.INV_ICM426XX_WOM_Y ? "Y" : "N");
		ESP_LOGI(TAG, "\tWOM_Z: \t%s", intconf.INV_ICM426XX_WOM_Z ? "Y" : "N");
		ESP_LOGI(TAG, "\tSTEP DET: \t%s", intconf.INV_ICM426XX_STEP_DET ? "Y" : "N");
		ESP_LOGI(TAG, "\tSTEP COUNT OVERFLOW: \t%s", intconf.INV_ICM426XX_STEP_CNT_OVFL ? "Y" : "N");
		ESP_LOGI(TAG, "\tTILT: \t%s", intconf.INV_ICM426XX_TILT_DET ? "Y" : "N");
		ESP_LOGI(TAG, "\tSLEEP: \t%s", intconf.INV_ICM426XX_SLEEP_DET ? "Y" : "N");
		ESP_LOGI(TAG, "\tWAKE: \t%s", intconf.INV_ICM426XX_WAKE_DET ? "Y" : "N");
		ESP_LOGI(TAG, "\tTAP: \t%s", intconf.INV_ICM426XX_TAP_DET ? "Y" : "N");
		ESP_LOGI(TAG, "Circuit:");
		ESP_LOGI(TAG, "\tPOLARITY: \t%s", intconf.int_pol ? "ACT. HIGH" : "ACT. LOW");
		ESP_LOGI(TAG, "\tWAKE: \t%s", intconf.int_drive ? "Push-Pull" : "Open-Drain");
		ESP_LOGI(TAG, "\tTAP: \t%s", intconf.int_mode ? "Latched" : "Pulsed");
		ESP_LOGI(TAG, "****************** END OF INTERRUPT CONFIGURATION *******************");

		/*
	Code here if we want to change any interrupt config setting.
	Afterwards use:
	esp_err_t ICM426XX_dev_config_int1(inv_icm426xx_interrupt_parameter_t *intconf);
	to set configuration.
	Please read datasheet before doing this.
	*/
		//NULL installs default callback
		ICM426XX_install_Int1_isr(NULL, &intconf);

		//We can now run the driver task
		vTaskResume(sICMtask);
		osSleep(1000);
		//This code below prints a task list to console
		{
			char *info = malloc(800);
			vTaskList(info);
			ESP_LOGW(TAG, "\n%s", info);
			free(info);
		}
	}
	else
	{
		ERROR("------ WE ARE FUCKED ------");
		INFO("<<<<REBOOTING>>>>>");
		abort();
	}
}