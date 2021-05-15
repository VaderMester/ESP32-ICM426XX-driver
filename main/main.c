#include <stdio.h>
#include <stdint.h>
#include "ICM426XX.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#define TAG "ICM426XX-Example"

#define SERIF_TYPE ICM426XX_UI_I2C

#ifdef COMFIG_ICM426XX_USE_SPI4
#define SERIF_TYPE ICM426XX_UI_SPI4
#endif

static void ICM426XX_rawDataFifoTask(void *pvParams)
{
	uint32_t prevTicks;
	agData_t recv;
	while(1)
	{
		prevTicks = xTaskGetTickCount();
		xSemaphoreTake(irqSem, portMAX_DELAY);
		xQueueReceive(xICMeventQ, &recv, portMAX_DELAY);
		ESP_LOGI(TAG, "RAW AG DATA: <%lld>\t[x]-%d\t[y]-%d\t[z]-%d\t-\t[Rx]-%d\t[Ry]-%d\t[Rz]-%d", recv.timestamp, recv.acc[0], recv.acc[1], recv.acc[2], recv.gyr[0], recv.gyr[1], recv.gyr[2]);
		vTaskDelayUntil(&prevTicks, 1); //Delay for at least 1 tick
	}
	vTaskDelete(NULL);
}

void app_main(void)
{
	TaskHandle_t sICMtask;
	struct inv_icm426xx_serif serif = {0};
	serif.context   = 0;        /* no need */
	serif.read_reg  = ESP32_HAL_read_reg;
	serif.write_reg = ESP32_HAL_write_reg;
	serif.max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	serif.max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	serif.serif_type = SERIF_TYPE;

	inv_icm426xx_interrupt_parameter_t intconf = {0};
	char icmName[9] = {0};
	ESP_ERROR_CHECK(ICM426XX_driver_init(&serif, &intconf, ICM426XX_rawDataFifoTask, &sICMtask, icmName));

	ESP_LOGI(TAG, "******************** %s *********************", icmName);

	ESP_LOGI(TAG, "%s current interrupt config:", icmName);
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
	ESP_LOGI(TAG, "****************** END OF INT CONF ******************");

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
}