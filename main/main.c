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

const static char icm426xx_intrstr[ICM426XX_NUM_INTRS+1][29] = {
	"INTR_ICM426XX_NONE",
	"INTR_ICM426XX_UI_FSYNC",
	"INTR_ICM426XX_PLL_RDY",
	"INTR_ICM426XX_RESET_DONE",
	"INTR_ICM426XX_UI_DRDY",
	"INTR_ICM426XX_FIFO_THS",
	"INTR_ICM426XX_FIFO_FULL",
	"INTR_ICM426XX_AGC_RDY",
	"INTR_ICM426XX_SMD",
	"INTR_ICM426XX_WOM_X",
	"INTR_ICM426XX_WOM_Y",
	"INTR_ICM426XX_WOM_Z",
	"INTR_ICM426XX_STEP_DET",
	"INTR_ICM426XX_STEP_CNT_OVFL",
	"INTR_ICM426XX_TILT_DET",
#if defined(ICM_FAMILY_BPLUS)
	"INTR_ICM426XX_SLEEP_DET",
	"INTR_ICM426XX_WAKE_DET",
#elif defined(ICM_FAMILY_CPLUS)
	"INTR_ICM426XX_LOWG_DET",
	"INTR_ICM426XX_FF_DET",
#endif
	"INTR_ICM426XX_TAP_DET"};


#define ECHK(val) do{int v = (val); if(v != 0) {  \
    ESP_LOGE(TAG,"TRACE %s(%d): %d", __FUNCTION__, __LINE__, v);  \
	for (int i = 5; i >= 0; i--) {\
	ESP_LOGE(TAG, "Restarting in %d seconds...\n", i); osSleep(1000);} \
	ESP_LOGE(TAG, "Restarting now.\n"); \
	fflush(stdout); \
	abort();} \
	}while(0)

#define ELOG(val) do{int v = (val); if(v != 0) {ESP_LOGE(TAG, "Error in %s(%d): %d", __FUNCTION__, __LINE__, v);}} while(0)
#define ELOG_RET(val) do{int v = (val); if(v != 0) {ESP_LOGE(TAG, "Error in %s(%d): %d", __FUNCTION__, __LINE__, v); return v;}} while(0)

const static char intdet[] = "interrupt detected";

static TaskHandle_t sICMirqTask = NULL;
static TaskHandle_t sICMprocessTask = NULL;

static void ICM426XX_processData(void *pvParams);
static void ICM426XX_handleIrqTask(void *pvParams);

static void ICM426XX_handleIrqTask(void *pvParams)
{
	int cnt, intnum, pcnt;
	ICM426XX_intr_status_t intstats[ICM426XX_NUM_INTRS] = {0};
	while (1)
	{
		cnt = uxSemaphoreGetCount(irqSem);
		ESP_LOGE(TAG, "Cnt= %d", cnt);
		if (xSemaphoreTake(irqSem, 0) == pdFALSE)
		{
			ESP_LOGW(TAG, "Waiting for an interrupt to happen!");
			xSemaphoreTake(irqSem, portMAX_DELAY);
		}
		else
		{
			ESP_LOGE(TAG, "Semaphore Taken");
		}
		intnum = ICM426XX_get_intr_events(&intstats[0], ICM426XX_NUM_INTRS);
		ESP_LOGW(TAG, "--> INTERRUPTS: Found: %d, detected: %d", intnum, cnt);
		if (intnum > 0)
		{
			pcnt = 0;
			for (int i = 0; i < intnum; i++)
			{
				ESP_LOGI(TAG, "%s %s (%d)", icm426xx_intrstr[intstats[i]], intdet, i);
				switch (intstats[i])
				{
				case INTR_ICM426XX_NONE:
				break;
				case INTR_ICM426XX_PLL_RDY:
				break;
				case INTR_ICM426XX_RESET_DONE:
				break;
				case INTR_ICM426XX_UI_DRDY:
				break;
				case INTR_ICM426XX_FIFO_THS:
					pcnt = ICM426XX_readFifo();
					ESP_LOGI(TAG, "FIFO -> %d packets read", pcnt);
					/*
					ICM426XX_readreg(MPUREG_FIFO_DATA, 16, datachk);
					printf("Raw FIFO Data:");
					for(int i = 0; i < 16; i++){
						printf("0x%02X ", datachk[i]);
					}
					printf("\n");
					*/
					break;
				case INTR_ICM426XX_FIFO_FULL:
					if(!pcnt){
						pcnt = ICM426XX_readFifo();
						ESP_LOGI(TAG, "FIFO -> %d packets read", pcnt);
					}
					break;
				case INTR_ICM426XX_AGC_RDY:
				break;
				case INTR_ICM426XX_SMD:
				break;
				case INTR_ICM426XX_WOM_X:
				break;
				case INTR_ICM426XX_WOM_Y:
				break;
				case INTR_ICM426XX_WOM_Z:
				break;
				case INTR_ICM426XX_STEP_DET:
				break;
				case INTR_ICM426XX_STEP_CNT_OVFL:
				break;
				case INTR_ICM426XX_TILT_DET:
				break;
#if defined(ICM_FAMILY_BPLUS)
				case INTR_ICM426XX_SLEEP_DET:
				break;
				case INTR_ICM426XX_WAKE_DET:
				break;
#elif defined(ICM_FAMILY_CPLUS)
				case INTR_ICM426XX_LOWG_DET:
				break;
				case INTR_ICM426XX_FF_DET:
				break;
#endif
				case INTR_ICM426XX_TAP_DET:
				break;
				default:
				break;
				}
			}
			continue;
		}
	}
	vTaskDelete(NULL);
}

static void ICM426XX_processData(void *pvParams)
{
	//uint32_t prevTicks;
	agData_t recv;
	while (1)
	{
		//prevTicks = xTaskGetTickCount();
		ESP_LOGD(TAG, "Data waiting on Queue: %d", uxQueueMessagesWaiting(xICMeventQ));
		xQueueReceive(xICMeventQ, &recv, portMAX_DELAY);
		ESP_LOGI(TAG, "RAW AG DATA: <%lld>\t[x]-%d\t[y]-%d\t[z]-%d\t-\t[Rx]-%d\t[Ry]-%d\t[Rz]-%d", recv.timestamp, recv.acc[0], recv.acc[1], recv.acc[2], recv.gyr[0], recv.gyr[1], recv.gyr[2]);
		//vTaskDelayUntil(&prevTicks, 1); //Delay for at least 1 tick
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
	uint8_t devFound = 0;
	struct inv_icm426xx_serif serif = {0};
	serif.context = 0; /* no need */
	serif.read_reg = ESP32_HAL_read_reg;
	serif.write_reg = ESP32_HAL_write_reg;
	serif.max_read = 1024 * 32;	 /* maximum number of bytes allowed per serial read */
	serif.max_write = 1024 * 32; /* maximum number of bytes allowed per serial write */
	serif.serif_type = SERIF_TYPE;
	ESP_ERROR_CHECK(ESP32_icm_serif_init(&serif));
	ESP_LOGI(TAG, "Scanning I2C bus for ICM DEVICE");
	uint8_t found[10];
	I2Cdev_scan(&found[0], 10);

	for (int i = 0; i < 10; i++)
	{
		if (found[i] == ICM426XX_ADDR_AD0_LO)
		{
			ESP_LOGI(TAG, "Found ICM device at 0x%.2x", found[i]);
			devFound = found[i];
			break;
		}
		if (found[i] == ICM426XX_ADDR_AD0_HI)
		{
			ESP_LOGI(TAG, "Found ICM device at 0x%.2x", found[i]);
			devFound = found[i];
			break;
		}
	}
	if (devFound == 0) ESP_LOGE(TAG, "Device not found on I2C bus");

	char icmName[9] = {0};
	ICM426XX_set_I2C_addr(devFound);
	ECHK(ICM426XX_driver_init(&serif, icmName));
	ECHK(ICM426XX_set_fifo_threshold(10));
	ECHK(ICM426XX_ConfigureUseFifo((uint8_t)IS_LOW_NOISE_MODE,
							(uint8_t)IS_HIGH_RES_MODE,
							ICM426XX_ACCEL_CONFIG0_FS_SEL_4g,
							ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps,
							ICM426XX_ACCEL_CONFIG0_ODR_50_HZ,
							ICM426XX_GYRO_CONFIG0_ODR_50_HZ,
							(uint8_t)USE_CLK_IN));

	ECHK(ICM426XX_sensor_en_all());
	inv_icm426xx_interrupt_parameter_t intconf = {0};
	/* Uncomment the interrupt event you want to watch */
	//intconf.INV_ICM426XX_UI_FSYNC  	= INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_UI_DRDY   	= INV_ICM426XX_ENABLE;
	intconf.INV_ICM426XX_FIFO_THS  	= INV_ICM426XX_ENABLE;
	intconf.INV_ICM426XX_FIFO_FULL 	= INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_SMD   		= INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_WOM_X 		= INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_WOM_Y 		= INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_WOM_Z 		= INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_STEP_DET      = INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_STEP_CNT_OVFL = INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_TILT_DET      = INV_ICM426XX_ENABLE;
	//intconf.INV_ICM426XX_TAP_DET       = INV_ICM426XX_ENABLE;
	intconf.int_pol = ICM426XX_INT_CONFIG_INT1_POLARITY_LOW;
	intconf.int_drive = ICM426XX_INT_CONFIG_INT1_DRIVE_CIRCUIT_PP;
	intconf.int_mode = ICM426XX_INT_CONFIG_INT1_PULSE;
	ELOG(ICM426XX_set_dev_int1_config(&intconf));

	int taskSusp = 1;
	ECHK(ICM426XX_create_driver_task(ICM426XX_handleIrqTask, &sICMirqTask, taskSusp));
	
/*Create the process task */
#ifdef CONFIG_FREERTOS_UNICORE
	xTaskCreate(ICM426XX_processData, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, CONFIG_ICM426XX_TASK_PRIORITY, sICMprocessTask);
#else
	xTaskCreatePinnedToCore(ICM426XX_processData, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, CONFIG_ICM426XX_TASK_PRIORITY, sICMprocessTask, CONFIG_ICM426XX_TASK_CORE_AFFINITY);
#endif
	if (taskSusp) vTaskResume(sICMirqTask);
	//After this, interrupts are active.
	ICM426XX_print_dev_int1_config(CONFIG_ICM426XX_DEV_NAME, ESP_LOG_INFO);
	ICM426XX_dumpBank0Regs();
	ICM426XX_dumpBank1Regs();

	//This starts data acquisition.
	ECHK(ICM426XX_resetFifo());
	ECHK(ICM426XX_install_Int1_isr(NULL, &intconf));
	//This code below prints a task list to console
	/*
	{
		char *info = malloc(800);
		vTaskList(info);
		ESP_LOGW(TAG, "\n%s", info);
		free(info);
	}
	*/
}