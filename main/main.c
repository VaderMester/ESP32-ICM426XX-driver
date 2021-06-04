#include <stdio.h>
#include <stdint.h>
#include "ICM426XX.h"
#include "malloc_local.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_err.h"

/* INCLUDE for serial interface */
#include "I2Cdev.h"

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
static int ESP32_HAL_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len);
static int ESP32_HAL_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len);
static int ESP32_icm_serif_init(struct inv_icm426xx_serif *serif);
/*
 * --------------------------------------------------------------------------------------
 *  ESP32 SPECIFIC HAL Functions 
 * --------------------------------------------------------------------------------------
 *
 * --------------------------------------------------------------------------------------
 *  INSTRUCTIONS
 *
 * Insert the functions you want to use below, as per the example
 * --------------------------------------------------------------------------------------
 */

/** @brief Below are the I2C HAL functions compatible with the "Icm426xxTransport.h"
struct inv_icm426xx_serif {
	void *     context;
	int      (*read_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);
	int      (*write_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * buf, uint32_t len);
	int      (*configure)(struct inv_icm426xx_serif * serif);
	uint32_t   max_read;
	uint32_t   max_write;
	ICM426XX_SERIAL_IF_TYPE_t serif_type;
};
*/
/**
 * @brief This is to connect in ESP32 HW functions for reading registers from the ICM426XX device
 * @param serif:  We only use ICM426XX_SERIAL_IF_TYPE_t serif_type to brach for the properly
 * @param reg:    address of register to be read
 * @param buf:    buffer pointer for data to be stored
 * @param len:    read length
 * 
 * @return: 0: OK
 *          Everything return that is not 0, means an error
 */
static int ESP32_HAL_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
	//Add your SPI read function here
		return -10;
	case ICM426XX_UI_I2C:
		return (I2Cdev_readBytes(ICM426XX_devAddr, reg, len, buf));
	default:
		return -1;
	}
	return 1;
}

/**
 * @brief This is to connect in ESP32 HW functions for writing to registers of ICM426XX device
 * @param serif:  We only use ICM426XX_SERIAL_IF_TYPE_t serif_type to brach for the properly
 * @param reg:    address of register to be read
 * @param buf:    buffer pointer for data to be stored
 * @param len:    read length
 * 
 * @return: 0: OK
 *          Everything return that is not 0, means an error
 */
static int ESP32_HAL_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
	//Add your SPI write function here
		return -10;
	case ICM426XX_UI_I2C:
		return I2Cdev_writeBytes(ICM426XX_devAddr, reg, len, buf);
	default:
		return -1;
	}
	return 1;
}

static int ESP32_icm_serif_init(struct inv_icm426xx_serif *serif)
{
	int rc;
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
		//TODO: Add SPI later
		return -10;
	case ICM426XX_UI_I2C:
		rc = I2Cdev_init(-1, -1);
		if (rc != ESP_OK)
		{
			ESP_LOGE(LOGNAME, "Failed to init I2Cdev: %s", esp_err_to_name(rc));
			return -1;
		}
		else
		{
			ESP_LOGI(LOGNAME, "I2C driver initialized!");
			return 0;
		}
	default:
		return -1;
	}
	return 1;
}

/*Driver Task */
static void ICM426XX_handleIrqTask(void *pvParams)
{
	int cnt, intnum;
	uint8_t convert = 1;
	ICM426XX_intr_status_t intstats[ICM426XX_NUM_INTRS] = {0};
	ag_buffer_t *buf = NULL;
	while (1)
	{
		cnt = uxSemaphoreGetCount(irqSem);
		ESP_LOGD(TAG, "Cnt= %d", cnt);
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
					//pcnt = ICM426XX_readFifo();
					buf = ICM426XX_readFifo(convert);
					if (buf)
					{
						if (!buf->err)
						{
							ESP_LOGI(TAG, "FIFO -> %d packets read", buf->pcnt);
							/* If Queue is full we remove discard the first (oldest) item */
							if (uxQueueSpacesAvailable(xICMeventQ) == 0)
							{
								ag_buffer_t *recv;
								xQueueReceive(xICMeventQ, &recv, 1);
							}
							xQueueSendToBack(xICMeventQ, (void *)&buf, 1);
							break;
						}
						ESP_LOGE(TAG, "FIFO -> FAILED TO READ: %d", buf->err);
						ICM426XX_delete_ag_buffer(buf);
						break;
					}
					ESP_LOGE(TAG, "FIFO -> FAILED TO READ");

				break;
				case INTR_ICM426XX_FIFO_FULL:
					buf = ICM426XX_readFifo(convert);
					if (buf)
					{
						if (buf->err == ESP_OK)
						{
							ESP_LOGI(TAG, "FIFO -> %d packets read", buf->pcnt);
							/* If Queue is full we remove discard the first (oldest) item */
							if (uxQueueSpacesAvailable(xICMeventQ) == 0)
							{
								ag_buffer_t *recv;
								xQueueReceive(xICMeventQ, &recv, 1);
							}
							xQueueSendToBack(xICMeventQ, (void *)&buf, 1);
							break;
						}
						ESP_LOGE(TAG, "FIFO -> FAILED TO READ: %s", esp_err_to_name(buf->err));
						ICM426XX_delete_ag_buffer(buf);
						break;
					}
					ESP_LOGE(TAG, "FIFO -> FAILED TO READ: Buffer not created");
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
	ag_buffer_t *recv = NULL;
	ag_sensor_data_t *dp;
	while (1)
	{
		//ESP_LOGW(TAG, "Data waiting on Queue: %d", uxQueueMessagesWaiting(xICMeventQ));
		xQueueReceive(xICMeventQ, &recv, portMAX_DELAY);
		dp = recv->data;
		for(int i = 0; i < recv->pcnt; i++)
		{
		ESP_LOGI(TAG, "RAW AG DATA: <%d>ms\t temp:%3.1fC\t[x]-%d\t[y]-%d\t[z]-%d\t-\t[Rx]-%d\t[Ry]-%d\t[Rz]-%d", dp->tmstp/10000, (float)(dp->temp/10), dp->accel[0], dp->accel[1], dp->accel[2], dp->gyro[0], dp->gyro[1], dp->gyro[2]);
		dp++;
		}
		ICM426XX_delete_ag_buffer(recv);
		continue;
	}
	vTaskDelete(NULL);
}

static void printTaskList(void)
{
	char *info = malloc_local(800);
	vTaskList(info);
	ESP_LOGW(TAG, "\n%s", info);
	free_local(info);
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
	osSleep(100);
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
	xTaskCreate(ICM426XX_processData, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, 5, sICMprocessTask);
#else
	xTaskCreatePinnedToCore(ICM426XX_processData, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, 5, sICMprocessTask, CONFIG_ICM426XX_TASK_CORE_AFFINITY);
#endif
	if (taskSusp) vTaskResume(sICMirqTask);
	//After this, interrupts in ICM device are active.
	ICM426XX_print_dev_int1_config(CONFIG_ICM426XX_DEV_NAME, ESP_LOG_INFO);

	/* Pouring info out of console */
	ICM426XX_dumpBank0Regs();
	ICM426XX_dumpBank1Regs();
	printTaskList();
	//This starts data acquisition.
	ECHK(ICM426XX_resetFifo());

	/*Install interrupt service to start receiving data */
	ECHK(ICM426XX_install_Int1_isr(NULL, &intconf));

	
}