/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The ICM426XX is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "sdkconfig.h"

#include "nvs_flash.h"

//#include "Message.h"

#include "ICM426XX.h"
#include "time.h"
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sntp.h"

#define LOGNAME "ICM426XX"
#define ICM426XX_NVS_NAME "ICM426XXNVS"
#define ICM426XX_NVS_KEY "biases"

#if CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 0
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 1
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 2
#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 3
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 4
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#elif CONFIG_ICM426XX_LOCAL_LOG_LEVEL == 5
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif

#include "logmacro.h"

#define RAD_TO_DEG(rad) ((float)rad * 57.2957795131)

#define ESP_INTR_FLAG_DEFAULT 0

/* Full Scale Range */
#if 0
#if IS_HIGH_RES_MODE
	#if defined(ICM42686)
		static const int32_t acc_fsr = 32;   /* +/- 32g */
		static const int32_t gyr_fsr = 4000; /* +/- 4000dps */
	#else
		static const int32_t acc_fsr = 16;   /* +/- 16g */
		static const int32_t gyr_fsr = 2000; /* +/- 2000dps */
	#endif
#else
	static const int32_t acc_fsr = 4;        /* +/- 4g */
	static const int32_t gyr_fsr = 2000;     /* +/- 2000dps */
#endif
#endif

/*
 * ICM mounting matrix
 * Coefficients are coded as Q30 integer
 */
#if defined(ICM_FAMILY_CPLUS)
static int32_t icm_mounting_matrix[9] = {  0,        -(1<<30),      0,
                                          (1<<30),     0,           0,
                                           0,          0,          (1<<30) };
#else
static int32_t icm_mounting_matrix[9] = { (1<<30),     0,           0,
                                           0,         (1<<30),      0,
                                           0,          0,          (1<<30) };
#endif


/* Just a handy variable to handle the icm426xx object */
static struct inv_icm426xx icm_driver;

/* structure allowing to handle clock calibration */
static clk_calib_t clk_calib;

/* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
RINGBUFFER(timestamp_buffer, 64, uint64_t);

/* MUX for entering critical section */
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

const uint8_t qSize = 10;

#if USE_MAG
/* Ak0991x driver object */
static inv_ak0991x_t ak_driver;

/* Buffer to keep track of the timestamp when ak09915 data ready interrupt fires. */
extern RINGBUFFER_VOLATILE(timestamp_buffer_mag, 64, uint64_t);

/*
 * Magnetometer mounting matrix
 * Coefficients are coded as Q30 integer
 */
static int32_t mag_mounting_matrix[9] = {  0,         (1<<30),     0,
                                         -(1<<30),     0,          0,
                                           0,          0,         (1<<30) };

/* Variable to keep track if the mag has initialized successfully */
extern int mag_init_successful;

#endif //#if USE_MAG


/* --------------------------------------------------------------------------------------
 *  Extern functions definition - Declared in Icm426xxExtFunc.h
 * -------------------------------------------------------------------------------------- */

/*
 * Icm426xx driver needs to get time in us. Let's give its implementation here.
 */
uint64_t inv_icm426xx_get_time_us(void)
{
	return ESP32_get_time_us();
}

/*
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 * ESP32 implementation uses osSleep() macro, which calls vTaskDelay.
 * This means the minimum resolution depends of the tick frequency.
 */
 void inv_icm426xx_sleep_us(uint32_t us)
{
	osSleep((TickType_t)(us/1000));
}

/*
 * --------------------------------------------------------------------------------------
 *  ESP32 SPECIFIC HAL Functions 
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
int ESP32_HAL_read_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
		//TODO: Add SPI later
		return 1;
	case ICM426XX_UI_I2C:
		if (I2Cdev_readBytes(ICM426XX_BUS_ADDR, reg, len, buf))
		{
			return 0;
		}
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
int ESP32_HAL_write_reg(struct inv_icm426xx_serif *serif, uint8_t reg, uint8_t *buf, uint32_t len)
{
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
		//TODO: Add SPI later
		return -1;
	case ICM426XX_UI_I2C:
		if (I2Cdev_writeBytes(ICM426XX_BUS_ADDR, reg, len, buf))
		{
			return 0;
		}
	default:
		return -1;
	}
	return 1;
}

int ESP32_icm_serif_init(struct inv_icm426xx_serif *serif)
{
	int rc;
	switch (serif->serif_type)
	{
	case ICM426XX_AUX1_SPI3:
	case ICM426XX_AUX1_SPI4:
	case ICM426XX_AUX2_SPI3:
	case ICM426XX_UI_SPI4:
		//TODO: Add SPI later
		return -1;
	case ICM426XX_UI_I2C:
		rc = I2Cdev_init(-1, -1);
		if (rc != ESP_OK)
		{
			ESP_LOGE(LOGNAME, "Failed to init I2Cdev: %s", esp_err_to_name(rc));
			return -1;
		}
		else
		{
			return 0;
		}
	default:
		return -1;
	}
	return 1;
}

static esp_err_t ESP32_ICM426XX_nvs_init(void)
{
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	icm_nvs_inited = ret;
	return ret;
}

esp_err_t ESP32_store_biases_in_flash(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3])
{
	esp_err_t err = ESP_FAIL;
	nvs_handle_t handle;
	uint8_t sensors_biases[36] = {0};
	uint8_t idx = 0;
	size_t size;

	memcpy(&sensors_biases[idx], acc_bias_q16, sizeof(acc_bias_q16[0]) * 3);
	idx += sizeof(acc_bias_q16[0]) * 3;

	memcpy(&sensors_biases[idx], gyr_bias_q16, sizeof(gyr_bias_q16[0]) * 3);
	idx += sizeof(gyr_bias_q16[0]) * 3;

	memcpy(&sensors_biases[idx], mag_bias_q16, sizeof(mag_bias_q16[0]) * 3);

	if (icm_nvs_inited != ESP_OK)
	{
		ESP32_ICM426XX_nvs_init();
	}
	if (icm_nvs_inited == ESP_OK)
	{
		err = nvs_open(ICM426XX_NVS_NAME, NVS_READWRITE, &handle);
		if (err == ESP_OK)
		{
			ESP_LOGI(LOGNAME, "Successfully openeded NVS with handle: %s", ICM426XX_NVS_NAME);
			size = sizeof(sensors_biases);
			err = nvs_set_blob(handle, ICM426XX_NVS_KEY, sensors_biases, size);
			if (err == ESP_OK)
			{
				err = nvs_commit(handle);
			}
			nvs_close(handle);
		}
		ESP_LOGE(LOGNAME, "%s store in nvs: 0x%04X", ICM426XX_NVS_NAME, err);
	}
	else
	{
		ESP_LOGE(LOGNAME, "%s NVS init: 0x%04X", ICM426XX_NVS_NAME, icm_nvs_inited);
	}
	return err;
}

esp_err_t ESP32_retrieve_stored_biases_from_flash(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3])
{
	esp_err_t err = ESP_FAIL;
	nvs_handle_t handle;
	uint8_t sensors_biases[36] = {0};
	uint8_t idx = 0;
	size_t size;

	if (icm_nvs_inited != ESP_OK)
	{
		ESP32_ICM426XX_nvs_init();
	}
	if (icm_nvs_inited == ESP_OK)
	{
		err = nvs_open(ICM426XX_NVS_NAME, NVS_READWRITE, &handle);
		if (err == ESP_OK)
		{
			ESP_LOGI(LOGNAME, "Successfully openeded NVS with handle: %s", ICM426XX_NVS_NAME);
			size = sizeof(sensors_biases);
			err = nvs_get_blob(handle, ICM426XX_NVS_KEY, sensors_biases, &size);
			nvs_close(handle);
		}
		ESP_LOGE(LOGNAME, "%s read from nvs: 0x%04X", ICM426XX_NVS_NAME, err);
		if (err == ESP_OK)
		{
			memcpy(acc_bias_q16, &sensors_biases[idx], sizeof(acc_bias_q16[0]) * 3);
			idx += sizeof(acc_bias_q16[0]) * 3;

			memcpy(gyr_bias_q16, &sensors_biases[idx], sizeof(gyr_bias_q16[0]) * 3);
			idx += sizeof(gyr_bias_q16[0]) * 3;

			memcpy(mag_bias_q16, &sensors_biases[idx], sizeof(mag_bias_q16[0]) * 3);
		}
	}
	else
	{
		ESP_LOGE(LOGNAME, "%s NVS init: 0x%04X", ICM426XX_NVS_NAME, icm_nvs_inited);
	}
	return err;
}

uint64_t ESP32_get_time_us(void)
{
	struct timeval tv_now;
	gettimeofday(&tv_now, NULL);
	uint64_t time_us = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;
	return time_us;
}

/* --------------------------------------------------------------------------------------
 *  static function declaration
 * -------------------------------------------------------------------------------------- */
static void ICM426XX_apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3]);

/*
 * --------------------------------------------------------------------------------------
 *  ICM426XX SPECIFIC Functions 
 * --------------------------------------------------------------------------------------
 */
static void IRAM_ATTR ICM426XX_irq_cb_default(void *arg)
{
	uint64_t timestamp = ESP32_get_time_us();
	if (!RINGBUFFER_FULL(&timestamp_buffer))
		RINGBUFFER_PUSH(&timestamp_buffer, &timestamp);
	//We increment the signal semaphore, to indicate an interrupt happened.
	xSemaphoreGiveFromISR(irqSem, NULL);
}

esp_err_t ICM426XX_install_Int1_isr(gpio_isr_t int1_isr_handler, inv_icm426xx_interrupt_parameter_t *int_config)
{
	esp_err_t rc = ESP_OK;
	//Setup INT1 gpio
	//interrupt of rising edge
	gpio_config_t io_conf = {0};
	if(int_config->int_pol == ICM426XX_INT_CONFIG_INT1_POLARITY_HIGH)
	{
    	io_conf.intr_type = GPIO_INTR_POSEDGE;
	}
	else
	{
		io_conf.intr_type = GPIO_INTR_NEGEDGE;
	}
    //bit mask of the pins, for INV_GPIO_INT1
    io_conf.pin_bit_mask = CONFIG_ICM426XX_DEV_INT1_GPIO;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    ERR_LOG_FUNC_RET(gpio_config(&io_conf));
	//installing ISR services
	ERR_LOG_FUNC_RET(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT));
    //hook isr handler for specific gpio pin
	if (int1_isr_handler != NULL)
	{
		ERR_LOG_FUNC_RET(gpio_isr_handler_add(INV_GPIO_INT1, int1_isr_handler, (void *)INV_GPIO_INT1));
	}
	else
	{
		ERR_LOG_FUNC_RET(gpio_isr_handler_add(INV_GPIO_INT1, ICM426XX_irq_cb_default, (void *)INV_GPIO_INT1));
	}
	return rc;
}

esp_err_t ICM426XX_whoami(uint8_t *who_am_i)
{
	return inv_icm426xx_read_reg(&icm_driver, MPUREG_WHO_AM_I, 1, who_am_i);
}

int ICM426XX_Configure(uint8_t is_low_noise_mode,
                       uint8_t is_high_res_mode,
                       ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                       ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
						uint8_t is_rtc_mode)
{
	int rc = 0;
	
	if (!is_rtc_mode) {
		/*
		 * Compute the time drift between the MCU and ICM clock
		 */
		rc |= clock_calibration_init(&icm_driver, &clk_calib);
	} else {
		clock_calibration_reset(&icm_driver, &clk_calib);
		clk_calib.coef[INV_ICM426XX_PLL] = 1.0f;
		clk_calib.coef[INV_ICM426XX_RC_OSC] = 1.0f;
		clk_calib.coef[INV_ICM426XX_WU_OSC] = 1.0f;
#if defined(ICM42633) 
		/* 
		 * ICM42633 is a triple interface device. To access CLKIN, AUX2 interface needs to be disabled.
		 * Use INV_ICM426XX_DUAL_INTERFACE mode. The following mode are also compatible:
		 *  - INV_ICM426XX_SINGLE_INTERFACE
		 *  - INV_ICM426XX_DUAL_INTERFACE_SPI4
		 */
		rc |= inv_icm426xx_interface_change_procedure(&icm_driver, INV_ICM426XX_DUAL_INTERFACE);
#endif
	}

	/* 
	 * Force or prevent CLKIN usage depending on example configuration
	 * Note that CLKIN can't be forced if part is not trimmed accordingly
	 * It can be always disabled however, whatever the part used
	 */
	rc |= inv_icm426xx_enable_clkin_rtc(&icm_driver, is_rtc_mode);

	if(is_high_res_mode)
		rc |= inv_icm426xx_enable_high_resolution_fifo(&icm_driver);
	else {
		rc |= inv_icm426xx_set_accel_fsr(&icm_driver, acc_fsr_g);
		rc |= inv_icm426xx_set_gyro_fsr(&icm_driver, gyr_fsr_dps);
	}
	
	rc |= inv_icm426xx_set_accel_frequency(&icm_driver, acc_freq);
	rc |= inv_icm426xx_set_gyro_frequency(&icm_driver, gyr_freq);
	
	if (is_low_noise_mode)
		rc |= inv_icm426xx_enable_accel_low_noise_mode(&icm_driver);
	else
		rc |= inv_icm426xx_enable_accel_low_power_mode(&icm_driver);
	
	rc |= inv_icm426xx_enable_gyro_low_noise_mode(&icm_driver);

	return rc;
}


void ICM426XX_handleFifoPacket_cb(inv_icm426xx_sensor_event_t * event)
{
	uint64_t irq_timestamp = 0, extended_timestamp;
	int32_t accel[3], gyro[3];
	
	/*
	 * Extract the timestamp that was buffered when current packet IRQ fired. See 
	 * ext_interrupt_cb() in main.c for more details.
	 * As timestamp buffer is filled in interrupt handler, we should pop it with
	 * interrupts disabled to avoid any concurrent access.
	 */
	portENTER_CRITICAL(&mux);
	if (!RINGBUFFER_EMPTY(&timestamp_buffer))
		RINGBUFFER_POP(&timestamp_buffer, &irq_timestamp);
	portEXIT_CRITICAL(&mux);
	
	/*
	 * Extend the 16-bit timestamp from the Icm426xx FIFO to a 64 bits timestamp.
	 */
	inv_helper_extend_timestamp_from_fifo(&icm_driver, &clk_calib, event->timestamp_fsync, irq_timestamp, event->sensor_mask, &extended_timestamp);	
	
	/*
	 * Compute raw data according to the format
	 */
	if(icm_driver.fifo_highres_enabled) {
		accel[0] = (((int32_t)event->accel[0] << 4)) | event->accel_high_res[0];
		accel[1] = (((int32_t)event->accel[1] << 4)) | event->accel_high_res[1];
		accel[2] = (((int32_t)event->accel[2] << 4)) | event->accel_high_res[2];
		
		gyro[0] = (((int32_t)event->gyro[0] << 4)) | event->gyro_high_res[0];
		gyro[1] = (((int32_t)event->gyro[1] << 4)) | event->gyro_high_res[1];
		gyro[2] = (((int32_t)event->gyro[2] << 4)) | event->gyro_high_res[2];
		
	} else {
		accel[0] = event->accel[0];
		accel[1] = event->accel[1];
		accel[2] = event->accel[2];
		
		gyro[0] = event->gyro[0];
		gyro[1] = event->gyro[1];
		gyro[2] = event->gyro[2];
	}
	
	ICM426XX_apply_mounting_matrix(icm_mounting_matrix, accel);
	ICM426XX_apply_mounting_matrix(icm_mounting_matrix, gyro);

	/*
	 * Output data on UART link
	 */
	
	if(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL) && event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, %d, %d, %d", (uint32_t)extended_timestamp,
		        accel[0], accel[1], accel[2], 
		        event->temperature,
		        gyro[0], gyro[1], gyro[2]);
	else if(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: NA, NA, NA, %d, %d, %d, %d", (uint32_t)extended_timestamp,
		        event->temperature,
		        gyro[0], gyro[1], gyro[2]);
	else if (event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL))
		INV_MSG(INV_MSG_LEVEL_INFO, "%u: %d, %d, %d, %d, NA, NA, NA", (uint32_t)extended_timestamp,
		        accel[0], accel[1], accel[2],
		        event->temperature);
	
	/* If Queue is full we remove discard the first (oldest) item */
	if(uxQueueSpacesAvailable(xICMeventQ)) {
		agData_t recv;
		xQueueReceive(xICMeventQ, &recv, 2);
	}
	agData_t send = {0};
	send.acc[0] = accel[0];
	send.acc[1] = accel[1];
	send.acc[2] = accel[2];
	send.gyr[0] = gyro[0];
	send.gyr[1] = gyro[1];
	send.gyr[2] = gyro[2];
	send.timestamp = extended_timestamp;
	xQueueSendToBack(xICMeventQ, (void *) &send, 10);
}

int ICM426XX_readFifo(void)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. 
	 * HandleInvDeviceFifoPacket) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	return inv_icm426xx_get_data_from_fifo(&icm_driver);
}

static void ICM426XX_apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3])
{
	unsigned i;
	int64_t data_q30[3];
	
	for(i = 0; i < 3; i++) {
		data_q30[i] =  ((int64_t)matrix[3*i+0] * raw[0]);
		data_q30[i] += ((int64_t)matrix[3*i+1] * raw[1]);
		data_q30[i] += ((int64_t)matrix[3*i+2] * raw[2]);
	}
	raw[0] = (int32_t)(data_q30[0]>>30);
	raw[1] = (int32_t)(data_q30[1]>>30);
	raw[2] = (int32_t)(data_q30[2]>>30);
}

esp_err_t ICM426XX_dev_config_int1(inv_icm426xx_interrupt_parameter_t *intconf)
{
	return inv_icm426xx_set_config_int1(&icm_driver, intconf);
}

esp_err_t ICM426XX_driver_init(struct inv_icm426xx_serif *icm_serif, inv_icm426xx_interrupt_parameter_t *intconf, void (*handleTask)(void *pvParams), TaskHandle_t *sICMtask, char *devName)
{

	esp_err_t rc = 0;
	uint8_t who_am_i;
	
	INFO("##### Initing ICM426xx driver #####");
	ERR_LOG_MSG_RET(inv_icm426xx_init(&icm_driver, icm_serif, ICM426XX_handleFifoPacket_cb), "!!! ERROR : failed to initialize Icm426xx.");
	/* Check WHOAMI */
	INFO("Check Icm426xx whoami value");
	ERR_LOG_MSG_RET(ICM426XX_whoami(&who_am_i), "!!! ERROR : failed to read Icm426xx whoami value.");

	char name[9];
	switch(who_am_i){
		
		case ICM42600_WHOAMI:
			sprintf(name, "ICM42600");
			break;
		case ICM42602_WHOAMI:
			sprintf(name, "ICM42602");
			break;
		case ICM42605_WHOAMI:
			sprintf(name, "ICM42605");
			break;
		case ICM42622_WHOAMI:
			sprintf(name, "ICM42622");
			break;
		case ICM42631_WHOAMI:
			sprintf(name, "ICM42631");
			break;
		case ICM42633_WHOAMI:
			sprintf(name, "ICM42633");
			break;
		case ICM42686_WHOAMI:
			sprintf(name, "ICM42686");
			break;
		case ICM42688_WHOAMI:
			sprintf(name, "ICM42688");
			break;
		case ICM42608_WHOAMI:
			sprintf(name, "ICM42608");
			break;
		default:
			sprintf(name, "unknown");
	}
	INFO("Found device: %s", name);
	if(who_am_i != ICM_WHOAMI) {
		ERROR("!!! ERROR :  detected: %s, driver is configured for: ICM%d!!!", name, CONFIG_ICM426XX_DEV_NAME);
		return ESP_FAIL;
	}
	snprintf(devName, strlen(name)+1, "%s", name);

	RINGBUFFER_CLEAR(&timestamp_buffer);
	return rc;
	ICM426XX_Configure((uint8_t )IS_LOW_NOISE_MODE,
                            (uint8_t )IS_HIGH_RES_MODE,
                            ICM426XX_ACCEL_CONFIG0_FS_SEL_4g,
                            ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps,
                            ICM426XX_ACCEL_CONFIG0_ODR_25_HZ,
                            ICM426XX_GYRO_CONFIG0_ODR_25_HZ,
                            (uint8_t )USE_CLK_IN);
	//Place accelgyro data into IRAM, hopefully
	qStore = heap_caps_malloc(sizeof(agData_t)*qSize, MALLOC_CAP_8BIT);
	xICMeventQ = xQueueCreateStatic(qSize, sizeof(agData_t), qStore, &xQbuf);
	irqSem = xSemaphoreCreateCounting(qSize, 0);
	ERR_LOG_MSG_RET(inv_icm426xx_get_config_int1(&icm_driver, intconf), "Error reading interrupt configuration");
	#ifdef CONFIG_FREERTOS_UNICORE
	xTaskCreate(handleTask, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, CONFIG_ICM426XX_TASK_PRIORITY, sICMtask);
	#else
	xTaskCreatePinnedToCore(handleTask, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, CONFIG_ICM426XX_TASK_PRIORITY, sICMtask, CONFIG_ICM426XX_TASK_CORE_AFFINITY);
	#endif //#ifdef CONFIG_FREERTOS_UNICORE
	vTaskSuspend(sICMtask);
}