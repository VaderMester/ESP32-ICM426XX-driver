/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The ICM426XX is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "nvs_flash.h"

//#include "Message.h"

#define ICM426XX_NVS_NAME "ICM426XXNVS"
#define ICM426XX_NVS_KEY "biases"

#include "ICM426XX.h"
#include "time.h"
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sntp.h"
#include "esp_log.h"
#include "malloc_local.h"

/* Helper macros to help better error checking */
#if CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0
#define ERR_RET(val) do{ int v = (val); if(v != 0) {return (v);}}while(0)
#define ERR_LOG_FUNC(val) do{ int v = (val); if(v) {ESP_LOG_LEVEL((esp_log_level_t)CONFIG_ICM426XX_DEBUG_LOG_LEVEL, LOGNAME, "Error in %s(%d): %d", __FUNCTION__, __LINE__, v);}} while(0)
#define ERR_LOG_FUNC_RET(val) do {int v = (val); if(v) {ESP_LOG_LEVEL((esp_log_level_t)CONFIG_ICM426XX_DEBUG_LOG_LEVEL, LOGNAME, "Error in %s(%d): %d", __FUNCTION__, __LINE__, v); return v;}}while(0)
#define ERR_LOG_MSG_RET(val, fmt, ...) do { int v = (val); if(v) {ESP_LOG_LEVEL((esp_log_level_t)CONFIG_ICM426XX_DEBUG_LOG_LEVEL, LOGNAME, fmt, ## __VA_ARGS__); return v;}}while(0)
#elif CONFIG_ICM426XX_DEBUG_LOG_LEVEL == 0
#define ERR_RET(val) (val)
#define ERR_LOG_FUNC(val) (val)
#define ERR_LOG_FUNC_RET(val) (val)
#define ERR_LOG_MSG_RET(val, fmt, ...) (val)
#endif // CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0

#define RAD_TO_DEG(rad) ((float)rad * 57.2957795131)
#define FLOAT_TO_INT(x) (((x)>=0)?(int)((x)+0.5):(int)((x)-0.5))

#define ESP_INTR_FLAG_DEFAULT 0

#define ICM426XX_BUS_ADDR CONFIG_ICM426XX_DEV_ADDRESS

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

/* structure allowing to handle clock calibration */
static clk_calib_t clk_calib;

/* Buffer to keep track of the timestamp when icm426xx data ready interrupt fires. */
RINGBUFFER(timestamp_buffer, 64, uint64_t);

uint8_t intcnt = 0;
uint8_t ICM426XX_devAddr = 0;

/* MUX for entering critical section */
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;


//const uint8_t qSize = 10;
const uint8_t qSize = 2;

//Helper for iterating through interrupt stauts bits
static const uint8_t intr_helper[ICM426XX_NUM_INTRS] = {
	BIT_INT_STATUS_UI_FSYNC,
	BIT_INT_STATUS_PLL_RDY,
	BIT_INT_STATUS_RESET_DONE,
	BIT_INT_STATUS_DRDY,
	BIT_INT_STATUS_FIFO_THS,
	BIT_INT_STATUS_FIFO_FULL,
	BIT_INT_STATUS_AGC_RDY,
	BIT_INT_STATUS2_SMD_INT,
	BIT_INT_STATUS2_WOM_Z_INT,
	BIT_INT_STATUS2_WOM_Y_INT,
	BIT_INT_STATUS2_WOM_X_INT,
	BIT_INT_STATUS3_STEP_DET,
	BIT_INT_STATUS3_STEP_CNT_OVFL,
	BIT_INT_STATUS3_TILT_DET,
#if defined(ICM_FAMILY_BPLUS)
	BIT_INT_STATUS3_WAKE_DET,
	BIT_INT_STATUS3_SLEEP_DET,
#elif defined(ICM_FAMILY_CPLUS)
	BIT_INT_STATUS3_LOWG_DET,
	BIT_INT_STATUS3_FF_DET,
#endif
	BIT_INT_STATUS3_TAP_DET
};

#if CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0
#define BYTE_TO_BINARY_PATTERN "b%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0') 

const static char icm426xx_bank0regstr[28][19] = {
	"INTF_FIFO_COUNTH",
	"INTF_FIFO_COUNTL",
	"INTF_CONFIG0\t",
	"INTF_CONFIG1\t",
	"PWR_MGMT_0\t",
	"GYRO_CONFIG0\t",
	"ACCEL_CONFIG0\t",
	"GYRO_CONFIG1\t",
	"ACCEL_GYRO_CONFIG0",
	"ACCEL_CONFIG1\t",
	"TMST_CONFIG\t",
	"APEX_CONFIG0\t",
	"SMD_CONFIG\t",
	"FIFO_CONFIG1\t",
	"FIFO_CONFIG2\t",
	"FIFO_CONFIG3\t",
	"FSYNC_CONFIG\t",
	"INT_CONFIG0\t",
	"INT_CONFIG1\t",
	"INT_SOURCE0\t",
	"INT_SOURCE1\t",
	"INT_SOURCE2\t",
	"INT_SOURCE3\t",
	"INT_SOURCE4\t",
	"INT_SOURCE5\t",
	"FIFO_LOST_PKT0\t",
	"SELF_TEST_CONFIG",
	"WHO_AM_I\t"};

const static uint8_t icm426xx_bank0reg[28] = {
	MPUREG_FIFO_COUNTH,
	MPUREG_FIFO_COUNTL,
	MPUREG_INTF_CONFIG0,
	MPUREG_INTF_CONFIG1,
	MPUREG_PWR_MGMT_0,
	MPUREG_GYRO_CONFIG0,
	MPUREG_ACCEL_CONFIG0,
	MPUREG_GYRO_CONFIG1,
	MPUREG_ACCEL_GYRO_CONFIG0,
	MPUREG_ACCEL_CONFIG1,
	MPUREG_TMST_CONFIG,
	MPUREG_APEX_CONFIG0,
	MPUREG_SMD_CONFIG,
	MPUREG_FIFO_CONFIG1,
	MPUREG_FIFO_CONFIG2,
	MPUREG_FIFO_CONFIG2+1,
	MPUREG_FSYNC_CONFIG,
	MPUREG_INT_CONFIG0,
	MPUREG_INT_CONFIG1,
	MPUREG_INT_SOURCE0,
	MPUREG_INT_SOURCE1,
	MPUREG_INT_SOURCE2,
	MPUREG_INT_SOURCE3,
	MPUREG_INT_SOURCE4,
	MPUREG_INT_SOURCE5,
	MPUREG_FIFO_LOST_PKT0,
	MPUREG_SELF_TEST_CONFIG,
	MPUREG_WHO_AM_I};

const static char icm426xx_bank1regstr[18][19] = {
	"SENSOR_CONFIG0\t",
	"GYRO_CONF_STATIC2",
	"GYRO_CONF_STATIC3",
	"GYRO_CONF_STATIC4",
	"GYRO_CONF_STATIC5",
	"GYRO_CONF_STATIC6",
	"GYRO_CONF_STATIC7",
	"GYRO_CONF_STATIC8",
	"GYRO_CONF_STATIC9",
	"GYRO_CONF_STATIC10",
	"XG_ST_DATA\t",
	"YG_ST_DATA\t",
	"ZG_ST_DATA\t",
	"TMST_VAL0\t",
	"TMST_VAL1\t",
	"TMST_VAL2\t",
	"INTF_CONFIG4\t",
	"INTF_CONFIG5\t"};

const static uint8_t icm426xx_bank1reg[18] = {
	MPUREG_SENSOR_CONFIG1_B1,
	MPUREG_GYRO_CONFIG_STATIC2_B1,
	MPUREG_GYRO_CONFIG_STATIC2_B1+1,
	MPUREG_GYRO_CONFIG_STATIC2_B1+2,
	MPUREG_GYRO_CONFIG_STATIC2_B1+3,
	MPUREG_GYRO_CONFIG_STATIC2_B1+4,
	MPUREG_GYRO_CONFIG_STATIC2_B1+5,
	MPUREG_GYRO_CONFIG_STATIC2_B1+6,
	MPUREG_GYRO_CONFIG_STATIC2_B1+7,
	MPUREG_GYRO_CONFIG_STATIC2_B1+8,
	MPUREG_XG_ST_DATA_B1,
	MPUREG_YG_ST_DATA_B1,
	MPUREG_ZG_ST_DATA_B1,
	MPUREG_TMST_VAL0_B1,
	MPUREG_TMST_VAL0_B1+1,
	MPUREG_TMST_VAL0_B1+2,
	MPUREG_INTF_CONFIG4_B1,
	MPUREG_INTF_CONFIG5_B1};
#endif //#if CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0

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
uint64_t
inv_icm426xx_get_time_us(void)
{
	return ESP32_get_time_us();
}

/*
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 * ESP32 implementation uses the system time do small delays.
 * This is going to burden the CPU, so for delays longer than
 * half of the FreeRTOS tick period, we use osSleep() macro, which calls vTaskDelay.
 */
void inv_icm426xx_sleep_us(uint32_t us)
{
	uint64_t end = ESP32_get_time_us() + us; //call this here to get a more accurate start of delay;
	if (1000 >= us)
	{
		while(end > ESP32_get_time_us());
	}
	else
	{
		osSleep((TickType_t)(us / 1000));
	}
}

/*
 * --------------------------------------------------------------------------------------
 *  ESP32 SPECIFIC Functions 
 * --------------------------------------------------------------------------------------
 */

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
 *  USEFUL FOR DEBUG
 * --------------------------------------------------------------------------------------
*/

esp_err_t ICM426XX_dumpBank0Regs(void)
{
	#if CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0
	uint8_t data;
	printf("**** BANK0 REGISTERS ****\n");
	ERR_LOG_FUNC_RET(ICM426XX_set_reg_bank(0));
	for (int i = 0; i < sizeof(icm426xx_bank0reg); i++)
	{
		data = 0xFF;
		ERR_LOG_FUNC_RET(ICM426XX_readreg(icm426xx_bank0reg[i], 1, &data));
		printf("%s(0x%.2X) - 0x%.2X - " BYTE_TO_BINARY_PATTERN "\n", icm426xx_bank0regstr[i], icm426xx_bank0reg[i], data, BYTE_TO_BINARY(data));
	}
	return ESP_OK;
	#elif CONFIG_ICM426XX_DEBUG_LOG_LEVEL == 0
	return ESP_OK;
	#endif //CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0
}

esp_err_t ICM426XX_dumpBank1Regs(void)
{
	#if CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0
	uint8_t data;
	printf("**** BANK1 REGISTERS ****\n");
	ERR_LOG_FUNC_RET(ICM426XX_set_reg_bank(1));
	for (int i = 0; i < sizeof(icm426xx_bank1reg); i++)
	{
		data = 0xFF;
		ERR_LOG_FUNC_RET(ICM426XX_readreg(icm426xx_bank1reg[i], 1, &data));
		printf("%s(0x%.2X) - 0x%.2X - " BYTE_TO_BINARY_PATTERN "\n", icm426xx_bank1regstr[i], icm426xx_bank1reg[i], data, BYTE_TO_BINARY(data));
	}
	ERR_LOG_FUNC_RET(ICM426XX_set_reg_bank(0));
	return ESP_OK;
	#elif CONFIG_ICM426XX_DEBUG_LOG_LEVEL == 0
	return ESP_OK;
	#endif //CONFIG_ICM426XX_DEBUG_LOG_LEVEL > 0
}


/* --------------------------------------------------------------------------------------
 *  static function declaration
 * --------------------------------------------------------------------------------------
*/

static void ICM426XX_apply_mounting_matrix(const int32_t matrix[9], int32_t raw[3]);
static void IRAM_ATTR ICM426XX_irq_cb_default(void *arg);

/*
 * --------------------------------------------------------------------------------------
 *  ICM426XX SPECIFIC Functions 
 * --------------------------------------------------------------------------------------
 */
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
    //bit mask of the pin&icm_driver, for INV_GPIO_INT1
    io_conf.pin_bit_mask = (1ULL << INV_GPIO_INT1);
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

	RINGBUFFER_CLEAR(&timestamp_buffer);
	return rc;
}

int ICM426XX_get_intr_events(ICM426XX_intr_status_t *statuses, int bufsize)
{
	if(!statuses){
		ESP_LOGE(LOGNAME, "%s(%d) - interrupt statuses array pointer is NULL", __FUNCTION__, __LINE__);
	}
	uint8_t int_stat[3];
	ERR_LOG_FUNC_RET(inv_icm426xx_read_reg(&icm_driver, MPUREG_INT_STATUS, 1, &int_stat[0]));
	ERR_LOG_FUNC_RET(inv_icm426xx_read_reg(&icm_driver, MPUREG_INT_STATUS2, 2, &int_stat[1])); //reads both STAUS2 and STATUS3 in sequence
	ESP_LOGD(LOGNAME, "INT_STATUSx: 0x%02X, 0x%02X, 0x%02X", int_stat[0], int_stat[1], int_stat[2]);
	uint8_t j = 0;
	uint8_t a = 0;
	for(int i = 0; i < ICM426XX_NUM_INTRS; i++)
	{
		if(i == 7) a = 1;
		if(i == 12) a = 2;
		if(int_stat[a] & intr_helper[i])
		{
				if(j < bufsize) {
				statuses[j] = (ICM426XX_intr_status_t)(i+1);
				}
				j++;
		}
	}
	return j;
}

esp_err_t ICM426XX_whoami(uint8_t *who_am_i)
{
	return inv_icm426xx_read_reg(&icm_driver, MPUREG_WHO_AM_I, 1, who_am_i);
}

esp_err_t ICM426XX_readreg(uint8_t reg, uint32_t len, uint8_t * buf)
{
	return inv_icm426xx_read_reg(&icm_driver, reg, len, buf);
}

esp_err_t ICM426XX_writereg(uint8_t reg, uint32_t len, uint8_t * buf)
{
	return inv_icm426xx_write_reg(&icm_driver, reg, len, buf);
}


esp_err_t ICM426XX_set_reg_bank(uint8_t bank)
{
	return inv_icm426xx_set_reg_bank(&icm_driver, bank);
}

esp_err_t ICM426XX_sensor_en_all(void)
{
	//Activate all sensor axes
	/*
	*	This few lines ow a little explanation.
	*	According to the datasheet, the default value of SENSOR_CONFIG0
	*	is 0x00 after reset. (In the driver the sensor name is wrong)
	*	However on my device the default reset value is 0x42.
	*	If you write 0x00 SENSOR_CONFIG0, the device start to fail to comminicate
	*	And weird stuff will happen. THIS IS NOT DOCUMENTED
	*	Instead of writing 0x00, we only set the lower 6 bits of it, to enable all sensors
	*/
	uint8_t data = 0;
	ERR_LOG_FUNC_RET(ICM426XX_set_reg_bank(1));
	ERR_LOG_FUNC_RET(inv_icm426xx_read_reg(&icm_driver, MPUREG_SENSOR_CONFIG1_B1, 1, &data));
	data &= (uint8_t)~0x3F;
	ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_SENSOR_CONFIG1_B1, 1, &data));

	ERR_LOG_FUNC_RET(ICM426XX_set_reg_bank(0));
	return ESP_OK;
}

int ICM426XX_ConfigureUseFifo(uint8_t is_low_noise_mode,
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

esp_err_t ICM426XX_resetFifo(void)
{
	return inv_icm426xx_reset_fifo(&icm_driver);
}

esp_err_t ICM426XX_convert_accel2mg(int32_t acc[3])
{
	float scaler, val;
	switch (icm_driver.accel_fsr)
	{
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_2g:
		scaler = 16.384;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_4g:
		scaler = 8.192;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_8g:
		scaler = 4.096;
		break;
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_16g:
		scaler = 2.048;
		break;
#ifdef ICM42686
	case ICM426XX_ACCEL_CONFIG0_FS_SEL_32g:
		scaler = 1.024;
		break;
#endif
	default:
		ESP_LOGD(LOGNAME, "Conversion error: Accel FSR not set");
		return ESP_FAIL;
	}
	for (int i = 0; i < 3; i++)
	{
		val = acc[i] / scaler;
		acc[i] = FLOAT_TO_INT(val);
	}
	return ESP_OK;
}

esp_err_t ICM426XX_convert_gyro2dps(int32_t gyr[3])
{
	float scaler, val;
	switch (icm_driver.gyro_fsr)
	{
	case ICM426XX_GYRO_CONFIG0_FS_SEL_16dps:
		scaler = 2048;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_31dps:
		scaler = 1024;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_62dps:
		scaler = 528.52;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_250dps:
		scaler = 131.072;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_500dps:
		scaler = 65.536;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_1000dps:
		scaler = 32.768;
		break;
	case ICM426XX_GYRO_CONFIG0_FS_SEL_2000dps:
		scaler = 16.384;
		break;
#ifdef ICM42686
	case ICM426XX_GYRO_CONFIG0_FS_SEL_4000dps:
		scaler = 8.192;
		break;
#endif
	default:
		ESP_LOGD(LOGNAME, "Conversion error: Accel FSR not set");
		return ESP_FAIL;
	}
	for (int i = 0; i < 3; i++)
	{
		val = gyr[i] / scaler;
		gyr[i] = FLOAT_TO_INT(val);
	}
	return ESP_OK;
}

void ICM426XX_delete_ag_buffer(ag_buffer_t *buffer)
{
	if (buffer)
	{
		if(buffer->data) free_local(buffer->data);
		free_local(buffer);
	}
}

ag_buffer_t* ICM426XX_readFifo(uint8_t converted)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. 
	 * HandleInvDeviceFifoPacket) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	int status = 0;
	uint8_t data[2];
	uint16_t packet_count_i, packet_count = 0;
	uint16_t packet_size = FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
	uint64_t irq_timestamp = 0, extended_timestamp;
	fifo_header_t *header;

	ag_buffer_t *ret = malloc_local(sizeof(ag_buffer_t));
	if (ret == NULL)
		return NULL;
	
	ret->err = 0;
	ag_sensor_data_t *dp;

	/* FIFO record mode configured at driver init, so we read packet number, not byte count */
	status = ICM426XX_set_reg_bank(0);
	status |= inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_COUNTH, 2, data);
	if (status != INV_ERROR_SUCCESS)
	{
		ERR_LOG_FUNC(status);
		ret->err = status;
		return ret;
	}
	inv_icm426xx_format_data(ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN, data, &packet_count);
	if (packet_count > 0)
	{
		ret->pcnt = (uint16_t)packet_count;
		/* Read FIFO only when data is expected in FIFO */
		/* fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE */
		uint16_t fifo_idx = 0;

		if (icm_driver.fifo_highres_enabled)
			packet_size = FIFO_20BYTES_PACKET_SIZE;

		ret->data = malloc_local(sizeof(ag_sensor_data_t) * packet_count);
		if (ret->data == NULL)
		{
			ret->err = ESP_ERR_NO_MEM;
			return ret;
		}
		dp = ret->data;

		if (icm_driver.transport.serif.serif_type == ICM426XX_UI_I3C)
		{
			/* in case of I3C, need to read packet by packet since INT is embedded on protocol so this can 
				happen that FIFO read is interrupted to handle IBI, and in that case FIFO is partially read.
				To handle this, 2 solution :
				- handle fifo lost packet & partial read
				- read packet by packet
				2nd solution prefered here because less heavy from driver point of view but it is less optimal
				for the timing because we have to initiate N transactions in any case */
			for (packet_count_i = 0; packet_count_i < packet_count; packet_count_i++)
			{
				if (!inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_DATA, packet_size, &icm_driver.fifo_data[packet_count_i * packet_size]))
				{
					/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
							  reset FIFO and try next chance */
					inv_icm426xx_reset_fifo(&icm_driver);
					ret->err = ESP_FAIL;
					return ret;
				}
			}
		}
		else
		{
			status |= inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_DATA, packet_size * packet_count, icm_driver.fifo_data);
			if (status)
			{
				/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
						  reset FIFO and try next chance */
				inv_icm426xx_reset_fifo(&icm_driver);
				ERR_LOG_FUNC(status);
				ret->err = status;
				return ret;
			}
		}

		for (packet_count_i = 0; packet_count_i < packet_count; packet_count_i++)
		{
			inv_icm426xx_sensor_event_t event;
			event.sensor_mask = 0;

			header = (fifo_header_t *)&(icm_driver.fifo_data[fifo_idx]);
			fifo_idx += FIFO_HEADER_SIZE;

			/* Decode packet */
			if (header->bits.msg_bit)
			{
				/* MSG BIT set in FIFO header, Resetting FIFO */
				inv_icm426xx_reset_fifo(&icm_driver);
				ERR_LOG_FUNC(status);
				ret->err = INV_ERROR;
				return ret;
			}

			if (header->bits.accel_bit)
			{
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[0 + fifo_idx]), (uint16_t *)&event.accel[0]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[2 + fifo_idx]), (uint16_t *)&event.accel[1]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[4 + fifo_idx]), (uint16_t *)&event.accel[2]);
				fifo_idx += FIFO_ACCEL_DATA_SIZE;
			}

			if (header->bits.gyro_bit)
			{
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[0 + fifo_idx]), (uint16_t *)&event.gyro[0]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[2 + fifo_idx]), (uint16_t *)&event.gyro[1]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[4 + fifo_idx]), (uint16_t *)&event.gyro[2]);
				fifo_idx += FIFO_GYRO_DATA_SIZE;
			}

			if ((header->bits.accel_bit) || (header->bits.gyro_bit))
			{
				if (header->bits.twentybits_bit)
				{
					inv_icm426xx_format_data(icm_driver.endianess_data, &icm_driver.fifo_data[0 + fifo_idx], (uint16_t *)&event.temperature);
					fifo_idx += FIFO_TEMP_DATA_SIZE + FIFO_TEMP_HIGH_RES_SIZE;

					/* new temperature data */
					if (event.temperature != INVALID_VALUE_FIFO)
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_TEMPERATURE);
				}
				else
				{
					event.temperature = (int8_t)icm_driver.fifo_data[0 + fifo_idx]; /* cast to int8_t since FIFO is in 16 bits mode (temperature on 8 bits) */
					fifo_idx += FIFO_TEMP_DATA_SIZE;

					/* new temperature data */
					if (event.temperature != INVALID_VALUE_FIFO_1B)
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_TEMPERATURE);
				}
			}

			if ((header->bits.timestamp_bit) || (header->bits.fsync_bit))
			{
				inv_icm426xx_format_data(icm_driver.endianess_data, &icm_driver.fifo_data[0 + fifo_idx], (uint16_t *)&event.timestamp_fsync);
				fifo_idx += FIFO_TS_FSYNC_SIZE;

				/* new fsync event */
				/* First FSYNC event after enable is irrelevant
					 * FSYNC tag and FSYNC data should be ignored on the first ODR after restart.
					 */
				if (header->bits.fsync_bit)
				{
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					if (icm_driver.fsync_to_be_ignored == 0)
#endif
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_FSYNC_EVENT);
				}
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
				icm_driver.fsync_to_be_ignored = 0;
#endif
			}

			if (header->bits.accel_bit)
			{
				if ((event.accel[0] != INVALID_VALUE_FIFO) &&
					(event.accel[1] != INVALID_VALUE_FIFO) &&
					(event.accel[2] != INVALID_VALUE_FIFO))
				{

					if (header->bits.twentybits_bit)
					{
						event.accel_high_res[0] = (icm_driver.fifo_data[0 + fifo_idx] >> 4) & 0xF;
						event.accel_high_res[1] = (icm_driver.fifo_data[1 + fifo_idx] >> 4) & 0xF;
						event.accel_high_res[2] = (icm_driver.fifo_data[2 + fifo_idx] >> 4) & 0xF;
					}

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					if (icm_driver.accel_start_time_us == UINT32_MAX)
					{
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
					}
					else
					{
						if (!header->bits.fsync_bit)
						{
							/* First data are noisy after enabling sensor
								 * Compare start time with current time to notify the event
								 */
							if ((inv_icm426xx_get_time_us() - icm_driver.accel_start_time_us) >= ICM426XX_ACC_STARTUP_TIME_US)
							{
								icm_driver.accel_start_time_us = UINT32_MAX;
								event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
							}
						}
					}
#else
					event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
#endif
				}
			}

			if (header->bits.gyro_bit)
			{
				if ((event.gyro[0] != INVALID_VALUE_FIFO) &&
					(event.gyro[1] != INVALID_VALUE_FIFO) &&
					(event.gyro[2] != INVALID_VALUE_FIFO))
				{

					if (header->bits.twentybits_bit)
					{
						event.gyro_high_res[0] = (icm_driver.fifo_data[0 + fifo_idx]) & 0xF;
						event.gyro_high_res[1] = (icm_driver.fifo_data[1 + fifo_idx]) & 0xF;
						event.gyro_high_res[2] = (icm_driver.fifo_data[2 + fifo_idx]) & 0xF;
					}

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					if (icm_driver.gyro_start_time_us == UINT32_MAX)
					{
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
					}
					else
					{
						if (!header->bits.fsync_bit)
						{
							/* First data are noisy after enabling sensor
								 * Compare start time with current time to notify the event
								 */
							if ((inv_icm426xx_get_time_us() - icm_driver.gyro_start_time_us) >= ICM426XX_GYR_STARTUP_TIME_US)
							{
								icm_driver.gyro_start_time_us = UINT32_MAX;
								event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
							}
						}
					}
#else
					event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
#endif
				}
			}

			if (header->bits.twentybits_bit)
				fifo_idx += FIFO_ACCEL_GYRO_HIGH_RES_SIZE;


			//portENTER_CRITICAL(&mux);
			if (!RINGBUFFER_EMPTY(&timestamp_buffer))
				RINGBUFFER_POP(&timestamp_buffer, &irq_timestamp);
			//portEXIT_CRITICAL(&mux);

			/*
			 * Extend the 16-bit timestamp from the Icm426xx FIFO to a 64 bits timestamp.
			*/
			inv_helper_extend_timestamp_from_fifo(&icm_driver, &clk_calib, event.timestamp_fsync, irq_timestamp, event.sensor_mask, &extended_timestamp);

			/*
	 		 * Compute raw data according to the format
			 */
			if (icm_driver.fifo_highres_enabled)
			{
				dp->accel[0] = (((int32_t)event.accel[0] << 4)) | event.accel_high_res[0];
				dp->accel[1] = (((int32_t)event.accel[1] << 4)) | event.accel_high_res[1];
				dp->accel[2] = (((int32_t)event.accel[2] << 4)) | event.accel_high_res[2];

				dp->gyro[0] = (((int32_t)event.gyro[0] << 4)) | event.gyro_high_res[0];
				dp->gyro[1] = (((int32_t)event.gyro[1] << 4)) | event.gyro_high_res[1];
				dp->gyro[2] = (((int32_t)event.gyro[2] << 4)) | event.gyro_high_res[2];
			}
			else
			{
				dp->accel[0] = event.accel[0];
				dp->accel[1] = event.accel[1];
				dp->accel[2] = event.accel[2];

				dp->gyro[0] = event.gyro[0];
				dp->gyro[1] = event.gyro[1];
				dp->gyro[2] = event.gyro[2];
			}
			dp->temp = event.temperature;
			dp->tmstp = event.timestamp_fsync;
			ICM426XX_apply_mounting_matrix(icm_mounting_matrix, dp->accel);
			ICM426XX_apply_mounting_matrix(icm_mounting_matrix, dp->gyro);

			if (converted)
			{
				ICM426XX_convert_accel2mg(dp->accel);
				ICM426XX_convert_gyro2dps(dp->gyro);
				dp->temp = (event.temperature * 5) + 250; //Make temperature to XX.Y°C
			}
			//increment inv_icm426xx_sensor_event_t *p pointer
			dp++;
		} /* end of FIFO read for loop */
	}
	/*else: packet_count was 0*/

	return ret;
}

esp_err_t ICM426XX_get_dev_int1_config(inv_icm426xx_interrupt_parameter_t *intconf) {
	return inv_icm426xx_get_config_int1(&icm_driver, intconf);
}


esp_err_t ICM426XX_set_dev_int1_config(inv_icm426xx_interrupt_parameter_t *intconf)
{
	return inv_icm426xx_set_config_int1(&icm_driver, intconf);
}

esp_err_t ICM426XX_print_dev_int1_config(uint32_t devType, esp_log_level_t level)
{
	inv_icm426xx_interrupt_parameter_t intconf = {0};
	ICM426XX_get_dev_int1_config(&intconf);
		ESP_LOG_LEVEL(level, LOGNAME, "************** ICM%d CURRENT INTERRUPT CONFIGURATION *************", devType);
		ESP_LOG_LEVEL(level, LOGNAME, "\tFSYNC: \t%s", intconf.INV_ICM426XX_UI_FSYNC ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tDATA RDY: \t%s", intconf.INV_ICM426XX_UI_DRDY ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tFIFO THS: \t%s", intconf.INV_ICM426XX_FIFO_THS ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tFIFO FULL: \t%s", intconf.INV_ICM426XX_FIFO_FULL ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tSMD: \t%s", intconf.INV_ICM426XX_SMD ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tWOM_X: \t%s", intconf.INV_ICM426XX_WOM_X ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tWOM_Y: \t%s", intconf.INV_ICM426XX_WOM_Y ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tWOM_Z: \t%s", intconf.INV_ICM426XX_WOM_Z ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tSTEP DET: \t%s", intconf.INV_ICM426XX_STEP_DET ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tSTEP COUNT OVERFLOW: \t%s", intconf.INV_ICM426XX_STEP_CNT_OVFL ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tTILT: \t%s", intconf.INV_ICM426XX_TILT_DET ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tSLEEP: \t%s", intconf.INV_ICM426XX_SLEEP_DET ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tWAKE: \t%s", intconf.INV_ICM426XX_WAKE_DET ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "\tTAP: \t%s", intconf.INV_ICM426XX_TAP_DET ? "Y" : "N");
		ESP_LOG_LEVEL(level, LOGNAME, "Circuit:");
		ESP_LOG_LEVEL(level, LOGNAME, "\tPOLARITY: \t%s", intconf.int_pol ? "ACT. HIGH" : "ACT. LOW");
		ESP_LOG_LEVEL(level, LOGNAME, "\tWAKE: \t%s", intconf.int_drive ? "Push-Pull" : "Open-Drain");
		ESP_LOG_LEVEL(level, LOGNAME, "\tTAP: \t%s", intconf.int_mode ? "Latched" : "Pulsed");
		ESP_LOG_LEVEL(level, LOGNAME, "****************** END OF INTERRUPT CONFIGURATION *******************");
	return ESP_OK;
}

esp_err_t ICM426XX_dev_reset(void)
{
	return inv_icm426xx_device_reset(&icm_driver);
}

void ICM426XX_set_I2C_addr(uint8_t addr)
{
	ICM426XX_devAddr = addr;
}

esp_err_t ICM426XX_driver_init(struct inv_icm426xx_serif *icm_serif, char *devName)
{
	//esp_err_t rc = 0;
	if(ICM426XX_devAddr == 0) {
		ESP_LOGE(LOGNAME, "ERROR: I2C address not set!");
		return ESP_FAIL;
	}
	uint8_t who_am_i;
	ESP_LOGI(LOGNAME, "##### Initing ICM426xx driver, at address 0x%.2x #####", ICM426XX_devAddr);
	ERR_LOG_MSG_RET(inv_icm426xx_init(&icm_driver, icm_serif, NULL), "!!! ERROR : failed to initialize Icm426xx.");
	/* Check WHOAMI */
	ESP_LOGI(LOGNAME, "Check Icm426xx whoami value");
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
	ESP_LOGI(LOGNAME, "Found device: %s", name);
	if(who_am_i != ICM_WHOAMI) {
		ESP_LOGE(LOGNAME, "!!! ERROR :  detected: %s, driver is configured for: ICM%d!!!", name, CONFIG_ICM426XX_DEV_NAME);
		return ESP_FAIL;
	}
	snprintf(devName, strlen(name)+1, "%s", name);

	return ESP_OK;
}

esp_err_t ICM426XX_set_fifo_threshold(uint16_t threshold)
{
	return inv_icm426xx_configure_fifo_wm(&icm_driver, threshold);
}

esp_err_t ICM426XX_create_driver_task(void (*handleTask)(void *pvParams), TaskHandle_t *sICMtask, int suspend)
{	
	qStore = malloc_local(sizeof(ag_buffer_t*) * qSize);
	xICMeventQ = xQueueCreateStatic(qSize, sizeof(ag_buffer_t*), qStore, &xQbuf);
	irqSem = xSemaphoreCreateCounting(qSize, 0);
	BaseType_t stat;
#ifdef CONFIG_FREERTOS_UNICORE
	stat = xTaskCreate(handleTask, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, CONFIG_ICM426XX_TASK_PRIORITY, sICMtask);
#else
	stat = xTaskCreatePinnedToCore(handleTask, CONFIG_ICM426XX_TASK_NAME, CONFIG_ICM426XX_TASK_STACK_SIZE, NULL, CONFIG_ICM426XX_TASK_PRIORITY, sICMtask, CONFIG_ICM426XX_TASK_CORE_AFFINITY);
#endif //#ifdef CONFIG_FREERTOS_UNICORE
	if (stat == pdPASS)
	{
		if (suspend)
		{
			vTaskSuspend(*sICMtask);
			ESP_LOGI(LOGNAME, "Task called '%s' created in suspended state!", CONFIG_ICM426XX_TASK_NAME);
		}
		else
		{
		ESP_LOGI(LOGNAME, "Task called '%s' created in running state!", CONFIG_ICM426XX_TASK_NAME);
		}
		return ESP_OK;
	}
	return ESP_FAIL;
}


/* --------------------------------------------------------------------------------------
 *  DEPRECATED
 * --------------------------------------------------------------------------------------
*/

#if 0
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
	//portENTER_CRITICAL(&mux);
	if (!RINGBUFFER_EMPTY(&timestamp_buffer))
		RINGBUFFER_POP(&timestamp_buffer, &irq_timestamp);
	//portEXIT_CRITICAL(&mux);
	
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
	/*
	if(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL) && event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))
	{
		ESP_LOGI(LOGNAME, "%u: %d, %d, %d, %d, %d, %d, %d", (uint32_t)extended_timestamp,
		        accel[0], accel[1], accel[2], 
		        event->temperature,
		        gyro[0], gyro[1], gyro[2]);
	}
	else if(event->sensor_mask & (1 << INV_ICM426XX_SENSOR_GYRO))
	{
		ESP_LOGI(LOGNAME,  "%u: NA, NA, NA, %d, %d, %d, %d", (uint32_t)extended_timestamp,
		        event->temperature,
		        gyro[0], gyro[1], gyro[2]);
	}
	else if (event->sensor_mask & (1 << INV_ICM426XX_SENSOR_ACCEL))
	{
		ESP_LOGI(LOGNAME,  "%u: %d, %d, %d, %d, NA, NA, NA", (uint32_t)extended_timestamp,
		        accel[0], accel[1], accel[2],
		        event->temperature);
	}
	*/
	/* If Queue is full we remove discard the first (oldest) item */
	if(uxQueueSpacesAvailable(xICMeventQ) == 0) {
		agData_t recv;
		xQueueReceive(xICMeventQ, &recv, 1);
	}
	agData_t send = {0};
	send.acc[0] = accel[0];
	send.acc[1] = accel[1];
	send.acc[2] = accel[2];
	send.gyr[0] = gyro[0];
	send.gyr[1] = gyro[1];
	send.gyr[2] = gyro[2];
	send.timestamp = extended_timestamp;
	xQueueSendToBack(xICMeventQ, (void *) &send, 2);
}

int ICM426XX_readFifo(void)
{
	/*
	 * Extract packets from FIFO. Callback defined at init time (i.e. 
	 * HandleInvDeviceFifoPacket) will be called for each valid packet extracted from 
	 * FIFO.
	 */
	int status = 0;
	uint8_t data[2];
	uint16_t packet_count_i, packet_count = 0;
	uint16_t packet_size = FIFO_HEADER_SIZE + FIFO_ACCEL_DATA_SIZE + FIFO_GYRO_DATA_SIZE + FIFO_TEMP_DATA_SIZE + FIFO_TS_FSYNC_SIZE;
	fifo_header_t *header;

	/* FIFO record mode configured at driver init, so we read packet number, not byte count */
	status = ICM426XX_set_reg_bank(0);
	status |= inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_COUNTH, 2, data);
	if (status != INV_ERROR_SUCCESS)
		return status;
	inv_icm426xx_format_data(ICM426XX_INTF_CONFIG0_DATA_LITTLE_ENDIAN, data, &packet_count);
	if (packet_count > 0)
	{
		/* Read FIFO only when data is expected in FIFO */
		/* fifo_idx type variable must be large enough to parse the FIFO_MIRRORING_SIZE */
		uint16_t fifo_idx = 0;

		if (icm_driver.fifo_highres_enabled)
			packet_size = FIFO_20BYTES_PACKET_SIZE;

		if (icm_driver.transport.serif.serif_type == ICM426XX_UI_I3C)
		{
			/* in case of I3C, need to read packet by packet since INT is embedded on protocol so this can 
				happen that FIFO read is interrupted to handle IBI, and in that case FIFO is partially read.
				To handle this, 2 solution :
				- handle fifo lost packet & partial read
				- read packet by packet
				2nd solution prefered here because less heavy from driver point of view but it is less optimal
				for the timing because we have to initiate N transactions in any case */
			for (packet_count_i = 0; packet_count_i < packet_count; packet_count_i++)
			{
				if(!inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_DATA, packet_size, &icm_driver.fifo_data[packet_count_i * packet_size]))
				{
					/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
							  reset FIFO and try next chance */
					inv_icm426xx_reset_fifo(&icm_driver);
					return status;
				}
			}
		}
		else
		{
			status |= inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_DATA, packet_size * packet_count, icm_driver.fifo_data);
			if (status)
			{
				/* sensor data is in FIFO according to FIFO_COUNT but failed to read FIFO,
						  reset FIFO and try next chance */
				inv_icm426xx_reset_fifo(&icm_driver);
				return status;
			}
		}

		for (packet_count_i = 0; packet_count_i < packet_count; packet_count_i++)
		{
			inv_icm426xx_sensor_event_t event;
			event.sensor_mask = 0;

			header = (fifo_header_t *)&(icm_driver.fifo_data[fifo_idx]);
			fifo_idx += FIFO_HEADER_SIZE;

			/* Decode packet */
			if (header->bits.msg_bit)
			{
				/* MSG BIT set in FIFO header, Resetting FIFO */
				inv_icm426xx_reset_fifo(&icm_driver);
				return INV_ERROR;
			}

			if (header->bits.accel_bit)
			{
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[0 + fifo_idx]), (uint16_t *)&event.accel[0]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[2 + fifo_idx]), (uint16_t *)&event.accel[1]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[4 + fifo_idx]), (uint16_t *)&event.accel[2]);
				fifo_idx += FIFO_ACCEL_DATA_SIZE;
			}

			if (header->bits.gyro_bit)
			{
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[0 + fifo_idx]), (uint16_t *)&event.gyro[0]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[2 + fifo_idx]), (uint16_t *)&event.gyro[1]);
				inv_icm426xx_format_data(icm_driver.endianess_data, &(icm_driver.fifo_data[4 + fifo_idx]), (uint16_t *)&event.gyro[2]);
				fifo_idx += FIFO_GYRO_DATA_SIZE;
			}

			if ((header->bits.accel_bit) || (header->bits.gyro_bit))
			{
				if (header->bits.twentybits_bit)
				{
					inv_icm426xx_format_data(icm_driver.endianess_data, &icm_driver.fifo_data[0 + fifo_idx], (uint16_t *)&event.temperature);
					fifo_idx += FIFO_TEMP_DATA_SIZE + FIFO_TEMP_HIGH_RES_SIZE;

					/* new temperature data */
					if (event.temperature != INVALID_VALUE_FIFO)
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_TEMPERATURE);
				}
				else
				{
					event.temperature = (int8_t)icm_driver.fifo_data[0 + fifo_idx]; /* cast to int8_t since FIFO is in 16 bits mode (temperature on 8 bits) */
					fifo_idx += FIFO_TEMP_DATA_SIZE;

					/* new temperature data */
					if (event.temperature != INVALID_VALUE_FIFO_1B)
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_TEMPERATURE);
				}
			}

			if ((header->bits.timestamp_bit) || (header->bits.fsync_bit))
			{
				inv_icm426xx_format_data(icm_driver.endianess_data, &icm_driver.fifo_data[0 + fifo_idx], (uint16_t *)&event.timestamp_fsync);
				fifo_idx += FIFO_TS_FSYNC_SIZE;

				/* new fsync event */
				/* First FSYNC event after enable is irrelevant
					 * FSYNC tag and FSYNC data should be ignored on the first ODR after restart.
					 */
				if (header->bits.fsync_bit)
				{
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					if (icm_driver.fsync_to_be_ignored == 0)
#endif
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_FSYNC_EVENT);
				}
#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
				icm_driver.fsync_to_be_ignored = 0;
#endif
			}

			if (header->bits.accel_bit)
			{
				if ((event.accel[0] != INVALID_VALUE_FIFO) &&
					(event.accel[1] != INVALID_VALUE_FIFO) &&
					(event.accel[2] != INVALID_VALUE_FIFO))
				{

					if (header->bits.twentybits_bit)
					{
						event.accel_high_res[0] = (icm_driver.fifo_data[0 + fifo_idx] >> 4) & 0xF;
						event.accel_high_res[1] = (icm_driver.fifo_data[1 + fifo_idx] >> 4) & 0xF;
						event.accel_high_res[2] = (icm_driver.fifo_data[2 + fifo_idx] >> 4) & 0xF;
					}

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					if (icm_driver.accel_start_time_us == UINT32_MAX)
					{
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
					}
					else
					{
						if (!header->bits.fsync_bit)
						{
							/* First data are noisy after enabling sensor
								 * Compare start time with current time to notify the event
								 */
							if ((inv_icm426xx_get_time_us() - icm_driver.accel_start_time_us) >= ICM426XX_ACC_STARTUP_TIME_US)
							{
								icm_driver.accel_start_time_us = UINT32_MAX;
								event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
							}
						}
					}
#else
					event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_ACCEL);
#endif
				}
			}

			if (header->bits.gyro_bit)
			{
				if ((event.gyro[0] != INVALID_VALUE_FIFO) &&
					(event.gyro[1] != INVALID_VALUE_FIFO) &&
					(event.gyro[2] != INVALID_VALUE_FIFO))
				{

					if (header->bits.twentybits_bit)
					{
						event.gyro_high_res[0] = (icm_driver.fifo_data[0 + fifo_idx]) & 0xF;
						event.gyro_high_res[1] = (icm_driver.fifo_data[1 + fifo_idx]) & 0xF;
						event.gyro_high_res[2] = (icm_driver.fifo_data[2 + fifo_idx]) & 0xF;
					}

#if (!INV_ICM426XX_LIGHTWEIGHT_DRIVER)
					if (icm_driver.gyro_start_time_us == UINT32_MAX)
					{
						event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
					}
					else
					{
						if (!header->bits.fsync_bit)
						{
							/* First data are noisy after enabling sensor
								 * Compare start time with current time to notify the event
								 */
							if ((inv_icm426xx_get_time_us() - icm_driver.gyro_start_time_us) >= ICM426XX_GYR_STARTUP_TIME_US)
							{
								icm_driver.gyro_start_time_us = UINT32_MAX;
								event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
							}
						}
					}
#else
					event.sensor_mask |= (1 << INV_ICM426XX_SENSOR_GYRO);
#endif
				}
			}

			if (header->bits.twentybits_bit)
				fifo_idx += FIFO_ACCEL_GYRO_HIGH_RES_SIZE;

			/* call sensor event callback */
			if (icm_driver.sensor_event_cb)
				icm_driver.sensor_event_cb(&event);

		} /* end of FIFO read for loop */
	}
	/*else: packet_count was 0*/

	return packet_count;
}


esp_err_t ICM426XX_configure_fifo(icm426xx_fifo_conf_t *fifocfg)
{
	esp_err_t ret = ESP_OK;
	uint8_t data;
	inv_icm426xx_interrupt_parameter_t config_int1 = {(inv_icm426xx_interrupt_value)0};
	inv_icm426xx_interrupt_parameter_t config_int2 = {(inv_icm426xx_interrupt_value)0};
	inv_icm426xx_interrupt_parameter_t config_ibi  = {(inv_icm426xx_interrupt_value)0};
	
	icm_driver.fifo_is_used = (INV_ICM426XX_FIFO_CONFIG_t)fifocfg->enable;
	ret = ICM426XX_set_reg_bank(0);

	switch (icm_driver.fifo_is_used)
	{
		case INV_ICM426XX_FIFO_ENABLED :
			/* Configure:
			 * - FIFO record mode i.e FIFO count unit is packet 
			 * - FIFO snapshot mode i.e drop the data when the FIFO overflows
			 * - Timestamp is logged in FIFO
			 * - Little Endian fifo_count
			*/
			ERR_LOG_FUNC_RET(inv_icm426xx_read_reg(&icm_driver, MPUREG_INTF_CONFIG0, 1, &data));
			ESP_LOGI(LOGNAME, "INTF_CONFIG0: Read: 0x%02X", data);

			data |= (uint8_t)(fifocfg->rec_type);
			data |= (uint8_t)BIT_FIFO_SREG_INVALID_IND_MASK;	//FIFO_HOLD_LAST_DATA_EN
			data &= (uint8_t)~BIT_FIFO_COUNT_ENDIAN_MASK; // little endian
			ESP_LOGI(LOGNAME, "INTF_CONFIG0: Writing: 0x%02X", data);
			ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_INTF_CONFIG0, 1, &data));

			data = (uint8_t)fifocfg->mode;
			ESP_LOGI(LOGNAME, "FIFO_CONFIG(mode): Writing: 0x%02X", data);
			ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_FIFO_CONFIG, 1, &data));

			ERR_LOG_FUNC_RET(inv_icm426xx_read_reg(&icm_driver, MPUREG_TMST_CONFIG, 1, &data));
			ESP_LOGI(LOGNAME, "TMST_CONFIG: Read: 0x%02X", data);
			data |= ICM426XX_TMST_CONFIG_TMST_EN;
			ESP_LOGI(LOGNAME, "TMST_CONFIG: Written: 0x%02X", data);
			ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_TMST_CONFIG, 1, &data));

			/* restart and reset FIFO configuration */
			data = 0;
			if(fifocfg->acc_en) data |= BIT_FIFO_CONFIG1_ACCEL_MASK;
			if(fifocfg->gyr_en) data |= BIT_FIFO_CONFIG1_GYRO_MASK;
			if(fifocfg->temp_en) data |= BIT_FIFO_CONFIG1_TEMP_MASK;
			//if(fifocfg->hires_en) data |= BIT_FIFO_CONFIG1_HIRES_MASK;
			if(fifocfg->tmst_fsync_en) data |= BIT_FIFO_CONFIG1_TMST_FSYNC_MASK;
			if(fifocfg->th_int_en) data |= BIT_FIFO_CONFIG1_WM_GT_TH_MASK;
			if(fifocfg->read_partial_en) data |= BIT_FIFO_CONFIG1_RESUME_PARTIAL_RD_MASK;
			ESP_LOGI(LOGNAME, "FIFO_CONFIG1: Written: 0x%02X", data);
			ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_FIFO_CONFIG1, 1, &data));
			/* Configure FIFO WM count which causes the interrupt to trigger */
			ERR_LOG_FUNC_RET(inv_icm426xx_configure_fifo_wm(&icm_driver, fifocfg->threshold_cnt));

			config_int1.INV_ICM426XX_FIFO_THS = INV_ICM426XX_ENABLE;
			inv_icm426xx_set_config_int1(&icm_driver, &config_int1);

			config_int2.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
			inv_icm426xx_set_config_int2(&icm_driver, &config_int2);

			config_ibi.INV_ICM426XX_FIFO_THS = INV_ICM426XX_DISABLE;
			inv_icm426xx_set_config_ibi(&icm_driver, &config_ibi);

#if defined(ICM_FAMILY_CPLUS)
			/* Set FIFO decimation rate to 0 */
			ERR_LOG_FUNC_RET(inv_icm426xx_set_fifo_dec_rate(&icm_driver, 0));
#endif
			ret = ESP_OK;
			break;
		
		case INV_ICM426XX_FIFO_DISABLED :
			/* make sure FIFO is disabled */
			data = ICM426XX_FIFO_CONFIG_MODE_BYPASS;
			ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_FIFO_CONFIG, 1, &data));
			
			/* restart and reset FIFO configuration */
			ERR_LOG_FUNC_RET(inv_icm426xx_read_reg(&icm_driver, MPUREG_FIFO_CONFIG1, 1, &data));
			data &= (uint8_t)~(BIT_FIFO_CONFIG1_GYRO_MASK | BIT_FIFO_CONFIG1_ACCEL_MASK);
			data |= (BIT_FIFO_CONFIG1_TEMP_MASK | BIT_FIFO_CONFIG1_TMST_FSYNC_MASK);
			ERR_LOG_FUNC_RET(inv_icm426xx_write_reg(&icm_driver, MPUREG_FIFO_CONFIG1, 1, &data));
			
			/* Enable Data Ready Interrupt */
			ERR_LOG_FUNC_RET(inv_icm426xx_get_config_int1(&icm_driver, &config_int1));
			config_int1.INV_ICM426XX_UI_DRDY = INV_ICM426XX_ENABLE;
			ERR_LOG_FUNC_RET(inv_icm426xx_set_config_int1(&icm_driver, &config_int1));
			ret = ESP_OK;
			break;

		default :
			ret = ESP_ERR_INVALID_ARG;
	}
	return ret;
}

esp_err_t ICM426XX_use_fsync(int useFsync)
{
	if (useFsync)
	{
		ERR_LOG_MSG_RET(inv_icm426xx_enable_fsync(&icm_driver), "Error when trying to disable Fsync usage");
	}
	ERR_LOG_MSG_RET(inv_icm426xx_disable_fsync(&icm_driver), "Error when trying to enable Fsync usage");
	return ESP_OK;
}
#endif