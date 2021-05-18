/* 07/13/2019 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Dragonfly default), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.
  The ICM42605 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef __ICM426XX_H__
#define __ICM426XX_H__

#include <stdint.h>
#include "sdkconfig.h"
#include "I2Cdev.h"
#include "esp_err.h"
#include "driver/gpio.h"

#include "Icm426xxDefs.h"
#include "Icm426xxTransport.h"
#include "Icm426xxDriver_HL.h"
#include "Icm426xxDriver_HL_apex.h"
#include "Icm426xxSelfTest.h"
#include "Ak0991x.h"
#include "Ak0991xSerif.h"

#include "helperClockCalib.h"
#include "Message.h"
#include "RingBuffer.h"
#include "ErrorHelper.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

//#include "logmacro.h"

/*
 * --------------------------------------------------------------------------------------
 *  ICM426XX Driver configurations
 * --------------------------------------------------------------------------------------
 */
/* 
 * Select communication link between SmartMotion and ICM426xx 
 */
#ifdef CONFIG_ICM426XX_USE_I2C
#define SERIF_TYPE ICM426XX_UI_I2C
#elif defined CONFIG_ICM426XX_USE_SPI4
#define SERIF_TYPE ICM426XX_UI_SPI4
#endif //#ifdef CONFIG_ICM426XX_USE_I2C

/*
 * Set this define to 0 to disable mag support
 * Recommended value: 1
 */
#define USE_MAG              0

/*
 * Set power mode flag
 * Set this flag to run example in low-noise mode.
 * Reset this flag to run example in low-power mode.
 * Note : low-noise mode is not available with sensor data frequencies less than 12.5Hz.
 */
#define IS_LOW_NOISE_MODE    1

/* 
 * Set this to 0 if you want to test timestamping mechanism without CLKIN 32k capability.
 * Please set a hardware bridge between PA17 (from MCU) and CLKIN pins (to ICM).
 * Warning: This option is not available for all ICM426XX. Please check the datasheet.
 */
#define USE_CLK_IN 0

/*
 * Accelerometer and gyroscope frequencies.
 * Recommended value: ICM426XX_GYRO_CONFIG0_ODR_1_KHZ and ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ (1000 Hz)
 * Possible values (same for accel, replace "GYRO" with "ACCEL"): 
 * - ICM426XX_GYRO_CONFIG0_ODR_1_KHZ  (1000 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_500_HZ (500 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_200_HZ (200 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_100_HZ (100 Hz)
 * - ICM426XX_GYRO_CONFIG0_ODR_50_HZ (50 Hz)
 */
#define GYRO_FREQ            ICM426XX_GYRO_CONFIG0_ODR_1_KHZ
#define ACCEL_FREQ           ICM426XX_ACCEL_CONFIG0_ODR_1_KHZ

/*
 * Select Fifo resolution Mode (default is low resolution mode)
 * Low resolution mode : 16 bits data format
 * High resolution mode : 20 bits data format
 * Warning: Enabling High Res mode will force FSR to 16g and 2000dps
 */
#define IS_HIGH_RES_MODE 0
/*
* Set INT1 interrupt pin
*/
#define INV_GPIO_INT1 CONFIG_ICM426XX_DEV_INT1_GPIO

/*
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_INFO

StaticQueue_t xQbuf;
QueueHandle_t xICMeventQ;
uint8_t *qStore;
SemaphoreHandle_t irqSem;


typedef struct agData {
  int32_t acc[3];
  int32_t gyr[3];
  uint64_t timestamp;
} agData_t;

agData_t *agDataBuf;

//global value of init status. Used by components's NVS functions, and initialized as ESP_FAIL;
esp_err_t icm_nvs_inited;
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
int ESP32_HAL_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);

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
int ESP32_HAL_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);

/**
 * @brief Initializes chosen serial interface. For I2C it inits an I2CDev driver.
 *        Before use, configure it using MENUCONFIG
 * @param serif:  We only use ICM426XX_SERIAL_IF_TYPE_t serif_type to brach for the properly
 * 
 * @return: 0: OK
 *          Everything return that is not 0, means an error, normally -1.
 */
int ESP32_icm_serif_init(struct inv_icm426xx_serif *serif);

/**
 * \brief Write sensor biases into Flash using NVS-Flash API
 * \note	This function automatically initializes NVS Api on first call.
 * \param[in] acc_bias_q16 acc bias to be written
 * \param[in] gyr_bias_q16 gyr bias to be written
 * \param[in] mag_bias_q16 mag bias to be written
 */
esp_err_t ESP32_store_biases_in_flash(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3]);

/**
 * \brief Reads sensor biases from Flash using NVS-Flash API
 * \note	This function automatically initializes NVS Api on first call.
 * \param[in] acc_bias_q16 acc bias to be written
 * \param[in] gyr_bias_q16 gyr bias to be written
 * \param[in] mag_bias_q16 mag bias to be written
 */
esp_err_t ESP32_retrieve_stored_biases_from_flash(int32_t acc_bias_q16[3], int32_t gyr_bias_q16[3], int32_t mag_bias_q16[3]);

//Helper to get microsecond time
uint64_t ESP32_get_time_us(void);

esp_err_t ICM426XX_install_Int1_isr(gpio_isr_t int1_isr_handler, inv_icm426xx_interrupt_parameter_t *int_config);

esp_err_t ICM426XX_whoami(uint8_t *who_am_i);

int ICM426XX_Configure(uint8_t is_low_noise_mode,
                       uint8_t is_high_res_mode,
                       ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                       ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
						uint8_t is_rtc_mode);

void ICM426XX_handleFifoPacket_cb(inv_icm426xx_sensor_event_t * event);

int ICM426XX_readFifo(void);

esp_err_t ICM426XX_dev_config_int1(inv_icm426xx_interrupt_parameter_t *intconf);

esp_err_t ICM426XX_driver_init(struct inv_icm426xx_serif *icm_serif, inv_icm426xx_interrupt_parameter_t *intconf, void (*handleTask)(void *pvParams), TaskHandle_t *sICMtask, char *devName);

#endif //#ifndef __ICM426XX_H__
