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

typedef struct agData {
  int32_t acc[3];
  int32_t gyr[3];
  uint64_t timestamp;
} agData_t;


typedef struct icm426xx_fifo_conf {
  uint8_t enable;
  uint8_t read_partial_en;
  uint8_t acc_en;
  uint8_t gyr_en;
  uint8_t temp_en;
  uint8_t th_int_en;
  uint8_t tmst_fsync_en;
  uint8_t hires_en;
  uint8_t mode;
  ICM426XX_INTF_CONFIG0_FIFO_COUNT_REC_t rec_type;
  uint16_t threshold_cnt;
} icm426xx_fifo_conf_t;

typedef struct ag_sensor_data {
  uint16_t tmstp;
  int32_t accel[3];
  int32_t gyro[3];
  int16_t temp;         // Temperature in 0.1 °C precision
} ag_sensor_data_t;

typedef struct ag_buffer {
  int err;
  uint64_t timestamp;
  uint16_t pcnt;
  uint8_t highres;
  ag_sensor_data_t *data;
} ag_buffer_t;

typedef enum {
  INTR_ICM426XX_NONE = 0,
	INTR_ICM426XX_UI_FSYNC = 1,
  INTR_ICM426XX_PLL_RDY,
  INTR_ICM426XX_RESET_DONE,
	INTR_ICM426XX_UI_DRDY,
	INTR_ICM426XX_FIFO_THS,
	INTR_ICM426XX_FIFO_FULL,
  INTR_ICM426XX_AGC_RDY,
	INTR_ICM426XX_SMD,
	INTR_ICM426XX_WOM_X,
	INTR_ICM426XX_WOM_Y,
	INTR_ICM426XX_WOM_Z,
	INTR_ICM426XX_STEP_DET,
	INTR_ICM426XX_STEP_CNT_OVFL,
	INTR_ICM426XX_TILT_DET,
#if defined(ICM_FAMILY_BPLUS) 
	INTR_ICM426XX_SLEEP_DET,
	INTR_ICM426XX_WAKE_DET,
#elif defined(ICM_FAMILY_CPLUS)
	INTR_ICM426XX_LOWG_DET,
	INTR_ICM426XX_FF_DET,
#endif
    INTR_ICM426XX_TAP_DET
  }ICM426XX_intr_status_t;

#define ICM426XX_NUM_INTRS 14
#if defined(ICM_FAMILY_BPLUS) 
#define ICM426XX_NUM_INTRS 17
#elif defined(ICM_FAMILY_CPLUS)
#define ICM426XX_NUM_INTRS 17
#endif


#define ICM426XX_ADDR_AD0_LO 0x68
#define ICM426XX_ADDR_AD0_HI 0x69

//global value of init status. Used by components's NVS functions, and initialized as ESP_FAIL;
esp_err_t icm_nvs_inited;

StaticQueue_t xQbuf;
QueueHandle_t xICMeventQ;
uint8_t *qStore;
SemaphoreHandle_t irqSem;
agData_t *agDataBuf;

extern uint8_t ICM426XX_devAddr;
/* Just a handy variable to handle the icm426xx object */
struct inv_icm426xx icm_driver;

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


/**
 * @brief   Debug helper functions to dump most registers from Bank0 and Bank1
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_dumpBank0Regs(void);
esp_err_t ICM426XX_dumpBank1Regs(void);
/*
 * --------------------------------------------------------------------------------------
 *  ICM426XX SPECIFIC Functions 
 * --------------------------------------------------------------------------------------
 */


/**
 * @brief   Installs GPIO interrupt to pin configured in Menuconfig Components->ICM426XX driver
 * 
 * @param   int1_isr_handler  You can pass your own interrupt handler here. If you pass this as NULL,
 *                             Driver default is used.
 * @param   int_config        Interrupt configuration struct. Used to determine int signal polirity and stuff
 * 
 * @note    Default isr does this:
 *          It reads system time (getTimeOfDay()) and stores it in a ring buffer.
 *          If also increments a counting semaphore: irqSem.
 *          This semaphore then can be used to control a task to read device for information and data
 * 
 * @return  ESP_OK succesfull, otherwise error
 */
esp_err_t ICM426XX_install_Int1_isr(gpio_isr_t int1_isr_handler, inv_icm426xx_interrupt_parameter_t *int_config);

/**
 * @brief	Reads interrupt status registers from ICM device. When an interrupt cause is found,
 * 			the function will write it to the buffer passed as the first argument.
 * 			When the function finished, the function set the num_ints argumetn with the number of interrupts found.
 * 
 * @param	statuses - Buffer pointer to store found interrupt flags
 * @param	bufsize	- size of the buffer pointed to statuses argument
 * @param	num_ints - Pointer to a byte variable, which will store the number of the found interrupts.
 * 						The number returned in with this argument is not dependent of the size of the buffer.
 * 						If the buffer is too small to hold all interrupt events, users can check which
 * 						events might have occured, because the intterupt flags are read sequentially.
 * 						The last interrupt flag represents the last in the sequence, which guarantees that
 * 						all interrupt events preceeding that flag has been checked and reported.
 * 
 * @return ESP_OK - Bus was read correctly, data is valid
 * 					Everything else returned are correct errors form the bus transaction.
 * 						
 * @example ICM426XX_intr_status_t intstats[ICM426XX_NUM_INTRS];
 * 			uint8_t intnum;
 * 			ICM426XX_get_intr_events(&intstats[0], ICM426XX_NUM_INTRS, &intnum);
 * 			
 * 			for(int i = 0; i < intnum, i++){
 * 				switch(instats[i]) {
 * 					case INTR_ICM426XX_UI_FSYNC:
 * 					.
 * 					.
 * 				}
 * 			}
 * 			.
 * 			.
*/
int ICM426XX_get_intr_events(ICM426XX_intr_status_t *statuses, int bufsize);

/**
 * @brief Until this function is called, not of the sensors should work.
 *        Explanation.
 *	      According to the datasheet, the default value of SENSOR_CONFIG0
 *	      is 0x00 after reset. However on my device the default reset value is 0x42.
 *	      If you write 0x00 SENSOR_CONFIG0, the device start to fail to comminicate
 *	      and weird stuff will happen. THIS IS NOT DOCUMENTED
 *        Instead of writing 0x00, we only set the lower 6 bits of it, to enable all sensors
 *        If this is not done, then Fifo will not work correctly, interrupts won't happen, etc.
 */
esp_err_t ICM426XX_sensor_en_all(void);

/**
 * @brief These two functions below convert raw register data into measurement units.
 *        For accelerometer, it convert to milligee-s(mg), for gyro it converts to dps (degrees/second)
 * 
 * @note  These functions are called by ICM426XX_readFifo() if convert argument is set to 1.
 */
esp_err_t ICM426XX_convert_accel2mg(int32_t acc[3]);
esp_err_t ICM426XX_convert_gyro2dps(int32_t gyr[3]);

/**
 * @brief Reads out fifo from device in burst-read mode (fastest)
 *        Returns a pointer to an ag_buffer_t struct which contains information
 *        and pointer to sensor data.
 * @param converter   0 - sensor data is raw, as read from the device
 *                    1 - Sensor data is converted to mg (milligee) and dps(degrees/second)
 * @return  Pointer to buffer or NULL if something is wrong
 *          A returned pointer has a member ->errno, which should be checked.
 *          If ->errno is ESP_ERR_NOMEM, then the sensor data buffer is not created and fifo is not read.
 *          In this case ->data is also NULL.
 *          If ->errno equals ESP_FAIL, communication error happened.
 */
ag_buffer_t* ICM426XX_readFifo(uint8_t converted);

/**
 * @brief Frees a previously created buffer struct and it's ->data member
 */
void ICM426XX_delete_ag_buffer(ag_buffer_t *buffer);

/**
 * @brief   Sets Fifo threshold in ICM device registers. If you are using FIFO-THS interrupt,
 *          this will set, how many records (or bytes) in Fifo shall trigger an interrupt
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_set_fifo_threshold(uint16_t threshold);


/**
 *  @brief 	Prints to console the interrupt configuration passed to the function
 *  @note	  This function does not read it from the device
 * 	@param 	intconf - interrupt configuration to be printed.
 * 	@param	devType	- 5digit type number
 * 	@param	loglevel - Log level of the function
 */
esp_err_t ICM426XX_print_dev_int1_config(uint32_t devType, esp_log_level_t level);

/**
 *  @brief 	Setter and getter functions for interrupt configurations of the ICM device.
 *
 * 	@param 	intconf - interrupt configuration to be printed.
 * 	
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_get_dev_int1_config(inv_icm426xx_interrupt_parameter_t *intconf);
esp_err_t ICM426XX_set_dev_int1_config(inv_icm426xx_interrupt_parameter_t *intconf);

/**
 * @brief   Software resets the ICM device
 * @note    Remember, that even though you reset your ESP, the ICM device does give a f***
 *          This can be called manually, and also called during init
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_dev_reset(void);

/**
 * @brief   Sets the ICM devices I2C address. You must call this manually before init.
 *          Useful if you want to scan your I2C bus for a device before initializing the driver,
 *          because the ICM device address can be change, depending on how you tie the address pins
 */
void ICM426XX_set_I2C_addr(uint8_t addr);

/**
 * @brief   Initializes driver struct, resets ICM device and configures it for communication
 * @note    If I2C is used for communication, ICM426XX_set_I2C_addr() must be called.
 * 
 * @param   icm_serif   Serif struct to set up serial communication
 * @param   devName     This is a string pointer given back by the init function, will contain the full device name.
 *                      The length must be at least 9 characters (8 characters+NULL)
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_driver_init(struct inv_icm426xx_serif *icm_serif, char *devName);

/**
 * @brief   Configures device with operation parameters
 * @note    ICM426XX_driver_init() must be called before this.
 * 
 * @param   is_low_nise_mode   Sets sensors to low noise (LN) mode
 * @param   is_high_res_mode   Sets measurement resolution to high_res (check datasheet)
 *                             In case of FIFO, data will be 20bits instead of 16bits.
 *                             Fifo functions handle this.
 * @param   acc_fsr_g   Full scale range of accelerometer
 * @param   gyr_fsr_dps Full scale range of gyroscope
 * @param   acc_freq    Accelerometer ODR frequency
 * @param   gyr_freq    Gyro ODR frequency
 * @param   is_rtc_mode Some ICM devices support external clocking. Can be used to activate that feature
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
int ICM426XX_ConfigureUseFifo(uint8_t is_low_noise_mode,
                       uint8_t is_high_res_mode,
                       ICM426XX_ACCEL_CONFIG0_FS_SEL_t acc_fsr_g,
                       ICM426XX_GYRO_CONFIG0_FS_SEL_t gyr_fsr_dps,
                       ICM426XX_ACCEL_CONFIG0_ODR_t acc_freq,
                       ICM426XX_GYRO_CONFIG0_ODR_t gyr_freq,
						uint8_t is_rtc_mode);

/**
 * @brief   Creates driver task, according to the parameters set in Menuconfig Components->ICM426XX driver
 * 
 * @param   handleTask   Function pointer to driver task function
 * @param   sICMTaskk    TaskHandle pointer passed by user. This can be used later to reference
 *                       the created driver task
 * @param   suspend      If it is set to 1, task is created in a suspended state. Call vTaskResume()
 *                       with the task handle to resume driver task
 * @return  ESP_OK if task is created successfully, ESP_FAIL otherwise
 */
esp_err_t ICM426XX_create_driver_task(void (*handleTask)(void *pvParams), TaskHandle_t *sICMtask, int suspend);

/**
 * @brief  Function wrappers for device read and write function to be used in user code if needed
 * 
 * @param   reg   Register address to be read/written. If len is more than 1, this is the start address.
 * @param   len   Length of the read/write
 * @param   buf   Buffer pointer. For read, read data is stored in the buffer pointed by buf.
 *                Write writes the content in buf to target register(s)
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_readreg(uint8_t reg, uint32_t len, uint8_t * buf);
esp_err_t ICM426XX_writereg(uint8_t reg, uint32_t len, uint8_t * buf);

/**
 * @brief   Function to change register banks. ICM devices have more register banks (typically 4)
 *          Read datasheet for more details
 * 
 * @param   bank   Bank number to be set
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_set_reg_bank(uint8_t bank);

/**
 * @brief  Reads device ID (WHO_AM_I) register. This is independent of bank setting.
 * 
 * @param   who_am_i   Pointer to byte in which the read value is stored
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_whoami(uint8_t *who_am_i);

/**
 * @brief  Resets device FIFO, which clears all data in it.
 * 
 * @return  ESP_OK if there was NO comminication error, some error code otherwise.
 *          Error returned is only related to communication issues.
 */
esp_err_t ICM426XX_resetFifo(void);

/* --------------------------------------------------------------------------------------
 *  DEPRECATED//untested
 * --------------------------------------------------------------------------------------
*/
//ag_buffer_t ICM426XX_readFifo(void);
//void ICM426XX_handleFifoPacket_cb(inv_icm426xx_sensor_event_t * event);
//esp_err_t ICM426XX_configure_fifo(icm426xx_fifo_conf_t *fifocfg);
//esp_err_t ICM426XX_use_fsync(int useFsync);
#endif //#ifndef __ICM426XX_H__
