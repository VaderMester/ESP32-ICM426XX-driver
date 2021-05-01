/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */


#include "example-pedometer.h"

/* InvenSense utils */
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"
#include "uart_mngr.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"

#include "system-interface.h"

/* std */
#include <stdio.h>


/* --------------------------------------------------------------------------------------
 *  Example configuration
 * -------------------------------------------------------------------------------------- */

/*
 * Select UART port on which INV_MSG() will be printed.
 */
#define LOG_UART_ID INV_UART_SENSOR_CTRL

/* 
 * Select communication link between SmartMotion and ICM426xx 
 */
#define SERIF_TYPE ICM426XX_UI_SPI4
// #define SERIF_TYPE ICM426XX_UI_I2C

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

/* 
 * Set of timers used throughout standalone applications 
 */
#define TIMEBASE_TIMER INV_TIMER1
#define DELAY_TIMER    INV_TIMER2

/*
 * Pedometer frequency 
 * Use type ICM426XX_APEX_CONFIG0_DMP_ODR_t to define pedometer frequency
 * These types are defined in Icm426xxDefs.h.
 *
 * \note The frequency modes to run the Pedometer are :
 * ICM426XX_APEX_CONFIG0_DMP_ODR_25Hz  (Low Power mode), 
 * ICM426XX_APEX_CONFIG0_DMP_ODR_50Hz  (Normal mode)
 */
#define ICM_PEDOMETER_FREQUENCY_MODE ICM426XX_APEX_CONFIG0_DMP_ODR_25Hz

/*
 * Pedometer power save mode
 * Use type ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_t to define pedometer power save mode
 * These types are defined in Icm426xxDefs.h.
 */
#define ICM_PEDOMETER_POWER_SAVE_MODE ICM426XX_APEX_CONFIG0_DMP_POWER_SAVE_EN
 
/*
 * Pedometer programmable parameters 
 */
 
/* 
 * Peak threshold value to be considered as a valid step (mg) 
 * Use type ICM426XX_APEX_CONFIG2_PEDO_AMP_TH_t to define the valid step peak threshold
 * These types are defined in Icm426xxDefs.h
 */
#define ICM_PEDOMETER_VALID_STEP_THRESHOLD ICM426XX_APEX_CONFIG2_PEDO_AMP_TH_62MG 	
		
/*
 * Minimum number of steps that must be detected before the pedometer step count begins incrementing 
 */
#define ICM_PEDOMETER_STEP_COUNTER_THRESHOLD 5

/* 
 * Duration of non-walk in number of samples to exit the current walk mode.
 * ICM_PEDOMETER_STEP_COUNTER_THRESHOLD number of steps above must again be detected before step count starts to increase
 * Use type ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_t to define the non-walk duration
 * These types are defined in Icm426xxDefs.h
 *
 * \note The recommended values according to the frequency mode selected are :
 * ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_100_SAMPLES in Low Power mode
 * ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_150_SAMPLES in Normal mode
 */
#define ICM_PEDOMETER_NON_WALK_DURATION ICM426XX_APEX_CONFIG3_PEDO_SB_TIMER_TH_100_SAMPLES

/* 
 * Minimum number of low latency steps that must be detected before the pedometer step count begins incrementing 
 */
#define ICM_PEDOMETER_STEP_DETECTOR_THRESHOLD 2

/*
 * Sensitivity mode: Normal
 * Use type ICM426XX_APEX_CONFIG9_SENSITIVITY_MODE_t to define sensitivity mode 
 * These types are defined in Icm426xxDefs.h.
 */
#define ICM_PEDOMETER_SENSITIVITY_MODE ICM426XX_APEX_CONFIG9_SENSITIVITY_MODE_NORMAL


/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

/* 
 * Buffer to keep track of the timestamp when icm426xx data ready interrupt fires.
 * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
 */
RINGBUFFER(timestamp_buffer, 64, uint64_t);



/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from icm426xx device irq handler */
static volatile int irq_from_device;


/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif);
static void ext_interrupt_inv_cb(void * context, unsigned int int_num);
void check_rc(int rc, const char * msg_context);
void msg_printer(int level, const char * str, va_list ap);


/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int main(void)
{
	int rc = 0;
    struct inv_icm426xx_serif icm426xx_serif;

	/* Initialize MCU hardware */
	SetupMCUHardware(&icm426xx_serif);

	/* Initialize Icm426xx */
	rc = SetupInvDevice(&icm426xx_serif);
	check_rc(rc, "error while setting up INV device");
	
	/* Configure Icm426xx */
	rc = ConfigureInvDevice(ICM_PEDOMETER_FREQUENCY_MODE,
							ICM_PEDOMETER_POWER_SAVE_MODE,
							ICM_PEDOMETER_VALID_STEP_THRESHOLD,
							ICM_PEDOMETER_STEP_COUNTER_THRESHOLD,
							ICM_PEDOMETER_NON_WALK_DURATION,
							ICM_PEDOMETER_STEP_DETECTOR_THRESHOLD,
							ICM_PEDOMETER_SENSITIVITY_MODE);
	
	check_rc(rc, "error while configuring INV device");
	
	do {
		/* Poll device for data */		
		if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {

			rc = GetDataFromInvDevice();
			check_rc(rc, "error while getting data from Icm426xx");

			inv_disable_irq();
			irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
			inv_enable_irq();
		}
	
	} while(1);
}



/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIOs so that MCU can receive interrupts from ICM426xx 
 *   - a microsecond timer requested by Icm426xx driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a serial link to communicate from MCU to Icm426xx
 */
static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif)
{
	inv_io_hal_board_init();
	
	/* configure UART */
	config_uart(LOG_UART_ID);

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	INV_MSG(INV_MSG_LEVEL_INFO, "#########################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Pedometer   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "#########################");

	/*
	 * Configure input capture mode GPIO connected to pin PB10 (arduino connector D6).
	 * This pin is connected to Icm426xx INT1 output and thus will receive interrupts 
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	*/
	inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_inv_cb, 0);
	
	/* Init timer peripheral for delay */
	inv_delay_init(DELAY_TIMER);

	/*
	 * Configure the timer for the timebase
	 */
	inv_timer_configure_timebase(1000000);
	inv_timer_enable(TIMEBASE_TIMER);
	
	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
	inv_io_hal_init(icm_serif);
}


/*
 * Icm426xx interrupt handler.
 * Function is executed when an Icm426xx interrupt rises on MCU.
 * This function get a timestamp and store it in a dedicated timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * is implemented for shared variable timestamp_buffer.
 */
void ext_interrupt_inv_cb(void * context, unsigned int int_num)
{
	(void)context;

	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);

	if(int_num == INV_GPIO_INT1) {
		if (!RINGBUFFER_FULL(&timestamp_buffer))
			RINGBUFFER_PUSH(&timestamp_buffer, &timestamp);
	}

	irq_from_device |= TO_MASK(int_num);
}

/*
 *  Helper function to check RC value and block programm execution
 */
void check_rc(int rc, const char * msg_context)
{
	if(rc < 0) {
		INV_MSG(INV_MSG_LEVEL_ERROR, "%s: error %d (%s)\r\n", msg_context, rc, inv_error_str(rc));
		while(1);
	}
}


/*
 * Printer function for message facility
 */
void msg_printer(int level, const char * str, va_list ap)
{
	static char out_str[256]; /* static to limit stack usage */
	unsigned idx = 0;
	const char * s[INV_MSG_LEVEL_MAX] = {
	    "",    // INV_MSG_LEVEL_OFF
	    "[E] ", // INV_MSG_LEVEL_ERROR
	    "[W] ", // INV_MSG_LEVEL_WARNING
	    "[I] ", // INV_MSG_LEVEL_INFO
	    "[V] ", // INV_MSG_LEVEL_VERBOSE
	    "[D] ", // INV_MSG_LEVEL_DEBUG
	};
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "%s", s[level]);
	if(idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if(idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if(idx >= (sizeof(out_str)))
		return;

	inv_uart_mngr_puts(LOG_UART_ID, out_str, idx);
}


/* --------------------------------------------------------------------------------------
 *  Extern functions definition
 * -------------------------------------------------------------------------------------- */

/*
 * Icm426xx driver needs to get time in us. Let's give its implementation here.
 */
uint64_t inv_icm426xx_get_time_us(void)
{
	return inv_timer_get_counter(TIMEBASE_TIMER);
}

/*
 * Clock calibration module needs to disable IRQ. Thus inv_helper_disable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_disable_irq(void)
{
	inv_disable_irq();
}

/*
 * Clock calibration module needs to enable IRQ. Thus inv_helper_enable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_enable_irq(void)
{
	inv_enable_irq();
}


/*
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 */
void inv_icm426xx_sleep_us(uint32_t us)
{
	inv_delay_us(us);
}

