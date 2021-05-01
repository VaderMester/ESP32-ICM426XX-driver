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


#include "example-selftest.h"

/* InvenSense utils */
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"
#include "uart.h"
#include "uart_mngr.h"
#include "delay.h"
#include "timer.h"
#include "gpio.h"

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
 * Timer used throughout standalone applications 
 */
#define DELAY_TIMER INV_TIMER3
#define TIMEBASE_TIMER INV_TIMER2

/* 
 * Select communication between SmartMotion and ICM426xx by setting correct SERIF_TYPE 
 */
#define SERIF_TYPE ICM426XX_UI_SPI4
//#define SERIF_TYPE ICM426XX_UI_I2C

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG


/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif);
static void check_rc(int rc, const char * msg_context);
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
	
	/* Perform Self-Test */
	RunSelfTest();

	/* Get Low Noise / Low Power bias computed by self-tests scaled by 2^16 */
	GetBias();

	/* Add a delay so the messages get fully printed out */
	inv_delay_us(1000000);
}


/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group
 *   - a microsecond timer requested by Icm426xx driver to compute some delay
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
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example Self-Test   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "#########################");

	/* Init timer peripheral for delay */
	inv_delay_init(DELAY_TIMER);
	
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
 * Helper function to check RC value and block programm execution
 */
static void check_rc(int rc, const char * msg_context)
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
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 */
 void inv_icm426xx_sleep_us(uint32_t us)
{
	inv_delay_us(us);
}
