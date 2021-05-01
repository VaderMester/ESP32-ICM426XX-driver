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

#include "example-high-rate-logger.h"

/* InvenSense utils */
#include "Invn/EmbUtils/Message.h"
#include "Invn/EmbUtils/ErrorHelper.h"
#include "Invn/EmbUtils/RingBuffer.h"

/* board driver */
#include "common.h"
#include "uart.h"
#include "uart_mngr.h"
#include "delay.h"
#include "gpio.h"
#include "timer.h"
#include "rtc_timer.h"
#include "spi_master.h"

#include "system-interface.h"

/* std */
#include <stdio.h>


/* --------------------------------------------------------------------------------------
 *  Example configuration
 * -------------------------------------------------------------------------------------- */

/*
 * Select UART port on which INV_MSG() will be printed.
 */
#define LOG_UART_ID INV_UART_LOG

/* 
 * Set of timers used throughout standalone applications 
 */
#define TIMEBASE_TIMER INV_TIMER1
#define DELAY_TIMER    INV_TIMER2

/* 
 * Select communication link between SmartMotion and ICM426xx 
 */
#define SERIF_TYPE ICM426XX_UI_SPI4

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from icm426xx device irq handler */
static volatile int irq_from_device;



/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif);
static void ext_interrupt_cb(void * context, unsigned int int_num);
static void check_rc(int rc, const char * msg_context);
void msg_printer(int level, const char * str, va_list ap);


/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int main(void)
{
	int rc = 0;
	uint8_t out_str[FRAME_SIZE_MAX];
	uint8_t payload_size_bytes = SENSOR_6AXIS_SIZE;
	uint8_t data_address = MPUREG_ACCEL_DATA_X0_UI;
	struct inv_icm426xx_serif icm426xx_serif;
	
	/* Initialize MCU hardware */
	SetupMCUHardware(&icm426xx_serif);
	
	/* Initialize Icm426xx */
	rc = SetupInvDevice(&icm426xx_serif);
	check_rc(rc, "error while setting up INV device");

	/* Configure Icm426xx */
	rc = ConfigureInvDevice();
	check_rc(rc, "error while configuring INV device");
	
	/* Frame Header, contains sensor odr information,
	 to be decoded properly by python script */
	out_str[HEADER_LSB_IDX] = HEADER_LSB;
	out_str[HEADER_MSB_IDX] = HEADER_MSB | SENSOR_ODR;

	/* Frame counter index reset*/
	out_str[FRAME_CNT_IDX] = 0;
	
	if (SENSOR_ODR == HIGH_RATE_32KHZ) {
	
		payload_size_bytes = SENSOR_1AXIS_SIZE;
		switch(AXIS_TO_LOG) {
			case ACCEL_X_AXIS: 
				data_address = MPUREG_ACCEL_DATA_X0_UI;
				INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Accel x-axis at 32 KHz");
				break;

			case ACCEL_Y_AXIS: 
				data_address = MPUREG_ACCEL_DATA_X0_UI + 2;
				INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Accel y-axis at 32 KHz");
				break;

			case ACCEL_Z_AXIS: 
				data_address = MPUREG_ACCEL_DATA_X0_UI + 4;
				INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Accel z-axis at 32 KHz");
				break;

			case GYRO_X_AXIS:  
				data_address = MPUREG_GYRO_DATA_X0_UI;
				INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Gyro x-axis at 32 KHz");
				break;

			case GYRO_Y_AXIS:  
				data_address = MPUREG_GYRO_DATA_X0_UI + 2;
				INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Gyro y-axis at 32 KHz");
				break;

			case GYRO_Z_AXIS:  
				data_address = MPUREG_GYRO_DATA_X0_UI + 4;
				INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Gyro z-axis at 32 KHz");
				break;

			default:
				return -1;

		}

	} else if (SENSOR_ODR == HIGH_RATE_16KHZ) {
	
		INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Accel x,y,z and Raw Gyro x,y,z at 16 KHz");
	
	} else if (SENSOR_ODR == HIGH_RATE_8KHZ) {

		INV_MSG(INV_MSG_LEVEL_INFO, "Start streaming: Raw Accel x,y,z and Raw Gyro x,y,z at 8 KHz");
	}

	do {
		/* Poll device for data */		
		if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {
			/* Increment counter */
			out_str[FRAME_CNT_IDX] ++;
			
			inv_io_hal_read_reg(&icm426xx_serif, data_address, &out_str[RACC_X_IDX], payload_size_bytes);

			/* Split uart send in several packet to allow host to suport 3Mbaud */
			while(inv_uart_puts(INV_UART_SENSOR_CTRL,(const char*)&out_str[HEADER_LSB_IDX], HEADER_SIZE) != 0);
			while(inv_uart_puts(INV_UART_SENSOR_CTRL,(const char*)&out_str[FRAME_CNT_IDX], FRAME_CNT_SIZE) != 0);
			while(inv_uart_puts(INV_UART_SENSOR_CTRL,(const char*)&out_str[RACC_X_IDX], payload_size_bytes) != 0);

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
 *   - interrupt priority group and GPIO so that MCU can receive interrupts from ICM426xx
 *   - a microsecond timer requested by Icm426xx driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a serial link to communicate from MCU to Icm426xx
 */
static void SetupMCUHardware(struct inv_icm426xx_serif * icm_serif)
{
	inv_io_hal_board_init();
	
	/* Configure Log UART */
	config_uart(LOG_UART_ID);

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	INV_MSG(INV_MSG_LEVEL_INFO, "##################################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example High Rate Logger     #");
	INV_MSG(INV_MSG_LEVEL_INFO, "##################################");

	/* Configure Streaming UART */
	inv_uart_mngr_init_struct_t uart_mngr_config;
	
	uart_mngr_config.uart_num = INV_UART_SENSOR_CTRL;
	uart_mngr_config.baudrate = 3000000;
	uart_mngr_config.flow_ctrl = INV_UART_FLOW_CONTROL_RTS_CTS;
	inv_uart_mngr_init(&uart_mngr_config);

	/*
	 * Configure input capture mode GPIO connected to pin EXT3-9 (pin PB03).
	 * This pin is connected to Icm426xx INT1 output and thus will receive interrupts 
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	*/
	inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_cb, 0);

	/* Init timer peripheral for delay */
	inv_delay_init(DELAY_TIMER);

	/*
	 * Configure the timer for the timebase
	 */
	inv_timer_configure_timebase(1000000);
	inv_timer_enable(TIMEBASE_TIMER);

#if USE_CLK_IN
	rtc_timer_init(NULL);
	/* Output 32kHz SLCK to PA17, it is up to user to connect it or not at board level to have CLKIN capability */
	inv_gpio_output_clk_on_pin(INV_GPIO_CLKIN);
#endif

	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
	/* Configure SPI to 24 MHz */
	inv_io_hal_configure_spi_speed(24);
	inv_io_hal_init(icm_serif);
	
}


/*
 * Icm426xx interrupt handler.
 * Function is executed when an Icm426xx interrupt rises on MCU.
 * This function get a timestamp and store it in the timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * are implemented for shared variable timestamp_buffer.
 */
static void ext_interrupt_cb(void * context, unsigned int int_num)
{
	(void)context;

	irq_from_device |= TO_MASK(int_num);
}


/*
 * Helper function to check RC value and block programm exectution
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
#if USE_CLK_IN
	return rtc_timer_get_time_us();
#else
	return inv_timer_get_counter(TIMEBASE_TIMER);
#endif
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
