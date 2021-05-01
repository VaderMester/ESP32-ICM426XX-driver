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

#include "example-algo.h"

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
#include "rtc_timer.h"

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
//#define SERIF_TYPE ICM426XX_UI_I2C

/* 
 * Define msg level 
 */
#define MSG_LEVEL INV_MSG_LEVEL_INFO

/* 
 * Set of timers used throughout standalone applications 
 */
#define TIMEBASE_TIMER     INV_TIMER1
#define DELAY_TIMER        INV_TIMER2
#define MAG_SAMPLING_TIMER INV_TIMER3
#define MAG_DATA_TIMER     INV_TIMER4


/* --------------------------------------------------------------------------------------
 *  Global variables
 * -------------------------------------------------------------------------------------- */

/* 
 * Buffer to keep track of the timestamp when icm426xx/ak09915 data ready interrupt fires.
 * The buffer can contain up to 64 items in order to store one timestamp for each packet in FIFO.
 */
RINGBUFFER_VOLATILE(timestamp_buffer_icm, 64, uint64_t);
#if USE_MAG
RINGBUFFER_VOLATILE(timestamp_buffer_mag, 64, uint64_t);
#endif

/*
 * Outptut data to print
 * Default behavior: only print accel, gyro, mag and 6-axis
 */
uint32_t data_to_print = MASK_PRINT_ACC_DATA
                       | MASK_PRINT_GYR_DATA
                       | MASK_PRINT_MAG_DATA
                       | MASK_PRINT_6AXIS_DATA;

/*
 * Define how often traces will be printed
 */
int print_period_us = 1000000; /* 1 s */  


/* --------------------------------------------------------------------------------------
 *  Static variables
 * -------------------------------------------------------------------------------------- */

/* Flag set from icm426xx device irq handler */
static volatile int irq_from_device;

#if USE_MAG
/* Flag set from irq handler of the timer used to trigger new data acquitision on Akm09915 */
static volatile int irq_from_ak09915_acquisition_timer = 0;

/* Flag set from irq handler of the timer used to trigger new data ready on Akm09915 */
static volatile int irq_from_ak09915_data_ready = 0;

/* Variable used to keep channel used for mag data ready */
static int mag_end_capture_channel = -1;

/* Variable to keep track if the mag has initialized successfully */
int mag_init_successful = 0;

#define MAG_DATA_RDY_DELAY_US     4200  /* Typical time for the compass to generate a data */
#endif

/* Flag set once a UART RX frame is received */
volatile int irq_event_main_uart = 1;//0;


/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static int SetupMCUHardware(struct inv_icm426xx_serif * icm_serif, struct inv_ak0991x_serif *akm_serif);
static void ext_interrupt_inv_cb(void * context, unsigned int_num);
#if USE_MAG
static void interrupt_timer_start_mag_cb(void* context);
static void interrupt_timer_data_rdy_mag_cb(void *context);
#endif
static char get_user_command_from_uart(void);
static void process_user_command(void);
static void print_help(void);
void check_rc(int rc, const char * msg_context);
void msg_printer(int level, const char * str, va_list ap);


/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

int main(void)
{
	int rc = 0;
	struct inv_icm426xx_serif icm426xx_serif;
	struct inv_ak0991x_serif ak09915_serif;

	/* Initialize MCU hardware */
	rc = SetupMCUHardware(&icm426xx_serif, &ak09915_serif);
	check_rc(rc, "Error while setting up MCU");

	/* Initialize ICM device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initializing ICM device...");
	rc = SetupInvDevice(&icm426xx_serif);
	check_rc(rc, "Error while setting up ICM device");
	INV_MSG(INV_MSG_LEVEL_INFO, "OK");

	/* Initialize algorithm */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initializing algorithm...");
	rc  = InitInvAGMBiases();
	rc |= InitInvAGMAlgo();
	check_rc(rc, "Error while initializing AGM algorithm");
	INV_MSG(INV_MSG_LEVEL_INFO, "OK");
	
	/* Configure ICM device */
	INV_MSG(INV_MSG_LEVEL_INFO, "Configuring ICM device...");
	rc = ConfigureInvDevice();
	check_rc(rc, "Error while configuring ICM device");
	INV_MSG(INV_MSG_LEVEL_INFO, "OK");


#if USE_MAG
	/* Initialize magnetomer */
	INV_MSG(INV_MSG_LEVEL_INFO, "Initializing Mag device...");
	rc = SetupMagDevice(&ak09915_serif);
	if (rc < 0) {
		INV_MSG(INV_MSG_LEVEL_WARNING, "Error while setting up Mag device: not support of mag-related sensors");
		mag_init_successful = 0;
	} else {
		INV_MSG(INV_MSG_LEVEL_INFO, "OK");
		mag_init_successful = 1;
		/* Configure the timer used to trigger new magnetometer data capture */
		inv_timer_configure_callback(MAG_SAMPLING_TIMER, period_us_to_frequency(MAG_ODR_US), 0, interrupt_timer_start_mag_cb);

	}
	
#endif

	/* Print reminder on how to use example */
	print_help();
	
	INV_MSG(INV_MSG_LEVEL_INFO, "Start processing");
	
	do {
#if USE_MAG
		if (mag_init_successful) {
			/* Check Ak09915 IRQ */
			if (irq_from_ak09915_data_ready) {
				inv_disable_irq();
				irq_from_ak09915_data_ready = 0;
				inv_enable_irq();

				rc = GetDataFromMagDevice();
				check_rc(rc, "error while getting data from Akm09915");
			}
			
			/* Check IRQ from timer ruling mag acquisition */
			if (irq_from_ak09915_acquisition_timer) {
				inv_disable_irq();
				irq_from_ak09915_acquisition_timer = 0;
				inv_enable_irq();

				StartMagDeviceAcquisition();

				/* Start time for the duration of the aquisition */
				mag_end_capture_channel = inv_timer_configure_callback(MAG_DATA_TIMER, 
					period_us_to_frequency(MAG_DATA_RDY_DELAY_US), 0, interrupt_timer_data_rdy_mag_cb);
			}
		}
#endif

		/* Check Icm426xx IRQ */
		if (irq_from_device & TO_MASK(INV_GPIO_INT1)) {
			
			rc = GetDataFromInvDevice();
			check_rc(rc, "error while getting data from Icm426xx");

			inv_disable_irq();
			irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
			inv_enable_irq();
		}
		
		if (irq_event_main_uart)
			process_user_command();
		
	} while(1);
}


/* --------------------------------------------------------------------------------------
 *  Functions definitions
 * -------------------------------------------------------------------------------------- */

void process_user_command(void)
{
	uint8_t command_from_uart=0;
	int rc;
	
	command_from_uart = get_user_command_from_uart();
	
	switch(command_from_uart) {
		case 'i': data_to_print ^= MASK_PRINT_INPUT_DATA;          break; /* Print input data */
		case 'a': data_to_print ^= MASK_PRINT_ACC_DATA;            break; /* Print accel data */
		case 'g': data_to_print ^= MASK_PRINT_GYR_DATA;            break; /* Print gyro data */
#if USE_MAG		
		case 'm': data_to_print ^= MASK_PRINT_MAG_DATA;            break; /* Print mag data */
		case '9': data_to_print ^= MASK_PRINT_9AXIS_DATA;          break; /* Print 9 axis data */
#endif		
		case '6': data_to_print ^= MASK_PRINT_6AXIS_DATA;          break; /* Print 6 axis data */ 

		case 'r':
			rc  = ResetInvAGMBiases();
			rc |= InitInvAGMAlgo();
			check_rc(rc, "Error while initializing VR Threedof algorithms");
			break; 
		case 'f': /* Toggle fast-mode (data printed every 20 ms or every 1 s) */
			print_period_us = (print_period_us == 1000000/*1s*/) ? 20000/*20ms*/ : 1000000/*1s*/;
			break;
		case 'G': data_to_print ^= MASK_PRINT_GRAVITY_DATA;        break; /* Print gravity data */
		case 'l': data_to_print ^= MASK_PRINT_LINEARACC_DATA;      break; /* Print linear acceleration data */
		case 'h':
		case 'H':
			print_help(); /* Print helper command */
			break;
		case 0:
			break; /* No command received */
		default: 
			INV_MSG(INV_MSG_LEVEL_INFO, "Unknown command : %c", command_from_uart);
			print_help();
			break;
	}	
}

void print_help(void)
{
	INV_MSG(INV_MSG_LEVEL_INFO, "##########################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Help - Example Algo  #");
	INV_MSG(INV_MSG_LEVEL_INFO, "##########################");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'i' : print input data (raw accel, raw gyro and raw mag)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'a' : print accel data");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'g' : print gyro data");
#if USE_MAG		
	if (mag_init_successful) {
		INV_MSG(INV_MSG_LEVEL_INFO, "\t'm' : print mag data");
		INV_MSG(INV_MSG_LEVEL_INFO, "\t'9' : print rv quaternion data and eulers angles (9axis fusion)");
	}
#endif	
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'6' : print grv quaternion data and eulers angles (6axis fusion)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'G' : print gravity estimation in sensor frame");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'l' : print linear acceleration estimation in sensor frame");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'r' : reset biases and accuracies (will also reinit algorithm)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'f' : toggle fast-mode (data printed every 20 ms or every 1 s)");
	INV_MSG(INV_MSG_LEVEL_INFO, "\t'h' : print this helper");
}

/* Callback called upon UART RX event */
void ext_interrupt_uart_main_cb(void * context)
{
	(void)context;
	irq_event_main_uart = 1;
}

/*
 * This function initializes MCU on which this software is running.
 * It configures:
 *   - a UART link used to print some messages
 *   - interrupt priority group and GPIOs so that MCU can receive interrupts from both
 *     ICM426xx and Akm09915
 *   - a microsecond timer requested by Icm426xx driver to compute some delay
 *   - a microsecond timer used to get some timestamps
 *   - a microsecond timer used to periodically starts magneto data acquisition
 *   - a serial link to communicate from MCU to Icm426xx
 *   - a serial link to communicate from MCU to Akm09915
 */
static int SetupMCUHardware(struct inv_icm426xx_serif * icm_serif, struct inv_ak0991x_serif * akm_serif)
{
	int rc = 0;

	inv_io_hal_board_init();

	/* configure UART */
	config_uart(LOG_UART_ID);

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	INV_MSG(INV_MSG_LEVEL_INFO, "###################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example AGM   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################");
	
	/*
	 * Configure input capture mode GPIO connected to pin PB10 (arduino connector D6).
	 * This pin is connected to Icm426xx INT1 output and thus will receive interrupts 
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	*/
	inv_gpio_sensor_irq_init(INV_GPIO_INT1, ext_interrupt_inv_cb, 0);
	
	/* Init timer peripheral for delay */
	rc |= inv_delay_init(DELAY_TIMER);

#if USE_CLK_IN
	/* Use CLKIN */
	rtc_timer_init(NULL);
	/* Output 32kHz SLCK to PA17, it is up to user to connect it or not at board level to have CLKIN capability */
	rc |= inv_gpio_output_clk_on_pin(INV_GPIO_CLKIN);
#else
	/* Configure the timer for the timebase */
	rc |= inv_timer_configure_timebase(1000000);
	inv_timer_enable(TIMEBASE_TIMER);
#endif

	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = inv_io_hal_read_reg;
	icm_serif->write_reg = inv_io_hal_write_reg;
	icm_serif->max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
	rc |= inv_io_hal_init(icm_serif);
	
#if USE_MAG
	/* Configure timer used to periodically start mag acquisition */
	if (TIMEBASE_TIMER != MAG_SAMPLING_TIMER) {
		inv_timer_enable(MAG_SAMPLING_TIMER);
	}

	/* Initialize serial inteface between MCU and Akm09911 */
	akm_serif->context   = 0;   /* no need */
	akm_serif->read_reg  = akm_io_hal_read_reg;
	akm_serif->write_reg = akm_io_hal_write_reg;
	akm_serif->max_read  = 64;  /* maximum number of bytes allowed per serial read */
	akm_serif->max_write = 64;  /* maximum number of bytes allowed per serial write */
	akm_serif->is_spi    = 0;
	rc |= akm_io_hal_init(akm_serif);
#else
	(void)akm_serif;
#endif

	return rc;
}

/*
 * Icm426xx interrupt handler.
 * Function is executed when an Icm426xx interrupt rises on MCU.
 * This function get a timestamp and store it in a dedicated timestamp buffer.
 * Note that this function is executed in an interrupt handler and thus no protection
 * is implemented for shared variable timestamp_buffer.
 */
void ext_interrupt_inv_cb(void * context, unsigned int_num)
{
	(void)context;

#if USE_CLK_IN
	/* Read timestamp from the RTC derived from SLCK since CLKIN is used */
	uint64_t timestamp = rtc_timer_get_time_us();
#else /* ICM42686 */
	/* Read timestamp from the timer */
	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);
#endif

	if (int_num == INV_GPIO_INT1) {
		if (!RINGBUFFER_VOLATILE_FULL(&timestamp_buffer_icm))
			RINGBUFFER_VOLATILE_PUSH(&timestamp_buffer_icm, &timestamp);
	}

	irq_from_device |= TO_MASK(int_num);
}

#if USE_MAG
/*
 * Interrupt handler of the timer used to trigger new magnetometer data acquisition.
 */
static void interrupt_timer_start_mag_cb(void *context)
{
	(void)context;

#if USE_CLK_IN
	/* Read timestamp from the RTC derived from SLCK since CLKIN is used */
	uint64_t timestamp = rtc_timer_get_time_us();
#else
	/* Read timestamp from the timer */
	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);
#endif

	if (!RINGBUFFER_VOLATILE_FULL(&timestamp_buffer_mag))
		RINGBUFFER_VOLATILE_PUSH(&timestamp_buffer_mag, &timestamp);

	irq_from_ak09915_acquisition_timer = 1;
}

/*
 * Interrupt handler of the timer used to trigger new magnetometer data acquisition.
 */
static void interrupt_timer_data_rdy_mag_cb(void *context)
{
	(void)context;

	irq_from_ak09915_data_ready = 1;

	inv_timer_channel_suspend(MAG_DATA_TIMER, mag_end_capture_channel);
}


#endif /* USE_MAG */

/* Get char command on the UART */
static char get_user_command_from_uart(void)
{
	char rchar, cmd = 0;
	if (irq_event_main_uart) {
		inv_disable_irq();
		// irq_event_main_uart = 0;
		inv_enable_irq();
		
		while(inv_uart_mngr_available(LOG_UART_ID)) {
			rchar = inv_uart_mngr_getc(LOG_UART_ID);
			if (rchar != '\n')
				cmd = rchar;
		}
	}
	return cmd;
}

/*
 * Helper function to check RC value and block programm execution
 */
void check_rc(int rc, const char * msg_context)
{
	if (rc < 0) {
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
	if (idx >= (sizeof(out_str)))
		return;
	idx += vsnprintf(&out_str[idx], sizeof(out_str) - idx, str, ap);
	if (idx >= (sizeof(out_str)))
		return;
	idx += snprintf(&out_str[idx], sizeof(out_str) - idx, "\r\n");
	if (idx >= (sizeof(out_str)))
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

