/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2017 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
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

#include "system-interface.h"

/* std */
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "esp_attr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "sdkconfig.h"
#include "time.h"
#include <sys/time.h>
#include "esp_system.h"
#include "esp_sntp.h"
#include "driver/gpio.h"

#include "ICM426XX.h"

#define TAG "ICM426XX"

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
//#define SERIF_TYPE ICM426XX_UI_SPI4
#define SERIF_TYPE ICM426XX_UI_I2C

/*
* Set INT1 interrupt pin
*/
#define INV_GPIO_INT1 CONFIG_I2CM426XX_DEV_INT1_GPIO

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

DRAM_ATTR const char * INV_LOGLEVEL[INV_MSG_LEVEL_MAX] = {
		"",    // INV_MSG_LEVEL_OFF
		"[E] ", // INV_MSG_LEVEL_ERROR
		"[W] ", // INV_MSG_LEVEL_WARNING
		"[I] ", // INV_MSG_LEVEL_INFO
		"[V] ", // INV_MSG_LEVEL_VERBOSE
		"[D] ", // INV_MSG_LEVEL_DEBUG
};
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

/* MUX for entering critical section */
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
/* Flag set once a UART RX frame is received */
volatile int irq_event_main_uart = 1;//0;
static QueueHandle_t uart0_queue;
const uint16_t uart_bufs = 1024;
uint8_t *uart_cmd_buf = NULL;
SemaphoreHandle_t sCmdSem;

#define EX_UART_NUM UART_NUM_0
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL  (1ULL<<INV_GPIO_INT1)

/* --------------------------------------------------------------------------------------
 *  Forward declaration
 * -------------------------------------------------------------------------------------- */

static int SetupMCUHardware(struct inv_icm426xx_serif * icm_serif, struct inv_ak0991x_serif *akm_serif);
static void IRAM_ATTR ext_interrupt_inv_cb(void * context);
#if USE_MAG
static void interrupt_timer_start_mag_cb(void* context);
static void interrupt_timer_data_rdy_mag_cb(void *context);
#endif
static char get_user_command_from_uart(void);
static void process_user_command(void);
static void print_help(void);
void check_rc(int rc, const char * msg_context);
void msg_printer(int level, const char * str, va_list ap);

void ESP32_uart_event_task(void *pvParameters);
void ESP32_setup_uart(void);
uint64_t ESP32_get_time_us();

/* --------------------------------------------------------------------------------------
 *  Main
 * -------------------------------------------------------------------------------------- */

void app_main(void)
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
				portENTER_CRITICAL(&mux);
				irq_from_ak09915_data_ready = 0;
				portEXIT_CRITICAL(&mux);

				rc = GetDataFromMagDevice();
				check_rc(rc, "error while getting data from Akm09915");
			}
			
			/* Check IRQ from timer ruling mag acquisition */
			if (irq_from_ak09915_acquisition_timer) {
				portENTER_CRITICAL(&mux);
				irq_from_ak09915_acquisition_timer = 0;
				portEXIT_CRITICAL(&mux);

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

			portENTER_CRITICAL(&mux);
			irq_from_device &= ~TO_MASK(INV_GPIO_INT1);
			portEXIT_CRITICAL(&mux);
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
			ResetInvAGMBiases();
			ICM_MSG(INV_MSG_LEVEL_INFO, "Reset command received");
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

void ESP32_uart_event_task(void *pvParameters)
{
    uart_event_t event;
    size_t buffered_size;
    uart_cmd_buf = (uint8_t*) malloc(uart_bufs);
	if(!uart_cmd_buf) {
		ESP_LOGE("UART BUFFER ERR: NOT ENOUGH MEMORY, ABORTING");
		abort();
	}
    for(;;) {
        //Waiting for UART event.
        if(xQueueReceive(uart0_queue, (void * )&event, (portTickType)portMAX_DELAY)) {
            bzero(uart_cmd_buf, uart_bufs);
            ESP_LOGI(TAG, "uart[%d] event:", EX_UART_NUM);
            switch(event.type) {
                //Event of UART receving data
                /*We'd better handler data event fast, there would be much more data events than
                other types of events. If we take too much time on data event, the queue might
                be full.*/
                case UART_DATA:
                    ESP_LOGI(TAG, "[UART DATA]: %d", event.size);
					xSemaphoreTake(sCmdSem, portMAX_DELAY);
                    uart_read_bytes(EX_UART_NUM, uart_cmd_buf, event.size, portMAX_DELAY);
                    xSemaphoreGive(sCmdSem);
					ext_interrupt_inv_cb();
                    break;
                //Event of HW FIFO overflow detected
                case UART_FIFO_OVF:
                    ESP_LOGI(TAG, "hw fifo overflow");
                    // If fifo overflow happened, you should consider adding flow control for your application.
                    // The ISR has already reset the rx FIFO,
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART ring buffer full
                case UART_BUFFER_FULL:
                    ESP_LOGI(TAG, "ring buffer full");
                    // If buffer full happened, you should consider encreasing your buffer size
                    // As an example, we directly flush the rx buffer here in order to read more data.
                    uart_flush_input(EX_UART_NUM);
                    xQueueReset(uart0_queue);
                    break;
                //Event of UART RX break detected
                case UART_BREAK:
                    ESP_LOGE(TAG, "uart rx break");
                    break;
                //Event of UART parity check error
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "uart parity error");
                    break;
                //Event of UART frame error
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "uart frame error");
                    break;
                //UART_PATTERN_DET
                case UART_PATTERN_DET:
                    break;
                //Others
                default:
                    ESP_LOGE(TAG, "uart event type: %d", event.type);
                    break;
            }
        }
    }
    free(uart_cmd_buf);
    uart_cmd_buf = NULL;
    vTaskDelete(NULL);
}

void ESP32_setup_uart(void)
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };
    //Install UART driver, and get the queue.
    uart_driver_install(EX_UART_NUM, uart_bufs * 2, uart_bufs * 2, 20, &uart0_queue, 0);
    uart_param_config(EX_UART_NUM, &uart_config);

    //Set UART pins (using UART0 default pins ie no changes.)
    uart_set_pin(EX_UART_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);


	sCmdSem = xSemaphoreCreateBinary();
	xSemaphoreGive(sCmdSem);
    //Create a task to handler UART event from ISR
    xTaskCreate(ESP32_uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);
}

uint64_t ESP32_get_time_us()
{
	struct timeval tv_now;
	gettimeofday(&tv_now, NULL);
	uint64_t time_us = (uint64_t)tv_now.tv_sec * 1000000L + (uint64_t)tv_now.tv_usec;
	return time_us;
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

	/* configure UART */
	//Setup uart0 to receive commands
	ESP32_setup_uart();

	/* Setup message facility to see internal traces from FW */
	INV_MSG_SETUP(MSG_LEVEL, msg_printer);

	INV_MSG(INV_MSG_LEVEL_INFO, "###################");
	INV_MSG(INV_MSG_LEVEL_INFO, "#   Example AGM   #");
	INV_MSG(INV_MSG_LEVEL_INFO, "###################");
	
	/*
	 * Configure input capture mode GPIO connected to pin set in INV_GPIO_INT1.
	 * This pin is connected to Icm426xx INT1 output and thus will receive interrupts 
	 * enabled on INT1 from the device.
	 * A callback function is also passed that will be executed each time an interrupt
	 * fires.
	*/
	//Setup INT1 gpio
	//interrupt of rising edge
	gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the pins, for INV_GPIO_INT1
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

	//installing ISR services
	gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(INV_GPIO_INT1, ext_interrupt_inv_cb, INV_GPIO_INT1);

#if 0
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
#endif //if0

	/* Initialize serial inteface between MCU and Icm426xx */
	icm_serif->context   = 0;        /* no need */
	icm_serif->read_reg  = ESP32_HAL_read_reg;
	icm_serif->write_reg = ESP32_HAL_write_reg;
	icm_serif->max_read  = 1024*32;  /* maximum number of bytes allowed per serial read */
	icm_serif->max_write = 1024*32;  /* maximum number of bytes allowed per serial write */
	icm_serif->serif_type = SERIF_TYPE;
	rc |= ESP32_icm_serif_init(icm_serif);


/*
*	MAG stuff is not done, and not tested due to lack of hardware.
*
*/
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
void ext_interrupt_inv_cb(void * context)
{
	(void)context;

	/* Read timestamp from the RTC derived from SLCK since CLKIN is used */
	uint64_t timestamp = ESP32_get_time_us();
    uint32_t int_num = (uint32_t) arg;

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

#if 0
#if USE_CLK_IN
	/* Read timestamp from the RTC derived from SLCK since CLKIN is used */
	uint64_t timestamp = rtc_timer_get_time_us();
#else
	/* Read timestamp from the timer */
	uint64_t timestamp = inv_timer_get_counter(TIMEBASE_TIMER);
#endif
#endif //#if 0
	uint64_t timestamp = ESP32_get_time_us();

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
	char cmd = 0;
	if (irq_event_main_uart) {
		//portENTER_CRITICAL(&mux);
		irq_event_main_uart = 0;
		//portEXIT_CRITICAL(&mux);
		xSemaphoreTake(sCmdSem, portMAX_DELAY);
		int i = 0;
		while(i < uart_bufs) {
			cmd = uart_cmd_buf[i];
			if (cmd != '\n') {
				break;
			} else {
				i++;
			}
		}
		xSemaphoreGive(sCmdSem);
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
		/*LOL... we are FreeRTOS, so this would not do much,
		maybe just startle the task watchdog. */
		// while(1);
		abort(0); //... because when there is error, panic! why not?
		// I'm pointing at you FreeRTOS ASSERT!!!! ,,|,,
		//Anyway, are you up to reading some tasty core dumps? Me neither!
		//At least you can find this abort call from the backtrace, and realize, it has been nothing :)
	}
}

/*
 * Printer function for message facility
 */
void msg_printer(int level, const char * str, va_list ap)
{
	va_list args;
	va_start(args, format);
	size_t msglen = vsnprintf(NULL, NULL, format, args);
	char *msg = malloc(msglen+1);
	if (!msg)
	{
		return;
	}
	//ESP_LOGW("DEB: ", "Taglen: %d", strlen(msg));
	vsnprintf(msg, msglen+1, format, args);
	va_end(args);
	ESP_LOG_LEVEL(level, TAG, "%s%s\r", INV_LOGLEVEL[level], msg);
}


/* --------------------------------------------------------------------------------------
 *  Extern functions definition
 * -------------------------------------------------------------------------------------- */

/*
 * Icm426xx driver needs to get time in us. Let's give its implementation here.
 */
uint64_t inv_icm426xx_get_time_us(void)
{
#if 0	//original code
#if USE_CLK_IN
	return rtc_timer_get_time_us();
#else
	return inv_timer_get_counter(TIMEBASE_TIMER);
#endif
#endif //#if 0
	return ESP32_get_time_us();
}

/*
 * Clock calibration module needs to disable IRQ. Thus inv_helper_disable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_disable_irq(void)
{
	portENTER_CRITICAL(&mux);
}

/*
 * Clock calibration module needs to enable IRQ. Thus inv_helper_enable_irq is
 * defined as extern symbol in clock calibration module. Let's give its implementation
 * here.
 */
void inv_helper_enable_irq(void)
{
	portEXIT_CRITICAL(&mux);
}

/*
 * Icm426xx driver needs a sleep feature from external device. Thus inv_icm426xx_sleep_us
 * is defined as extern symbol in driver. Let's give its implementation here.
 */
void inv_icm426xx_sleep_us(uint32_t us)
{
	inv_delay_us(us);
}

