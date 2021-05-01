/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2016-2016 InvenSense Inc. All rights reserved.
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

/* board driver */
#include "uart_mngr.h"
#include "delay.h"
#include "i2c_master.h"
#include "spi_master.h"
#include "gpio.h"
#include "common.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#include "system-interface.h"

#include "Invn/EmbUtils/Message.h"

/* I2C number and slave address for INV device */
#if (SM_BOARD_REV == SM_REVB_OB)
#define INV_SPI_AP INV_SPI_ONBOARD_REVB
#define ICM_I2C_ADDR     0x69
#else
/* SM_REVB_DB and SM_REVG have same SPI/I2C configuration */
#define INV_SPI_AP INV_SPI_REVG
#define ICM_I2C_ADDR     0x68
#endif

/* I2C address for Ak09915 */
#define AK_I2C_ADDR      0x0E

/* Default SPI frequency is 6 Mhz */
static uint8_t spi_default_freq_mhz = 6;

void config_uart(inv_uart_num_t log_uart_id)
{
	inv_uart_mngr_init_struct_t uart_mngr_config;
	
	uart_mngr_config.uart_num = log_uart_id;
	uart_mngr_config.baudrate = 921600;
	uart_mngr_config.flow_ctrl = INV_UART_FLOW_CONTROL_NONE;
	inv_uart_mngr_init(&uart_mngr_config);
}

void config_command_uart(inv_uart_num_t cmd_uart_id, void ext_interrupt_uart_main_cb(void *))
{
	inv_uart_mngr_init_struct_t uart_mngr_config;
	
	uart_mngr_config.uart_num = cmd_uart_id;
	uart_mngr_config.baudrate = 2000000;
	uart_mngr_config.flow_ctrl = INV_UART_FLOW_CONTROL_RTS_CTS;
	inv_uart_mngr_init(&uart_mngr_config);
	
}

/******************************************************/
/* Low-level serial interface function implementation */
/******************************************************/

void inv_io_hal_board_init(void)
{
	inv_board_hal_init();

#if (SM_BOARD_REV == SM_REVB_OB)
	/* Set board revision revB if define is set */
	inv_gpio_set_board_rev(INV_GPIO_BOARD_REVB);
#elif (SM_BOARD_REV == SM_REVB_DB)
	/* Set board revision revB DB if define is set */
	inv_gpio_set_board_rev(INV_GPIO_BOARD_REVB_DB);
#elif (SM_BOARD_REV == SM_REVG)
	/* Set board revision revG if define is set */
	inv_gpio_set_board_rev(INV_GPIO_BOARD_REVG);
#else
	#error Please select a board revision
#endif

}

void inv_io_hal_configure_spi_speed(uint8_t spi_freq_mhz)
{
	spi_default_freq_mhz = spi_freq_mhz;
}


int inv_io_hal_init(struct inv_icm426xx_serif * serif)
{
	uint8_t spi_freq_mhz = spi_default_freq_mhz;

	switch (serif->serif_type) {
		case ICM426XX_AUX1_SPI3:
		case ICM426XX_AUX1_SPI4:
		case ICM426XX_AUX2_SPI3:
		case ICM426XX_UI_SPI4:
		{
#if (SM_BOARD_REV == SM_REVB_DB)
			/* Applicable for RevB+DB only */
			uint8_t dummy = 0;
			/* To avoid SPI disturbance on ICM DB, on-chip ICM is forced to SPI by doing a dummy-write*/
			inv_spi_master_init(INV_SPI_ONBOARD_REVB, 6*1000*1000);
			/* Write to register MPUREG_WHO_AM_I */
			inv_spi_master_write_register(INV_SPI_ONBOARD_REVB, 0x76, 1, &dummy);
#endif
			inv_spi_master_init(INV_SPI_AP, spi_freq_mhz*1000*1000);
			break;
		}

		case ICM426XX_UI_I2C:
#if (SM_BOARD_REV == SM_REVG)
			/* Force AD0 to be driven as output level low */
			inv_gpio_init_pin_out(INV_GPIO_AD0);
			inv_gpio_set_pin_low(INV_GPIO_AD0);
#endif
			/* Set I2C clock is 400kHz by default */
			inv_i2c_master_init();
			break;
		
		default:
			return -1;
	}

	return 0;
}
int inv_io_hal_configure(struct inv_icm426xx_serif * serif)
{
	switch (serif->serif_type) {
		default:
			return -1;
	}
}

int inv_io_hal_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	switch (serif->serif_type) {
		case ICM426XX_AUX1_SPI3:
		case ICM426XX_AUX1_SPI4:
		case ICM426XX_AUX2_SPI3:
		case ICM426XX_UI_SPI4:
			return inv_spi_master_read_register(INV_SPI_AP, reg, rlen, rbuffer);
		case ICM426XX_UI_I2C:
			while(inv_i2c_master_read_register(ICM_I2C_ADDR, reg, rlen, rbuffer)) {
				inv_delay_us(32000); // Loop in case of I2C timeout
			}
			return 0;
		default:
			return -1;
	}
}

int inv_io_hal_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	int rc;

	switch (serif->serif_type) {
		case ICM426XX_AUX1_SPI3:
		case ICM426XX_AUX1_SPI4:
		case ICM426XX_AUX2_SPI3:
		case ICM426XX_UI_SPI4:
			for(uint32_t i=0; i<wlen; i++) {
				rc = inv_spi_master_write_register(INV_SPI_AP, reg+i, 1, &wbuffer[i]);
				if(rc)
					return rc;
			}
			return 0;
		case ICM426XX_UI_I2C:
			while(inv_i2c_master_write_register(ICM_I2C_ADDR, reg, wlen, wbuffer)) {
				inv_delay_us(32000); // Loop in case of I2C timeout
			}
			return 0;
		default:
			return -1;
	}
}

/* Ak09915 Serif object definition for I2C ***************************************/

int akm_io_hal_init(void * serif)
{
	(void)serif;

	inv_i2c_master_init();
	return 0;
}

int akm_io_hal_read_reg(void * serif, uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)serif;

	return inv_i2c_master_read_register(AK_I2C_ADDR, reg, rlen, rbuffer);
}

int akm_io_hal_write_reg(void * serif, uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)serif;

	return inv_i2c_master_write_register(AK_I2C_ADDR, reg, wlen, wbuffer);
}
