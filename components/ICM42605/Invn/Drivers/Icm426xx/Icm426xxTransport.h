/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
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

/** @defgroup DriverIcm426xxTransport Icm426xx driver transport
 *  @brief    Low-level Icm426xx register access
 *  @ingroup  DriverIcm426xx
 *  @{
 */

/** @file Icm426xxTransport.h
 * Low-level Icm426xx register access
 */

#ifndef _INV_ICM426XX_TRANSPORT_H_
#define _INV_ICM426XX_TRANSPORT_H_


#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/* forward declaration */
struct inv_icm426xx;


/** @brief enumeration  of serial interfaces available on icm426xx */
typedef enum
{
  ICM426XX_UI_I2C,
  ICM426XX_UI_SPI4,
  ICM426XX_UI_I3C,
  ICM426XX_AUX1_SPI3,
  ICM426XX_AUX2_SPI3,
  ICM426XX_AUX1_SPI4
  
} ICM426XX_SERIAL_IF_TYPE_t;
 
/** @brief basesensor serial interface
 */
struct inv_icm426xx_serif {
	void *     context;
	int      (*read_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);
	int      (*write_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * buf, uint32_t len);
	int      (*configure)(struct inv_icm426xx_serif * serif);
	uint32_t   max_read;
	uint32_t   max_write;
	ICM426XX_SERIAL_IF_TYPE_t serif_type;
};

/** @brief transport interface
 */
struct inv_icm426xx_transport {
	struct inv_icm426xx_serif serif; /**< Warning : this field MUST be the first one of struct inv_icm426xx_transport */

	/** @brief Contains mirrored values of some IP registers */
	struct register_cache {
		uint8_t intf_cfg_1_reg;   /**< INTF_CONFIG1, Bank: 0, Address: 0x4D */
		uint8_t pwr_mngt_0_reg;   /**< PWR_MGMT_0, Bank: 0, Address: 0x4E */
		uint8_t gyro_cfg_0_reg;   /**< GYRO_CONFIG0, Bank: 0, Address: 0x4F */
		uint8_t accel_cfg_0_reg;  /**< ACCEL_CONFIG0, Bank: 0, Address: 0x50 */
		uint8_t tmst_cfg_reg;     /**< TMST_CONFIG, Bank: 0, Address: 0x54 */
		uint8_t bank_sel_reg;     /**< MPUREG_REG_BANK_SEL, All banks, Address 0x76*/
	} register_cache; /**< Store mostly used register values on SRAM. 
	                    *  MPUREG_OTP_SEC_STATUS_B1 and MPUREG_INT_STATUS registers
	                    *  are read before the cache has a chance to be initialized. 
	                    *  Therefore, these registers shall never be added to the cache 
						*  Registers from bank 1,2,3 or 4 shall never be added to the cache
	                    */
};

/** @brief Init cache variable.
 * @return            0 in case of success, -1 for any error
 */
int inv_icm426xx_init_transport(struct inv_icm426xx * s);

/** @brief Reads data from a register on Icm426xx.
 * @param[in] reg    register address to be read
 * @param[in] len    number of byte to be read
 * @param[out] buf   output data from the register
 * @return            0 in case of success, -1 for any error
 */
int inv_icm426xx_read_reg(struct inv_icm426xx * s, uint8_t reg, uint32_t len, uint8_t * buf);

/** @brief Writes data to a register on Icm426xx.
 * @param[in] reg    register address to be written
 * @param[in] len    number of byte to be written
 * @param[in] buf    input data to write
 * @return            0 in case of success, -1 for any error
 */
int inv_icm426xx_write_reg(struct inv_icm426xx * s, uint8_t reg, uint32_t len, const uint8_t * buf);

/** @brief Enable MCLK so that MREG are clocked and system beyond SOI can be safely accessed
 * @param[out] idle_en Value of IDLE bit prior to function access, this might be useful to understand if MCLK was already enabled or not
 * @return            0 in case of success, -1 for any error
 */

#ifdef __cplusplus
}
#endif

#endif /* _INV_ICM426XX_TRANSPORT_H_ */

/** @} */
