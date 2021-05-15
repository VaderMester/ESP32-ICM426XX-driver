/*
 * I2CDev.h
 *
 *  Created on: 2020. jan. 14.
 *      Author: Admin
 */

#ifndef MAIN_I2CDEV_H_
#define MAIN_I2CDEV_H_

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
//#define PIN_SDA 						21
//#define PIN_CLK 						22

void I2Cdev_SelectRegister(uint8_t dev, uint8_t reg);

bool I2Cdev_writeBytes(uint8_t devAdd, uint8_t regAddr, uint32_t length, uint8_t *data);
bool I2Cdev_writeByte(uint8_t devAdd, uint8_t regAddr, uint8_t data);

uint8_t I2Cdev_readBytes(uint8_t devAdd, uint8_t regAddr, uint32_t length, uint8_t *data);
uint8_t I2Cdev_readByte(uint8_t devAdd, uint8_t regAddr);
uint8_t I2Cdev_readBits(uint8_t devAdd, uint8_t regAddr, uint8_t bitStart, uint32_t length, uint8_t *data);
bool I2Cdev_writeBits(uint8_t devAdd, uint8_t regAddr, uint8_t bitStart, uint32_t length, uint8_t data);
bool I2Cdev_writeBit(uint8_t devAdd, uint8_t regAddr, uint8_t bitNum, uint8_t data);
uint8_t I2Cdev_readWord(uint8_t devAdd, uint8_t regAddr, uint16_t *data);
bool I2Cdev_writeWord(uint8_t devAddr, uint8_t regAddr, uint16_t data);
esp_err_t I2Cdev_init(int gpio_num_sda, int gpio_num_scl);

#endif /* MAIN_I2CDEV_H_ */
