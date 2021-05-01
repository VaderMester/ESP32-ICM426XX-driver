/* 09/23/2017 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Butterfly default), respectively, and it uses the Butterfly STM32L433CU Breakout Board.
  The ICM42605 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "ICM42605.h"
#include "I2Cdev.h"
#include "sdkconfig.h"
#include <cubeOS.h>

#include "Invn/Drivers/Icm426xx/Icm426xxTransport.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDefs.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL.h"
#include "Invn/Drivers/Icm426xx/Icm426xxDriver_HL_apex.h"
#include "Invn/Drivers/Icm426xx/Icm426xxSelfTest.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LOGNAME "ICM42605"

//0: No output
//1: ERRORS, DEBUG
//2: ERRORS, DEBUG, WARNINGS
//3: VERBOSE
#define CONFIG_ICM426XX_API_LOCAL_LOG_LEVEL 0

#if CONFIG_ICM42605_API_LOCAL_LOG_LEVEL == 0
#define ERROR(fmt, ...)   {}
#define WARNING(fmt, ...) {}
#define DEBUG(fmt, ...)   {}
#define TRACE(fmt, ...)   {}
#define INFO(fmt, ...)    {}
#endif//CONFIG_ICM42605_API_LOCAL_LOG_LEVEL == 0

#if CONFIG_ICM42XX_API_LOCAL_LOG_LEVEL == 1
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) {}
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   {}
#define INFO(fmt, ...)    {}
#endif//CONFIG_ICM42605_API_LOCAL_LOG_LEVEL == 1

#if CONFIG_ICM426XX_API_LOCAL_LOG_LEVEL == 2
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   {}
#define INFO(fmt, ...)    {}
#endif//CONFIG_ICM42605_API_LOCAL_LOG_LEVEL == 2

#if CONFIG_ICM42XX_API_LOCAL_LOG_LEVEL == 3
#define ERROR(fmt, ...)   ESP_LOGE(LOGNAME, fmt, ## __VA_ARGS__)
#define WARNING(fmt, ...) ESP_LOGW(LOGNAME, fmt, ## __VA_ARGS__)
#define DEBUG(fmt, ...)   ESP_LOGD(LOGNAME, fmt, ## __VA_ARGS__)
#define TRACE(fmt, ...)   ESP_LOGV(LOGNAME, fmt, ## __VA_ARGS__)
#define INFO(fmt, ...)   ESP_LOGI(LOGNAME, fmt, ## __VA_ARGS__)
#endif//CONFIG_ICM42605_API_LOCAL_LOG_LEVEL == 2

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

float _aRes, _gRes;
uint8_t devAddr;

uint8_t buffer[14];
const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

#define ICM42605_BUS_ADDR CONFIG_ICM42  //AD0 is low
//#define ICM42605_BUS_ADDR 0x69  //AD0 is high

/** @brief Below are the I2C HAL functions compatible with the "Icm426xxTransport.h"
struct inv_icm426xx_serif {
	void *     context;
	int      (*read_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len);
	int      (*write_reg)(struct inv_icm426xx_serif * serif, uint8_t reg, const uint8_t * buf, uint32_t len);
	int      (*configure)(struct inv_icm426xx_serif * serif);
	uint32_t   max_read;
	uint32_t   max_write;
	ICM426XX_SERIAL_IF_TYPE_t serif_type;
};
*/
/**
 * @brief This is to connect in ESP32 HW functions for reading registers from the ICM426XX device
 * @param serif:  We only use ICM426XX_SERIAL_IF_TYPE_t serif_type to brach for the properly
 * @param reg:    address of register to be read
 * @param buf:    buffer pointer for data to be stored
 * @param len:    read length
 * 
 * */
int ESP32_HW_read_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len)
{
  if(serif.serif_type == ICM426XX_UI_I2C || serif.serif_type == ICM426XX_UI_I3C)
  {
    if(I2Cdev_readBytes(ICM42605_BUS_ADDR, reg, len, buf) != 0)
    {
      return 0;
    } else {
      return 1;
    }
  }
  else
  {
    return 1;
  }
}

int ESP32_HW_write_reg(struct inv_icm426xx_serif * serif, uint8_t reg, uint8_t * buf, uint32_t len)
{
  if(serif.serif_type == ICM426XX_UI_I2C || serif.serif_type == ICM426XX_UI_I3C)
  {
    if(I2Cdev_writeBytes(ICM42605_BUS_ADDR, reg, len, buf)
    {
      return 0;
    } else {
      return 1;
    }
  }
  else
  {
    return 1;
  }
}

uint8_t ICM42605_getChipID()
{
  uint8_t c = I2Cdev_readByte(devAddr, ICM42605_WHO_AM_I);
  return c;
}

float ICM42605_getAres(uint8_t Ascale) {
  switch (Ascale)
  {
    // Possible accelerometer scales (and their register bit settings) are:
    // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
    case AFS_2G:
      _aRes = 2.0f / 32768.0f;
      break;
    case AFS_4G:
      _aRes = 4.0f / 32768.0f;
      break;
    case AFS_8G:
      _aRes = 8.0f / 32768.0f;
      break;
    case AFS_16G:
      _aRes = 16.0f / 32768.0f;
      break;
  }
  return _aRes;
}

float ICM42605_getGres(uint8_t Gscale) {
  switch (Gscale)
  {
    case GFS_15_125DPS:
      _gRes = 15.125f / 32768.0f;
      break;
    case GFS_31_25DPS:
      _gRes = 31.25f / 32768.0f;
      break;
    case GFS_62_5DPS:
      _gRes = 62.5f / 32768.0f;
      break;
    case GFS_125DPS:
      _gRes = 125.0f / 32768.0f;
      break;
    case GFS_250DPS:
      _gRes = 250.0f / 32768.0f;
      break;
    case GFS_500DPS:
      _gRes = 500.0f / 32768.0f;
      break;
    case GFS_1000DPS:
      _gRes = 1000.0f / 32768.0f;
      break;
    case GFS_2000DPS:
      _gRes = 2000.0f / 32768.0f;
      break;
  }
  return _gRes;
}

void ICM42605_reset()
{
  // reset device
  uint8_t temp = I2Cdev_readByte(devAddr, ICM42605_DEVICE_CONFIG);
  I2Cdev_writeByte(devAddr, ICM42605_DEVICE_CONFIG, temp | 0x01); // Set bit 0 to 1 to reset ICM42605
  osSleep(10); // Wait for all registers to reset
}

uint8_t ICM42605_status()
{
  // reset device
  uint8_t temp = I2Cdev_readByte(devAddr, ICM42605_INT_STATUS);
  return temp;
}

void ICM42605_init(int gpio_sda, int gpio_scl, uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR)
{
	devAddr=ICM42605_ADDRESS;
  I2Cdev_init(gpio_sda, gpio_scl);
  ICM42605_reset();

  uint8_t temp = I2Cdev_readByte(devAddr, ICM42605_PWR_MGMT0); // make sure not to disturb reserved bit values
  I2Cdev_writeByte(devAddr, ICM42605_PWR_MGMT0, temp | 0x0F);  // enable gyro and accel in low noise mode

  temp = I2Cdev_readByte(devAddr, ICM42605_GYRO_CONFIG0);
  I2Cdev_writeByte(devAddr, ICM42605_GYRO_CONFIG0, temp | GODR | Gscale << 5); // gyro full scale and data rate

  temp = I2Cdev_readByte(devAddr, ICM42605_ACCEL_CONFIG0);
  I2Cdev_writeByte(devAddr, ICM42605_ACCEL_CONFIG0, temp | AODR | Ascale << 5); // set accel full scale and data rate

  temp = I2Cdev_readByte(devAddr, ICM42605_GYRO_CONFIG1);
  I2Cdev_writeByte(devAddr, ICM42605_GYRO_CONFIG1, temp | 0xD0); // set temperature sensor low pass filter to 5Hz, use first order gyro filter

  temp = I2Cdev_readByte(devAddr, ICM42605_INT_CONFIG);
  I2Cdev_writeByte(devAddr, ICM42605_INT_CONFIG, temp | 0x18 | 0x03 ); // set both interrupts active high, push-pull, pulsed

  temp = I2Cdev_readByte(devAddr, ICM42605_INT_CONFIG1);
  I2Cdev_writeByte(devAddr, ICM42605_INT_CONFIG1, temp & ~(0x10) ); // set bit 4 to zero for proper function of INT1 and INT2
 
  temp = I2Cdev_readByte(devAddr, ICM42605_INT_SOURCE0);
  I2Cdev_writeByte(devAddr, ICM42605_INT_SOURCE0, temp | 0x08 ); // route data ready interrupt to INT1
 
  temp = I2Cdev_readByte(devAddr, ICM42605_INT_SOURCE3);
  I2Cdev_writeByte(devAddr, ICM42605_INT_SOURCE3, temp | 0x01 ); // route AGC interrupt interrupt to INT2

  // Select Bank 4
  temp = I2Cdev_readByte(devAddr, ICM42605_REG_BANK_SEL);
  I2Cdev_writeByte(devAddr, ICM42605_REG_BANK_SEL, temp | 0x04 ); // select Bank 4

  temp = I2Cdev_readByte(devAddr, ICM42605_APEX_CONFIG5);
  I2Cdev_writeByte(devAddr, ICM42605_APEX_CONFIG5, temp & ~(0x07) ); // select unitary mounting matrix

  temp = I2Cdev_readByte(devAddr, ICM42605_REG_BANK_SEL);
  I2Cdev_writeByte(devAddr, ICM42605_REG_BANK_SEL, temp & ~(0x07) ); // select Bank 0

  temp = ICM42605_getChipID();
}

/*
void ICM42605_selfTest()
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int16_t accelPTest[3] = {0, 0, 0}, accelNTest[3] = {0, 0, 0}, gyroPTest[3] = {0, 0, 0}, gyroNTest[3] = {0, 0, 0};
  int16_t accelNom[3] = {0, 0, 0}, gyroNom[3] = {0, 0, 0};

  readData(temp);
  accelNom[0] = temp[4];
  accelNom[1] = temp[5];
  accelNom[2] = temp[6];
  gyroNom[0]  = temp[1];
  gyroNom[1]  = temp[2];
  gyroNom[2]  = temp[3];

  I2Cdev_writeByte(devAddr, ICM42605_CTRL5_C, 0x01); // positive accel self test
  osSleep(100); // let accel respond
  readData(temp);
  accelPTest[0] = temp[4];
  accelPTest[1] = temp[5];
  accelPTest[2] = temp[6];

  I2Cdev_writeByte(devAddr, ICM42605_CTRL5_C, 0x03); // negative accel self test
  osSleep(100); // let accel respond
  readData(temp);
  accelNTest[0] = temp[4];
  accelNTest[1] = temp[5];
  accelNTest[2] = temp[6];

  I2Cdev_writeByte(devAddr, ICM42605_CTRL5_C, 0x04); // positive gyro self test
  osSleep(100); // let gyro respond
  readData(temp);
  gyroPTest[0] = temp[1];
  gyroPTest[1] = temp[2];
  gyroPTest[2] = temp[3];

  I2Cdev_writeByte(devAddr, ICM42605_CTRL5_C, 0x0C); // negative gyro self test
  osSleep(100); // let gyro respond
  readData(temp);
  gyroNTest[0] = temp[1];
  gyroNTest[1] = temp[2];
  gyroNTest[2] = temp[3];

  I2Cdev_writeByte(devAddr, ICM42605_CTRL5_C, 0x00); // normal mode
  osSleep(100); // let accel and gyro respond

  INFO("Accel Self Test:");
  Serial.print("+Ax results:"); Serial.print(  (accelPTest[0] - accelNom[0]) * _aRes * 1000.0); Serial.println(" mg");
  Serial.print("-Ax results:"); Serial.println((accelNTest[0] - accelNom[0]) * _aRes * 1000.0);
  Serial.print("+Ay results:"); Serial.println((accelPTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("-Ay results:"); Serial.println((accelNTest[1] - accelNom[1]) * _aRes * 1000.0);
  Serial.print("+Az results:"); Serial.println((accelPTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.print("-Az results:"); Serial.println((accelNTest[2] - accelNom[2]) * _aRes * 1000.0);
  Serial.println("Should be between 90 and 1700 mg");

  Serial.println("Gyro Self Test:");
  Serial.print("+Gx results:"); Serial.print((gyroPTest[0] - gyroNom[0]) * _gRes); Serial.println(" dps");
  Serial.print("-Gx results:"); Serial.println((gyroNTest[0] - gyroNom[0]) * _gRes);
  Serial.print("+Gy results:"); Serial.println((gyroPTest[1] - gyroNom[1]) * _gRes);
  Serial.print("-Gy results:"); Serial.println((gyroNTest[1] - gyroNom[1]) * _gRes);
  Serial.print("+Gz results:"); Serial.println((gyroPTest[2] - gyroNom[2]) * _gRes);
  Serial.print("-Gz results:"); Serial.println((gyroNTest[2] - gyroNom[2]) * _gRes);
  Serial.println("Should be between 20 and 80 dps");
  osSleep(2000);
}
*/

/*
void MPU6050_setXAccelOffset(int16_t offset) {
    I2Cdev_writeWord(devAddr, MPU6050_RA_XA_OFFS_H, offset);
}

void MPU6050_setYAccelOffset(int16_t offset) {
	I2Cdev_writeWord(devAddr, MPU6050_RA_YA_OFFS_H, offset);
}

void MPU6050_setZAccelOffset(int16_t offset) {
	I2Cdev_writeWord(devAddr, MPU6050_RA_ZA_OFFS_H, offset);
}


void MPU6050_setXGyroOffset(int16_t offset) {
	I2Cdev_writeWord(devAddr, MPU6050_RA_XG_OFFS_USRH, offset);
}

void MPU6050_setYGyroOffset(int16_t offset) {
	I2Cdev_writeWord(devAddr, MPU6050_RA_YG_OFFS_USRH, offset);
}

void MPU6050_setZGyroOffset(int16_t offset) {
	I2Cdev_writeWord(devAddr, MPU6050_RA_ZG_OFFS_USRH, offset);
}

void SetOffsets(int TheOffsets[6])
  { MPU6050_setXAccelOffset(TheOffsets [iAx]);
  	MPU6050_setYAccelOffset(TheOffsets [iAy]);
  	MPU6050_setZAccelOffset(TheOffsets [iAz]);
  	MPU6050_setXGyroOffset (TheOffsets [iGx]);
  	MPU6050_setYGyroOffset (TheOffsets [iGy]);
  	MPU6050_setZGyroOffset (TheOffsets [iGz]);
  }
*/

void ICM42605_readData(int16_t * destination)
{
  uint8_t rawData[14];  // x/y/z accel register data stored here
  I2Cdev_readBytes(devAddr, ICM42605_TEMP_DATA1, 14, &rawData[0]);  // Read the 14 raw data registers into data array
  destination[0] = ((int16_t)rawData[0] << 8) | rawData[1] ;  // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[2] << 8) | rawData[3] ;
  destination[2] = ((int16_t)rawData[4] << 8) | rawData[5] ;
  destination[3] = ((int16_t)rawData[6] << 8) | rawData[7] ;
  destination[4] = ((int16_t)rawData[8] << 8) | rawData[9] ;
  destination[5] = ((int16_t)rawData[10] << 8) | rawData[11] ;
  destination[6] = ((int16_t)rawData[12] << 8) | rawData[13] ;
}

void ICM42605_offsetBias(float * dest1, float * dest2)
{
  int16_t temp[7] = {0, 0, 0, 0, 0, 0, 0};
  int32_t sum[7] = {0, 0, 0, 0, 0, 0, 0};

  DEBUG("Calculate accel and gyro offset biases: keep sensor flat and motionless!");
  osSleep(4000);

  for (int ii = 0; ii < 128; ii++)
  {
    ICM42605_readData(temp);
    sum[1] += temp[1];
    sum[2] += temp[2];
    sum[3] += temp[3];
    sum[4] += temp[4];
    sum[5] += temp[5];
    sum[6] += temp[6];
    osSleep(50);
  }

  dest1[0] = sum[1] * _aRes / 128.0f;
  dest1[1] = sum[2] * _aRes / 128.0f;
  dest1[2] = sum[3] * _aRes / 128.0f;
  dest2[0] = sum[4] * _gRes / 128.0f;
  dest2[1] = sum[5] * _gRes / 128.0f;
  dest2[2] = sum[6] * _gRes / 128.0f;

  if (dest1[0] > 0.8f)  {
    dest1[0] -= 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[0] < -0.8f) {
    dest1[0] += 1.0f; // Remove gravity from the x-axis accelerometer bias calculation
  }
  if (dest1[1] > 0.8f)  {
    dest1[1] -= 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[1] < -0.8f) {
    dest1[1] += 1.0f; // Remove gravity from the y-axis accelerometer bias calculation
  }
  if (dest1[2] > 0.8f)  {
    dest1[2] -= 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }
  if (dest1[2] < -0.8f) {
    dest1[2] += 1.0f; // Remove gravity from the z-axis accelerometer bias calculation
  }

}
