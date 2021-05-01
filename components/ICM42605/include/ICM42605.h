/* 07/13/2019 Copyright Tlera Corporation

    Created by Kris Winer

  This sketch uses SDA/SCL on pins 21/20 (Dragonfly default), respectively, and it uses the Dragonfly STM32L476RE Breakout Board.
  The ICM42605 is a sensor hub with embedded accel and gyro, here used as 6 DoF in a 9 DoF absolute orientation solution.

  Library may be used freely and without limit with attribution.

*/

#ifndef __ICM42605_H__
#define __ICM42605_H__

#include <stdint.h>

  float ICM42605_getAres(uint8_t Ascale);
  float ICM42605_getGres(uint8_t Gscale);
  uint8_t ICM42605_getChipID();
  void ICM42605_init(int gpio_sda, int gpio_scl, uint8_t Ascale, uint8_t Gscale, uint8_t AODR, uint8_t GODR);
  void ICM42605_offsetBias(float * dest1, float * dest2);
  void ICM42605_reset();
  void ICM42605_selfTest();
  void ICM42605_readData(int16_t * destination);
  uint8_t ICM42605_status();

#endif
