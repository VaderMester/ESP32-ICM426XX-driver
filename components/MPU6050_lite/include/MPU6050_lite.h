

#ifndef MAIN_MPU6050_LITE_H_
#define MAIN_MPU6050_LITE_H_

#define MPU6050_RA_GYRO_CONFIG      	0x1B
#define MPU6050_RA_PWR_MGMT_1      	 	0x6B
#define MPU6050_PWR1_CLKSEL_BIT         2
#define MPU6050_PWR1_CLKSEL_LENGTH      3
#define MPU6050_GCONFIG_FS_SEL_BIT      4
#define MPU6050_GCONFIG_FS_SEL_LENGTH   2
#define MPU6050_CLOCK_PLL_XGYRO         0x01
#define MPU6050_GYRO_FS_250         	0x00
#define MPU6050_ACCEL_FS_2          	0x00
#define MPU6050_RA_ACCEL_CONFIG     	0x1C
#define MPU6050_ACONFIG_AFS_SEL_BIT     4
#define MPU6050_ACONFIG_AFS_SEL_LENGTH  2
#define MPU6050_PWR1_SLEEP_BIT          6
#define MPU6050_RA_WHO_AM_I        		0x75
#define MPU6050_WHO_AM_I_BIT        	6
#define MPU6050_WHO_AM_I_LENGTH     	6
#define MPU6050_RA_USER_CTRL        	0x6A
#define MPU6050_USERCTRL_FIFO_RESET_BIT 2
#define MPU6050_USERCTRL_DMP_RESET_BIT  3
#define MPU6050_RA_TEMP_OUT_H       	0x41
#define MPU6050_RA_XA_OFFS_H        	0x06
#define MPU6050_RA_YA_OFFS_H        	0x08
#define MPU6050_RA_ZA_OFFS_H        	0x0A
#define MPU6050_RA_XG_OFFS_USRH   	  	0x13
#define MPU6050_RA_YG_OFFS_USRH     	0x15
#define MPU6050_RA_ZG_OFFS_USRH     	0x17
#define MPU6050_RA_ACCEL_XOUT_H     	0x3B


//extern void sAccelGyroReadTask(void *pvParam); 
//extern int AccelGyro_SemaphoreCreate(void);

/**
  @brief Async callable read function. Stores acceleration and gyro values into an accessable variable.

*/
//extern void setAccelGyro(void);

/**
  @brief  Async callable get function. If the accelerometer is read by a periodic task,
          one should be able to get the previously read value using this function.

*/
extern void getAccelGyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz);


float map(float val, float I_Min, float I_Max, float O_Min, float O_Max);
void MPU6050_setClockSource(uint8_t source) ;
void MPU6050_setFullScaleGyroRange(uint8_t range);
void MPU6050_setFullScaleAccelRange(uint8_t range) ;
void MPU6050_setSleepEnabled(bool enabled) ;
uint8_t MPU6050_getDeviceID() ;
void MPU6050_resetFIFO() ;
void MPU6050_resetDMP() ;
void MPU6050_PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops);
void MPU6050_CalibrateAccel(uint8_t Loops );
void MPU6050_CalibrateGyro(uint8_t Loops );
bool MPU6050_testConnection();
void MPU6050_initialize();
int16_t MPU6050_getTemperature() ;
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) ;
void MPU6050_setXAccelOffset(int16_t offset) ;
void MPU6050_setYAccelOffset(int16_t offset) ;
void MPU6050_setZAccelOffset(int16_t offset);
void MPU6050_setXGyroOffset(int16_t offset) ;
void MPU6050_setYGyroOffset(int16_t offset);
void MPU6050_setZGyroOffset(int16_t offset);
void SetOffsets(int TheOffsets[6]);
void GetSmoothed();
void ForceHeader();
void ShowProgress();
void PullBracketsOut();
void SetAveraging(int NewN);
void PullBracketsIn();
void MPU6050_calibrate();




#endif /* MAIN_MPU6050_LITE_H_ */
