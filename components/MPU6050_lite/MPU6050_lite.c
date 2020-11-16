

#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include "esp_log.h"
#include "MPU6050_lite.h"

#include "I2Cdev.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#define MS2TICKS(ms)              ((ms) / portTICK_PERIOD_MS)
#define osSleep(ms)               vTaskDelay(MS2TICKS(ms));

#define READ_FREQ 25        //Reading frequency of the accelerometer

const char *TAG = "AccelGyro";


uint8_t devAddr;
uint8_t buffer[14];



const char LBRACKET = '[';
const char RBRACKET = ']';
const char COMMA    = ',';
const char BLANK    = ' ';
const char PERIOD   = '.';

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

const int usDelay = 3150;
const int NFast =  1000;
const int NSlow = 10000;
const int LinesBetweenHeaders = 5;
      int LowValue[6];
      int HighValue[6];
      int Smoothed[6];
      int LowOffset[6];
      int HighOffset[6];
      int Target[6];
      int LinesOut;
      int N;

      float map(float val, float I_Min, float I_Max, float O_Min, float O_Max){
          return(val/(I_Max-I_Min)*(O_Max-O_Min) + O_Min);
      }

/*
static int16_t accelgyroval [6]; // global to be read and written from
xSemaphoreHandle mpuSem = NULL;
StaticSemaphore_t mpuSembuf;

int AccelGyro_SemaphoreCreate(void)
{
    mpuSem = xSemaphoreCreateBinaryStatic(&mpuSembuf);
    xSemaphoreGive(mpuSem);
    if(mpuSem == NULL) {
        mpuSem = xSemaphoreCreateBinaryStatic(&mpuSembuf);
        if(mpuSem == NULL) {
          return 0;
        }
    }
    return 1;
}
*/
/*
void sAccelGyroReadTask(void *pvParam)
{
    while(1) {
      uint32_t prevTick = xTaskGetTickCount();
      setAccelGyro();
      //ESP_LOGI(TAG, "temp = %.2f  ", (MPU6050_getTemperature()/340.0+36.53));
      vTaskDelayUntil(&prevTick, MS2TICKS(1000/READ_FREQ));
    }
    vTaskDelete(NULL);
}
*/
/**
  @brief Async callable read function. Stores acceleration and gyro values into an accessable variable.

*/

/*
void setAccelGyro(void)
{
    xSemaphoreTake(mpuSem, portMAX_DELAY);
    MPU6050_getMotion6(&accelgyroval[iAx], &accelgyroval[iAy], &accelgyroval[iAz],
                             &accelgyroval[iGx], &accelgyroval[iGy], &accelgyroval[iGz]);
    xSemaphoreGive(mpuSem);
}
*/
/**
  @brief  Async callable get function. If the accelerometer is read by a periodic task,
          one should be able to get the previously read value using this function.

*/
void getAccelGyro(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
    /*
    xSemaphoreTake(mpuSem, portMAX_DELAY);
    MPU6050_getMotion6(&accelgyroval[iAx], &accelgyroval[iAy], &accelgyroval[iAz],
                             &accelgyroval[iGx], &accelgyroval[iGy], &accelgyroval[iGz]);
    *ax = accelgyroval[iAx];
    *ay = accelgyroval[iAy];
    *az = accelgyroval[iAz];
    *gx = accelgyroval[iGx];
    *gy = accelgyroval[iGy];
    *gz = accelgyroval[iGz];
    xSemaphoreGive(mpuSem);
    */
    MPU6050_getMotion6(ax, ay, az, gx, gy, gz);
}

void MPU6050_setClockSource(uint8_t source) {
    I2Cdev_writeBits(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_CLKSEL_BIT, MPU6050_PWR1_CLKSEL_LENGTH, source);
}

void MPU6050_setFullScaleGyroRange(uint8_t range) {
    I2Cdev_writeBits(devAddr, MPU6050_RA_GYRO_CONFIG, MPU6050_GCONFIG_FS_SEL_BIT, MPU6050_GCONFIG_FS_SEL_LENGTH, range);
}

void MPU6050_setFullScaleAccelRange(uint8_t range) {
    I2Cdev_writeBits(devAddr, MPU6050_RA_ACCEL_CONFIG, MPU6050_ACONFIG_AFS_SEL_BIT, MPU6050_ACONFIG_AFS_SEL_LENGTH, range);
}

void MPU6050_setSleepEnabled(bool enabled) {
    	I2Cdev_writeBit(devAddr, MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, enabled);
}

uint8_t MPU6050_getDeviceID() {
    I2Cdev_readBits(devAddr, MPU6050_RA_WHO_AM_I, MPU6050_WHO_AM_I_BIT, MPU6050_WHO_AM_I_LENGTH, buffer);
    return buffer[0];
}



void MPU6050_resetFIFO() {
    I2Cdev_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_FIFO_RESET_BIT, true);
}

void MPU6050_resetDMP() {
    I2Cdev_writeBit(devAddr, MPU6050_RA_USER_CTRL, MPU6050_USERCTRL_DMP_RESET_BIT, true);
}

void MPU6050_PID(uint8_t ReadAddress, float kP,float kI, uint8_t Loops){
    uint8_t SaveAddress = (ReadAddress == 0x3B)?((MPU6050_getDeviceID() < 0x38 )? 0x06:0x77):0x13;

    int16_t  Data;
    float Reading;
    int16_t BitZero[3];
    uint8_t shift =(SaveAddress == 0x77)?3:2;
    float Error, PTerm, ITerm[3];
    int16_t eSample;
    uint32_t eSum ;
    for (int i = 0; i < 3; i++) {
        I2Cdev_readWord(devAddr, SaveAddress + (i * shift), (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
        Reading = Data;
        if(SaveAddress != 0x13){
            BitZero[i] = Data & 1;										 // Capture Bit Zero to properly handle Accelerometer calibration
            ITerm[i] = ((float)Reading) * 8;
        } else {
            ITerm[i] = Reading * 4;
        }
    }
    for (int L = 0; L < Loops; L++) {
        eSample = 0;
        for (int c = 0; c < 100; c++) {// 100 PI Calculations
            eSum = 0;
            for (int i = 0; i < 3; i++) {
                I2Cdev_readWord(devAddr, ReadAddress + (i * 2), (uint16_t *)&Data); // reads 1 or more 16 bit integers (Word)
                Reading = Data;
                if ((ReadAddress == 0x3B)&&(i == 2)) Reading -= 16384;	//remove Gravity
                Error = -Reading;
                eSum += abs(Reading);
                PTerm = kP * Error;
                ITerm[i] += (Error * 0.001) * kI;				// Integral term 1000 Calculations a second = 0.001
                if(SaveAddress != 0x13){
                    Data = round((PTerm + ITerm[i] ) / 8);		//Compute PID Output
                    Data = ((Data)&0xFFFE) |BitZero[i];			// Insert Bit0 Saved at beginning
                } else Data = round((PTerm + ITerm[i] ) / 4);	//Compute PID Output
                I2Cdev_writeWord(devAddr, SaveAddress + (i * shift),Data);
            }
            if((c == 99) && eSum > 1000){						// Error is still to great to continue
                c = 0;
            }
            if((eSum * ((ReadAddress == 0x3B)?.05: 1)) < 5) eSample++;	// Successfully found offsets prepare to  advance
            if((eSum < 100) && (c > 10) && (eSample >= 10)) break;		// Advance to next Loop
//            delay(1);
        }
        kP *= .75;
        kI *= .75;
        for (int i = 0; i < 3; i++){
            if(SaveAddress != 0x13) {
                Data = round((ITerm[i] ) / 8);		//Compute PID Output
                Data = ((Data)&0xFFFE) |BitZero[i];	// Insert Bit0 Saved at beginning
            } else Data = round((ITerm[i]) / 4);
            I2Cdev_writeWord(devAddr, SaveAddress + (i * shift), Data);
        }
    }
    MPU6050_resetFIFO();
    MPU6050_resetDMP();
}

void MPU6050_CalibrateAccel(uint8_t Loops ) {

    float kP = 0.3;
    float kI = 20;
    float x;
    x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
    kP *= x;
    kI *= x;
    MPU6050_PID( 0x3B, kP, kI,  Loops);
}

void MPU6050_CalibrateGyro(uint8_t Loops ) {
    double kP = 0.3;
    double kI = 90;
    float x;
    x = (100 - map(Loops, 1, 5, 20, 0)) * .01;
    kP *= x;
    kI *= x;

    MPU6050_PID( 0x43,  kP, kI,  Loops);
}




bool MPU6050_testConnection() {

    return MPU6050_getDeviceID() == 0x34;
}

void MPU6050_initialize() {
	I2Cdev_init();
	devAddr=0x68;
	MPU6050_setClockSource(MPU6050_CLOCK_PLL_XGYRO);
	MPU6050_setFullScaleGyroRange(MPU6050_GYRO_FS_250);
	MPU6050_setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
	MPU6050_setSleepEnabled(false); // thanks to Jack Elston for pointing this one out!
	ESP_LOGI(TAG, "%s", MPU6050_testConnection() ? "MPU6050 connection successful\n" : "MPU6050 connection failed\n");
}



int16_t MPU6050_getTemperature() {
    I2Cdev_readBytes(devAddr, MPU6050_RA_TEMP_OUT_H, 2, buffer);
    return (((int16_t)buffer[0]) << 8) | buffer[1];
}

void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
    I2Cdev_readBytes(devAddr, MPU6050_RA_ACCEL_XOUT_H, 14, buffer);
    *ax = (((int16_t)buffer[0]) << 8) | buffer[1];
    *ay = (((int16_t)buffer[2]) << 8) | buffer[3];
    *az = (((int16_t)buffer[4]) << 8) | buffer[5];
    *gx = (((int16_t)buffer[8]) << 8) | buffer[9];
    *gy = (((int16_t)buffer[10]) << 8) | buffer[11];
    *gz = (((int16_t)buffer[12]) << 8) | buffer[13];
}

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
  } // SetOffsets

void GetSmoothed()
  { int16_t RawValue[6];
    int i;
    long Sums[6];
    for (i = iAx; i <= iGz; i++)
      { Sums[i] = 0; }
//    unsigned long Start = micros();

    for (i = 1; i <= N; i++)
      { // get sums
    	MPU6050_getMotion6(&RawValue[iAx], &RawValue[iAy], &RawValue[iAz],
                             &RawValue[iGx], &RawValue[iGy], &RawValue[iGz]);

        for (int j = iAx; j <= iGz; j++)
          Sums[j] = Sums[j] + RawValue[j];

      } // get sums
//    unsigned long usForN = micros() - Start;
//    Serial.print(" reading at ");
//    Serial.print(1000000/((usForN+N/2)/N));
//    Serial.println(" Hz");
    for (i = iAx; i <= iGz; i++)
      { Smoothed[i] = (Sums[i] + N/2) / N ; }
  } // GetSmoothed

void ForceHeader()
  { LinesOut = 99; }

void ShowProgress()
  { if (LinesOut >= LinesBetweenHeaders)
      { // show header
        ESP_LOGI(TAG, "\n\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
        LinesOut = 0;
      } // show header
    ESP_LOGI(TAG, "%c",BLANK);
    for (int i = iAx; i <= iGz; i++)
      { //ESP_LOGI(TAG, "%c%d%c%d] --> [%d%c%d%c",LBRACKET, LowOffset[i], COMMA, HighOffset[i], LowValue[i], COMMA, HighValue[i], RBRACKET);
      printf("%s: %c", TAG, LBRACKET);
      printf("%d",LowOffset[i]);
    		  printf("%c",COMMA);
      printf("%d",HighOffset[i]);
      printf("] --> [");
      printf("%d",LowValue[i]);
      printf("%c",COMMA);
      printf("%d",HighValue[i]);
        if (i == iGz)
          { printf("%c\n",RBRACKET); }
        else
          { printf("]\t"); }
      }

    LinesOut++;
  } // ShowProgres

void PullBracketsOut()
  { bool Done = false;
    int NextLowOffset[6];
    int NextHighOffset[6];

   ESP_LOGI(TAG, "expanding:");
    ForceHeader();

    while (!Done)
      { Done = true;
        SetOffsets(LowOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got low values
            LowValue[i] = Smoothed[i];
            if (LowValue[i] >= Target[i])
              { Done = false;
                NextLowOffset[i] = LowOffset[i] - 1000;
              }
            else
              { NextLowOffset[i] = LowOffset[i]; }
          } // got low values

        SetOffsets(HighOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // got high values
            HighValue[i] = Smoothed[i];
            if (HighValue[i] <= Target[i])
              { Done = false;
                NextHighOffset[i] = HighOffset[i] + 1000;
              }
            else
              { NextHighOffset[i] = HighOffset[i]; }
          } // got high values
        ShowProgress();
        for (int i = iAx; i <= iGz; i++)
          { LowOffset[i] = NextLowOffset[i];   // had to wait until ShowProgress done
            HighOffset[i] = NextHighOffset[i]; // ..
          }
     } // keep going
  } // PullBracketsOut

void SetAveraging(int NewN)
  { N = NewN;
    ESP_LOGI(TAG, "averaging %d readings each time", N);
   } // SetAveraging

void PullBracketsIn()
  { bool AllBracketsNarrow;
    bool StillWorking;
    int NewOffset[6];

    ESP_LOGI(TAG, "\nclosing in:");
    AllBracketsNarrow = false;
    ForceHeader();
    StillWorking = true;
    while (StillWorking)
      { StillWorking = false;
        if (AllBracketsNarrow && (N == NFast)){

        	//SetAveraging(NSlow);
          }
        else{ AllBracketsNarrow = true; }// tentative
        for (int i = iAx; i <= iGz; i++)
          { if (HighOffset[i] <= (LowOffset[i]+1))
              { NewOffset[i] = LowOffset[i]; }
            else
              { // binary search
                StillWorking = true;
                NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
                if (HighOffset[i] > (LowOffset[i] + 10))
                  { AllBracketsNarrow = false; }
              } // binary search
          }
        SetOffsets(NewOffset);
        GetSmoothed();
        for (int i = iAx; i <= iGz; i++)
          { // closing in
            if (Smoothed[i] > Target[i])
              { // use lower half
                HighOffset[i] = NewOffset[i];
                HighValue[i] = Smoothed[i];
              } // use lower half
            else
              { // use upper half
                LowOffset[i] = NewOffset[i];
                LowValue[i] = Smoothed[i];
              } // use upper half
          } // closing in
        ShowProgress();
      } // still working
    //ESP_LOGI(TAG,"\n Done \n");
  } // PullBracketsIn


void MPU6050_calibrate(){
		MPU6050_CalibrateAccel(6);
		  MPU6050_CalibrateGyro(6);
		  MPU6050_CalibrateAccel(1);
		  MPU6050_CalibrateGyro(1);
		  MPU6050_CalibrateAccel(1);
		  MPU6050_CalibrateGyro(1);
		  MPU6050_CalibrateAccel(1);
		  MPU6050_CalibrateGyro(1);
		  MPU6050_CalibrateAccel(1);
		  MPU6050_CalibrateGyro(1);
    /*
		for (int i = iAx; i <= iGz; i++)
	        { // set targets and initial guesses
	          Target[i] = 0; // must fix for ZAccel
	          HighOffset[i] = 0;
	          LowOffset[i] = 0;
	        } // set targets and initial guesses
	  	  Target[iAz] = 16384;
	      SetAveraging(NFast);
	     PullBracketsOut();
	     PullBracketsIn();
    */
	     ESP_LOGI(TAG, "\n--------------------- Done ---------------------\n");
}

