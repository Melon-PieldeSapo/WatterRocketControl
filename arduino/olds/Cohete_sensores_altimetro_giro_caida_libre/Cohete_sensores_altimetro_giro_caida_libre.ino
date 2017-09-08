/*
  AirQuality library v1.0
  2010 Copyright (c) Seeed Technology Inc.  All right reserved.
 
  Original Author: Bruce.Qin
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __AIRQUALITY_H__
#define __AIRQUALITY_H__
#include"Arduino.h"
class AirQuality
{
public:
    int i ;
    long vol_standard;
    int init_voltage;
    int first_vol;
    int last_vol;
    int temp;
    int counter;
    boolean timer_index;
    boolean error;
    void init(int pin);
    int slope(void);
private:
    int _pin;
    void avgVoltage(void);
};
#endif
/*
  AirQuality library v1.0
  2010 Copyright (c) Seeed Technology Inc.  All right reserved.
 
  Original Author: Bruce.Qin
  
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

//Get the avg voltage in 5 minutes.

void AirQuality::avgVoltage()
{
  if(i==150)//sum 5 minutes
  {
    vol_standard=temp/150; 
    temp=0;
    Serial.print("Vol_standard in 5 minutes:");     
    Serial.println(vol_standard);   
    i=0;    
  }
    else 
  {
    temp+=first_vol;
    i++;
  }
}
void AirQuality::init(int pin)
{
    _pin=pin;
    pinMode(_pin,INPUT);
    unsigned char i=0;
    delay(3000);
    Serial.println("sys_starting...");
    delay(20000);//200000
    init_voltage=analogRead(_pin);

    Serial.println("The init voltage is ...");
    Serial.println(init_voltage);
    while(init_voltage)
    {
        if(init_voltage<798 && init_voltage>10)// the init voltage is ok
        {
            first_vol=analogRead(A0);//initialize first value
            last_vol=first_vol;
            vol_standard=last_vol;
            Serial.println("Sensor ready.");
            error=false;;
            break;
        }
        else if(init_voltage>798||init_voltage<=10)
        { 
            i++;
            Serial.println("waitting sensor init..(it takes 60 seconds to init)");
            delay(60000);//60000
            init_voltage=analogRead(A0);
            if(i==5)
            {
                i=0;
                error=true;
                Serial.println("Sensor Error!");
            }
          }
        else 
        break;
    }
    //init the timer 
    TCCR1A=0;//normal model
    TCCR1B=0x02;//set clock as 8*(1/16M)
    TIMSK1=0x01;//enable overflow interrupt
    Serial.println("Test begin...");
    sei();
}
int AirQuality::slope(void)
{
  while(timer_index)
  {
      if(first_vol-last_vol>400||first_vol>700)
        {
            Serial.println("High pollution! Force signal active.");   
            timer_index=0;  
            avgVoltage(); 
            return 0; 
        }
      else if((first_vol-last_vol>400&&first_vol<700)||first_vol-vol_standard>150)
        { 
            Serial.print("sensor_value:");    
            Serial.print(first_vol);          
            Serial.println("\t High pollution!");   
            timer_index=0;  
            avgVoltage();
            return 1;
            
        }
      else if((first_vol-last_vol>200&&first_vol<700)||first_vol-vol_standard>50)
        {
            //Serial.println(first_vol-last_vol);
            Serial.print("sensor_value:");
            Serial.print(first_vol);          
            Serial.println("\t Low pollution!");    
            timer_index=0;
            avgVoltage();
            return 2; 
        }
      else
        {
            avgVoltage(); 
            Serial.print("sensor_value:");
            Serial.print(first_vol);
            Serial.println("\t Air fresh");
            timer_index=0;
            return 3;
        }
  }
    return -1;
}



/*
 * File name  : kalmanFilter.h
 * Description:  
 * Author     : Oliver Wang from Seeed studio
 * Version    : V0.1
 * Create Time: 2014/04
 * Change Log :
*/

#ifndef _KALMANFILTER_H
#define _KALMANFILTER_H
/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <Arduino.h>
#include <inttypes.h>
/****************************************************************************/
/***        Local variables                                               ***/
/****************************************************************************/


/****************************************************************************/
/***        Class Definitions                                             ***/
/****************************************************************************/
class KalmanFilter
{
    public:
    KalmanFilter();  
    float Filter(float);
  private:
  /* variables */
  float X_pre, X_post, P_pre, P_post, K_cur; 
  float Gaussian_Noise_Cov(void);
  
};
extern KalmanFilter kalmanFilter;
#endif
/*
 * File name  : HP20x_dev.h
 * Description: Driver for I2C PRECISION BAROMETER AND ALTIMETER [HP206C]
 * Author     : Oliver Wang from Seeed studio
 * Version    : V0.1
 * Create Time: 2014/04
 * Change Log :
*/
#ifndef _HP20X_DEV_H
#define _HP20X_DEV_H
/****************************************************************************/
/***        Including Files                                               ***/
/****************************************************************************/
#include <Wire.h>
#include <Arduino.h>
/****************************************************************************/
/***        Macro Definitions                                             ***/
/****************************************************************************/
typedef unsigned int    uint;
typedef unsigned char   uchar;
typedef unsigned long   ulong;

#define HP20X_I2C_DEV_ID       (0xEC)>>1    //CSB PIN is VDD level(address is 0x76)
#define HP20X_I2C_DEV_ID2      (0XEE)>>1    //CSB PIN is GND level(address is 0x77)
#define HP20X_SOFT_RST         0x06
#define HP20X_WR_CONVERT_CMD   0x40
#define HP20X_CONVERT_OSR4096  0<<2
#define HP20X_CONVERT_OSR2048  1<<2
#define HP20X_CONVERT_OSR1024  2<<2
#define HP20X_CONVERT_OSR512   3<<2
#define HP20X_CONVERT_OSR256   4<<2
#define HP20X_CONVERT_OSR128   5<<2

#define HP20X_READ_P           0x30   //read_p command
#define HP20X_READ_A           0x31   //read_a command
#define HP20X_READ_T           0x32   //read_t command
#define HP20X_READ_PT          0x10   //read_pt command
#define HP20X_READ_AT          0x11   //read_at command
#define HP20X_READ_CAL       0X28   //RE-CAL ANALOG

#define HP20X_WR_REG_MODE      0xC0
#define HP20X_RD_REG_MODE      0x80

#define ERR_WR_DEVID_NACK       0x01    
#define ERR_RD_DEVID_NACK       0x02    
#define ERR_WR_REGADD_NACK      0x04   
#define ERR_WR_REGCMD_NACK      0x08   
#define ERR_WR_DATA_NACK        0x10     
#define ERR_RD_DATA_MISMATCH    0x20 

#define I2C_DID_WR_MASK         0xFE
#define I2C_DID_RD_MASK         0x01

#define T_WIN_EN                0X01
#define PA_WIN_EN               0X02
#define T_TRAV_EN               0X04
#define PA_TRAV_EN              0X08
#define PA_RDY_EN               0X20
#define T_RDY_EN                0X10

#define T_WIN_CFG               0X01
#define PA_WIN_CFG              0X02
#define PA_MODE_P               0X00
#define PA_MODE_A               0X40

#define T_TRAV_CFG              0X04

#define OK_HP20X_DEV            0X80    //HP20x_dev successfully initialized
#define REG_PARA                0X0F        //Status register

/****************************************************************************/
/***        Class Definitions                                             ***/
/****************************************************************************/
class HP20x_dev : public TwoWire
{
  /* Public variables and functions */
  public:
    uchar OSR_CFG;
  uint  OSR_ConvertTime;
    /* Constructor */
    HP20x_dev();  
  void begin();
  uchar isAvailable();
  
  /* Read sensor data */
  ulong ReadTemperature(void);
  ulong ReadPressure(void);
  ulong ReadAltitude(void);
  
  /* Private variables and functions */
  private:
    /* Write a command to HP20x */
  void HP20X_IIC_WriteCmd(uchar uCmd);  
  /* Read register value */
  uchar HP20X_IIC_ReadReg(uchar bReg);  
  void HP20X_IIC_WriteReg(uchar bReg,uchar bData);    
  ulong HP20X_IIC_ReadData(void);
  ulong HP20X_IIC_ReadData3byte(void);
};
extern HP20x_dev HP20x;
#endif

/*
 * File name  : KalmanFilter.cpp
 * Description: Kalman Filter class 
 * Author     : Oliver Wang from Seeed studio
 * Version    : V0.1
 * Create Time: 2014/04
 * Change Log :
*/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <Arduino.h>
#include <inttypes.h>
#include <stdlib.h>
#include <stdio.h>

/* random number table */
float Rand_Table[100]={
0.5377,1.8339,-2.2588,0.8622,0.3188,-1.3077,-0.4336,0.342,3.5784, 
2.7694,-1.3499,3.0349,0.7254,-0.0631,0.7147,-0.2050,-0.1241,1.4897, 
1.4090,1.4172,0.6715,-1.2075,0.7172,1.6302,0.4889,1.0347,0.7269, 
-0.3034,0.2939,-0.7873,0.8884,-1.1471,-1.0689,-0.8095,-2.9443,1.4384, 
0.3252,-0.7549,1.3703,-1.7115,-0.1022,-0.2414,0.3192,0.3129,-0.8649, 
-0.0301,-0.1649,0.6277,1.0933,1.1093,-0.8637,0.0774,-1.2141,-1.1135, 
-0.0068,1.5326,-0.7697,0.3714,-0.2256,1.1174,-1.0891,0.0326,0.5525, 
1.1006,1.5442,0.0859,-1.4916,-0.7423,-1.0616,2.3505,-0.6156,0.7481, 
-0.1924,0.8886,-0.7648,-1.4023,-1.4224,0.4882,-0.1774,-0.1961,1.4193, 
0.2916,0.1978,1.5877,-0.8045,0.6966,0.8351,-0.2437,0.2157,-1.1658, 
-1.1480,0.1049,0.7223,2.5855,-0.6669,0.1873,-0.0825,-1.9330,-0.439, 
-1.7947};

/* Extern variables */
KalmanFilter kalmanFilter;

KalmanFilter::KalmanFilter()
{
    X_pre = 0;
  P_pre = 0;   
  X_post = 0;
  P_post = 0;
  K_cur = 0;
}

float KalmanFilter::Gaussian_Noise_Cov(void)
{
    int index = 0;
  float tmp[10]={0.0};
  float average = 0.0;
  float sum = 0.0;
  /* Initialize random number generator */
  srand((int)analogRead(0));
  
  /* Get random number */
  for(int i=0; i<10; i++)
  {
      index = (int)rand()%100;
        tmp[i] = Rand_Table[index];
        sum += tmp[i];    
  }
  
  /* Calculate average */
  average = sum/10;
  
  /* Calculate Variance */
  float Variance = 0.0;   
  for(int j = 0; j < 10; j++)
  {
      Variance += (tmp[j]-average)*(tmp[j]-average);
  }
  Variance/=10.0;
  
  return Variance;
}

float KalmanFilter::Filter(float origin)
{
    float modelNoise = 0.0;
  float observeNoise = 0.0;
  
  /* Get model and observe Noise */
  modelNoise = Gaussian_Noise_Cov();
  observeNoise = Gaussian_Noise_Cov();
  
  /* Algorithm */
  X_pre = X_post;
  P_pre = P_post + modelNoise;
  K_cur = P_pre/(P_pre + observeNoise);
  P_post = (1 - K_cur)*P_pre;
  X_post = X_pre + K_cur*(origin - X_pre);
  
  return X_post;
}


/*
 * File name  : HP20x_dev.cpp
 * Description: Driver for I2C PRECISION BAROMETER AND ALTIMETER [HP206C]
 * Author     : Oliver Wang from Seeed studio
 * Version    : V0.1
 * Create Time: 2014/04
 * Change Log :
*/

/****************************************************************************/
/***        Include files                                                 ***/
/****************************************************************************/
#include <Wire.h>
#include <Arduino.h>
/****************************************************************************/
/***       Local Variable                                                 ***/
/****************************************************************************/
 HP20x_dev HP20x;


/****************************************************************************/
/***       Class member Functions                                         ***/
/****************************************************************************/
/*
 **@ Function name: HP20x_dev
 **@ Description: Constructor
 **@ Input: none
 **@ OutPut: none
 **@ Retval: none
*/
HP20x_dev::HP20x_dev()
{
    OSR_CFG = HP20X_CONVERT_OSR1024;
    OSR_ConvertTime = 25; 
}

/*
 **@ Function name: begin
 **@ Description: Initialize HP20x_dev
 **@ Input: none
 **@ OutPut: none
 **@ Retval: none
*/
void HP20x_dev::begin()
{
  //Wire.begin();
  /* Reset HP20x_dev */
  HP20x.HP20X_IIC_WriteCmd(HP20X_SOFT_RST);
}

/*
 **@ Function name: isAvailable
 **@ Description: Indicate whether it's available
 **@ Input: none
 **@ OutPut: none
 **@ Retval: uchar 
*/
uchar HP20x_dev::isAvailable()
{
  uchar ret = HP20x.HP20X_IIC_ReadReg(REG_PARA);
  return ret;
}
/*
 **@ Function name: ReadTemperature
 **@ Description: Read Temperature from HP20x_dev
 **@ Input:
 **@ OutPut:
 **@ Retval:
*/
ulong HP20x_dev::ReadTemperature(void)
{
    uchar Temp;
    uchar Temp0;
     
  HP20X_IIC_WriteCmd(HP20X_WR_CONVERT_CMD|OSR_CFG); //ADC convert
 
    delay(OSR_ConvertTime);                         //difference OSR_CFG will be difference OSR_ConvertTime
    HP20X_IIC_WriteCmd(HP20X_READ_T);      
    ulong Temperature = HP20X_IIC_ReadData();
    return Temperature;   
}

/*
 **@ Function name: ReadPressure
 **@ Description: Read Pressure value
 **@ Input:
 **@ OutPut: 
 **@ Retval: value
*/
 
ulong HP20x_dev::ReadPressure(void)
{
    HP20X_IIC_WriteCmd(HP20X_WR_CONVERT_CMD|OSR_CFG);
    delay(OSR_ConvertTime);
    HP20X_IIC_WriteCmd(HP20X_READ_P);
    ulong Pressure = HP20X_IIC_ReadData();             
    return Pressure;
} 

/*
 **@ Function name: ReadAltitude
 **@ Description: Read Pressure value
 **@ Input:
 **@ OutPut: 
 **@ Retval: value
*/
ulong HP20x_dev::ReadAltitude(void)
{
    HP20X_IIC_WriteCmd(HP20X_READ_A);
    ulong Altitude = HP20X_IIC_ReadData();   
    return Altitude;    
} 
 
/*
void ReadPressureAndTemperature(void)
{
        HP20X_IIC_WriteCmd(HP20X_WR_CONVERT_CMD|OSR_CFG);
        Timer_Delayxms(OSR_ConvertTime*2);
        HP20X_IIC_WriteCmd(HP20X_READ_PT);
        
        Temperature=HP20X_IIC_ReadData();
       
        Pressure=HP20X_IIC_ReadData3byte();       
}

void IIC_ReadAltitudeAndTemperature(void)
{

       HP20X_IIC_WriteCmd(HP20X_WR_CONVERT_CMD|OSR_CFG);
       Timer_Delayxms(OSR_ConvertTime*2);
       HP20X_IIC_WriteCmd(HP20X_READ_AT);
        
        Temperature=HP20X_IIC_ReadData();
        IIC_ACK();
        Altitude=HP20X_IIC_ReadData3byte();
        IIC_NoAck();      
        IIC_Stop();  
                   
}*/
/****************************************************************************/
/***       Local Functions                                                ***/
/****************************************************************************/

/*
 **@ Function name: HP20X_IIC_WriteCmd
 **@ Description:
 **@ Input:
 **@ OutPut:
 **@ Retval:
*/
void HP20x_dev::HP20X_IIC_WriteCmd(uchar uCmd)
{   
  /* Port to arduino */
  Wire.beginTransmission(HP20X_I2C_DEV_ID);
  Wire.write(uCmd);
  Wire.endTransmission();
}

/*
 **@ Function name: HP20X_IIC_ReadReg
 **@ Description:
 **@ Input:
 **@ OutPut:
 **@ Retval:  
*/
uchar HP20x_dev::HP20X_IIC_ReadReg(uchar bReg)
{
    /* Port to arduino */
    uchar Temp = 0;
  
  /* Send a register reading command */
    HP20X_IIC_WriteCmd(bReg|HP20X_RD_REG_MODE); 
   
  Wire.requestFrom(HP20X_I2C_DEV_ID, 1);   
  while(Wire.available())
  {
      Temp = Wire.read();
  }
   
  return Temp;
} 
/*
 **@ Function name: HP20X_IIC_WriteReg
 **@ Description:
 **@ Input:
 **@ OutPut:
 **@ Retval:
*/
void HP20x_dev::HP20X_IIC_WriteReg(uchar bReg,uchar bData)
{       
  Wire.beginTransmission(HP20X_I2C_DEV_ID);
  Wire.write(bReg|HP20X_WR_REG_MODE);
  Wire.write(bData);
  Wire.endTransmission();
}


/*
 **@ Function name: HP20X_IIC_ReadData
 **@ Description:
 **@ Input:
 **@ OutPut:
 **@ Retval:
*/
ulong HP20x_dev::HP20X_IIC_ReadData(void)
{                        
  /* Port to arduino */  
  ulong Temp = HP20X_IIC_ReadData3byte(); 
  return Temp;
}

/*
 **@ Function name: HP20X_IIC_ReadData3byte
 **@ Description:
 **@ Input:
 **@ OutPut:
 **@ Retval:
*/
ulong HP20x_dev::HP20X_IIC_ReadData3byte(void)
{ 
  ulong TempData = 0;
  ulong tmpArray[3]={0};
  int cnt = 0;
  
  /* Require three bytes from slave */
  Wire.requestFrom(HP20X_I2C_DEV_ID, 3);      

    while(Wire.available())     // slave may send less than requested
    { 
      uchar c = Wire.read();    // receive a byte as character        
    tmpArray[cnt] = (ulong)c;
    cnt++;
  }
  
  /* MSB */
  TempData = tmpArray[0]<<16 | tmpArray[1]<<8 | tmpArray[2];

  
    if(TempData&0x800000)
    {
      TempData|=0xff000000;
  }

 /*   // 24 bit to 32 bit 
  if(TempData&0x800000)
  {
    // 1:minus 
    TempData |= 0x80000000;
    TempData &= 0xff7fffff;
  }
  else
  {
    // 0:plus 
    //do noting
  }  */
  return TempData;
} 


/**************************************END OF FILE**************************************/
/* LSM303DLM Example Code base on LSM303DLH example code by Jim Lindblom SparkFun Electronics
   
   date: 9/6/11
   license: Creative commons share-alike v3.0
   
   Modified by:Frankie.Chu
   Modified by:Jacky.Zhang 2014-12-11: Ported to 6-Axis Accelerometer&Compass of Seeed Studio
   Modified by:Jacky.Zhang 2015-1-6: added SPI driver
   
   Summary:
   Show how to calculate level and tilt-compensated heading using
   the snazzy LSM303DLH 3-axis magnetometer/3-axis accelerometer.
   
   Firmware:
   You can set the accelerometer's full-scale range by setting
   the SCALE constant to either 2, 4, or 8. This value is used
   in the initLSM303() function. For the most part, all other
   registers in the LSM303 will be at their default value.
   
   Use the write() and read() functions to write
   to and read from the LSM303's internal registers.
   
   Use getLSM303_accel() and getLSM303_mag() to get the acceleration
   and magneto values from the LSM303. You'll need to pass each of
   those functions an array, where the data will be stored upon
   return from the void.
   
   getHeading() calculates a heading assuming the sensor is level.
   A float between 0 and 360 is returned. You need to pass it a
   array with magneto values. 
   
   getTiltHeading() calculates a tilt-compensated heading.
   A float between 0 and 360 degrees is returned. You need
   to pass this function both a magneto and acceleration array.
   
   Headings are calculated as specified in AN3192:
   http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf
*/

#ifndef Accelerometer_Compass_LSM303D_H
#define Accelerometer_Compass_LSM303D_H

#include <Arduino.h>

#define ACCELE_SCALE 2  // accelerometer full-scale, should be 2, 4, or 8

#define X 0
#define Y 1
#define Z 2

class LSM303D
{
  public:

    char initI2C();
        char initSPI(char cspin);
        char config();
    unsigned char read(unsigned char address);
    void write(unsigned char data, unsigned char address);
    char isMagReady();
    void getMag(int * rawValues);
    void getAccel(int * rawValues);
    float getHeading(int * magValue);
    float getTiltHeading(int * magValue, float * accelValue);
  private:
        unsigned char _mode;
        unsigned char _cs;
};

extern  LSM303D Lsm303d;

#endif

/* LSM303DLM Example Code base on LSM303DLH example code by Jim Lindblom SparkFun Electronics
   
   date: 9/6/11
   license: Creative commons share-alike v3.0
   
   Modified by:Frankie.Chu
   Modified by:Jacky.Zhang 2014-12-11: Ported to 6-Axis Accelerometer&Compass of Seeed Studio
   Modified by:Jacky.Zhang 2015-1-6: added SPI driver
   
   Summary:
   Show how to calculate level and tilt-compensated heading using
   the snazzy LSM303DLH 3-axis magnetometer/3-axis accelerometer.
   
   Firmware:
   You can set the accelerometer's full-scale range by setting
   the SCALE constant to either 2, 4, or 8. This value is used
   in the initLSM303() function. For the most part, all other
   registers in the LSM303 will be at their default value.
   
   Use the write() and read() functions to write
   to and read from the LSM303's internal registers.
   
   Use getLSM303_accel() and getLSM303_mag() to get the acceleration
   and magneto values from the LSM303. You'll need to pass each of
   those functions an array, where the data will be stored upon
   return from the void.
   
   getHeading() calculates a heading assuming the sensor is level.
   A float between 0 and 360 is returned. You need to pass it a
   array with magneto values. 
   
   getTiltHeading() calculates a tilt-compensated heading.
   A float between 0 and 360 degrees is returned. You need
   to pass this function both a magneto and acceleration array.
   
   Headings are calculated as specified in AN3192:
   http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf
*/

/*
hardware & software comment

I2C mode:
1, solder the jumper "I2C EN" and the jumper of ADDR to 0x1E
2, use Lsm303d.initI2C() function to initialize the Grove by I2C

SPI mode:

1, break the jumper "I2C_EN" and the jumper ADDR to any side
2, define a pin as chip select for SPI protocol.
3, use Lsm303d.initSPI(SPI_CS) function to initialize the Grove by SPI
SPI.h sets these for us in arduino
const int SDI = 11;
const int SDO = 12;
const int SCL = 13;
*/


#include <Wire.h>
#include <SPI.h>

/* LSM303 Address definitions */
#define LSM303D_ADDR  0x1E  // assuming SA0 grounded

/* LSM303 Register definitions */
#define TEMP_OUT_L    0x05
#define TEMP_OUT_H    0x06
#define STATUS_REG_M  0x07
#define OUT_X_L_M     0x08
#define OUT_X_H_M     0x09
#define OUT_Y_L_M     0x0A
#define OUT_Y_H_M     0x0B
#define OUT_Z_L_M     0x0C
#define OUT_Z_H_M     0x0D
#define WHO_AM_I    0x0F
#define INT_CTRL_M    0x12
#define INT_SRC_M   0x13
#define INT_THS_L_M   0x14
#define INT_THS_H_M   0x15
#define OFFSET_X_L_M  0x16
#define OFFSET_X_H_M  0x17
#define OFFSET_Y_L_M  0x18
#define OFFSET_Y_H_M  0x19
#define OFFSET_Z_L_M  0x1A
#define OFFSET_Z_H_M  0x1B
#define REFERENCE_X   0x1C
#define REFERENCE_Y   0x1D
#define REFERENCE_Z   0x1E
#define CTRL_REG0     0x1F
#define CTRL_REG1     0x20
#define CTRL_REG2   0x21
#define CTRL_REG3     0x22
#define CTRL_REG4     0x23
#define CTRL_REG5     0x24
#define CTRL_REG6     0x25
#define CTRL_REG7     0x26
#define STATUS_REG_A  0x27
#define OUT_X_L_A     0x28
#define OUT_X_H_A     0x29
#define OUT_Y_L_A     0x2A
#define OUT_Y_H_A     0x2B
#define OUT_Z_L_A     0x2C
#define OUT_Z_H_A     0x2D
#define FIFO_CTRL   0x2E
#define FIFO_SRC    0x2F
#define IG_CFG1     0x30
#define IG_SRC1     0x31
#define IG_THS1     0x32
#define IG_DUR1     0x33
#define IG_CFG2     0x34
#define IG_SRC2     0x35
#define IG_THS2     0x36
#define IG_DUR2     0x37
#define CLICK_CFG   0x38
#define CLICK_SRC   0x39
#define CLICK_THS   0x3A
#define TIME_LIMIT    0x3B
#define TIME_LATENCY  0x3C
#define TIME_WINDOW   0x3D
#define ACT_THS     0x3E
#define ACT_DUR     0x3F

#define MAG_SCALE_2   0x00//full-scale is +/-2Gauss
#define MAG_SCALE_4   0x20//+/-4Gauss
#define MAG_SCALE_8   0x40//+/-8Gauss
#define MAG_SCALE_12  0x60//+/-12Gauss

byte Read  = 0B10000000;
byte Write = 0B00000000;

//I2C mode
char LSM303D::initI2C()
{
  char rtn = -1;
    
    _mode = 0;//I2C mode
  Wire.begin();  // Start up I2C, required for LSM303 communication
  rtn = config();
    
  return rtn;
}

//SPI mode
char LSM303D::initSPI(char cspin)
{
  char rtn = -1;
    
    _mode = 1;//SPI mode
    _cs = cspin;
    pinMode(_cs, OUTPUT);//initialize the chip select pins;
    SPI.begin();//start the SPI library;
    rtn = config();
    
  return rtn;
}

char LSM303D::config()
{
    char rtn = -1;
    
    if(read(WHO_AM_I) != 0x49) return rtn; // return wrong if no LSM303D was found 
    
  write(0x57, CTRL_REG1);  // 0x57 = ODR=50hz, all accel axes on
  write((3<<6)|(0<<3), CTRL_REG2);  // set full-scale
  write(0x20, CTRL_REG3);  // no interrupt
  write(0x20, CTRL_REG4);  // no interrupt
  write(0x13, CTRL_REG5);  //(4<<2)- 0x10 = mag 50Hz output rate
  write(MAG_SCALE_2, CTRL_REG6); //magnetic scale = +/-1.3Gauss
  write(0x00, CTRL_REG7);  // 0x00 = continouous conversion mode
  write(0x2A,IG_CFG2);  //con los up/down determinamos si queremos que dispare cuando el valor leido sea mas alto o mas bajo que el humbral.
  write(0x47,IG_THS2); //determina el valor umbral al que se dispara la señal
  write(0x10,IG_CFG1);
  write(0x0A,IG_THS1);
  rtn = 0;
    
  return rtn;
}

unsigned char LSM303D::read(unsigned char address)
{
  char temp = 0x00;
    
    if(_mode == 0)//I2C mode
    {
        Wire.beginTransmission(LSM303D_ADDR);
        Wire.write(address);
        Wire.endTransmission();
        Wire.requestFrom(LSM303D_ADDR, 1);
        while(!Wire.available());
        temp = Wire.read();
        Wire.endTransmission();
    }
    else//SPI Mode
    {
        digitalWrite(_cs, LOW); 
        SPI.transfer(Read | address);
        temp = SPI.transfer(0x00);
        digitalWrite(_cs, HIGH);
    }
    
  return temp;
}

void LSM303D::write(unsigned char data, unsigned char address)
{
    if(_mode == 0)
    {
        Wire.beginTransmission(LSM303D_ADDR); 
        Wire.write(address);
        Wire.write(data);
        Wire.endTransmission();
    }
    else
    {
        digitalWrite(_cs, LOW);
        SPI.transfer(Write | address);
        SPI.transfer(data);
        digitalWrite(_cs, HIGH);
    }
}

char LSM303D::isMagReady()
{
  char temp;
    
  temp = read(STATUS_REG_M) & 0x03;
    
  return temp;
}

void LSM303D::getMag(int * rawValues)
{
  rawValues[X] = ((int)read(OUT_X_H_M) << 8) | (read(OUT_X_L_M));
  rawValues[Y] = ((int)read(OUT_Y_H_M) << 8) | (read(OUT_Y_L_M));
  rawValues[Z] = ((int)read(OUT_Z_H_M) << 8) | (read(OUT_Z_L_M));
}

void LSM303D::getAccel(int * rawValues)
{
  rawValues[X] = ((int)read(OUT_X_H_A) << 8) | (read(OUT_X_L_A));
  rawValues[Y] = ((int)read(OUT_Y_H_A) << 8) | (read(OUT_Y_L_A));
  rawValues[Z] = ((int)read(OUT_Z_H_A) << 8) | (read(OUT_Z_L_A));
}

float LSM303D::getHeading(int * magValue)
{
  // see section 1.2 in app note AN3192
  float heading = 180*atan2(magValue[Y], magValue[X])/PI;  // assume pitch, roll are 0

  if (heading <0)
  heading += 360;

  return heading;
}

float LSM303D::getTiltHeading(int * magValue, float * accelValue)
{
  // see appendix A in app note AN3192 
  float pitch = asin(-accelValue[X]);
  float roll = asin(accelValue[Y]/cos(pitch));

  float xh = magValue[X] * cos(pitch) + magValue[Z] * sin(pitch);
  float yh = magValue[X] * sin(roll) * sin(pitch) + magValue[Y] * cos(roll) - magValue[Z] * sin(roll) * cos(pitch);
  float zh = -magValue[X] * cos(roll) * sin(pitch) + magValue[Y] * sin(roll) + magValue[Z] * cos(roll) * cos(pitch);
  float heading = 180 * atan2(yh, xh)/PI;

  if (yh >= 0)
    return heading;
  else
    return (360 + heading);
}

LSM303D Lsm303d;
/* LSM303DLM Example Code base on LSM303DLH example code by Jim Lindblom SparkFun Electronics
   
   date: 9/6/11
   license: Creative commons share-alike v3.0
   
   Modified by:Frankie.Chu
   Modified by:Jacky.Zhang 2014-12-11: Ported to 6-Axis Accelerometer&Compass of Seeed Studio
   Modified by:Jacky.Zhang 2015-1-6: added SPI driver
   
   Summary:
   Show how to calculate level and tilt-compensated heading using
   the snazzy LSM303DLH 3-axis magnetometer/3-axis accelerometer.
   
   Firmware:
   You can set the accelerometer's full-scale range by setting
   the SCALE constant to either 2, 4, or 8. This value is used
   in the initLSM303() function. For the most part, all other
   registers in the LSM303 will be at their default value.
   
   Use the write() and read() functions to write
   to and read from the LSM303's internal registers.
   
   Use getLSM303_accel() and getLSM303_mag() to get the acceleration
   and magneto values from the LSM303. You'll need to pass each of
   those functions an array, where the data will be stored upon
   return from the void.
   
   getHeading() calculates a heading assuming the sensor is level.
   A float between 0 and 360 is returned. You need to pass it a
   array with magneto values. 
   
   getTiltHeading() calculates a tilt-compensated heading.
   A float between 0 and 360 degrees is returned. You need
   to pass this function both a magneto and acceleration array.
   
   Headings are calculated as specified in AN3192:
   http://www.sparkfun.com/datasheets/Sensors/Magneto/Tilt%20Compensated%20Compass.pdf
*/

/*
hardware & software comment

I2C mode:
1, solder the jumper "I2C EN" and the jumper of ADDR to 0x1E
2, use Lsm303d.initI2C() function to initialize the Grove by I2C

SPI mode:

1, break the jumper "I2C_EN" and the jumper ADDR to any side
2, define a pin as chip select for SPI protocol.
3, use Lsm303d.initSPI(SPI_CS) function to initialize the Grove by SPI
SPI.h sets these for us in arduino
const int SDI = 11;
const int SDO = 12;
const int SCL = 13;
*/

/* Global variables */
int accel[3];  // we'll store the raw acceleration values here
int mag[3];  // raw magnetometer values stored here
float realAccel[3];  // calculated acceleration values here
float heading, titleHeading;
KalmanFilter t_filter;    //temperature filter
KalmanFilter p_filter;    //pressure filter
KalmanFilter a_filter;    //altitude filter

unsigned char ret = 0;
#define SPI_CS 10

#include <SoftwareSerial.h>

int state = 0; /*0 subiendo, 1 arriba, 2 callendo**/
float sum = 0;
float altura=0;
SoftwareSerial SoftSerial(4, 5);
SoftwareSerial Radio(7, 8);
unsigned char buffer[64];                   // buffer array for data receive over serial port
unsigned char gps_data[64];                   // buffer array for data receive over serial port
int count=0;   
int gps_count=0;   
//GPS PARSING VARIABLES
        bool found_g = false;
        bool found_a = false;
        bool found_end=false;
        int a_position = 0;
        int end_position=0;
        String gps_payload="";

//AIR QUALITY        
AirQuality airqualitysensor;
int current_quality =-1;

void setup()
{
  char rtn = 0;
    Serial.begin(9600);  // Serial is used for debugging
    delay(2000); //allow serial inicialization;
    Serial.println("\r\npower on");
    SoftSerial.begin(9600);
    delay(2000);
    Serial.println("Radio on");
    rtn = Lsm303d.initI2C();
    //rtn = Lsm303d.initSPI(SPI_CS);
    if(rtn != 0)  // Initialize the LSM303, using a SCALE full-scale range
  {
    Serial.println("\r\nLSM303D is not found");
    while(1);
  }
  else
  {
    Serial.println("\r\nLSM303D is found");
  }
  delay(150);
  /* Reset HP20x_dev */
  //HP20x.begin();
  delay(100);
    /* Determine HP20x_dev is available or not */
  ret = HP20x.isAvailable();
  if(OK_HP20X_DEV == ret)
  {
    Serial.println("HP20x_dev is available.\n");    
  }
  else
  {
    Serial.println("HP20x_dev isn't available.\n");
  }
  Serial.println("Starting Airquality");
    airqualitysensor.init(14);
  Serial.println("All ready to go");

}

void loop()
{
  //Serial.println("\r\n**************");
  //getLSM303_accel(accel);  // get the acceleration values and store them in the accel array
  Lsm303d.getAccel(accel);
  while(!Lsm303d.isMagReady());// wait for the magnetometer readings to be ready
  Lsm303d.getMag(mag);  // get the magnetometer values, store them in mag
  
  for (int i=0; i<3; i++)
  {
    realAccel[i] = accel[i] / pow(2, 15) * ACCELE_SCALE;  // calculate real acceleration values, in units of g
  }
  heading = Lsm303d.getHeading(mag);
  titleHeading = Lsm303d.getTiltHeading(mag, realAccel);
 //Serial.print("0x");
 // Serial.print(Lsm303d.read(IG_SRC2));
 //  Serial.print(" - 0x");
 // Serial.print(Lsm303d.read(IG_SRC1)); //mostrar el valor de la interrupcion (para limpiar el registro)
 //Serial.print(" - ");
 // printValues();
 // printTest();
  //varometer();
  //test_vuelo_1();//Acelerometro
  //test_vuelo_2();//Altimetro
  //test_vuelo_3();//con el gps
 //gps();
 gps_parsed(); //se procesa 1 de cada 3 veces
 //Serial.println("bucle principal");
 print_serial();

  delay(300);  // delay for serial readability (delay 200
}

void gps(){
      if (SoftSerial.available())                     // if date is coming from software serial port ==> data is coming from SoftSerial shield
    {
        while(SoftSerial.available())               // reading data into char array
        {
            buffer[count++]=SoftSerial.read();      // writing data into array
            if(count == 64)break;
        }
        Serial.write(buffer,count);                 // if no data transmission ends, write buffer to hardware serial port
        clearBufferArray();                         // call clearBufferArray function to clear the stored data from the array
        count = 0;                                  // set counter of while loop to zero 
    }
//    if (Serial.available())                 // if data is available on hardware serial port ==> data is coming from PC or notebook
//    SoftSerial.write(Serial.read());
  }
  void gps_parsed(){
     if (SoftSerial.available())                     // if date is coming from software serial port ==> data is coming from SoftSerial shield
    {
        while(SoftSerial.available())               // reading data into char array
        {
            buffer[count++]=SoftSerial.read();      // writing data into array
            if(count == 64)break;
        }
        for(int i=0;i<count;i++){
          //Serial.print(buffer[i]);
          //Serial.print(" :- ");
          //Serial.write(&buffer[i],1);
          if(buffer[i]==0x47){
            //Serial.println("\nFound G");
            found_g = true;
          }else if(buffer[i]==0x41 && found_g){
            found_a=true;
            a_position=i; //+13
            //Serial.println("\nFound A");
          }else if(found_g){
            found_g=false;
          }
          if(found_a && buffer[i]==0x4D){
            end_position=i+1;
            found_end=true;
            break;
            //Serial.println("\nFound *");
          }
        }
        //Serial.println("#endbuf");
        if(found_a==true){
          //Serial.print("we found A");
          if(found_end==false){
            //Serial.println(" with no end");
            for(int i=a_position;i<count;i++){
              gps_data[gps_count++]=buffer[i];
            }
          }else{
           // Serial.println(" with end");
            for(int i=a_position;i<end_position;i++){
              gps_data[gps_count++]=buffer[i];
            }
            found_a = false;
            found_g = false;
            found_end=false;
            a_position=0;
            end_position=0;
           //write(gps_data,(gps_count));                 // if no data transmission ends, write buffer to hardware serial port
          String test = gps_data;
//          Serial.println(test);
          int first = test.indexOf("00,")+3;
          int second = test.indexOf("W,")+1;
//          Serial.println(test.substring(first,second));
//          Serial.println(test.substring(second+5+(test.substring(second+5)).indexOf(",")));
          gps_payload ="["+test.substring(first,second) + test.substring(second+5+(test.substring(second+5)).indexOf(","))+"]";
           //todo: hacer esto con parses y sacar un string en condificones
           //Serial.write(gps_data,(24)); 
           //Serial.write(&gps_data[33],(8)); 
           //Serial.print('\n');
           clearGpsBufferArray();                         // call clearBufferArray function to clear the stored data from the array
           gps_count=0;
          }
        }
     // Serial.write(buffer,count);                 // if no data transmission ends, write buffer to hardware serial port
        clearBufferArray();                         // call clearBufferArray function to clear the stored data from the array
        count = 0;                                  // set counter of while loop to zero 
    }
    }

  
void clearGpsBufferArray()                     // function to clear buffer array
{
    for (int i=0; i<gps_count;i++)
    {
        gps_data[i]=NULL;
    }                      // clear all index of array with command NULL
}
  
void clearBufferArray()                     // function to clear buffer array
{
    for (int i=0; i<count;i++)
    {
        buffer[i]=NULL;
    }                      // clear all index of array with command NULL
}

void test_vuelo_3(){
  
  }

void test_vuelo_2(){//impreciso.
  //hacer varias medidas y acumular las variaciones.
    float t=0;
    long Altitude_1 = HP20x.ReadAltitude();
    t += Altitude_1/100.0;
    delay(10);
    long Altitude_2 = HP20x.ReadAltitude();
    t += Altitude_2/100.0;
    delay(10);
    long Altitude_3 = HP20x.ReadAltitude();
    t += Altitude_3/100.0;
    t = t/3;
    float altura_actual = a_filter.Filter(t);
    int sentido = 0;
    
    if(altura_actual - (altura+0.5) < 0){ //delta hay que estudiarlo (ver cual hay que pner)
      sentido = -1;//estoy llegando
    }else{
      sentido = 1; //nanay - ni idea.
    }
    altura = altura_actual;
  }

void test_vuelo_1(){

sum=0;
  for (int i=0; i<3; i++)
  {
    sum+=abs(realAccel[i]);
  }
  if(sum > 1 && state==1){
      state=2;
  }
  if(sum < 0.3 && state==0){
    state=1; //estoy llegando
    }
  if(sum < 0.2 && state==1){
    state=2;
    }
switch(state){
  case(0):
  Serial.println("Subiendo");
  break;
  case(1):
  Serial.println("LLegando");
  break;
  case(2):
  Serial.println("Arriba");
  break;
  case(3):
  Serial.println("Callendo");
  break;
    
  }

}


void varometer(){
    char display[40];
    if(OK_HP20X_DEV == ret)
    { 
    long Temper = HP20x.ReadTemperature();
    float t = Temper/100.0;
//    Serial.print(t_filter.Filter(t));
 
    long Pressure = HP20x.ReadPressure();
    t = Pressure/100.0;
    Radio.print("[");
   Radio.print(p_filter.Filter(t));
    
    long Altitude = HP20x.ReadAltitude();
    t = Altitude/100.0;
    Radio.print(",");
    Radio.print(a_filter.Filter(t));
    Radio.print("]");
    }
}


void print_serial(){
  Radio.print("#");
  Radio.print("[");
  for (int i=0; i<3; i++)
  {
    Radio.print(realAccel[i]);
    Radio.print("g,");
  }
  Radio.print("]");
  varometer();
  //gps();
  Radio.print(gps_payload);
  Radio.print("[");
  Radio.print(airqualitysensor.first_vol);
  Radio.print("]");
  Radio.println(";");

}


ISR(TIMER1_OVF_vect)
{
  if(airqualitysensor.counter==61)//set 2 seconds as a detected duty
  {

      airqualitysensor.last_vol=airqualitysensor.first_vol;
      airqualitysensor.first_vol=analogRead(A0);
      airqualitysensor.counter=0;
      airqualitysensor.timer_index=1;
      PORTB=PORTB^0x20;
  }
  else
  {
    airqualitysensor.counter++;
  }
}
