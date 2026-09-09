/***************************************************************************
 *   Copyright (C) 2024 by DTU                             *
 *   jcan@dtu.dk                                                    *
 *
 * Interface to MPU9250
 *
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */




#ifndef UIMU2_H
#define UIMU2_H

#include <MPU9250_asukiaaa.h>


// I2C address 0x69 could be 0x68 depends on your wiring.
#define ADDRESS_MPU				   0x68
// #define ADDRESS_COM          0x0C // not used (Magnetometer)

class ULog;

class UImu2 // : public USubss
{
public:
  // Functions
  void setup();
  /**
   * decode calibration request */
  bool decode(const char * cmd);
  /**
   * Send command list */
  void sendHelp();
  /**
   * Update measurements */
  void tick();
  /**
   * save scale and offset */
  void eePromSave();
  /**
   * load scale and offset */
  void eePromLoad();
  /**
   * Initialization of MPU communication */
  void initMpu();  
  /**
   * Calibration handling */
  uint32_t gyroOffsetStartCnt = 1000;
  bool     gyroOffsetDone;
  /**
   * Newest obtained gyro values
   * in degrees per second */
  float gyro[3] = {0}; // in degrees per second
  /**
   * Newest accelerometer values
   * in G, e.g. units of 9.82 m/s^2 */
  float acc[3] = {0}; // in G

public:
  /** internal filter values */
  float tiltu1  = 0; // old value for complementary filter
  float accAng;   // for debug
  float gyroTiltRate;
  /**
   * Implement a complementary filter using both acc and gyro
   * but only for tilt around the y-axis (to the left)
   */
  void estimateTilt();
  /**
   * IMU communication is fine, if the value is > 0 */
  int imuAvailable = 10; // count down on error
  /**
   * Estimated tilt value in radians (around the y-axis) */
  float tilt = 0; // tilt angle in radians (complement filter around y-axis)
  // SVN version
  const char * versionh = "$Id: uimu2.h 28 2024-11-15 12:18:05Z jcan $";
  const char * versioncpp = nullptr;

protected:
  /**
   * Offset of gyro, when calibrated */
  float offsetGyro[3] = {0};
private:
  /**
   * interface object for IMU */
  MPU9250_asukiaaa mpu{ADDRESS_MPU};
  /// last read time (in us) from IMU
  uint32_t lastRead = 0;
  ///
  uint32_t tickCnt = 0;
  /** debug counter of interrups
   * usefull, to debug heavy noise on interrupt lines */
  int sampleTimeus = 1000;
};
  
extern UImu2 imu2;

#endif // UIMU_H
