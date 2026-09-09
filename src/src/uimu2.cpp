 /***************************************************************************
  *   Copyright (C) 2024 by DTU                             *
  *   jcan@dtu.dk                                                    *
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
 
#include "uimu2.h"
#include "ueeconfig.h"
#include "uencoder.h"
#include "urobot.h"


UImu2 imu2;



void UImu2::setup()
{
  initMpu();
  versioncpp = "$Id: uimu2.cpp 35 2025-01-01 08:24:35Z jcan $";
}

void UImu2::initMpu()
{
  #if defined(REGBOT_HW4)
  Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_INT, I2C_RATE_1000 );
  #else
    #if defined(REGBOT_HW41) || defined(REGBOT_HW63_35)
    Wire.begin();
    Wire.setClock(1000000);
    #else
    Wire.begin ( I2C_MASTER, 0x00, I2C_PINS_16_17, I2C_PULLUP_EXT, I2C_RATE_1000 );
    #endif
  #endif
  /* Initialize and configure IMU */
  mpu.setWire(&Wire);
  uint8_t id = 0;
  uint8_t retval = mpu.readId(&id);
  if (retval != 0)
  { // could not read from IPU
    usb.send("%% Error initializing communication with IMU\r\n");
    imuAvailable = 0;
  }
  else
  { // print who am i
//     const int MSL = 100;
//     char s[MSL];
//     snprintf(s, MSL, "%% MPU9250 'who_am_i'=%d (%x)\r\n", id, id);
//     usb.send(s);
    //
    mpu.beginAccel(ACC_FULL_SCALE_4_G);
    mpu.beginGyro(GYRO_FULL_SCALE_1000_DPS);
  }
}


bool UImu2::decode(const char* cmd)
{
  bool found = true;
  if (strncmp(cmd, "gyroc", 5) == 0)
  {
    gyroOffsetDone = false;
  }
  else
    found = false;
  return found;
}
//
void UImu2::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("%% IMU -------\r\n");
  snprintf(reply, MRL, "%% -- \tgyroc \tStart gyro calibration (finished=%d)\r\n", gyroOffsetDone);
  usb.send(reply);
}

void UImu2::tick()
{ // read data - first time will fail
  tickCnt++;
  if (imuAvailable > 0)
  {
    int32_t nt = micros();
    int dt = nt - lastRead;
    if (true /*dt >= 90 and dt < 10000*/)
    { // data should be imuAvailable
      // there seems to be a problem, if dt <= 1000 (1ms).
      lastRead = nt;
      sampleTimeus = dt;
      if (mpu.accelUpdate() == 0)
      { // rectify raw data - except magnetometer
//         int16_t * a = mpu.getAcc();
        float accw[3];
        accw[0] = mpu.accelX();
        accw[1] = mpu.accelY();
        accw[2] = mpu.accelZ();
        //
        for (int i = 0; i < 3; i++)
          acc[i] = accw[i]/* * accScale[i]*/; // - accOffset[i];
      }
      // Gyro
      if (mpu.gyroUpdate() == 0)
      { // gyro
        if (gyroOffsetDone)
        { // production
          gyro[0] = mpu.gyroX() - offsetGyro[0];
          gyro[1] = mpu.gyroY() - offsetGyro[1];
          gyro[2] = mpu.gyroZ() - offsetGyro[2];
        }
        else
        { // calibrate
          // use raw values
          gyro[0] = mpu.gyroX();
          gyro[1] = mpu.gyroY();
          gyro[2] = mpu.gyroZ();
          // start calibration
          if (tickCnt < gyroOffsetStartCnt)
          { // zero offset before 1000 summations
            offsetGyro[0] = 0;
            offsetGyro[1] = 0;
            offsetGyro[2] = 0;
          }
          else if (tickCnt <= gyroOffsetStartCnt + 1000)
          { // not finished offset calculation
            // summation over 1 second
            offsetGyro[0] += gyro[0];
            offsetGyro[1] += gyro[1];
            offsetGyro[2] += gyro[2];
            if ((tickCnt - gyroOffsetStartCnt) % 100 == 0)
            {
              const int MSL = 150;
              char s[MSL];
              snprintf(s, MSL,"%% UImu2::gyrooffset n=%lu, gx=%g sumgx=%g\r\n",
                       tickCnt - gyroOffsetStartCnt,
                       gyro[0],
                       offsetGyro[0]/(tickCnt - gyroOffsetStartCnt));
              usb.send(s);
            }
            if (tickCnt == gyroOffsetStartCnt + 1000)
            { // set average offset
              offsetGyro[0] /= 1000;
              offsetGyro[1] /= 1000;
              offsetGyro[2] /= 1000;
//               usb.send("%% gyro offset finished\r\n");
              gyroOffsetDone  = true;
              const int MSL = 150;
              char s[MSL];
              snprintf(s, MSL,"%% UImu2::gyro offset: %g %g %g\r\n", offsetGyro[0], offsetGyro[1], offsetGyro[2]);
              usb.send(s);
            }
          }
          else
            // redo of calibrate requested
            gyroOffsetStartCnt = tickCnt + 10;
        }
        if (imuAvailable < 10)
          imuAvailable++;
      }
      else
      {
        imuAvailable--;
        if (imuAvailable == 0)
          usb.send("%% message failed to read from MPU9250 10 times in a row, stopped trying\r\n");
      }
    }
    { //  complementary tilt filter only
      estimateTilt();
    }
  }
}


////////////////////////////////////////////////

void UImu2::eePromSave()
{
  uint8_t f = 1;
  // f |= useMadgwich << 1;
  eeConfig.pushByte(f);
  eeConfig.pushFloat(offsetGyro[0]);
  eeConfig.pushFloat(offsetGyro[1]);
  eeConfig.pushFloat(offsetGyro[2]);
}

void UImu2::eePromLoad()
{
  /*uint8_t flags =*/ eeConfig.readByte();
  //
  offsetGyro[0] = eeConfig.readFloat();
  offsetGyro[1] = eeConfig.readFloat();
  offsetGyro[2] = eeConfig.readFloat();
  gyroOffsetDone = true;
  //
  // debug
  // const int MSL = 100;
  // char s[MSL];
  // snprintf(s, MSL, "%% UImu2:: gyro offset: %g %g %g\r\n", offsetGyro[0], offsetGyro[1], offsetGyro[2]);
  // usb.send(s);
}


/**
 * estimate tilt angle, as complementary filter with gyro and acc 
 *       1     tau s                     1
 * Gyro ---  ----------  + acc_pitch --------
 *       s    tau s + 1              tau s + 1
 *
 *     1        T/(T+2.*tau) + *T/(T+2.*tau) * z^-1
 * --------- = -------------------------------------
 *  tau s + 1     1 + (T-2.*tau)/(T+2.*tau) * z^-1
 *
 * T = 0.001 (using actual sample time);
 * tau = 1.0;
 *  */
void UImu2::estimateTilt()
{ // use actual sample time
  float T = sampleTimeus * 1e-6;
  float tau = 2.0; // seems to give good response
  float b = T/(T + 2 * tau);
  float a = -(T - 2 * tau)/(T + 2 * tau);
  float u; // input to filter
  float est; // estimated angle
  // gyro mounted on top plate!, using X and Z
  accAng = atan2f(acc[2], -acc[0]);

  // gyro is running in mode 2 (0= 250 grader/sek, 1 = 500 deg/s, 2=1000 deg/s 3=2000 deg/s)
  // rotation around Y-axis
  gyroTiltRate = gyro[1] * M_PI / 180.0; // radianer pr sekund
  // add gyro and accelerometer reading
  u = accAng + gyroTiltRate * tau;
  if (true) // imuGyro[0] < 245 and imuGyro[0] > -245)
  { // gyro not saturated
    // filter
    if (accAng > 0.0 and tilt < -M_PI/2.0)
      est = a * (tilt + 2 * M_PI) + b * u + b * tiltu1;
    else if (accAng < 0.0 and tilt > M_PI/2.0)
      est = a * (tilt - 2 * M_PI) + b * u + b * tiltu1;
    else
      est = a * tilt + b * u + b * tiltu1;

    if (false)
    {
      const int MSL = 200;
      char s[MSL];
      snprintf(s, MSL, "# Tilt:: accAng:%g, gyrotiltrate:%g, u:%g, est:%g \n", accAng, gyroTiltRate, u, est);
      usb.send(s);
    }
  }
  else
    // else use angle as is from accelerometer
    est = accAng;
  //
  if (est > M_PI)
  { // folded 
    est -= 2 * M_PI;
    // save last value of u in right angle space
    tiltu1 = accAng - 2 * M_PI + gyroTiltRate * tau;
  }
  else if (est < -M_PI)
  { // folded
    est += 2 * M_PI;
    tiltu1 = accAng + 2 * M_PI + gyroTiltRate * tau;
  }
  else
  { // no folding
    tiltu1 = u;
  }
  //
  tilt = est;
}
