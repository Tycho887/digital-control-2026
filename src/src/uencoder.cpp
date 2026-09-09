/***************************************************************************
 *   Copyright (C) 2024 by DTU
 *   jcan@dtu.dk
 * 
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

#include <stdlib.h>
#include "main.h"
#include "uencoder.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
#include "umotor.h"
#include "udisplay.h"

UEncoder encoder;


void UEncoder::setup()
{ // make interrupt on all edges og encoder
  attachInterrupt ( M1ENC_A, m1EncoderA, CHANGE );
  attachInterrupt ( M2ENC_A, m2EncoderA, CHANGE );
  attachInterrupt ( M1ENC_B, m1EncoderB, CHANGE );
  attachInterrupt ( M2ENC_B, m2EncoderB, CHANGE );
  // use hysteresis on input levels (to avoid extra trigger on slow transitions)
  *digital_pin_to_info_PGM[M1ENC_A].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M1ENC_B].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M2ENC_A].pad |= IOMUXC_PAD_HYS;
  *digital_pin_to_info_PGM[M2ENC_B].pad |= IOMUXC_PAD_HYS;
  // init CPU cycle counter
  // for time difference measurement
  ARM_DEMCR |= ARM_DEMCR_TRCENA;
  ARM_DWT_CTRL |= ARM_DWT_CTRL_CYCCNTENA;
  //
  versioncpp = "$Id: uencoder.cpp 31 2024-11-26 20:54:50Z jcan $";
}

void UEncoder::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("%% Encoder settings -------\r\n");
  snprintf(reply, MRL, "%% -- \tconfw ppr\tSet configuration (ppr: pulse per revolution, is %d)\r\n", pulsPerRev);
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tencf F C\tUse fast estimate (F=0 delay but cleaner, F=1 faster, is %d; C=0 don't compensate, U=1 compensate, is=%d)\r\n", velEstFast, velEstFastCompensate);
  usb.send(reply);

}
//
//
bool UEncoder::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "confw ", 5) == 0)
  { // robot configuration
    const char * p1 = &buf[5];
    pulsPerRev = strtol(p1, (char**) &p1, 10);
    if (pulsPerRev < 1)
      pulsPerRev = 1;
    anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  }
  else if (strncmp(buf, "encf ", 5) == 0)
  { // robot configuration
    const char * p1 = &buf[5];
    velEstFast = strtol(p1, (char**) &p1, 10);
    velEstFastCompensate = strtol(p1, (char**) &p1, 10);
  }
  else
    used = false;
  return used;
}

void UEncoder::tick()
{ // Update motor velocity estimates
  // estimate sample time
  uint32_t t_CPU = ARM_DWT_CYCCNT;
  uint32_t dt = t_CPU - lastSample_CPU;
  lastSample_CPU = t_CPU;
  // filter over 50 samples (float)
  sampleTime_us = (sampleTime_us * 49 + dt * CPU_us)/50.0;
  // and in CPU clocks (integer)
  // sampleTime_CPU = (sampleTime_CPU * 49 + dt)/50;
  //
  // estimate velocity
  updateVelocityEstimate();
  if (velEstFast)
  {
    if (velEstFastCompensate)
    {
      motorVelocity[0] = -motorVelocityFastMod[0];
      motorVelocity[1] = motorVelocityFastMod[1];
    }
    else
    {
      motorVelocity[0] = -motorVelocityFast[0];
      motorVelocity[1] = motorVelocityFast[1];
    }
  }
}

///////////////////////////////////////////////////////

void UEncoder::eePromSave()
{ // nothing relevant to save
  uint16_t flags = velEstFast |
                   (velEstFastCompensate << 0x01);
  eeConfig.pushWord(flags);
  for (int m = 0; m < MOTOR_CNT; m++)
  { // all motors
    for (int i = 0; i < 16; i++)
    { // load velocity compensation for encoder edge combinations
      if (i  == 2 or i == 3 or (i >=6 and i <= 9) or i == 12 or i == 13)
      { // these are the valid 8 cobinations (4 forward and 4 reverse)
        eeConfig.pushFloat(velDif[m][i]);
      }
    }
  }
}

void UEncoder::eePromLoad()
{
  uint16_t flags = eeConfig.readWord();
  velEstFast = (flags & 0x01) > 0;
  velEstFastCompensate = (flags & 0x02) > 0;
  for (int m = 0; m < MOTOR_CNT; m++)
  { // all motors
    for (int i = 0; i < 16; i++)
    { // load velocity compensation for encoder edge combinations
      if (i  == 2 or i == 3 or (i >=6 and i <= 9) or i == 12 or i == 13)
      { // these are the valid 8 cobinations (4 forward and 4 reverse)
        velDif[m][i] = eeConfig.readFloat();
      }
    }
  }
}

void UEncoder::updateVelocityEstimate()
{
  const float    one_sec_in_cpu  = F_CPU;
  const int32_t cpu_200ms = F_CPU/5;
  const uint32_t cpu_500ms = F_CPU/2;
  const int32_t cpu_2ms = F_CPU/500;
  // motor 1 velocity
  int j = active;
  active = (j + 1) % 2;
  float velSum[MOTOR_CNT] = {0};
  int velSumCnt[MOTOR_CNT] = {0};
  // angle for full period of encoder
  const float app = anglePerPuls * 4;
  uint32_t timeStartEdge = 0;
  uint32_t timeEndEdge = 0;
  uint32_t timeStartNoEdge = 0;
  uint32_t nowCpu = ARM_DWT_CYCCNT;
  const float minMotorVel = 2.0; // radians/s on motor before gear.
  // bool overflow = false;
  //
  for (int m = 0; m < MOTOR_CNT; m++)
  { // save increment counter - debug
    for (int ab4 = 0; ab4 < 4; ab4++)
    { // just for debug
      dEncoder[m][ab4] = incrEncoder[m][j][ab4];
      // debug end
      if (incrEncoder[m][j][ab4] == 0)
      { // no increment since last
        incrEnc[m][ab4] = 0;
      }
      else
      { // use time since saved transition of same edge type
        uint32_t dt_cpu = transitionTime_cpu[m][j][ab4] - lastTransitionTime_cpu[m][ab4];
        float v = 0;
        if (dt_cpu > 0 and dt_cpu < cpu_500ms)
        { // no overload, and valid timing ('incrEncoder' holds the sign)
          v = app * one_sec_in_cpu / dt_cpu * incrEncoder[m][j][ab4];
          velocityPart[m][ab4] = v;
          //
          // find also average delay time
          if (velSumCnt[m] == 0)
          { // first value
            timeStartEdge = lastTransitionTime_cpu[m][ab4];
            timeEndEdge = transitionTime_cpu[m][j][ab4];
          }
          else if (timeStartNoEdge < lastTransitionTime_cpu[m][ab4])
          { // more than one - use the newest
            timeStartEdge = lastTransitionTime_cpu[m][ab4];
            timeEndEdge = transitionTime_cpu[m][j][ab4];
          }
        }
        // save new transition time as last
        lastTransitionTime_cpu[m][ab4] = transitionTime_cpu[m][j][ab4];
        // edges used in this sample
        incrEnc[m][ab4] = incrEncoder[m][j][ab4];
        // prepare for next period
        incrEncoder[m][j][ab4] = 0;
        // use this velocity
        velSum[m] += v;
        velSumCnt[m]++;
      }
    }
    uint32_t delayAvg;
    if (velSumCnt[m] > 0)
    { // there were edges in this sample period
      // delay time in CPU clocks of newest data
      uint32_t dataPeriod = timeEndEdge - timeStartEdge;
      delayAvg = dataPeriod/2.0 + nowCpu - timeEndEdge;
      encoderDelay[m] = float(delayAvg)/one_sec_in_cpu;
      // motor velocity estimate in rad/s
      motorVelocity[m] = velSum[m] / velSumCnt[m];
    }
    else
    { // no edges in this sample
      // velocity estimate
      // use the fastest estimate that is slower than last
      // edge based velocity
      int32_t etime;
      // time since positive A edge
      int32_t maxTime = 1; // in CPU clocks
      float va; // absolute velocity
      for (int ab4 = 0; ab4 < 4; ab4++)
      { // find longest time since edge
        etime = nowCpu - lastTransitionTime_cpu[m][ab4];
        if (etime > maxTime)
          maxTime = etime;
      }
      va = app / (maxTime / one_sec_in_cpu);
      if (va < minMotorVel) // approx 5mm/sec for regbot
        va = 0;
      // velocity need update ('va' always positive)
      if (va < fabsf(motorVelocity[m]))
      { // keep old velocity if faster than this estimate
        if (motorVelocity[m] > 0)
          motorVelocity[m] = va;
        else
          motorVelocity[m] = -va;//velSlowSum[m] / velSlowSumCnt[m];
      }
      // avg delay is half time since last edge of newest data
      encoderDelay[m] = float(maxTime/2)/one_sec_in_cpu;
    }
    if (true or velEstFast)
    { // use 2 newest edges
      uint32_t newest = 0;
      uint32_t nextNewest = 0;
      int ab[2]{0};
      for (int ab4 = 0; ab4 < 4; ab4++)
      { // find the newest
        // velAge[m][ab4] = (nowCpu - lastTransitionTime_cpu[m][ab4])/(one_sec_in_cpu / 1000); // (ms) not used
        if (newest < lastTransitionTime_cpu[m][ab4])
        {
          newest = lastTransitionTime_cpu[m][ab4];
          ab[0] = ab4;
        }
      }
      for (int ab4 = 0; ab4 < 4; ab4++)
      { // find the next newest
        if (ab4 != ab[0] and nextNewest < lastTransitionTime_cpu[m][ab4])
        {
          nextNewest = lastTransitionTime_cpu[m][ab4];
          ab[1] = ab4;
        }
      }
      int32_t dataPeriod = newest - nextNewest;
      int32_t since = nowCpu - newest;
      delayAvg = dataPeriod/2.0 + since;
      if (velEstFast)
        encoderDelay[m] = float(delayAvg)/one_sec_in_cpu;
      bool calculated = false;
      // motor velocity estimate in rad/s
      if (since < 0 or since > cpu_200ms or dataPeriod <= 0)
      {  // too slow or time overflow
        motorVelocityFast[m] = 0;
      }
      else if (since > dataPeriod + cpu_2ms)
      { // use time since last edge
        motorVelocityFast[m] = anglePerPuls / (float(since)/one_sec_in_cpu);
      }
      else
      {  // use time of longest period
        motorVelocityFast[m] = anglePerPuls / (float(dataPeriod)/one_sec_in_cpu);
        calculated = true;
      }
      // add velocity sign
      if (encCCV[m])
        motorVelocityFast[m] *= -1;
      // save difference - first index, then values
      // index 2,3, 6,7,8,9, 12, 13 should be used only (3,6,8,13 for reverse)
      velDif[m][0] = ab[0];
      velDif[m][1] = ab[1];
      int e = ab[0]*4 + ab[1];
      if (fabs(motorVelocity[m]) > minMotorVel and calculated)
        velDif[m][e] = velDif[m][e] * (0.95) + (motorVelocity[m] - motorVelocityFast[m])/motorVelocity[m] * 0.05;
      motorVelocityFastMod[m] = motorVelocityFast[m] * (1 + velDif[m][e]);
    }

  }
}

void UEncoder::updateVelocityEstimateNot()
{
  const float    one_sec_in_cpu  = F_CPU;
  const uint32_t half_sec_in_cpu = F_CPU/2;
  // motor 1 velocity
  int j = active;
  active = (j + 1) % 2;
  float velSum[MOTOR_CNT] = {0};
  int velSumCnt[MOTOR_CNT] = {0};
  float velSlowSum[MOTOR_CNT] = {0};
  int velSlowSumCnt[MOTOR_CNT] = {0};
  //
  for (int m = 0; m < MOTOR_CNT; m++)
  { // save increment counter - debug
    for (int ab4 = 0; ab4 < 4; ab4++)
    { // just for debug
      dEncoder[m][ab4] = incrEncoder[m][j][ab4];
      // debug end
      if (incrEncoder[m][j][ab4] == 0)
      { // no increment since last
        // calculate velocity based on current time
        uint32_t dt_cpu = ARM_DWT_CYCCNT - lastTransitionTime_cpu[m][ab4];
        float v = 0;
        if (dt_cpu > 0 and dt_cpu < half_sec_in_cpu)
        { // no overload, and valid timing
          // calculate velocity based on this
          v = one_sec_in_cpu/dt_cpu;
          if (fabsf(v) * anglePerPuls < fabsf(motorVelocity[m]))
          { // use only if slower than last time
            // keep sign from last estimate
            if (velocityPart[m][ab4] > 0)
              velocityPart[m][ab4] = v * anglePerPuls;
            else
              velocityPart[m][ab4] = -v * anglePerPuls;
          }
        }
        else
          velocityPart[m][ab4] = 0;
        // edges used in this sample
        incrEnc[m][ab4] = 0;
        velSlowSum[m] +=  velocityPart[m][ab4];
        velSlowSumCnt[m]++;
      }
      else
      { // use time since saved transition
        uint32_t dt_cpu = transitionTime_cpu[m][j][ab4] - lastTransitionTime_cpu[m][ab4];
        float v = 0;
        if (dt_cpu > 0 and dt_cpu < half_sec_in_cpu)
        { // no overload, and valid timing
          v = one_sec_in_cpu / dt_cpu * incrEncoder[m][j][ab4] * anglePerPuls;
          velocityPart[m][ab4] = v;
        }
        // save new transition time as last
        lastTransitionTime_cpu[m][ab4] = transitionTime_cpu[m][j][ab4];
        // edges used in this sample
        incrEnc[m][ab4] = incrEncoder[m][j][ab4];
        // prepare for next period
        incrEncoder[m][j][ab4] = 0;
        // use this velocity
        velSum[m] += v;
        velSumCnt[m]++;
      }
    }
    if (velSumCnt[m] > 0)
      motorVelocity[m] = velSum[m] / velSumCnt[m];
    else
      motorVelocity[m] = velSlowSum[m] / velSlowSumCnt[m];
  }
}


void UEncoder::encoderInterrupt(int m, bool encA)
{ // get interrupt timing
  uint32_t edge_cpu = ARM_DWT_CYCCNT;
  uint8_t pA, pB;
  // get encoder values for this motor
  pA = digitalReadFast(encApin[m]);
  pB = digitalReadFast(encBpin[m]);
  bool err = false;
  // edge index: A-up = 0, A-down = 1, B-up = 2, B-down = 3
  int ab4;
  if (encA)
  { // encode pin A interrupt
    encCCV[m] = pA == pB;
    err = pA == lastA[m];
    lastA[m] = pA;
    if (err)
      errCntA[m][pA]++;
    if (pA)
      ab4 = 0;
    else
      ab4 = 1;
  }
  else
  { // encode pin B interrupt
    encCCV[m] = pA != pB;
    err = pB == lastB[m];
    lastB[m] = pB;
    if (err)
      errCntB[m][pB]++;
    if (pB)
      ab4 = 2;
    else
      ab4 = 3;
  }
  if (err)
  { // this was a spurious interrupt, ignore
    // encoder value didn't change
    return;
  }
  // use this set of data to save values
  int j = active;
  if (encCCV[m])
  {
    encoder[m]--;
    // and within sample period
    incrEncoder[m][j][ab4]--;
  }
  else
  {
    encoder[m]++;
    // and within sample period
    incrEncoder[m][j][ab4]++;
  }
  transitionTime_cpu[m][j][ab4] = edge_cpu;
}



//////////////////////////////////////////////////////////////

/**
 * Interrupt routines */
void m1EncoderA()
{ // motor 1 encoder A change
  encoder.encoderInterrupt(0, true);
  encoder.intCnt++;
}

void m2EncoderA()
{ // motor 2 encoder A
    encoder.encoderInterrupt(1, true);
    encoder.intCnt++;
}

void m1EncoderB()
{ // motor 1 encoder pin B
    encoder.encoderInterrupt(0, false);
    encoder.intCnt++;
}

void m2EncoderB()
{ // motor 2 encoder pin B
    encoder.encoderInterrupt(1, false);
    encoder.intCnt++;
}
