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

#ifndef UENCODER_H
#define UENCODER_H

#include <stdint.h>
#include <math.h>
#include "main.h"

/**
 * interrupt for motor encoder */
void m1EncoderA();
void m1EncoderB();
void m2EncoderA();
void m2EncoderB();


class UEncoder // : public USubss
{
public:
  /**
  * set PWM port of frekvens */
  void setup();
  /**
   * send command help */
  void sendHelp();
  /**
   * decode commands */
  bool decode(const char * buf);
  /**
   * sample update */
  void tick();
  /**
   * save configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * Estimate motor velocity from time between encoder ticks.
   * Uses CPU clock counter for more accurate timing.
   * Note: difference in encoder magnet size makes estimate noisy */
  void updateVelocityEstimate();
  void updateVelocityEstimateNot();
  /**
   * Called on all encoder interrupts */
  void encoderInterrupt(int m, bool a);
  
public:
  static const int MOTOR_CNT = 2;
  const int encApin[MOTOR_CNT] = {M1ENC_A, M2ENC_A};
  const int encBpin[MOTOR_CNT] = {M1ENC_B, M2ENC_B};
  /**
   * Rotation direction for each motor */
  bool encCCV[MOTOR_CNT];
  uint32_t encStartTime_cpu[MOTOR_CNT];
  /**
   * overload protection on clockcycle counter */
  bool encTimeOverload_cpu[MOTOR_CNT];
  /**
   * Encoder count for each motor (left, right) */
  int32_t encoder[MOTOR_CNT];
  int dEncoder[MOTOR_CNT][4];
  int32_t lastEncoder[MOTOR_CNT];
  // set by interrupt
  int incrEncoder[MOTOR_CNT][2][4];
  // for print only
  int incrEnc[MOTOR_CNT][4];
  uint32_t transitionTime_cpu[MOTOR_CNT][2][4];
  // active set written by interrupt
  int active = 0;
  // saved value set by tick
  uint32_t lastTransitionTime_cpu[MOTOR_CNT][4];
  /** Velocity of motor in radians per second */
  float motorVelocity[MOTOR_CNT];
  float motorVelocityFast[MOTOR_CNT];
  float motorVelocityFastMod[MOTOR_CNT];
  float encoderDelay[2] = {0.0}; // Average velocity estimate delay (sec)

  // velocity estimate for each A up+down, B up+down
  float velocityPart[MOTOR_CNT][4];
  /// just tick count
  int intCnt = 0; // debug
// #ifdef basebot_6
  /** (Regbot values)
   * Number of pulse edges in one rotation, counting both A and B edges */
  uint16_t pulsPerRev = 48; //  using all edges (48 (regbot) or 68 (robobot))
  /**
   * Gear ratio (not used in basebot) */
  // float gear = 9.68; // 9.68 (regbot), RoboBot: 10.0 or 30.0 or 18.0 or 19.0
  //
  bool velEstFast = false;
  bool velEstFastCompensate = false;

private:
  int tickCnt = 0;
  // in radians
  float anglePerPuls = 2.0 * M_PI / (pulsPerRev);
  // velocity difference from sensor A to B, B to A, and A to A or B to B
  // one value for each edge combination (4 forward and 4 reverse,
  // but 16 values allocated to make indexing easy
  float  velDif[MOTOR_CNT][16]{{0}};
  // float  velAge[MOTOR_CNT][4]{{0}};

public:
  // constant factor to get from dt in CPU clocks to us
  const float CPU_us = 1000000.0/float(F_CPU);
  // estimated sample time in micro seconds
  float sampleTime_us = 1000;
  // last sample time (in CPU clocks)
  uint32_t lastSample_CPU = 0;
  const char * versionh = "$Id: uencoder.h 36 2025-08-28 13:06:04Z jcan $";
  const char * versioncpp = nullptr;

private:
  uint32_t missionStart;
  bool lastA[MOTOR_CNT];
  bool lastB[MOTOR_CNT];
public:
  /** should be 1 or 0 - typically one error at startup */
  int errCntA[MOTOR_CNT][2] = {{0}};
  int errCntB[MOTOR_CNT][2] = {{0}};
};

extern UEncoder encoder;

#endif
