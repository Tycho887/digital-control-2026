#line 1 "/Users/michaeljohansen/Autumn2026/digital-control/week-01/basebot_6/src/umotor.h"
/***************************************************************************
 *   Copyright (C) 2014-2022 by DTU
 *   jca@elektro.dtu.dk            
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

#ifndef UMOTOR_H
#define UMOTOR_H

#include <stdint.h>
#include "main.h"
// #include "usubss.h"

class UMotor //: public USubss
{
public:
  // motor voltage
  float motorVoltage[2];
  bool motorEnable[2] = {false};
  /** PWM frequency (100kHz is probably maximum) */
  int PWMfrq = 68000; //
  /**
   * if motor gets full power in more than 1 second, then stop */
  int overloadCount;
  //
  bool m1ok = true;
  bool m2ok = true;
  // motor reversed
  // Pololu motors not, but big ones from China is opposite (reversed)
//   bool motorReversed = false;
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
   * implement motor voltage (escValue) as PWM signal */
//   void implementMotorVoltage();
  /**
   * set PWM frequency, should be less than 100000 (100 kHz) */
  void setPWMfrq(int frq);
  /**
   * implement sample tick */
  void tick();
  /**
   * save configuration to (EE)disk */
  void eePromSave();
  /**
   * load configuration from EE-prom */
  void eePromLoad();
  /**
   * set enable flag, and for HW 8 disable -> sleep mode for motor driver */
  void motorSetEnable(bool e1, bool e2);
  
  const char * versionh = "$Id: umotor.h 36 2025-08-28 13:06:04Z jcan $";
  const char * versioncpp = nullptr;

protected:

  void motorSetPWM(int m1PWM, int m2PWM);
  
  void motorSetAnchorVoltage();
  
  
private:
  //
  int16_t motorAnkerPWM[2];
  int16_t motorAnkerDir[2];
  int16_t motorSleeping[2];
  int pinMotor2Dir;
  int pinMotor2Pwm;
  // int tickCnt = 0;
  int setupCnt = 0;
  int MAX_PWM = 4096;
  int pwmDeadband[2] = {0};
};

extern UMotor motor;

#endif // UMOTOR_H
