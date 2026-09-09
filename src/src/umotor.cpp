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
#include "umotor.h"
#include "ueeconfig.h"
#include "urobot.h"
#include "uusb.h"
#include "math.h"

UMotor motor;

void UMotor::setup()
{
  analogWriteResolution(12); /// resolution (12 bit)
  //
  motorSetEnable(1,1);
  pinMode(PIN_LEFT_DIR,OUTPUT); // motor 1 IN1 (LEFT_IN_+)
  pinMode(PIN_RIGHT_DIR,OUTPUT); // motor 2 IN1 (RIGHT_IN_+)
  pinMode(PIN_LEFT_PWM,OUTPUT); // motor 1 IN2 (LEFT_IN_-)
  pinMode(PIN_RIGHT_PWM,OUTPUT); //motor 2 IN2 (RIGHT_IN_-)
  motorSetPWM(0,0);
  setPWMfrq(PWMfrq);
  motorSetPWM(0,0);
  setupCnt++;
  // version
  versioncpp = "$Id: umotor.cpp 36 2025-08-28 13:06:04Z jcan $";
}


/**
 * e2 used on hardware < 3 only */
void UMotor::motorSetEnable(bool e1, bool e2)
{
  // reset overload
  if (e1 and not motorEnable[0])
    overloadCount = 0;
  // enable motors (or disable)
  motorEnable[0] = e1;
  motorEnable[1] = e2;
  //
  if (not motorEnable[0])
  {// disable motor - set to sleep
    pinMode(PIN_LEFT_DIR,OUTPUT); 
    pinMode(PIN_LEFT_PWM,OUTPUT); 
    digitalWriteFast(PIN_LEFT_DIR, LOW);
    digitalWriteFast(PIN_LEFT_PWM, LOW);
    motorSleeping[0] = true;
  }
  if (not motorEnable[1])
  { // disable motor - set to sleep
    pinMode(PIN_RIGHT_DIR,OUTPUT); 
    pinMode(PIN_RIGHT_PWM,OUTPUT); 
    digitalWriteFast(PIN_RIGHT_DIR, LOW);
    digitalWriteFast(PIN_RIGHT_PWM, LOW);
    motorSleeping[1] = true;
  }
}


void UMotor::motorSetAnchorVoltage()
{
  float batteryNominalVoltage = robot.batteryVoltage;
  if (batteryNominalVoltage < 5.5)
    // not on battery - just for test and avoid divide by zero
    batteryNominalVoltage = 11.1;
  const float voltageLoss = 1.0; // from battery to motor driver
  float scaleFactor = float(MAX_PWM) / (batteryNominalVoltage - voltageLoss);
  // overload check
  if (overloadCount > 500 and motorEnable[0])
  { // disable motor (after 0.5 second)
    motorSetEnable(false, false);
    //
    usb.send("%% UMotor::motorSetAnchorVoltage: overload, disabled motors\n");
    //
  }
  // compensate for minimum pulsewidth
  float v1, v2;
  const float minV = 0.4; // else no PWM out of motor driver
  if (motorVoltage[0] > 0.001)
    v1 = motorVoltage[0] + minV;
  else if (motorVoltage[0] < -0.001)
    v1 = motorVoltage[0] - minV;
  else
    v1 = 0;
  if (motorVoltage[1] > 0.001)
    v2 = motorVoltage[1] + minV;
  else if (motorVoltage[1] < -0.001)
    v2 = motorVoltage[1] - minV;
  else
    v2 = 0;
  // convert to PWM values (at 20khz)
  int w1, w2;
  //   normal for pololu motors
  w1 = int16_t(v1 * scaleFactor);
  w2 = int16_t(v2 * scaleFactor);
  // positive voltage is counter clockwise
  motorSetPWM(w1, w2);
}

/** 
 * allowed input is +/- 2048, where 2048 is full battery voltage
 * */
void UMotor::motorSetPWM(int m1PWM, int m2PWM)
{ // PWM is 12 bit
  const int max_pwm = MAX_PWM;
  // for logging
  if (m1PWM > 0)
    motorAnkerPWM[0] = m1PWM + pwmDeadband[0];
  else if (m1PWM < 0)
    motorAnkerPWM[0] = m1PWM - pwmDeadband[0];
  else
    motorAnkerPWM[0] = 0;
  if (m2PWM > 0)
    motorAnkerPWM[1] = m2PWM + pwmDeadband[1];
  else if (m2PWM < 0)
    motorAnkerPWM[1] = m2PWM - pwmDeadband[1];
  else
    motorAnkerPWM[1] = 0;
  motorAnkerDir[0] = m1PWM >= 0;
  motorAnkerDir[1] = m2PWM >= 0;
  
  // LEFT motor
  if (not motorSleeping[0])
  {
    if (m1PWM > 0)
    {
      pinMode(PIN_LEFT_DIR,OUTPUT);
      digitalWriteFast(PIN_LEFT_DIR, HIGH);
      analogWrite(PIN_LEFT_PWM, max_pwm - motorAnkerPWM[0]);
    }
    else if (m1PWM < 0)
    { // move PWM to other pin
      pinMode(PIN_LEFT_PWM, OUTPUT); // motor 1 PWM
      digitalWriteFast(PIN_LEFT_PWM, HIGH);
      analogWrite(PIN_LEFT_DIR, max_pwm + motorAnkerPWM[0]);
    }
    else
    { // break
      pinMode(PIN_LEFT_DIR,OUTPUT);
      pinMode(PIN_LEFT_PWM,OUTPUT);
      digitalWriteFast(PIN_LEFT_DIR, HIGH);
      digitalWriteFast(PIN_LEFT_PWM, HIGH);
    }
  }
  else if (m1PWM != 0)
  { // set to not sleeping by 1,1 for > 400us
    // one sample time should be sufficient
    digitalWriteFast(PIN_LEFT_DIR, HIGH);
    digitalWriteFast(PIN_LEFT_PWM, HIGH);
    motorSleeping[0] = false;
  }
  // RIGHT motor
  if (not motorSleeping[1])
  {
    if (m2PWM > 0)
    {
      pinMode(PIN_RIGHT_DIR,OUTPUT);
      digitalWriteFast(PIN_RIGHT_DIR, HIGH);
      analogWrite(PIN_RIGHT_PWM, max_pwm - motorAnkerPWM[1]);
    }
    else if (m2PWM < 0)
    { // move PWM to other pin
      pinMode(PIN_RIGHT_PWM,OUTPUT);
      digitalWriteFast(PIN_RIGHT_PWM, HIGH);
      analogWrite(PIN_RIGHT_DIR, max_pwm + motorAnkerPWM[1]);
    }
    else
    { // break
      pinMode(PIN_RIGHT_DIR,OUTPUT);
      pinMode(PIN_RIGHT_PWM,OUTPUT);
      digitalWriteFast(PIN_RIGHT_DIR, HIGH);
      digitalWriteFast(PIN_RIGHT_PWM, HIGH);
    }
  }
  else if (m2PWM != 0)
  { // set to not sleeping by 1,1 for >400us
    // one sample time should be sufficient
    digitalWriteFast(PIN_RIGHT_DIR, HIGH);
    digitalWriteFast(PIN_RIGHT_PWM, HIGH);
    motorSleeping[1] = false;
  }
  if (false)
  { // debug print
    const int MSL = 120;
    char s[MSL];
    snprintf(s, MSL, "%% Setting motor voltage to %.2f, %.2f\n", motorVoltage[0], motorVoltage[1]);
    usb.send(s);
    snprintf(s, MSL, "%% Motor %.1f, %.1f V, PWM is %d, %d (of %d), dir = %d, %d\n",
             motorVoltage[0], motorVoltage[1],
             motorAnkerPWM[0], motorAnkerPWM[1], MAX_PWM,
             motorAnkerDir[0], motorAnkerDir[1]);
    usb.send(s);
  }
}


void UMotor::sendHelp()
{
  const int MRL = 300;
  char reply[MRL];
  usb.send("%% Motor -------\r\n");
//   snprintf(reply, MRL, "%% -- \tmotr V \tSet motor reversed; V=0 for small motors, V=1 for some big motors\r\n");
//   usb.send(reply);
  snprintf(reply, MRL, "%% -- \tmotv m1 m2 \tSet motor voltage -12.0..12.0 - and enable motors\r\n");
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tmotfrq \tSet motor PWM frequency [100..100000], is %d\r\n", PWMfrq);
  usb.send(reply);
  snprintf(reply, MRL, "%% -- \tstop \tCalls the function stop(0.02)\r\n");
  usb.send(reply);
}

void stop(float after);

bool UMotor::decode(const char* buf)
{
  bool used = true;
  if (strncmp(buf, "motv", 4) == 0)
  {
    float m1, m2;
    const char * p1 = &buf[4];
    // get two values - if no value, then 0 is returned
    m1 = strtof(p1, (char**)&p1);
    m2 = strtof(p1, (char**)&p1);
    motorVoltage[0] = m1;
    motorVoltage[1] = m2;
    if ((fabs(m1) < 0.01) and (fabs(m2) < 0.01))
      motorSetEnable(0, 0);
    else
      motorSetEnable(1,1);
    // debug
    // should be in tick() only
    motorSetAnchorVoltage();
    //
    const int MSL = 120;
    char s[MSL];
    snprintf(s, MSL, "%% Setting motor voltage to %.2f, %.2f\n", motorVoltage[0], motorVoltage[1]);
    usb.send(s);
    snprintf(s, MSL, "%% Motor PWM is %d, %d (of %d)\n", motorAnkerPWM[0], motorAnkerPWM[1], MAX_PWM);
    usb.send(s);
    // debug end
  }
  else if (strncmp(buf, "motfrq ", 7) == 0)
  {
    const char * p1 = &buf[7];
    int frq = strtol(p1, nullptr, 10);
    if (frq < 100)
      frq = 100;
    setPWMfrq(frq);
  }
  else if (strncmp(buf, "stop", 4) == 0)
  { // stop the robot mission
    stop(0.02);
  }
  else
    used = false;
  return used;
}


void UMotor::setPWMfrq(int frq)
{
  // pwmfrq 400\n
  if (frq > 150000) // limit'ish of motor driver
    PWMfrq=65000; // default
  else
    PWMfrq = frq;
  analogWriteFrequency(PIN_LEFT_DIR, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_LEFT_PWM, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_RIGHT_DIR, PWMfrq); /// frequency (Hz)
  analogWriteFrequency(PIN_RIGHT_PWM, PWMfrq); /// frequency (Hz)
}

void UMotor::tick()
{ //
    motorSetAnchorVoltage();
    m1ok = motorEnable[0];
    m2ok = motorEnable[1];
}

///////////////////////////////////////////////////////

void UMotor::eePromSave()
{
  // save desired PWM FRQ
//   uint16_t flags = 0;
//   eeConfig.pushWord(flags);
//   // save in kHz
//   eeConfig.pushWord(PWMfrq/1000);
}

void UMotor::eePromLoad()
{
  /*uint16_t flags =*/ //eeConfig.readWord();
//   PWMfrq = eeConfig.readWord() * 1000;
//   setup();
}



