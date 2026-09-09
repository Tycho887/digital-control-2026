/***************************************************************************
*   Copyright (C) 2024 by DTU                             *
*   jcan@dtu.dk                                                    *
*
*   Base Teensy firmware
*   build for Teensy 4.1,
*   intended for digital control course
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

void printLog();

#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include <IntervalTimer.h>
// Support functions
#include "udisplay.h"
#include "urobot.h"
#include "uusb.h"
#include "umotor.h"
#include "uimu2.h"
#include "uencoder.h"

/**
 * Global variables */
// Sample time can not go lower than 300us
const uint32_t sampleTimeUs = 10000; // desired sample time in us
// const float ts = sampleTimeUs * 1e-6; // sample time in seconds
// Robot configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float wheelBase = 0.14;
// pose values
float distA = 0; // trip distance
float pose[4];
float poseVel[2];
bool velSaturated = false;

// ////////////////////////////////////////
// define a structure for log elements
typedef struct
{
  float time;
  int state;
  float desiredValue;
  float desiredHeading;
  float motorVel[2];
  float motorVolt[2];
  int32_t encoder[2];
  float motorCurrent[2];
  float battery;
  float pose[4];
  float distA;
  float poseVel[2];
  float inVelLeft;
  float inVelRight;
  float gyro[3];
  float headU;
  float accVel;
  float accTurn;
  float gyro1;
  float gyro2;
  float gyro3;

} LogData;
//
// allocate memory for data log in memory block 2 available only through malloc/new
// (specific for this micro processor, a Teensy 4)
const int logsMax = 400000 / sizeof(LogData); // allocates 400kBytes
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
int logsCnt = 0; // next log entry to use / i.e. number of used log entries

void printLog()
{
  LogData * d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  // hard-coded settings
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, Gear ");  Serial.print(gear);
  Serial.print(", Wheels ");  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  // Columns
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3   Desired value");
  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");
  Serial.println("% 11  Pose (x,y,theta) (m, m, rad)");

  Serial.println("% 15-16 motor current (left, right) (A)");
  for (int i = 0; i < logsCnt; i++)
  {
    // Serial.print(" ");
    Serial.print(d->time,4);    // 1
    Serial.print(" ");    Serial.print(d->state);          // 2
    Serial.print(" ");    Serial.print(d->desiredValue,2); // 3
    Serial.print(" ");    Serial.print(d->motorVolt[0],2); // 4 (V)
    Serial.print(" ");    Serial.print(d->motorVolt[1],2); // 5 (V)
    Serial.print(" ");    Serial.print(d->encoder[0]);     // 6
    Serial.print(" ");    Serial.print(d->encoder[1]);     // 7
    Serial.print(" ");    Serial.print(d->motorVel[0],2);  // 8 (rad/s)
    Serial.print(" ");    Serial.print(d->motorVel[1],2);  // 9 (rad/s)
    Serial.print(" ");    Serial.print(d->battery,2);      // 10 (V)
    Serial.print(" ");    Serial.print(d->pose[0],4);      // 11 (m)
    Serial.print(" ");    Serial.print(d->pose[1],4);      // 12 (m)
    Serial.print(" ");    Serial.print(d->pose[2],4);      // 13 (rad)
    Serial.print(" ");    Serial.print(d->distA,4);        // 14 (m)
    Serial.print(" ");    Serial.print(d->motorCurrent[0],2);      // 15 (A)
    Serial.print(" ");    Serial.print(d->motorCurrent[1],2);      // 16 (A)
    Serial.println("");
    d++;
  }
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

// ////////////////////////////////////////

/**
 * Global variables for timing */
bool isSampleTime = false;
float time_sec = 0;

// Timer that controls sample time (interrupt based)
IntervalTimer sampleTimer;

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}
/**
 * declaration, that there is a reset function */
void reset();

/**
 * Global variables for state machine */
int state = 0;               // actual sequence state
float endTime = 0;           // for current state
float desiredValue = 0;      // desired (reference) value send to controller
float desiredHeading = 0;

void start()
{ // Start timing
  // reset log and encoders
  logsCnt = 0;
  // change motor PWM frequency for sampling time test
  // should not be above 100000 (100kHz), default is 80kHz.
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; // left motor encoder
  encoder.encoder[1] = 0; // right motor encoder
  // Update of display takes too long for fast sampling
  // so, disable during sequence.
  display.useDisplay = false;
}

void stop(float after)
{ // stop, but allow continued logging for a while
  state = 2;
  desiredValue = 0;
  // end log after an additional short time
  endTime = time_sec + after;
}

void finished()
{ // prepare for a new start
  // stop motors
  motor.motorVoltage[0] = 0; // left motor
  motor.motorVoltage[1] = 0;  // right motor
  // set default PWM frequency
  motor.setPWMfrq(80000);
  // start updating display
  display.useDisplay = true;
  // wait for next button press
  state = 0;
  printLog();
}

/**
 * Simple 2-step sequence */
void sequenceTwoSteps()
{ // this function is called at every sample time
  // and should never wait in a loop.
  // Update variables as needed and return.
  bool button;
  //
  // this is a state machine
  // state 0: wait for start button press
  // other states are part of a sequence
  switch (state)
  { // run mission, initial value
    case 0: // State 0 is just inactive, waiting for the start signal.
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { // starting by a short time to get zero velocity data to the log
        // reset for new run
        start();
        // Prepare next state
        desiredValue = 0; // reference value to the controller
        // to get start the log with no velocity
        endTime = time_sec + 0.020; // new state to end after 20ms
        state = 10;
      }
      break;
    case 10: // Waiting for first step (zero velocity to log) to finish.
      if (time_sec > endTime)
      { // change to next values - drive
        desiredValue = 1.5;  // should be in meters/sec, but is motor voltage for now,
        endTime = time_sec + 0.48 ; // ~ 0.5 second
        state = 11;
      }
      break;
    case 11: // First step
      // test if ready for next state
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 3;
        endTime = time_sec + 0.50;
        state = 12;
      }
      break;
    case 12: // Second step
      // test if this state is finished
      if (time_sec > endTime)
      { // change to next values
        desiredValue = 6;
        endTime = time_sec + 0.5;
        state = 100;
      }
      break;
    case 100: // Maintain last step
      // stop after end time
      if (time_sec > endTime)
      { // stop, but continue logging for a while (0.3 sec)
        stop(0.3); // actually sets state to 999 (i.e. default:)
      }
      break;
    default: // Hold until finished
      // this state is needed to enable a new start
      if (time_sec > endTime)
        finished();
      break;
  }
}

/**
 * make the control */
void controlUpdate()
{ // do control during a mission only.
  // called at every tick
  //
  float velRef = desiredValue;
  //
  // For a start, the desiredValue
  // is just used as motor voltage
  //
  // Left motor voltage (-9V to +9V)
  motor.motorVoltage[0] = -velRef;
  // Right motor voltage (-9V to +9V)
  motor.motorVoltage[1] = velRef;
  //
  // in the main loop, there is a motor.tick() that actually send the
  // values to the motor.
}

void updateLog()
{ // step response mode
  if (state > 0 and logsCnt < logsMax)
  { // save data for this sample into dataLog structure
    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue = desiredValue;
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0]; // The left motor runs backwards, therefore the sign.
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    // the last part is offset of current measurement (in Amps)
    // take the initial current readings and add or subtract as needed
    // these values fits Tania: (+0.260A, -0.275A)
    logs[logsCnt].desiredHeading = desiredHeading;
    logs[logsCnt].motorCurrent[0] = -(analogRead(A1) - 2048) * 0.0045 + 0.260;
    logs[logsCnt].motorCurrent[1] =  (analogRead(A0) - 2048) * 0.0045 - 0.275;
    logs[logsCnt].poseVel[0] = poseVel[0];
    logs[logsCnt].poseVel[1] = poseVel[1];
    // add more items as needed, NB! also the the struct definition at top.
    // We store the parameters from the Gyro, from imu2
    logs[logsCnt].gyro[0] = imu2.gyro[0];
    logs[logsCnt].gyro[1] = imu2.gyro[1];
    logs[logsCnt].gyro[2] = imu2.gyro[2];
    logsCnt++;
  }
}


void setup()   // INITIALIZATION
{ // to be able to print to USB interface
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  // initialize sensors, safety and display
  robot.setup(); // alive LED, display and battery
  motor.setup(); // motor voltage
  encoder.setup(); // motor encoders to velocity
  encoder.pulsPerRev = 48;
  imu2.setup(); // gyro and accelerometer
  motor.setPWMfrq(0x13000); // 0x13000 = 77824 Hz
  usb.setup();
  display.useDisplay = true;
}

/**
* Main loop
* */
void loop ( void )
{ // init sample time
  while ( true )
  { // main loop
    // loop until time for next sample
    if (isSampleTime) // start of new control cycle
    { // time for next tick
      isSampleTime = false;
      // read sensors
      imu2.tick();
      encoder.tick();
      // updatePose();
      sequenceTwoSteps();
      //
      if (state > 0)
      { // Only if started
        // Calculate new motor voltage
        controlUpdate();
      }
      // give value to actuators
      motor.tick();
      // save relevant values
      updateLog();
      // support functions
      robot.tick();   // measure battery voltage etc.
      display.tick(); // update O-LED display
    }
    usb.tick(); // listen to incoming from USB
  }
}

/////////////////////////////////////////////////////////////////
