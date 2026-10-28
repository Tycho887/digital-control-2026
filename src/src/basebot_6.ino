/***************************************************************************
-   Copyright (C) 2024 by DTU                             *
-   jcan@dtu.dk                                                    *
*
-   Base Teensy firmware
-   build for Teensy 4.1,
-   intended for digital control course
- 
- The MIT License (MIT)  https://mit-license.org/
- 
- Permission is hereby granted, free of charge, to any person obtaining a copy of this software
- and associated documentation files (the “Software”), to deal in the Software without restriction, 
- including without limitation the rights to use, copy, modify, merge, publish, distribute, 
- sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
- is furnished to do so, subject to the following conditions:
- 
- The above copyright notice and this permission notice shall be included in all copies 
- or substantial portions of the Software.
- 
- THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
- INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
- PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
- FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
- ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
- THE SOFTWARE. */

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
- Global variables */
const uint32_t sampleTimeUs = 1000; // desired sample time in us
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
  uint32_t captureUs;
  uint16_t currentADC[2];
  int16_t pwmRequest[2];
  uint8_t driverState; // bits: enabled L/R, sleeping L/R
  float encoderDelay[2];
  float controlSignal[2]; // Computed PID control signals (left, right)

} LogData;

// The forward sequence lasts 0.2 + 3.0 + 0.4 = 3.6 s. Retain every 1 ms
// sample, with margin for state transitions (3750 rows at 112 bytes/row).
const int logBufferBytes = 420000;
const int logsMax = logBufferBytes / sizeof(LogData);
static_assert(logsMax * sampleTimeUs >= 3700000,
              "Log buffer must cover the complete forward sequence plus margin");
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
const float currentScale = 0.0045; // A/count; requires hardware calibration
const float currentOffset[2] = {0.260, -0.275}; // legacy Tania offsets
uint32_t captureStartUs = 0;
int logsCnt = 0; // next log entry to use / i.e. number of used log entries
bool logPending = false; // preserve a completed capture until USB is connected
bool logTruncated = false;

// State is also used to prevent a USB log request from blocking control.
int state = 0;
float endTime = 0;
float desiredValue = 0;
float desiredHeading = 0;

class AlphaBetaFilter {
  public:
    // ---- Tuning ----
    float alpha;   // position gain,        0 < alpha < 2
    float beta;    // velocity gain,        0 < beta < alpha
    float Ts;      // sample time (seconds)

    // ---- State ----
    float position;  // smoothed position estimate  (same units as measurement)
    float velocity;  // smoothed velocity estimate  (measurement units per second)

    AlphaBetaFilter(float alpha_ = 0.30f, float beta_ = 0.05f, float Ts_ = 0.025f)
      : alpha(alpha_), beta(beta_), Ts(Ts_), position(0.0f), velocity(0.0f) {}

    void setSampleTime(float Ts_) { Ts = Ts_; }

    // Reset to a known state (call from start(), alongside controller.reset())
    void reset(float initialPosition = 0.0f, float initialVelocity = 0.0f) {
      position = initialPosition;
      velocity = initialVelocity;
    }

    // Prime with the very first measurement (optional; reduces start-up lag)
    void primeWith(float firstMeasurement) {
      position = firstMeasurement;
      velocity = 0.0f;
    }

    // Feed a new position measurement, return the smoothed velocity.
    float update(float measuredPosition) {
      // 1. Predict using the constant-velocity model
      float posPred = position + velocity * Ts;

      // 2. Innovation (residual)
      float residual = measuredPosition - posPred;

      // 3. Correct
      position = posPred + alpha * residual;
      velocity = velocity + (beta / Ts) * residual;

      return velocity;
    }
};

AlphaBetaFilter leftVelFilter (0.5f, 0.1f, sampleTimeUs * 1e-6f);
AlphaBetaFilter rightVelFilter(0.8f, 0.2f, sampleTimeUs * 1e-6f);

struct PID_Coefficients 
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
};

class WheelController {
  public: 
    PID_Coefficients coeffs;
    float u1 = 0.0f, u2 = 0.0f;
    float e1 = 0.0f, e2 = 0.0f;
    float error = 0.0f;
    float ComputeCCDE(float error);
    float ComputePID(float error);
    void reset() {
      u1 = 0.0f;
      u2 = 0.0f;
      e1 = 0.0f;
      e2 = 0.0f;
      error = 0.0f;
    }
    WheelController(PID_Coefficients x) {
      coeffs = x;
    }

};

float WheelController::ComputeCCDE(float err)
{
    error = err;
    // Difference equation execution
    float u = (coeffs.b0 * error) + (coeffs.b1 * e1) + (coeffs.b2 * e2) - (coeffs.a1 * u1) - (coeffs.a2 * u2);

    // State updates for next sample step
    e2 = e1;
    e1 = error;
    
    // Saturation 
    if (u > 3.0f) u = 3.0f;
    else if (u < -3.0f) u = -3.0f;

    u2 = u1;
    u1 = u;

    return u;
}

float WheelController::ComputePID(float err)
{
    // ---- Tunable parameters ----
    // Ts comes from the global sampleTimeUs so this stays correct
    // whatever sample time you experiment with.
    const float Ts   = sampleTimeUs * 1e-6f;   // seconds
    const float Kp   = 0.10f;                  // V per rad/s of error
    const float Ki   = 1.0f;                   // V per (rad/s · s) of error
    const float uMax = 3.0f;

    // ---- PI update ----
    // u[k] = Kp*e[k] + I[k]
    // I[k] = I[k-1] + Ki * Ts * e[k]     (with anti-windup below)
    //
    // Pure P  : leave Ki = 0.0f   (u2 stays at 0, u = Kp*err)
    // Pure I  : leave Kp = 0.0f   (u2 is the only state)
    // PI      : both non-zero     (recommended)

    error = err;

    // Integrate the error
    u2 += Ki * Ts * err;

    // Compute unsaturated output
    float u = Kp * err + u2;

    // Saturate and back-calculate the integrator (anti-windup).
    // Setting u2 = u - Kp*err makes the very next call start from a
    // state that produces exactly the saturated output, so the
    // integrator does not "wind up" while clamped.
    if (u > uMax) {
        u  = uMax;
        u2 = u - Kp * err;
    } else if (u < -uMax) {
        u  = -uMax;
        u2 = u - Kp * err;
    }

    u1 = u;   // kept for continuity with the rest of your code
    return u;
}


// Instantiate controller coefficients and controller objects globally
// Update to these PI parameters for left: u[k] = 0.273519*e[k] + -0.267766*e[k-1] + 0.000000*e[k-2] - (-1.000000*u[k-1] + 0.000000*u[k-2])
PID_Coefficients left_wheel_coeffs  = {0.273519f, -0.267766f, 0.0f, -1.000000f, 0.0f};
PID_Coefficients right_wheel_coeffs = {0.275100f, -0.271207f, 0.0f, -1.000000f, 0.0f};

WheelController leftController(left_wheel_coeffs);
WheelController rightController(right_wheel_coeffs);

float computedControl[2] = {0.0f, 0.0f};

void printLog()
{
  if (state > 0)
  {
    usb.send("% Log capture in progress; request log after the run finishes.\n");
    return;
  }
  if (!Serial)
    return;
  if (logs == nullptr)
  {
    Serial.println("% Log buffer allocation failed; run disabled.");
    return;
  }
  logPending = false;
  LogData * d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, Gear ");  Serial.print(gear);
  Serial.print(", Wheels ");  Serial.print(wheelRadius);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  if (logTruncated)
    Serial.println("% WARNING: log buffer filled; final samples are missing.");
  // Columns
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3   Desired value");
  Serial.println("% 4-5 motor voltage (left,right) (V)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");
  Serial.println("% 11-13 pose (x,y,theta) (m, m, rad)");
  Serial.println("% 14 trip distance (m)");
  Serial.println("% 15-16 legacy calibrated current (left, right) (A), forward-positive");
  Serial.println("% 17 capture timestamp (us since start), ADC read follows actuator update");
  Serial.println("% 18-19 raw current ADC counts (left A1, right A0), 12 bit");
  Serial.println("% 20-21 signed PWM request (left,right), forward-positive, full scale 4096");
  Serial.println("% 22 driver flags: bits 0/1 enable L/R, bits 2/3 sleeping L/R");
  Serial.println("% 23-24 encoder estimate delay (left,right) (s)");
  Serial.println("% 25-26 computed control signal (left,right) (V)");
  
  Serial.print("% Current A/count "); Serial.print(currentScale,7);
  Serial.print(" offsets L/R "); Serial.print(currentOffset[0],6);
  Serial.print(" "); Serial.println(currentOffset[1],6);
  Serial.print("% PWM frequency Hz "); Serial.println(motor.PWMfrq);
  Serial.println("% Voltage columns 4-5 are requests, NOT measured terminal voltages.");
  Serial.println("% Driver assumes supply loss 1 V, adds +/-0.4 V; battery <5.5 V uses 11.1 V fallback.");
  
  for (int i = 0; i < logsCnt; i++)
  {
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
    Serial.print(" ");    Serial.print(d->motorCurrent[0],6);      // 15 (A)
    Serial.print(" ");    Serial.print(d->motorCurrent[1],6);      // 16 (A)
    Serial.print(" "); Serial.print(d->captureUs); // 17
    Serial.print(" "); Serial.print(d->currentADC[0]); // 18
    Serial.print(" "); Serial.print(d->currentADC[1]); // 19
    Serial.print(" "); Serial.print(d->pwmRequest[0]); // 20
    Serial.print(" "); Serial.print(d->pwmRequest[1]); // 21
    Serial.print(" "); Serial.print(d->driverState); // 22
    Serial.print(" "); Serial.print(d->encoderDelay[0],7); // 23
    Serial.print(" "); Serial.print(d->encoderDelay[1],7); // 24
    Serial.print(" "); Serial.print(d->controlSignal[0],4); // 25
    Serial.print(" "); Serial.print(d->controlSignal[1],4); // 26
    Serial.println("");
    d++;
  }
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
}

/**
- Global variables for timing */
bool isSampleTime = false;
float time_sec = 0;

IntervalTimer sampleTimer;

void timerInterrupt()
{
  isSampleTime = true;
  time_sec += sampleTimeUs / 1e6;
}

void reset();

bool start()
{ 
  if (logs == nullptr)
  {
    usb.send("% Log buffer allocation failed; run disabled.\n");
    return false;
  }
  logPending = false;
  logTruncated = false;
  logsCnt = 0;
  captureStartUs = micros();
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; 
  encoder.encoder[1] = 0; 
  display.useDisplay = false;
  
  // Reset controllers for a clean run
  leftController.reset();
  rightController.reset();
  return true;
}

void stop(float after)
{ 
  state = 2;
  desiredValue = 0;
  endTime = time_sec + after;
}

void finished()
{ 
  motor.motorVoltage[0] = 0; 
  motor.motorVoltage[1] = 0;  
  // Apply zero PWM before any blocking serial transfer. PI outputs during
  // the settling phase may still be nonzero even with a zero speed target.
  motor.tick();
  motor.setPWMfrq(80000);
  display.useDisplay = true;
  state = 0;
  logPending = true;
}

/**
- Simple 3-step sequence */
void SpikeSequence()
{ 
  bool button;
  switch (state)
  { 
    case 0: 
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { 
        if (!start()) break;
        desiredValue = 0; 
        endTime = time_sec + 0.020; 
        state = 10;
      }
      break;
    case 10: 
      if (time_sec > endTime)
      { 
        desiredValue = 6;  
        endTime = time_sec + 0.48 ; 
        state = 11;
      }
      break;
    case 11: 
      if (time_sec > endTime)
      { 
        desiredValue = 0;
        endTime = time_sec + 0.50;
        state = 12;
      }
      break;
    case 12: 
      if (time_sec > endTime)
      { 
        desiredValue = 6;
        endTime = time_sec + 0.5;
        state = 13;
      }
      break;
    case 13: 
      if (time_sec > endTime)
      { 
        desiredValue = 0;
        endTime = time_sec + 0.5;
        state = 14;
      }
      break;
    case 14: 
      if (time_sec > endTime)
      { 
        desiredValue = 6;
        endTime = time_sec + 0.5;
        state = 100;
      }
      break;
    case 100: 
      if (time_sec > endTime)
      { 
        stop(0.6); 
      }
      break;
    default: 
      if (time_sec > endTime)
        finished();
      break;
  }
}

/**
- Simple 2-step sequence */
void sequenceTwoSteps()
{ 
  bool button;
  switch (state)
  { 
    case 0: 
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      { 
        if (!start()) break;
        desiredValue = 0; 
        endTime = time_sec + 0.020; 
        state = 10;
      }
      break;
    case 10: 
      if (time_sec > endTime)
      { 
        desiredValue = 1.5;  
        endTime = time_sec + 0.48 ; 
        state = 11;
      }
      break;
    case 11: 
      if (time_sec > endTime)
      { 
        desiredValue = 3;
        endTime = time_sec + 0.50;
        state = 12;
      }
      break;
    case 12: 
      if (time_sec > endTime)
      { 
        desiredValue = 6;
        endTime = time_sec + 0.5;
        state = 100;
      }
      break;
    case 100: 
      if (time_sec > endTime)
      { 
        stop(0.3); 
      }
      break;
    default: 
      if (time_sec > endTime)
        finished();
      break;
  }
}

/**
- Forward at 0.1 cm/s (0.001 m/s) for 3.0 seconds */
void forwardThreeSeconds()
{
  bool button;
  switch (state)
  {
    case 0:
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      {
        if (!start()) break;
        desiredValue = 0.0f;
        endTime = time_sec + 0.2f; // 0.2s baseline at standstill
        state = 10;
      }
      break;
    case 10:
      if (time_sec > endTime)
      {
        desiredValue = 0.5f;       // 2 cm/s (0.02 m/s) forward
        endTime = time_sec + 3.00f; // Hold for 3 seconds
        state = 100;
      }
      break;
    case 100:
      if (time_sec > endTime)
      {
        stop(0.40f);                // Command 0 m/s for 0.4s to capture deceleration
      }
      break;
    default:
      if (time_sec > endTime)
        finished();
      break;
  }
}

struct TrajectoryPoint 
{
  float time; 
  float yaw_rate; 
  float velocity;   
};

struct MotorCoefficients 
{
  float L;
  float R;
  float Kt;
  float Kb;
  float J;
  float D;
};

struct Trajectory {
  float timestamp;
  float velocity;
  float turn_rate;
};

// Here we used the v = (x_L + x_R) * 0.5, h' = (x_L - x_R) / B
// Putting this in matrix form and inverting, gives the system below.

float left_wheel_rad_p_s(float velocity, float turn_rate) {
  return velocity - 0.5 * wheelBase * turn_rate;
}
float right_wheel_rad_p_s(float velocity, float turn_rate) {
  return velocity + 0.5 * wheelBase * turn_rate;
}
/**
- make the control */
void controlUpdate()
{
  float velRef = desiredValue * (gear / wheelRadius);
  float left_wheel_setpoint = left_wheel_rad_p_s(velRef, 0.0f);
  float right_wheel_setpoint = right_wheel_rad_p_s(velRef, 0.0f);

  // Feed encoder counts (with sign convention: forward positive on both sides)
  float leftCounts  = -encoder.encoder[0];
  float rightCounts =  encoder.encoder[1];

  // Smoothed velocity, in encoder counts per second
  float leftCountsPerSec  = leftVelFilter.update(leftCounts);
  float rightCountsPerSec = rightVelFilter.update(rightCounts);

  // Convert counts/s to motor-shaft rad/s
  const float countsToRad = 2.0f * PI / encoder.pulsPerRev;
  float velLeftMeas  = leftCountsPerSec  * countsToRad;
  float velRightMeas = rightCountsPerSec * countsToRad;

  float errorLeft  = left_wheel_setpoint - velLeftMeas;
  float errorRight = right_wheel_setpoint - velRightMeas;

  computedControl[0] = leftController.ComputePID(errorLeft);
  computedControl[1] = rightController.ComputePID(errorRight);

  motor.motorVoltage[0] = -computedControl[0];
  motor.motorVoltage[1] =  computedControl[1];
}

void updateLog()
{ 
  if (state > 0 and logsCnt >= logsMax)
    logTruncated = true;
  if (logs != nullptr and state > 0 and logsCnt < logsMax)
  { 
    logs[logsCnt].time = time_sec;
    logs[logsCnt].state = state;
    logs[logsCnt].desiredValue = desiredValue;
    logs[logsCnt].motorVolt[0] = -motor.motorVoltage[0]; 
    logs[logsCnt].motorVolt[1] = motor.motorVoltage[1];
    logs[logsCnt].encoder[0] = -encoder.encoder[0];
    logs[logsCnt].encoder[1] = encoder.encoder[1];
    logs[logsCnt].motorVel[0] = -encoder.motorVelocity[0];
    logs[logsCnt].motorVel[1] = encoder.motorVelocity[1];
    logs[logsCnt].battery = robot.batteryVoltage;
    logs[logsCnt].pose[0] = pose[0];
    logs[logsCnt].pose[1] = pose[1];
    logs[logsCnt].pose[2] = pose[2];
    logs[logsCnt].pose[3] = pose[3];
    logs[logsCnt].distA = distA;
    logs[logsCnt].desiredHeading = desiredHeading;
    logs[logsCnt].captureUs = micros() - captureStartUs;
    logs[logsCnt].currentADC[0] = analogRead(A1);
    logs[logsCnt].currentADC[1] = analogRead(A0);
    logs[logsCnt].motorCurrent[0] = -(int(logs[logsCnt].currentADC[0]) - 2048) * currentScale + currentOffset[0];
    logs[logsCnt].motorCurrent[1] =  (int(logs[logsCnt].currentADC[1]) - 2048) * currentScale + currentOffset[1];
    logs[logsCnt].pwmRequest[0] = -motor.getPWMRequest(0);
    logs[logsCnt].pwmRequest[1] = motor.getPWMRequest(1);
    logs[logsCnt].driverState = motor.motorEnable[0] | (motor.motorEnable[1] << 1)
        | (motor.isSleeping(0) << 2) | (motor.isSleeping(1) << 3);
    logs[logsCnt].encoderDelay[0] = encoder.encoderDelay[0];
    logs[logsCnt].encoderDelay[1] = encoder.encoderDelay[1];
    logs[logsCnt].poseVel[0] = poseVel[0];
    logs[logsCnt].poseVel[1] = poseVel[1];
    
    // Store computed control signals in log
    logs[logsCnt].controlSignal[0] = computedControl[0];
    logs[logsCnt].controlSignal[1] = computedControl[1];

    logsCnt++;
  }
}

void setup()   
{ 
  Serial.begin(12000000);
  sampleTimer.begin(timerInterrupt, sampleTimeUs);
  robot.setup(); 
  motor.setup(); 
  encoder.setup(); 
  encoder.pulsPerRev = 48;
  imu2.setup(); 
  motor.setPWMfrq(0x13000); 
  usb.setup();
  display.useDisplay = true;
}

/**
- Main loop
- */
void loop ( void )
{ 
  while ( true )
  { 
    if (isSampleTime) 
    { 
      isSampleTime = false;
      imu2.tick();
      encoder.tick();
      forwardThreeSeconds();
      if (state > 0)
      { 
        controlUpdate();
      }
      motor.tick();
      updateLog();
      robot.tick();   
      display.tick(); 
    }
    usb.tick(); 
    if (logPending and state == 0 and Serial)
      printLog();
  }
}
