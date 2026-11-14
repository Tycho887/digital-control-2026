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
#include <string.h>
#include <math.h>
#include <IntervalTimer.h>
// Support functions
#include "udisplay.h"
#include "urobot.h"
#include "uusb.h"
#include "umotor.h"
#include "uimu2.h"
#include "uencoder.h"
#include "AS5X47.h"
#include "kalman.h"

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
// Kalman filter estimator state
// IMU mounting: X down (vertical), Y right, Z backwards. A mounting change
// must update these axis/sign choices together with the magnetometer mapping
// in uimu2.cpp.
const int imuForwardAxis = 2;             // forward = -IMU Z
const float imuForwardAccelSign = -1.0f;
const int imuYawAxis = 0;                  // CCW yaw = -IMU X
const float imuYawRateSign = -1.0f;
const float gravity = 9.80665f;
const float gyroToRad = PI / 180.0f;
float kfAccelBias = 0.0f;
uint32_t kfAccelCalCnt = 0;
bool kfAccelCalDone = false;
bool kfHeadingInit = false;
float kfStartS = 0.0f, kfStartTheta = 0.0f;
float kfX = 0.0f, kfY = 0.0f;
float kfPrevS = 0.0f, kfPrevTheta = 0.0f;
uint32_t kfLastUs = 0;
bool velSaturated = false;

// Absolute angle wheel sensors (AS5047P/AS5147) on the motor shafts, sharing
// the hardware SPI bus (MOSI 11, MISO 12, SCK 13). Chip selects are
// assumptions to be confirmed against the harness.
const uint8_t AS5X47_LEFT_CS = 24;
const uint8_t AS5X47_RIGHT_CS = 25;
// Sensor direction vs forward travel; flip a sign if a sensor is mounted
// mirrored. The left sign matches the quadrature encoder convention below.
const float AS5X47_LEFT_DIR = -1.0f;
const float AS5X47_RIGHT_DIR = 1.0f;
// Constructed on first use so SPI.begin() runs after the Teensy core init().
AS5X47 &leftWheelSensor()
{
  static AS5X47 sensor(AS5X47_LEFT_CS);
  return sensor;
}
AS5X47 &rightWheelSensor()
{
  static AS5X47 sensor(AS5X47_RIGHT_CS);
  return sensor;
}
// 10 ms angle-differencing state, in motor-shaft radians
bool wheelOdoInit = false;
float wheelOdoPrevAngle[2] = {0.0f, 0.0f};
float wheelOdoAccum[2] = {0.0f, 0.0f};
uint32_t wheelOdoStartUs = 0;
// Latest AS5X47 values, exposed for the calibration capture.
bool as5Valid = false;
bool as5Fresh = false;
float as5AngleLast[2] = {0.0f, 0.0f}; // rad, after direction sign
float as5VelLast[2] = {0.0f, 0.0f};   // wheel m/s, forward-positive
float as5Window = 0.0f;               // s

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
  float controlSetpoint[2]; // Wheel-speed reference (left, right) (motor rad/s)
  float kfState[KF_N]; // [s, theta, v, omega, b_a, b_g, b_m]
  float kfCov[KF_N * (KF_N + 1) / 2]; // lower triangle of P, row-major

} LogData;

// Calibration capture. The row is smaller than LogData, so it reuses the
// same buffer at a higher sample count. Standard and calibration captures
// are mutually exclusive.
enum CalMode : uint8_t
{
  CAL_NONE = 0,
  CAL_MAG = 1,
  CAL_NOISE = 2,
  CAL_TRACK = 3,
};

enum CalFlag : uint16_t
{
  CAL_ACCEL_FRESH = 1 << 0,
  CAL_GYRO_FRESH = 1 << 1,
  CAL_MAG_FRESH = 1 << 2,
  CAL_MAG_VALID = 1 << 3,
  CAL_MAG_AVAILABLE = 1 << 4,
  CAL_AS5_VALID = 1 << 5,
  CAL_AS5_FRESH = 1 << 6,
  CAL_KF_HEALTHY = 1 << 7,
};

enum KfUpdateFlag : uint16_t
{
  CAL_KF_GYRO = 1 << 0,
  CAL_KF_WHEEL_V = 1 << 1,
  CAL_KF_WHEEL_OMEGA = 1 << 2,
  CAL_KF_MAG = 1 << 3,
  CAL_KF_MAG_REJECTED = 1 << 4,
};

typedef struct
{
  float time;
  uint16_t mode;
  uint16_t flags;
  uint16_t kfMask;
  uint16_t pad;
  float setpoint[2];       // motor rad/s
  float feedback[2];       // PI feedback, motor rad/s
  float motorVolt[2];      // V request, forward-positive
  float motorVel[2];       // encoder motor rad/s, forward-positive
  float gyro[3];           // deg/s, offset-corrected
  float acc[3];            // g
  float mag[3];            // adjusted counts (0.15 uT/LSB)
  float as5Angle[2];       // rad, after direction sign
  float as5Vel[2];         // wheel m/s, forward-positive
  float wheelWindow;       // s
  float magHeading;        // rad, NaN if unavailable
  float kfV;
  float kfOmega;
  float kfTheta;
  float kfInnov[DiffDriveKalmanFilter::M_COUNT];
  float kfInnovVar[DiffDriveKalmanFilter::M_COUNT];
} CalData;

// The active sequenceTwoSteps run lasts 1.8 s. A row is now 260 bytes after
// adding the controller setpoints and Kalman state/covariance, so the buffer
// covers about 1.9 s at 1 kHz. For the 3.6 s forwardThreeSeconds run set
// logDivider to 2 (500 Hz) and logCoverageUs to 3700000.
const int logBufferBytes = 495000; // leaves ~16 kB of the OCRAM heap free
const int logsMax = logBufferBytes / sizeof(LogData);
const int logDivider = 1; // log one row per N control ticks
const uint32_t logCoverageUs = 1900000;
static_assert(logDivider >= 1, "logDivider must be at least 1");
static_assert(logsMax * sampleTimeUs * logDivider >= logCoverageUs,
              "Log buffer must cover the complete sequence plus margin");
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
CalData * calLogs = (CalData*)logs;
const int calMax = (logsMax * sizeof(LogData)) / sizeof(CalData);
static_assert(calMax >= 3000,
              "Calibration buffer must hold the 6 s track capture at 500 Hz");
const float currentScale = 0.0045; // A/count; requires hardware calibration
const float currentOffset[2] = {0.260, -0.275}; // legacy Tania offsets
uint32_t captureStartUs = 0;
int logsCnt = 0; // next log entry to use / i.e. number of used log entries
uint32_t logTickCnt = 0; // control ticks since start, for logDivider
bool logPending = false; // preserve a completed capture until USB is connected
bool logTruncated = false;

// Calibration experiment state.
CalMode calMode = CAL_NONE;
int calState = 0;        // 0 idle, 1 running
bool calArmed = false;   // track experiment waiting for the start button
bool calPending = false; // completed capture waiting for USB
bool calTruncated = false;
uint32_t calStartUs = 0;
float calEndTime = 0.0f;
int calCnt = 0;
int calDivider = 1;
int calTickCnt = 0;
int calTrackIndex = 0;
float calTrackStepEnd = 0.0f;

// State is also used to prevent a USB log request from blocking control.
int state = 0;
float endTime = 0;
float desiredValue = 0;
float desiredHeading = 0;

// The abstract filter class is supposed to let us implement different filters.
// The two main ones are low-pass filters for the wheel speed and a KF for the dead-reckoning.
// It needs methods to allow for updates of internal states and sampling/interpolation of internal states
// This is critical for sensors / controllers operating at different speeds

struct WheelSpeeds 
{
  float left;
  float right;
};

struct Polar
{
  float speed;
  float heading;
};

WheelSpeeds polar2Cartesian(float speed, float heading)
{
  WheelSpeeds ws;
  ws.left = speed - (heading * wheelBase / 2.0f);
  ws.right = speed + (heading * wheelBase / 2.0f);
  return ws;
}



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

// Actuator limit for the wheel-speed PI. The week07 tuning used 9 V; 3 V is
// retained here for safety, and the tracking experiment reports saturation.
const float controlVoltageLimit = 3.0f;

struct PID_Coefficients 
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
};

class YawController {
  public:
    PID_Coefficients coeffs;
    float u1 = 0.0f, u2 = 0.0f;
    float e1 = 0.0f, e2 = 0.0f;
    float error = 0.0f;
    float ComputeCCDE(float error);
    void reset() {
      u1 = 0.0f;
      u2 = 0.0f;
      e1 = 0.0f;
      e2 = 0.0f;
      error = 0.0f;
    }
    YawController(PID_Coefficients x) {
      coeffs = x;
    }

};

class VelocityController {
  public:
    PID_Coefficients coeffs;
    float u1 = 0.0f, u2 = 0.0f;
    float e1 = 0.0f, e2 = 0.0f;
    float error = 0.0f;
    float ComputeCCDE(float error);
    void reset() {
      u1 = 0.0f;
      u2 = 0.0f;
      e1 = 0.0f;
      e2 = 0.0f;
      error = 0.0f;
    }
    VelocityController(PID_Coefficients x) {
      coeffs = x;
    }
};

class WheelController {
  public: 
    PID_Coefficients coeffs;
    float u1 = 0.0f, u2 = 0.0f;
    float e1 = 0.0f, e2 = 0.0f;
    float error = 0.0f;
    float ComputeCCDE(float error);
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
    
    // Saturation. The incremental form stores the clamped output, so the
    // next update starts from the limited value and does not wind up.
    if (u > controlVoltageLimit) u = controlVoltageLimit;
    else if (u < -controlVoltageLimit) u = -controlVoltageLimit;

    u2 = u1;
    u1 = u;

    return u;
}

// Week07-tuned PI difference equations, valid for Ts = 1 ms:
//   u[k] = u[k-1] + b0*e[k] + b1*e[k-1]
// with Kp = -b1 and Ki = (b0 + b1)/Ts.
PID_Coefficients left_wheel_coeffs  = {0.273519f, -0.267766f, 0.0f, -1.000000f, 0.0f};
PID_Coefficients right_wheel_coeffs = {0.275100f, -0.271207f, 0.0f, -1.000000f, 0.0f};

WheelController leftController(left_wheel_coeffs);
WheelController rightController(right_wheel_coeffs);

float computedControl[2] = {0.0f, 0.0f};
float controlSetpoint[2] = {0.0f, 0.0f};
float controlFeedback[2] = {0.0f, 0.0f}; // PI feedback, forward-positive motor rad/s
uint16_t kfUpdateMask = 0;               // KF updates applied on the current tick

void printLogValue(float value)
{
  char buf[24];
  snprintf(buf, sizeof(buf), " %.9g", value);
  Serial.print(buf);
}

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
  Serial.println("% 27-28 Kalman velocity and yaw rate (m/s, rad/s)");
  Serial.println("% 29 magnetometer heading (rad, NaN if unavailable)");
  Serial.println("% 30-31 controller setpoint (left,right) (motor rad/s)");
  Serial.println("% 32-38 Kalman state [s, theta, v, omega, b_a, b_g, b_m]");
  Serial.println("%       (m, rad, m/s, rad/s, m/s^2, rad/s, rad)");
  Serial.println("% 39-66 Kalman covariance lower triangle, row-major:");
  Serial.println("%       P00; P10 P11; P20 P21 P22; ...; P66");
  Serial.println("%       P is symmetric, so the lower triangle is the full matrix.");
  
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
    Serial.print(" "); Serial.print(d->poseVel[0],4); // 27
    Serial.print(" "); Serial.print(d->poseVel[1],4); // 28
    Serial.print(" "); Serial.print(d->pose[3],4); // 29
    for (int k = 0; k < 2; k++) printLogValue(d->controlSetpoint[k]); // 30-31
    for (int k = 0; k < KF_N; k++) printLogValue(d->kfState[k]); // 32-38
    for (int k = 0; k < KF_N * (KF_N + 1) / 2; k++) printLogValue(d->kfCov[k]); // 39-66
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
  if (calState > 0)
  {
    usb.send("% Calibration capture active; run disabled.\n");
    return false;
  }
  logPending = false;
  logTruncated = false;
  // The standard log shares the buffer with calibration captures.
  calPending = false;
  calArmed = false;
  calMode = CAL_NONE;
  logsCnt = 0;
  logTickCnt = 0;
  controlSetpoint[0] = 0.0f;
  controlSetpoint[1] = 0.0f;
  controlFeedback[0] = 0.0f;
  controlFeedback[1] = 0.0f;
  computedControl[0] = 0.0f;
  computedControl[1] = 0.0f;
  captureStartUs = micros();
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; 
  encoder.encoder[1] = 0; 
  // Restart the wheel-speed window so the run begins from a clean seed.
  wheelOdoInit = false;
  wheelOdoAccum[0] = 0.0f;
  wheelOdoAccum[1] = 0.0f;
  display.useDisplay = false;
  
  // Reset controllers and velocity filters for a clean run. Without the
  // filter resets, the first samples inherit the previous run's velocity.
  leftController.reset();
  rightController.reset();
  leftVelFilter.reset();
  rightVelFilter.reset();
  // Capture the pose origin; the filter keeps its learned biases.
  kfStartS = kf.x[DiffDriveKalmanFilter::S];
  kfStartTheta = kf.x[DiffDriveKalmanFilter::THETA];
  kfX = 0.0f; kfY = 0.0f;
  kfPrevS = kfStartS; kfPrevTheta = kfStartTheta;
  // Make the first logged row use the new origin, before the next estimator tick.
  pose[0] = 0.0f; pose[1] = 0.0f; pose[2] = 0.0f;
  pose[3] = imu2.magHeadingValid ? imu2.magHeading : NAN;
  poseVel[0] = 0.0f; poseVel[1] = 0.0f;
  distA = 0.0f;
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
        desiredValue = 0.05f;  
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
        desiredValue = 0.10f;
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

/**
- Wheel-speed PI on the quadrature encoders.
- Setpoints are motor-shaft rad/s, forward-positive. The alpha-beta filtered
  encoder velocity is recorded as controlFeedback so the tracking experiment
  can compare the controller's own signal with AS5X47/KF velocity.
 */
void wheelSpeedControl(float leftSetpoint, float rightSetpoint)
{
  controlSetpoint[0] = leftSetpoint;
  controlSetpoint[1] = rightSetpoint;

  // Feed encoder counts (with sign convention: forward positive on both sides)
  float leftCounts  = -encoder.encoder[0];
  float rightCounts =  encoder.encoder[1];

  // Smoothed velocity, in encoder counts per second
  float leftCountsPerSec  = leftVelFilter.update(leftCounts);
  float rightCountsPerSec = rightVelFilter.update(rightCounts);

  // Convert counts/s to motor-shaft rad/s
  const float countsToRad = 2.0f * PI / encoder.pulsPerRev;
  controlFeedback[0] = leftCountsPerSec  * countsToRad;
  controlFeedback[1] = rightCountsPerSec * countsToRad;

  computedControl[0] = leftController.ComputeCCDE(leftSetpoint - controlFeedback[0]);
  computedControl[1] = rightController.ComputeCCDE(rightSetpoint - controlFeedback[1]);

  motor.motorVoltage[0] = -computedControl[0];
  motor.motorVoltage[1] =  computedControl[1];
}

/**
- Convert the sequence's linear speed in m/s to motor rad/s and run the PI. */
void controlUpdate()
{
  float velRef = desiredValue * (gear / wheelRadius);
  wheelSpeedControl(velRef, velRef);
}

/**
- Read one AS5X47 compensated angle in motor-shaft radians.
- Returns false when the SPI frame is flagged, so a failed transfer cannot
  turn into a fake angle step (a zero word would look like a jump).
- ANGLE_COMPSATED (0x3FFF) is the same register AS5X47::readAngle() uses. */
bool readWheelAngle(AS5X47 &sensor, float &angleRad)
{
  ReadDataFrame frame = sensor.readRegister(ANGLE_COMPSATED);
  if (frame.values.ef or frame.values.pard)
    return false;
  Angle angle;
  angle.raw = frame.values.data;
  angleRad = angle.values.cordicang * (2.0f * PI / 16384.0f);
  return true;
}

/**
- AS5X47 wheel speed, 1 kHz reads over a 10 ms window.
- Both absolute angles are read every tick and their unwrapped differences
  accumulated. Every KF_WHEEL_WINDOW the accumulators are converted to wheel
  speed and fed to the filter as v and omega.
- The sensors sit on the motor shafts, so wheel angle = sensor angle / gear.
- AS5X47::readVel() is deliberately not used: its 24.141 deg/s LSB is
  unusable at the robot's low speeds. */
void wheelOdometryUpdate()
{
  as5Fresh = false;
  float angleLeft = 0.0f, angleRight = 0.0f;
  if (!readWheelAngle(leftWheelSensor(), angleLeft) or
      !readWheelAngle(rightWheelSensor(), angleRight))
  {
    as5Valid = false;
    return;
  }
  angleLeft *= AS5X47_LEFT_DIR;
  angleRight *= AS5X47_RIGHT_DIR;
  as5Valid = true;
  as5AngleLast[0] = angleLeft;
  as5AngleLast[1] = angleRight;

  uint32_t nowUs = micros();
  if (!wheelOdoInit)
  { // seed the previous angles; the first window has no usable delta yet
    wheelOdoPrevAngle[0] = angleLeft;
    wheelOdoPrevAngle[1] = angleRight;
    wheelOdoAccum[0] = 0.0f;
    wheelOdoAccum[1] = 0.0f;
    wheelOdoStartUs = nowUs;
    wheelOdoInit = true;
    return;
  }

  wheelOdoAccum[0] += wrapAngle(angleLeft - wheelOdoPrevAngle[0]);
  wheelOdoAccum[1] += wrapAngle(angleRight - wheelOdoPrevAngle[1]);
  wheelOdoPrevAngle[0] = angleLeft;
  wheelOdoPrevAngle[1] = angleRight;

  float window = (nowUs - wheelOdoStartUs) * 1e-6f;
  if (window < KF_WHEEL_WINDOW)
    return;

  float wheelAngleLeft = wheelOdoAccum[0] / gear;
  float wheelAngleRight = wheelOdoAccum[1] / gear;
  float vLeft = wheelRadius * wheelAngleLeft / window;
  float vRight = wheelRadius * wheelAngleRight / window;
  kf.updateWheelV(0.5f * (vLeft + vRight), window);
  kf.updateWheelOmega((vRight - vLeft) / wheelBase, window);
  if (kf.lastInnovationValid[DiffDriveKalmanFilter::M_WHEEL_V])
    kfUpdateMask |= CAL_KF_WHEEL_V;
  if (kf.lastInnovationValid[DiffDriveKalmanFilter::M_WHEEL_OMEGA])
    kfUpdateMask |= CAL_KF_WHEEL_OMEGA;

  as5VelLast[0] = vLeft;
  as5VelLast[1] = vRight;
  as5Window = window;
  as5Fresh = true;

  wheelOdoAccum[0] = 0.0f;
  wheelOdoAccum[1] = 0.0f;
  wheelOdoStartUs = nowUs;
}

/**
- Kalman filter update, 1 kHz */
void estimatorUpdate()
{
  uint32_t nowUs = micros();
  float dt = (nowUs - kfLastUs) * 1e-6f;
  kfLastUs = nowUs;
  kfUpdateMask = 0;
  if (dt <= 0.0f || dt > 0.020f)
  {
    kfPrevS = kf.x[DiffDriveKalmanFilter::S];
    kfPrevTheta = kf.x[DiffDriveKalmanFilter::THETA];
    return;
  }
  float aRaw = imuForwardAccelSign * imu2.acc[imuForwardAxis] * gravity;
  if (!kfAccelCalDone && imu2.accelFresh)
  {
    kfAccelCalCnt++;
    kfAccelBias += (aRaw - kfAccelBias) / kfAccelCalCnt;
    if (kfAccelCalCnt >= 2000) kfAccelCalDone = true;
  }
  kf.predict(kfAccelCalDone ? aRaw - kfAccelBias : 0.0f, dt);
  if (imu2.gyroFresh && imu2.gyroOffsetDone)
  {
    kf.updateGyro(imuYawRateSign * imu2.gyro[imuYawAxis] * gyroToRad);
    if (kf.lastInnovationValid[DiffDriveKalmanFilter::M_GYRO])
      kfUpdateMask |= CAL_KF_GYRO;
  }
  // Wheel speed now comes from the AS5X47 absolute angles only. The stock
  // quadrature encoders remain in use by controlUpdate() but not here.
  wheelOdometryUpdate();
  if (imu2.magFresh && imu2.magHeadingValid)
  {
    if (!kfHeadingInit)
    {
      float delta = imu2.magHeading - kf.x[DiffDriveKalmanFilter::THETA];
      kf.x[DiffDriveKalmanFilter::THETA] = imu2.magHeading;
      kf.x[DiffDriveKalmanFilter::BM] = 0.0f;
      if (state > 0) kfStartTheta += delta;
      kfPrevTheta += delta;
      kfHeadingInit = true;
    }
    else
    {
      kf.updateMag(imu2.magHeading);
      if (kf.magRejected)
        kfUpdateMask |= CAL_KF_MAG_REJECTED;
      else if (kf.lastInnovationValid[DiffDriveKalmanFilter::M_MAG])
        kfUpdateMask |= CAL_KF_MAG;
    }
  }
  float s = kf.x[DiffDriveKalmanFilter::S];
  float theta = kf.x[DiffDriveKalmanFilter::THETA];
  float ds = s - kfPrevS;
  float headingRel = theta - kfStartTheta;
  float headingRelPrev = kfPrevTheta - kfStartTheta;
  float mid = 0.5f * (headingRel + headingRelPrev);
  kfX += ds * cosf(mid);
  kfY += ds * sinf(mid);
  kfPrevS = s;
  kfPrevTheta = theta;
  pose[0] = kfX;
  pose[1] = kfY;
  pose[2] = headingRel;
  pose[3] = imu2.magHeadingValid ? imu2.magHeading : NAN;
  poseVel[0] = kf.x[DiffDriveKalmanFilter::V];
  poseVel[1] = kf.x[DiffDriveKalmanFilter::OMEGA];
  distA = s - kfStartS;
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

    logs[logsCnt].controlSetpoint[0] = controlSetpoint[0];
    logs[logsCnt].controlSetpoint[1] = controlSetpoint[1];
    for (int k = 0; k < KF_N; k++)
      logs[logsCnt].kfState[k] = kf.x[k];
    // P is symmetric; store the lower triangle row by row.
    int covIndex = 0;
    for (int i = 0; i < KF_N; i++)
      for (int j = 0; j <= i; j++)
        logs[logsCnt].kfCov[covIndex++] = kf.P[i * KF_N + j];

    logsCnt++;
  }
}

// ////////////////////////////////////////
// Calibration experiments

const float calTrackSpeed[] = {
  0.0f, 0.05f, 0.10f, 0.15f, 0.0f, -0.05f, -0.10f, -0.15f, 0.0f
};
const float calTrackDuration[] = {
  0.3f, 0.8f, 0.8f, 0.8f, 0.5f, 0.8f, 0.8f, 0.8f, 0.4f
};
const int calTrackSteps = sizeof(calTrackSpeed) / sizeof(calTrackSpeed[0]);

const char * calModeName(CalMode mode)
{
  switch (mode)
  {
    case CAL_MAG: return "mag";
    case CAL_NOISE: return "noise";
    case CAL_TRACK: return "track";
    default: return "none";
  }
}

void resetWheelControl()
{
  leftController.reset();
  rightController.reset();
  leftVelFilter.reset();
  rightVelFilter.reset();
  controlSetpoint[0] = controlSetpoint[1] = 0.0f;
  controlFeedback[0] = controlFeedback[1] = 0.0f;
  computedControl[0] = computedControl[1] = 0.0f;
}

bool calStart(CalMode mode)
{
  if (logs == nullptr or calLogs == nullptr)
  {
    usb.send("% Calibration buffer unavailable.\r\n");
    return false;
  }
  if (state > 0)
  {
    usb.send("% Standard run active; calibration disabled.\r\n");
    return false;
  }
  if (calState > 0)
  {
    usb.send("% Calibration capture already active.\r\n");
    return false;
  }
  logPending = false;
  calPending = false;
  calTruncated = false;
  calMode = mode;
  calCnt = 0;
  calTickCnt = 0;
  calStartUs = micros();
  calDivider = (mode == CAL_MAG) ? 5 : (mode == CAL_NOISE) ? 1 : 2;
  motor.motorVoltage[0] = 0.0f;
  motor.motorVoltage[1] = 0.0f;
  motor.tick();
  resetWheelControl();

  if (mode == CAL_TRACK)
  {
    calState = 0;
    calArmed = true;
    usb.send("% cal track armed: clear the floor, then press start\r\n");
    return true;
  }
  calArmed = false;
  calState = 1;
  calEndTime = time_sec + ((mode == CAL_MAG) ? 15.0f : 3.0f);
  if (mode == CAL_MAG)
    usb.send("% cal mag: motors off. Rotate 360 deg by hand and back within 15 s.\r\n");
  else
    usb.send("% cal noise: motors off. Keep the robot still for 3 s.\r\n");
  return true;
}

void calBeginTrack()
{
  calArmed = false;
  calState = 1;
  calMode = CAL_TRACK;
  calCnt = 0;
  calTickCnt = 0;
  calDivider = 2;
  calStartUs = micros();
  calTrackIndex = 0;
  calTrackStepEnd = time_sec + calTrackDuration[0];
  motor.setPWMfrq(80000);
  float total = 0.0f;
  for (int i = 0; i < calTrackSteps; i++)
    total += calTrackDuration[i];
  calEndTime = time_sec + total;
  resetWheelControl();
  usb.send("% cal track started\r\n");
}

void calFinish()
{
  calState = 0;
  calArmed = false;
  motor.motorVoltage[0] = 0.0f;
  motor.motorVoltage[1] = 0.0f;
  motor.tick();
  resetWheelControl();
  calPending = true;
}

void calStop()
{
  calArmed = false;
  if (calState > 0)
  {
    calState = 0;
    calPending = (calCnt > 0);
  }
  motor.motorVoltage[0] = 0.0f;
  motor.motorVoltage[1] = 0.0f;
  motor.tick();
  resetWheelControl();
  if (calPending)
    usb.send("% cal stopped; capture pending\r\n");
  else
    usb.send("% cal stopped\r\n");
}

void calTick()
{
  if (calMode == CAL_TRACK)
  {
    if (time_sec >= calTrackStepEnd)
    {
      calTrackIndex++;
      if (calTrackIndex >= calTrackSteps)
      {
        calFinish();
        return;
      }
      calTrackStepEnd += calTrackDuration[calTrackIndex];
    }
    float motorSp = calTrackSpeed[calTrackIndex] * (gear / wheelRadius);
    wheelSpeedControl(motorSp, motorSp);
  }
  else
  {
    // Sensor-only experiments keep the motors off.
    motor.motorVoltage[0] = 0.0f;
    motor.motorVoltage[1] = 0.0f;
    controlSetpoint[0] = controlSetpoint[1] = 0.0f;
  }

  if (time_sec > calEndTime)
    calFinish();
}

void updateCalLog()
{
  if (calLogs == nullptr or calState == 0)
    return;
  if (calCnt >= calMax)
  {
    calTruncated = true;
    return;
  }
  CalData &d = calLogs[calCnt];
  d.time = time_sec;
  d.mode = (uint16_t)calMode;
  uint16_t flags = 0;
  if (imu2.accelFresh) flags |= CAL_ACCEL_FRESH;
  if (imu2.gyroFresh) flags |= CAL_GYRO_FRESH;
  if (imu2.magFresh) flags |= CAL_MAG_FRESH;
  if (imu2.magHeadingValid) flags |= CAL_MAG_VALID;
  if (imu2.magAvailable) flags |= CAL_MAG_AVAILABLE;
  if (as5Valid) flags |= CAL_AS5_VALID;
  if (as5Fresh) flags |= CAL_AS5_FRESH;
  if (kf.healthy) flags |= CAL_KF_HEALTHY;
  d.flags = flags;
  d.kfMask = kfUpdateMask;
  d.pad = 0;

  for (int k = 0; k < 2; k++)
  {
    d.setpoint[k] = controlSetpoint[k];
    d.feedback[k] = controlFeedback[k];
    d.motorVolt[k] = (k == 0) ? -motor.motorVoltage[0] : motor.motorVoltage[1];
    d.motorVel[k] = (k == 0) ? -encoder.motorVelocity[0] : encoder.motorVelocity[1];
    d.as5Angle[k] = as5AngleLast[k];
    d.as5Vel[k] = as5VelLast[k];
  }
  for (int k = 0; k < 3; k++)
  {
    d.gyro[k] = imu2.gyro[k];
    d.acc[k] = imu2.acc[k];
    d.mag[k] = imu2.mag[k];
  }
  d.wheelWindow = as5Window;
  d.magHeading = imu2.magHeadingValid ? imu2.magHeading : NAN;
  d.kfV = kf.x[DiffDriveKalmanFilter::V];
  d.kfOmega = kf.x[DiffDriveKalmanFilter::OMEGA];
  d.kfTheta = kf.x[DiffDriveKalmanFilter::THETA];
  for (int i = 0; i < DiffDriveKalmanFilter::M_COUNT; i++)
  {
    d.kfInnov[i] = kf.lastInnovation[i];
    d.kfInnovVar[i] = kf.lastInnovationVar[i];
  }
  calCnt++;
}

void printCalLog()
{
  if (calState > 0)
  {
    usb.send("% Calibration capture in progress; request log after it finishes.\r\n");
    return;
  }
  if (!Serial)
    return;
  if (calLogs == nullptr)
  {
    Serial.println("% Calibration buffer allocation failed.");
    return;
  }
  calPending = false;
  Serial.print("% CAL ");
  Serial.println(calModeName(calMode));
  Serial.print("% max cal samples ");
  Serial.print(calMax);
  Serial.print(" each sized ");
  Serial.print(sizeof(CalData));
  Serial.println(" bytes");
  if (calTruncated)
    Serial.println("% WARNING: cal buffer filled; final samples are missing.");
  Serial.println("% 1 time (s)");
  Serial.println("% 2 mode (1 mag, 2 noise, 3 track)");
  Serial.println("% 3 flags (bit0 accel fresh, bit1 gyro fresh, bit2 mag fresh, bit3 mag valid, bit4 mag available, bit5 AS5X47 valid, bit6 AS5X47 velocity fresh, bit7 KF healthy)");
  Serial.println("% 4 kfMask (bit0 gyro, bit1 wheel v, bit2 wheel omega, bit3 mag, bit4 mag rejected)");
  Serial.println("% 5-6 setpoint (left,right) (motor rad/s)");
  Serial.println("% 7-8 PI feedback (left,right) (motor rad/s)");
  Serial.println("% 9-10 motor voltage request (left,right) (V, forward-positive)");
  Serial.println("% 11-12 encoder motor velocity (left,right) (rad/s, forward-positive)");
  Serial.println("% 13-15 gyro raw (x,y,z) (deg/s, offset-corrected)");
  Serial.println("% 16-18 accel raw (x,y,z) (g)");
  Serial.println("% 19-21 mag raw (x,y,z) (adjusted counts, 0.15 uT/LSB)");
  Serial.println("% 22-23 AS5X47 angle (left,right) (rad, after direction sign)");
  Serial.println("% 24-25 AS5X47 wheel velocity (left,right) (m/s, forward-positive)");
  Serial.println("% 26 wheel window (s)");
  Serial.println("% 27 mag heading (rad, NaN if unavailable)");
  Serial.println("% 28-30 KF v, omega, theta (m/s, rad/s, rad)");
  Serial.println("% 31-34 KF innovation y (gyro, wheel v, wheel omega, mag)");
  Serial.println("% 35-38 KF innovation variance S for the same four updates");

  for (int i = 0; i < calCnt; i++)
  {
    CalData &d = calLogs[i];
    Serial.print(d.time, 4);
    Serial.print(" "); Serial.print(d.mode);
    Serial.print(" "); Serial.print(d.flags);
    Serial.print(" "); Serial.print(d.kfMask);
    for (int k = 0; k < 2; k++) printLogValue(d.setpoint[k]);
    for (int k = 0; k < 2; k++) printLogValue(d.feedback[k]);
    for (int k = 0; k < 2; k++) printLogValue(d.motorVolt[k]);
    for (int k = 0; k < 2; k++) printLogValue(d.motorVel[k]);
    for (int k = 0; k < 3; k++) printLogValue(d.gyro[k]);
    for (int k = 0; k < 3; k++) printLogValue(d.acc[k]);
    for (int k = 0; k < 3; k++) printLogValue(d.mag[k]);
    for (int k = 0; k < 2; k++) printLogValue(d.as5Angle[k]);
    for (int k = 0; k < 2; k++) printLogValue(d.as5Vel[k]);
    printLogValue(d.wheelWindow);
    printLogValue(d.magHeading);
    printLogValue(d.kfV);
    printLogValue(d.kfOmega);
    printLogValue(d.kfTheta);
    for (int i2 = 0; i2 < DiffDriveKalmanFilter::M_COUNT; i2++)
      printLogValue(d.kfInnov[i2]);
    for (int i2 = 0; i2 < DiffDriveKalmanFilter::M_COUNT; i2++)
      printLogValue(d.kfInnovVar[i2]);
    Serial.println("");
  }
  Serial.print("% sent ");
  Serial.print(calCnt);
  Serial.println(" cal lines");
}

bool decodeExperiment(const char *buf)
{
  if (strncmp(buf, "cal", 3) != 0)
    return false;
  const char *p = buf + 3;
  while (*p == ' ')
    p++;
  if (strncmp(p, "mag", 3) == 0)
    calStart(CAL_MAG);
  else if (strncmp(p, "noise", 5) == 0)
    calStart(CAL_NOISE);
  else if (strncmp(p, "track", 5) == 0)
    calStart(CAL_TRACK);
  else if (strncmp(p, "stop", 4) == 0)
    calStop();
  else if (strncmp(p, "log", 3) == 0)
    printCalLog();
  else if (strncmp(p, "retrymag", 8) == 0)
    imu2.retryMag();
  else
    usb.send("% cal commands: mag | noise | track | stop | log | retrymag\r\n");
  return true;
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
  // Construct the AS5X47 sensors after the core init(), so SPI.begin() does
  // not run from a global constructor or inside the first control tick.
  (void)leftWheelSensor();
  (void)rightWheelSensor();
  kf.reset(0.0f);
  kfLastUs = micros();
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
      estimatorUpdate();
      if (calState > 0)
      {
        calTick();
      }
      else if (calArmed and
               (digitalReadFast(PIN_START_BUTTON) or robot.missionStart))
      {
        calBeginTrack();
        calTick();
      }
      else
      {
        sequenceTwoSteps();
        //forwardThreeSeconds();
        if (state > 0)
        {
          controlUpdate();
        }
      }
      motor.tick();
      if (logTickCnt % logDivider == 0)
        updateLog();
      logTickCnt++;
      if (calState > 0 and calTickCnt % calDivider == 0)
        updateCalLog();
      calTickCnt++;
      robot.tick();   
      display.tick(); 
    }
    usb.tick(); 
    if (logPending and state == 0 and Serial)
      printLog();
    if (calPending and calState == 0 and Serial)
      printCalLog();
  }
}
