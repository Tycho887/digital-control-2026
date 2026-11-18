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

void printLog(bool manual = false);
void clearLog();
void printStatus();

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
#include "pid.h"
#include "trajectory.h"
#include "tuning.h"

// Linker heap bounds, used by the status command.
extern "C"
{
extern char *__brkval;
extern char _heap_end;
}

/**
- Global variables */
const uint32_t sampleTimeUs = 1000; // desired sample time in us
static_assert(sampleTimeUs == 1000,
              "controller coefficients assume a 1 ms sample time");
// Robot configuration
const float gear = 9.6;
const float wheelRadius = 0.0315; // (m)
const float wheelBase = 0.14;
// pose values
float distA = 0; // trip distance
float pose[4];
float poseVel[2];
// Filter state
// IMU measurement frame -> body. Fitted from
// src/logs/device-monitor-261118-111000.log: the spin axis is
// [-0.4517, 0.0208, 0.8919] (0.08 deg from the previous fit), and row 0
// is the forward-accel regression direction (0.87 deg from the projected
// +X axis; fit slope 0.718, corr 0.758). The library negates the accel
// axes, so row 0 points opposite the geometric body forward; row 2 is
// the body +Z yaw axis. The static row-0 projection is 0.607 m/s^2 and is
// removed by the startup kfAccelBias average. A mounting change must
// refit this matrix.
constexpr float imuToBody[3][3] = {
    {-0.892062f,  0.004577f, -0.451891f},
    {-0.013500f, -0.999772f,  0.016524f},
    {-0.451712f,  0.020841f,  0.891920f},
};
// Legacy single-axis mapping, kept for the log header; the estimator uses
// imuToBody instead.
const int imuForwardAxis = 2;             // forward = -IMU Z
const float imuForwardAccelSign = -1.0f;
const int imuYawAxis = tuning::yawAxis;
const float imuYawRateSign = tuning::yawSign;
const float gravity = 9.80665f;
const float gyroToRad = PI / 180.0f;
float kfAccelBias = 0.0f;
uint32_t kfAccelCalCnt = 0;
bool kfAccelCalDone = false;
float poseHeading = 0.0f;      // dead-reckoned heading, not a filter state
float poseStartTheta = 0.0f;   // heading at start(), for run-relative pose
float poseHeadingTotal = 0.0f;      // unwrapped trapezoidal heading
float poseStartHeadingTotal = 0.0f; // unwrapped heading at start()
float posePrevV = 0.0f;        // previous v for trapezoidal dead reckoning
float posePrevOmega = 0.0f;    // previous omega for trapezoidal heading
uint32_t kfLastUs = 0;
// Outer-loop commands from the last control tick, for logging.
float controlVCommand = 0.0f;     // m/s
float controlOmegaCommand = 0.0f; // rad/s
float controlOmegaRefLimited = 0.0f;
float controlHeadingCorrection = 0.0f;
float kfForwardAccelInput = 0.0f; // m/s^2, after startup offset removal
float kfGyroYawInput = 0.0f;      // rad/s, forward KF sign convention
uint32_t kfGyroTimestampUs = 0;
uint32_t kfWheelTimestampUs = 0;
float kfWheelVInput = 0.0f;
float kfWheelOmegaInput = 0.0f;
bool kfWheelEncoderSource = false;
const bool useKfFeedback = true;  // false: outer heading/yaw feed-forward only
constexpr TrajectoryConfig closedLoopConfig{};

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
// A blocked main loop can stretch the angle window; reject such a window
// instead of converting it into a bogus velocity.
const float maxWheelWindow = 0.030f; // s
// Encoder fallback window, used while the AS5X47 reads are invalid.
bool wheelEncoderInit = false;
uint32_t wheelEncoderStartUs = 0;
// A working sensor must report motion when the encoders do. These thresholds
// catch a dead SPI bus that returns a valid all-zero angle frame.
const float as5StallSpeed = 0.002f; // m/s
const float encoderMotion = 5.0f;   // motor rad/s
// Latest AS5X47 values, exposed for the calibration capture.
bool as5Valid = false;
bool as5Fresh = false;
float as5AngleLast[2] = {0.0f, 0.0f}; // rad, after direction sign
float as5VelLast[2] = {0.0f, 0.0f};   // wheel m/s, forward-positive
float as5Window = 0.0f;               // s

struct WheelMagneticSample
{
  int16_t magnitude, angle, diaagc;  // 14-bit payloads, -1 until read or on SPI error
};
WheelMagneticSample wheelMagnetic[2] = {{-1, -1, -1}, {-1, -1, -1}};

// ////////////////////////////////////////
// define a structure for log elements
typedef struct
{
  float time;
  int state;
  float desiredValue;     // forward velocity reference (m/s)
  float desiredYawRate;   // yaw-rate reference (rad/s)
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
  float kfState[5];       // [v, omega, dead-reckoned heading, b_a, reserved]
  float vError;           // controlVCommand - KF v (m/s)
  float yawRateError;     // yaw-rate PID error (rad/s)
  float vCommand;         // outer-loop v command (m/s)
  float yawRateCommand;   // outer-loop yaw-rate command (rad/s)
  float controlFeedback[2]; // filtered PI feedback (motor rad/s)
  float as5Vel[2];        // independent wheel velocity (m/s)
  float desiredHeading;   // rad
  float headingError;     // actual heading-controller input (rad)
  float omegaRefLimited;  // limited turn reference before yaw correction (rad/s)
  float headingCorrection; // heading-controller output (rad/s)
  float kfAccelInput;     // forward acceleration used by predict (m/s^2)
  float kfGyroInput;      // yaw-rate input used by updateGyro (rad/s)
  float kfInnov[KF_MEAS_COUNT];
  float gyroRaw[3];       // imu2.gyro[], deg/s, offset-corrected
  float accRaw[3];        // imu2.acc[], g
  float totalAngle;       // rad, unwrapped trapezoidal integral since start
  float yawRateCorrection; // rad/s, yaw-rate PID output
  uint16_t kfMask;
  uint16_t diagnosticFlags;
  WheelMagneticSample wheelMagnetic[2];

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
  CAL_UPSIDE_DOWN = 4,
  CAL_YAW = 5,
  CAL_TURN_RATE = 6,
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
  CAL_MAG_ATTEMPT = 1 << 8,
  CAL_MAG_DRDY = 1 << 9,
  CAL_MAG_OVERFLOW = 1 << 10,
  CAL_GYRO_CALIBRATED = 1 << 11,
  CAL_ACCEL_CALIBRATED = 1 << 12,
  CAL_GYRO_CONFIG_VALID = 1 << 13,
};

enum KfUpdateFlag : uint16_t
{
  CAL_KF_GYRO = 1 << 0,
  CAL_KF_WHEEL_V = 1 << 1,
  CAL_KF_WHEEL_OMEGA = 1 << 2,
  CAL_KF_RESERVED3 = 1 << 3,
  CAL_KF_RESERVED4 = 1 << 4,
  CAL_KF_WHEEL_ENCODER = 1 << 5,
};

enum LogFlag : uint16_t
{
  LOG_KF_FEEDBACK = 1 << 0,
  LOG_KF_HEALTHY = 1 << 1,
  LOG_AS5_VALID = 1 << 2,
  LOG_AS5_FRESH = 1 << 3,
  LOG_WHEEL_ENCODER_FALLBACK = 1 << 4,
  LOG_LEFT_SATURATED = 1 << 5,
  LOG_RIGHT_SATURATED = 1 << 6,
  LOG_HEADING_SATURATED = 1 << 7,
  LOG_YAW_RATE_SATURATED = 1 << 8,
  LOG_GYRO_FRESH = 1 << 9,
  LOG_GYRO_OFFSET_DONE = 1 << 10,
  LOG_GYRO_CONFIG_VALID = 1 << 11,
  LOG_ACCEL_FRESH = 1 << 12,
  LOG_GYRO_SATURATED = 1 << 13,
};

typedef struct
{
  float time;
  uint16_t mode;
  uint16_t flags;
  uint16_t kfMask;
  uint16_t phase;
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
  float kfTheta;   // dead-reckoned heading, not a filter state
  float kfBA;
  float kfBG;      // reserved (0); kept for log column compatibility
  float kfInnov[KF_MEAS_COUNT];
  float kfInnovVar[KF_MEAS_COUNT];
  uint16_t magStatus;      // AK8963 ST1 (low byte) and ST2 (high byte)
  uint16_t imuDtUs;        // measured IMU interval, microseconds
  int16_t encoder[2];      // low 16 bits of the raw quadrature count
  float yawReference;
  float yawReferenceLimited;
  float yawCommand;
  float gyroInput;
  float wheelVInput;
  float wheelOmegaInput;
  uint32_t gyroTimestampUs;
  uint32_t wheelTimestampUs;
  uint32_t accelTimestampUs;
  uint16_t diagnosticFlags;
  int16_t gyroCounts[3];
  float gyroIntegral[3];
  uint32_t gyroSampleCount;
} CalData;

// Fit both straight legs, both turns and the final hold in one format-4 capture.
const int logBufferBytes = 495000; // leaves ~16 kB of the OCRAM heap free
const int logsMax = logBufferBytes / sizeof(LogData);
const int logDivider = tuning::logDivider(closedLoopConfig.duration(), logsMax);
static_assert((logsMax - 1) * sampleTimeUs * logDivider >= closedLoopConfig.duration() * 1e6f,
              "Log buffer must cover the complete sequence plus margin");
LogData * logs = (LogData*)malloc(logsMax * sizeof(LogData));
CalData * calLogs = (CalData*)logs;
const int calMax = (logsMax * sizeof(LogData)) / sizeof(CalData);
static_assert(sizeof(CalData) == 224,
              "CalData layout changed; update the printCalLog columns");
static_assert(calMax >= 1502,
              "Calibration buffer must hold 3 s noise at 500 Hz plus margin");
static_assert(tuning::logDivider(tuning::duration(tuning::turnRateCalibration), calMax) *
                  (calMax - 1) * sampleTimeUs >= 40000000,
              "Calibration buffer must cover the complete turn-rate sequence");
const float currentScale = 0.0045; // A/count; requires hardware calibration
const float currentOffset[2] = {0.260, -0.275}; // legacy Tania offsets
uint32_t captureStartUs = 0;
int logsCnt = 0; // next log entry to use / i.e. number of used log entries
uint32_t logTickCnt = 0; // control ticks since start, for logDivider
bool logPending = false; // preserve a completed capture until USB is connected
bool logTruncated = false;
uint32_t lastStartRefusalMs = 0;
// Low-rate live telemetry, written only when the USB TX buffer has room.
bool liveLogEnabled = false;
uint32_t liveLogTickCnt = 0;
bool liveLogHeaderSent = false;
const uint32_t liveLogDivider = 50; // 20 Hz at the 1 kHz control rate

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
uint16_t calPhase = 0;
turnrate::Run turnRateRun;
turnrate::ButtonLatch turnRateButton;
struct TurnRateMetadata
{
  bool readable = false, valid = false, calibrated = false;
  uint8_t config = 0, dlpf = 0, divider = 0;
  float gyroOffsets[3] = {};
  uint32_t gyroSamples = 0;
} turnRateMetadata;

// Unbuffered sensor stream for long static captures (noise, Allan, mag).
// It does not use the log buffer and is printed while the robot is idle.
bool streamActive = false;
int streamDivider = 10;
uint32_t streamStartMs = 0;
int streamTickCnt = 0;
const uint32_t streamMaxMs = 20UL * 60UL * 1000UL;

// State is also used to prevent a USB log request from blocking control.
int state = 0;
float endTime = 0;
float desiredValue = 0;    // forward velocity reference (m/s)
float desiredYawRate = 0;  // yaw-rate reference (rad/s)
float desiredHeading = 0;  // heading reference (rad)

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

AlphaBetaFilter leftVelFilter (tuning::leftVelAlpha, tuning::leftVelBeta,
                               sampleTimeUs * 1e-6f);
AlphaBetaFilter rightVelFilter(tuning::rightVelAlpha, tuning::rightVelBeta,
                               sampleTimeUs * 1e-6f);

const float controlVoltageLimit = tuning::wheelVoltageLimit;

// Runtime copies of the controller defaults in tuning.h. They stay mutable so
// `cal gains` can apply RAM-only trial coefficients; start() and calStart()
// snapshot them for the logged controller headers.
PID_Coefficients left_wheel_coeffs = tuning::leftWheelCoeffs;
PID_Coefficients right_wheel_coeffs = tuning::rightWheelCoeffs;

PIDController leftController(left_wheel_coeffs, controlVoltageLimit);
PIDController rightController(right_wheel_coeffs, controlVoltageLimit);

PID_Coefficients yaw_pid_coeffs = tuning::yawRateCoeffs;
PID_Coefficients heading_coeffs = tuning::headingCoeffs;

/**
 * Reference limiter as in the Simulink acc_limit_turn blocks:
 *   u[k] = u[k-1] + Ts * sat(Kp * (ref - u[k-1]), lower, upper)
 * A step reference therefore becomes a bounded ramp followed by a
 * first-order approach. */
class ReferenceLimiter
{
public:
  ReferenceLimiter(float gain_, float rateLower_, float rateUpper_)
      : Kp(gain_), rateLower(rateLower_), rateUpper(rateUpper_) {}

  void reset(float value = 0.0f) { y = value; }

  float update(float ref)
  {
    float rate = Kp * (ref - y);
    if (rate > rateUpper)
      rate = rateUpper;
    else if (rate < rateLower)
      rate = rateLower;
    y += rate * sampleTimeUs * 1e-6f;
    return y;
  }

private:
  float Kp;
  float rateLower;
  float rateUpper;
  float y = 0.0f;
};

ReferenceLimiter velLimiter(tuning::KpLV, tuning::Lim_VL, tuning::Lim_VU);
ReferenceLimiter turnLimiter(tuning::KpLT, tuning::Lim_TL, tuning::Lim_TU);

// The heading and yaw-rate corrections are bounded at the turn-reference
// limit, so the total turn command is the reference plus at most twice that
// limit.
PIDController headingPid(heading_coeffs, tuning::Lim_TU);
PIDController yawRatePid(yaw_pid_coeffs, tuning::Lim_TU);

struct ControllerSnapshot
{
  PID_Coefficients left, right, yaw;
};
ControllerSnapshot runControllers = {left_wheel_coeffs, right_wheel_coeffs, yaw_pid_coeffs};
ControllerSnapshot calControllers = runControllers;

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

void printPidHeader(const char *name, const PID_Coefficients &c, float limit)
{
  Serial.print("% "); Serial.print(name); Serial.print(" b0,b1,b2,a1,a2,limit");
  printLogValue(c.b0); printLogValue(c.b1); printLogValue(c.b2);
  printLogValue(c.a1); printLogValue(c.a2); printLogValue(limit);
  Serial.println();
}

void printLog(bool manual)
{
  if (state > 0)
  {
    usb.send("% Log capture in progress; request log after the run finishes.\n");
    return;
  }
  if (manual)
  {
    if (!usb.usbIsUp)
      return;
  }
  else if (!Serial)
    return;
  streamActive = false;
  if (logs == nullptr)
  {
    Serial.println("% Log buffer allocation failed; run disabled.");
    return;
  }
  LogData * d = logs;
  Serial.print("% Basebot log for ");
  Serial.println(robot.getRobotName());
  Serial.println("% log format 4 (IMU + encoder estimator, no mag fusion, no covariance)");
  Serial.print("% Sample time ");  Serial.print(sampleTimeUs);
  Serial.print(" us, log divider "); Serial.print(logDivider);
  Serial.print(", gear ");  Serial.print(gear, 7);
  Serial.print(", wheel radius ");  Serial.print(wheelRadius, 7);
  Serial.print(" m, wheel base "); Serial.print(wheelBase, 7);
  Serial.print(" m, PPR ");  Serial.println(encoder.pulsPerRev);
  Serial.print("% maximum log samples ");  Serial.print(logsMax);
  Serial.print(" each sized ");  Serial.print(sizeof(LogData)); Serial.println(" bytes");
  Serial.print("% KF heading/yaw feedback "); Serial.println(useKfFeedback ? 1 : 0);
  Serial.print("% Yaw mapping axis,sign,scale ");
  Serial.print(imuYawAxis); printLogValue(imuYawRateSign);
  printLogValue(tuning::yawScale); Serial.println();
  Serial.print("% IMU to body rows (row0=accel forward, row2=yaw):");
  for (int row = 0; row < 3; row++)
    for (int col = 0; col < 3; col++)
      printLogValue(imuToBody[row][col]);
  Serial.println();
  Serial.print("% Trajectory baseline,forward duration,turn duration,settling (s) ");
  printLogValue(closedLoopConfig.initialRest);
  printLogValue(closedLoopConfig.forwardDuration);
  printLogValue(closedLoopConfig.turnDuration);
  printLogValue(closedLoopConfig.finalRest); Serial.println();
  Serial.print("% Trajectory post-turn settle,return duration (s) ");
  printLogValue(closedLoopConfig.turnSettle);
  printLogValue(closedLoopConfig.forwardDuration); Serial.println();
  Serial.print("% Trajectory forward speed (m/s), turn rate (rad/s) ");
  printLogValue(closedLoopConfig.forwardSpeed);
  printLogValue(closedLoopConfig.turnRate); Serial.println();
  Serial.print("% Final turn duration (s), mean rate (rad/s), pre-turn stop (s)");
  printLogValue(closedLoopConfig.turnDuration);
  printLogValue(-closedLoopConfig.turnRate);
  printLogValue(closedLoopConfig.preTurnSettle); Serial.println();
  printPidHeader("Wheel PI L", runControllers.left, controlVoltageLimit);
  printPidHeader("Wheel PI R", runControllers.right, controlVoltageLimit);
  printPidHeader("Heading", heading_coeffs, tuning::Lim_TU);
  printPidHeader("Yaw rate", runControllers.yaw, tuning::Lim_TU);
  Serial.print("% Reference limiters KpLV,KpLT,Lim_VL,Lim_VU,Lim_TL,Lim_TU");
  printLogValue(tuning::KpLV); printLogValue(tuning::KpLT);
  printLogValue(tuning::Lim_VL); printLogValue(tuning::Lim_VU);
  printLogValue(tuning::Lim_TL); printLogValue(tuning::Lim_TU);
  Serial.println();
  if (logTruncated)
    Serial.println("% WARNING: log buffer filled; final samples are missing.");
  // Columns
  Serial.println("% 1 time (sec)");
  Serial.println("% 2 state");
  Serial.println("% 3   forward velocity reference (m/s)");
  Serial.println("% 4-5 commanded motor voltage (left,right) (V, wheel PI output; not measured terminal voltage)");
  Serial.println("% 6-7 Encoder count (left, right)");
  Serial.println("% 8-9 motor vel (left, right) (rad/s)");
  Serial.println("% 10  Battery voltage (V)");
  Serial.println("% 11-13 pose (x,y,theta) (m, m, rad)");
  Serial.println("% 14 total distance (m, trapezoidal)");
  Serial.println("% 15-16 legacy calibrated current (left, right) (A), forward-positive");
  Serial.println("% 17 capture timestamp (us since start), ADC read follows actuator update");
  Serial.println("% 18-19 raw current ADC counts (left A1, right A0), 12 bit");
  Serial.println("% 20-21 signed PWM request (left,right), forward-positive, full scale 4096");
  Serial.println("% 22 driver flags: bits 0/1 enable L/R, bits 2/3 sleeping L/R");
  Serial.println("% 23-24 encoder estimate delay (left,right) (s)");
  Serial.println("% 25-26 wheel PI outputs (left,right) (V)");
  Serial.println("% 27-28 estimator velocity and yaw rate (m/s, rad/s)");
  Serial.println("% 29 magnetometer heading (rad, diagnostic only, NaN if unavailable)");
  Serial.println("% 30-31 controller setpoint (left,right) (motor rad/s)");
  Serial.println("% 32 yaw-rate reference (rad/s)");
  Serial.println("% 33-37 KF state [v, omega, wrapped dead-reckoned heading, b_a, reserved]");
  Serial.println("%       (m/s, rad/s, rad, m/s^2, 0)");
  Serial.println("% 38 forward-velocity control error (vCmd - v) (m/s)");
  Serial.println("% 39 yaw-rate PID error (omegaRefLim - omega) (rad/s)");
  Serial.println("% 40 outer-loop forward-velocity command (m/s)");
  Serial.println("% 41 outer-loop yaw-rate command (rad/s)");
  Serial.println("% 42-43 filtered wheel PI feedback (left,right) (motor rad/s)");
  Serial.println("% 44-45 AS5X47 wheel velocity (left,right) (m/s, forward-positive)");
  Serial.println("% 46 desired heading (rad)");
  Serial.println("% 47 actual heading-controller error (rad)");
  Serial.println("% 48 limited turn reference before yaw correction (rad/s)");
  Serial.println("% 49 heading-controller correction (rad/s)");
  Serial.println("% 50 forward acceleration used by KF predict (m/s^2)");
  Serial.println("% 51 yaw rate used by KF gyro update (rad/s)");
  Serial.println("% 52-55 KF innovation (gyro, wheel v, wheel omega, reserved)");
  Serial.println("% 56 KF update mask (same bits as calibration column 4)");
  Serial.println("% 57 diagnostic flags: bit0 KF feedback, bit1 KF healthy, bit2 AS5 valid, bit3 AS5 fresh, bit4 encoder fallback, bits5/6 wheel L/R saturation, bit7 heading saturation, bit8 yaw-rate saturation, bit9 gyro fresh, bit10 gyro offset done, bit11 gyro config valid, bit12 accel fresh, bit13 gyro saturated");
  Serial.println("% 58-60 gyro raw (x,y,z) (deg/s, offset-corrected)");
  Serial.println("% 61-63 accel raw (x,y,z) (g)");
  Serial.println("% 64 total angle (rad, unwrapped trapezoidal since start)");
  Serial.println("% 65 yaw-rate PID output (rad/s)");
  Serial.println("% 66-67 AS5047P/AS5147 MAG L/R (14-bit field magnitude counts, CS 24/25)");
  Serial.println("% 68-69 AS5047P/AS5147 ANGLECOM L/R (native motor-shaft counts, 16384/rev)");
  Serial.println("% 70-71 AS5047P/AS5147 DIAAGC L/R (bits 7:0 AGC, 8 LF, 9 COF, 10 MAGH, 11 MAGL)");
  Serial.println("% 66-71: -1 if unread or SPI error/parity failure; MAG and DIAAGC sampled every 16 ticks, angle every tick.");
  Serial.println("% Static gains: no covariance is propagated or logged.");
  
  Serial.print("% Current A/count "); Serial.print(currentScale,7);
  Serial.print(" offsets L/R "); Serial.print(currentOffset[0],6);
  Serial.print(" "); Serial.println(currentOffset[1],6);
  Serial.print("% PWM frequency Hz "); Serial.println(motor.PWMfrq);
  Serial.println("% Voltage columns 4-5 are requests, NOT measured terminal voltages.");
  Serial.println("% Driver assumes supply loss 1 V, adds +/-0.4 V; battery <5.5 V uses 11.1 V fallback.");
  Serial.println("% PID map: wheel L/R setpoint 30/31 - feedback 42/43 -> output 25/26;");
  Serial.println("% yaw rate error 39 (ref 48, feedback 34) -> output 65; heading error 47 (ref 46, feedback 35) -> output 49.");
  
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
    printLogValue(d->desiredYawRate); // 32
    for (int k = 0; k < 5; k++) printLogValue(d->kfState[k]); // 33-37
    printLogValue(d->vError); // 38
    printLogValue(d->yawRateError); // 39
    printLogValue(d->vCommand); // 40
    printLogValue(d->yawRateCommand); // 41
    for (int k = 0; k < 2; k++) printLogValue(d->controlFeedback[k]); // 42-43
    for (int k = 0; k < 2; k++) printLogValue(d->as5Vel[k]); // 44-45
    printLogValue(d->desiredHeading); // 46
    printLogValue(d->headingError); // 47
    printLogValue(d->omegaRefLimited); // 48
    printLogValue(d->headingCorrection); // 49
    printLogValue(d->kfAccelInput); // 50
    printLogValue(d->kfGyroInput); // 51
    for (int k = 0; k < KF_MEAS_COUNT; k++) printLogValue(d->kfInnov[k]); // 52-55
    Serial.print(" "); Serial.print(d->kfMask); // 56
    Serial.print(" "); Serial.print(d->diagnosticFlags); // 57
    for (int k = 0; k < 3; k++) printLogValue(d->gyroRaw[k]); // 58-60
    for (int k = 0; k < 3; k++) printLogValue(d->accRaw[k]);  // 61-63
    printLogValue(d->totalAngle); // 64
    printLogValue(d->yawRateCorrection); // 65
    for (const auto &wheel : d->wheelMagnetic) printLogValue(wheel.magnitude); // 66-67
    for (const auto &wheel : d->wheelMagnetic) printLogValue(wheel.angle);     // 68-69
    for (const auto &wheel : d->wheelMagnetic) printLogValue(wheel.diaagc);    // 70-71
    Serial.println("");
    d++;
  }
  Serial.print("% sent ");
  Serial.print(logsCnt);
  Serial.println(" log lines");
  logPending = false;
  display.setLine("");
}

void clearLog()
{
  if (state > 0)
  {
    usb.send("% log clear refused: run active\r\n");
    return;
  }
  logPending = false;
  logsCnt = 0;
  logTruncated = false;
  display.setLine("");
  usb.send("% log capture cleared\r\n");
}

void printStatus()
{
  char msg[256];
  unsigned long heapFree = 0;
  if (__brkval != nullptr)
    heapFree = (unsigned long)((char *)&_heap_end - __brkval);
  int n = snprintf(msg, sizeof(msg),
      "%% status state=%d calState=%d logs=%d/%d logPending=%d calPending=%d "
      "usb=%d dtr=%d heap=%lu truncated=%d live=%d\r\n",
      state, calState, logsCnt, logsMax,
      logPending ? 1 : 0, calPending ? 1 : 0,
      usb.usbIsUp ? 1 : 0, Serial ? 1 : 0, heapFree,
      logTruncated ? 1 : 0, liveLogEnabled ? 1 : 0);
  if (n > 0)
    usb.send(msg);
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
  if (logPending)
  {
    uint32_t now = millis();
    if (usb.usbIsUp and now - lastStartRefusalMs > 500)
    {
      usb.send("% start blocked: previous log pending; open the monitor, send 'log' or 'log clear'\r\n");
      lastStartRefusalMs = now;
    }
    return false;
  }
  if (calPending)
  {
    uint32_t now = millis();
    if (usb.usbIsUp and now - lastStartRefusalMs > 500)
    {
      usb.send("% start blocked: calibration capture pending; send 'cal log' or 'cal clear'\r\n");
      lastStartRefusalMs = now;
    }
    return false;
  }
  logTruncated = false;
  // The standard log shares the buffer with calibration captures.
  calArmed = false;
  calMode = CAL_NONE;
  logsCnt = 0;
  logTickCnt = 0;
  liveLogEnabled = true;
  liveLogTickCnt = 0;
  liveLogHeaderSent = false;
  controlSetpoint[0] = 0.0f;
  controlSetpoint[1] = 0.0f;
  controlFeedback[0] = 0.0f;
  controlFeedback[1] = 0.0f;
  computedControl[0] = 0.0f;
  computedControl[1] = 0.0f;
  captureStartUs = micros();
  runControllers = {left_wheel_coeffs, right_wheel_coeffs, yaw_pid_coeffs};
  motor.setPWMfrq(80000);
  time_sec = 0;
  encoder.encoder[0] = 0; 
  encoder.encoder[1] = 0; 
  // Restart the wheel-speed windows so the run begins from a clean seed.
  wheelOdoInit = false;
  wheelOdoAccum[0] = 0.0f;
  wheelOdoAccum[1] = 0.0f;
  wheelEncoderInit = false;
  for (auto &wheel : wheelMagnetic)
  {
    wheel.magnitude = -1;
    wheel.diaagc = -1;
  }
  display.useDisplay = false;
  
  // Reset controllers and velocity filters for a clean run. Without the
  // filter resets, the first samples inherit the previous run's velocity.
  leftController.reset();
  rightController.reset();
  leftVelFilter.reset();
  rightVelFilter.reset();
  headingPid.reset();
  yawRatePid.reset();
  velLimiter.reset();
  turnLimiter.reset();
  controlVCommand = 0.0f;
  controlOmegaCommand = 0.0f;
  controlOmegaRefLimited = 0.0f;
  controlHeadingCorrection = 0.0f;
  kfForwardAccelInput = 0.0f;
  kfGyroYawInput = 0.0f;
  desiredYawRate = 0.0f;
  // A run starts with clean motion states and a clean residual accelerometer
  // bias. The stationary startup average (kfAccelBias) is measured at
  // power-on and kept so the offset is not re-learned as b_a. The
  // dead-reckoned heading stays.
  estimator.v = 0.0f;
  estimator.omega = 0.0f;
  estimator.b_a = 0.0f;
  // Capture the pose origin and hold the current heading.
  desiredHeading = poseHeading;
  poseStartTheta = poseHeading;
  poseStartHeadingTotal = poseHeadingTotal;
  posePrevV = estimator.v;
  posePrevOmega = estimator.omega;
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
  desiredYawRate = 0;
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
  liveLogEnabled = false;
  state = 0;
  logPending = logsCnt > 0;
  if (logPending)
  {
    display.setLine("log pending");
    char msg[96];
    snprintf(msg, sizeof(msg),
             "%% run complete: %d samples pending; open the monitor or send 'log'\r\n",
             logsCnt);
    usb.send(msg);
  }
  else
    display.setLine("");
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
- Yaw-rate step: 0.05 m/s forward, then a 0.2 rad/s turn for 0.5 s.
- Not the default sequence; call it from loop() to exercise the turn-rate
  and heading loops. */
void yawRateStep()
{
  bool button;
  switch (state)
  {
    case 0:
      button = digitalReadFast(PIN_START_BUTTON);
      if (button or robot.missionStart)
      {
        if (!start()) break;
        desiredValue = 0.05f;
        desiredYawRate = 0.0f;
        endTime = time_sec + 0.5f; // settle the forward loop first
        state = 10;
      }
      break;
    case 10:
      if (time_sec > endTime)
      {
        desiredYawRate = 0.2f;
        endTime = time_sec + 0.5f; // 0.5 s yaw-rate step
        state = 11;
      }
      break;
    case 11:
      // Move the heading reference with the commanded turn so the heading
      // loop does not oppose the yaw-rate step.
      desiredHeading = wrapAngle(desiredHeading +
                                 desiredYawRate * sampleTimeUs * 1e-6f);
      if (time_sec > endTime)
      {
        desiredValue = 0;
        desiredYawRate = 0;
        stop(0.3f);
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

bool gyroFeedbackReady()
{
  // Use the same 20 ms sample-age limit as the yaw calibration experiment.
  return imu2.gyroConfigurationValid && imu2.gyroOffsetDone &&
         estimator.lastInnovationValid[KF_M_GYRO] &&
         micros() - kfGyroTimestampUs <= 20000u;
}

void closedLoopTrajectory()
{
  // Consume a button press or USB start request even when the gyro is unavailable.
  static bool startButtonLatched = false;
  const bool button = digitalReadFast(PIN_START_BUTTON) or robot.missionStart;
  const bool newPress = button and !startButtonLatched;
  startButtonLatched = button;
  if (state == 0 && !newPress) return;

  if (useKfFeedback && !gyroFeedbackReady())
  {
    if (state != 0) finished();  // zero PWM and preserve the captured rows
    usb.send("% trajectory disabled: gyro feedback unavailable; check 'cal imu', then press start again\r\n");
    return;
  }

  if (state == 0)
  {
    if (!start()) return;
    state = 10;
    desiredValue = 0.0f;
    desiredYawRate = 0.0f;
    endTime = closedLoopConfig.duration();
    return;
  }

  const float elapsed = (micros() - captureStartUs) * 1e-6f;
  TrajectoryCommand command = trajectoryCommand(closedLoopConfig, elapsed);
  if (command.phase == 0)
  {
    finished();
    return;
  }

  state = command.phase;
  desiredValue = command.velocity;
  desiredYawRate = command.yawRate;
  if (state == 2)
    desiredHeading = poseStartTheta;
  else if (desiredYawRate != 0.0f)
  {
    desiredHeading = wrapAngle(desiredHeading +
                               desiredYawRate * sampleTimeUs * 1e-6f);
  }
}

/**
- Wheel-speed PI on the quadrature encoders.
- Setpoints are motor-shaft rad/s, forward-positive. The alpha-beta filtered
  encoder velocity is recorded as controlFeedback so the tracking experiment
  can compare the controller's own signal with AS5X47/estimator velocity.
 */
void updateWheelFeedback()
{
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
}

void wheelSpeedControl(float leftSetpoint, float rightSetpoint)
{
  controlSetpoint[0] = leftSetpoint;
  controlSetpoint[1] = rightSetpoint;
  updateWheelFeedback();

  computedControl[0] = leftController.ComputeCCDE(leftSetpoint - controlFeedback[0]);
  computedControl[1] = rightController.ComputeCCDE(rightSetpoint - controlFeedback[1]);

  motor.motorVoltage[0] = -computedControl[0];
  motor.motorVoltage[1] =  computedControl[1];
}

/**
 * Week10 reference-limited cascade:
 *   vCmd     = velLimiter(vRef)
 *   omegaRef = turnLimiter(desiredYawRate + heading PID(wrapped heading error))
 *   omegaCmd = omegaRef + yaw-rate PID(omegaRef - selected yaw feedback)
 * The per-wheel PI is the forward-rate controller, so vCmd stays vRef
 * (after the velocity limiter). useKfFeedback selects estimator feedback;
 * when false, heading/yaw errors are zero and only feed-forward remains.
 * headingRef is in rad; vRef and omegaRef in m/s and rad/s. */
void trackingControlUpdate(float vRef, float omegaRef, float headingRef,
                           bool enableHeading = true, bool useGyroYaw = false)
{
  desiredValue = vRef;
  desiredYawRate = omegaRef;

  const float vCmd = velLimiter.update(vRef);
  const float headingFeedback = useKfFeedback ? poseHeading : headingRef;
  controlHeadingCorrection = enableHeading ?
      headingPid.ComputeCCDE(wrapAngle(headingRef - headingFeedback)) : 0.0f;
  controlOmegaRefLimited = turnLimiter.update(omegaRef + controlHeadingCorrection);
  const float omegaFeedback = useGyroYaw ? kfGyroYawInput :
      (useKfFeedback ? estimator.omega : controlOmegaRefLimited);
  const float omegaCmd = controlOmegaRefLimited +
      yawRatePid.ComputeCCDE(controlOmegaRefLimited - omegaFeedback);

  controlVCommand = vCmd;
  controlOmegaCommand = omegaCmd;

  // Convert polar (v, omega) to left/right wheel linear speeds (m/s),
  // then scale to motor-shaft rad/s for the 1 kHz inner wheel PI.
  WheelSpeeds ws = polar2Cartesian(vCmd, omegaCmd);
  const float mpsToMotorRadSec = gear / wheelRadius;
  wheelSpeedControl(ws.left * mpsToMotorRadSec, ws.right * mpsToMotorRadSec);
}

void controlUpdate()
{
  trackingControlUpdate(desiredValue, desiredYawRate, desiredHeading);
}

// The AS5047P and AS5147 both use 0x3FFC for DIAAGC.
// Stagger these read-only diagnostics after actuation, one SPI read per selected tick.
void sampleWheelMagnetics()
{
  constexpr uint16_t diaagcRegister = 0x3FFC;
  switch (logTickCnt % 16)
  {
    case 0:  wheelMagnetic[0].magnitude = leftWheelSensor().readCheckedRegister(MAG_REG); break;
    case 4:  wheelMagnetic[1].magnitude = rightWheelSensor().readCheckedRegister(MAG_REG); break;
    case 8:  wheelMagnetic[0].diaagc = leftWheelSensor().readCheckedRegister(diaagcRegister); break;
    case 12: wheelMagnetic[1].diaagc = rightWheelSensor().readCheckedRegister(diaagcRegister); break;
  }
}

/**
- Feed one wheel-velocity pair (m/s, forward-positive) to the estimator. */
void applyWheelMeasurements(float vLeft, float vRight)
{
  kfWheelVInput = 0.5f * (vLeft + vRight);
  kfWheelOmegaInput = (vRight - vLeft) / wheelBase;
  kfWheelTimestampUs = micros();
  kfWheelEncoderSource = false;
  estimator.updateWheelV(kfWheelVInput);
  estimator.updateWheelOmega(kfWheelOmegaInput);
  if (estimator.lastInnovationValid[KF_M_WHEEL_V])
    kfUpdateMask |= CAL_KF_WHEEL_V;
  if (estimator.lastInnovationValid[KF_M_WHEEL_OMEGA])
    kfUpdateMask |= CAL_KF_WHEEL_OMEGA;
}

/**
- AS5X47 wheel speed, 1 kHz reads over a 10 ms window.
- Both absolute angles are read every tick and their unwrapped differences
  accumulated. Every KF_WHEEL_WINDOW the accumulators are converted to wheel
  speed and fed to the filter as v and omega. Windows stretched beyond
  30 ms by a blocked loop are discarded.
- When the AS5X47 read is invalid, the smoothed quadrature controlFeedback
  (motor rad/s) provides v and omega at the same 10 ms rate, flagged with
  CAL_KF_WHEEL_ENCODER in the calibration mask.
- The sensors sit on the motor shafts, so wheel angle = sensor angle / gear.
- AS5X47::readVel() is deliberately not used: its 24.141 deg/s LSB is
  unusable at the robot's low speeds. */
void wheelOdometryUpdate()
{
  as5Fresh = false;
  const uint32_t nowUs = micros();
  // controlFeedback is only refreshed while a wheel-speed loop runs.
  const bool wheelLoopActive =
      state > 0 or (calState > 0 and
                    (calMode == CAL_TRACK or calMode == CAL_UPSIDE_DOWN or
                     calMode == CAL_YAW or calMode == CAL_TURN_RATE));
  wheelMagnetic[0].angle = leftWheelSensor().readCheckedRegister(ANGLECOM_REG);
  wheelMagnetic[1].angle = rightWheelSensor().readCheckedRegister(ANGLECOM_REG);
  if (wheelMagnetic[0].angle < 0 or wheelMagnetic[1].angle < 0)
  {
    as5Valid = false;
    if (!wheelLoopActive)
    {
      wheelEncoderInit = false;
      return;
    }
    if (!wheelEncoderInit)
    {
      wheelEncoderStartUs = nowUs;
      wheelEncoderInit = true;
      return;
    }
    float window = (nowUs - wheelEncoderStartUs) * 1e-6f;
    if (window < KF_WHEEL_WINDOW)
      return;
    wheelEncoderStartUs = nowUs;
    if (window > maxWheelWindow)
      return;
    float vLeft = controlFeedback[0] * wheelRadius / gear;
    float vRight = controlFeedback[1] * wheelRadius / gear;
    applyWheelMeasurements(vLeft, vRight);
    kfWheelEncoderSource = true;
    kfUpdateMask |= CAL_KF_WHEEL_ENCODER;
    return;
  }
  wheelEncoderInit = false;
  const float radiansPerCount = 2.0f * PI / 16384.0f;
  const float angleLeft = AS5X47_LEFT_DIR * wheelMagnetic[0].angle * radiansPerCount;
  const float angleRight = AS5X47_RIGHT_DIR * wheelMagnetic[1].angle * radiansPerCount;
  as5Valid = true;
  as5AngleLast[0] = angleLeft;
  as5AngleLast[1] = angleRight;

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
  if (window > maxWheelWindow)
  { // a blocked loop makes the averaged window unusable; restart it
    wheelOdoAccum[0] = 0.0f;
    wheelOdoAccum[1] = 0.0f;
    wheelOdoStartUs = nowUs;
    return;
  }
  if (window < KF_WHEEL_WINDOW)
    return;

  float wheelAngleLeft = wheelOdoAccum[0] / gear;
  float wheelAngleRight = wheelOdoAccum[1] / gear;
  float vLeft = wheelRadius * wheelAngleLeft / window;
  float vRight = wheelRadius * wheelAngleRight / window;

  // A dead SPI bus returns a valid all-zero frame: the angle never changes,
  // so the AS5X47 speed is zero while the encoders show motion. Fall back to
  // the encoder velocity instead of feeding the filter a silent zero.
  const bool as5Stalled =
      (fabsf(vLeft) < as5StallSpeed and fabsf(controlFeedback[0]) > encoderMotion) or
      (fabsf(vRight) < as5StallSpeed and fabsf(controlFeedback[1]) > encoderMotion);
  if (wheelLoopActive and as5Stalled)
  {
    as5Valid = false;
    applyWheelMeasurements(controlFeedback[0] * wheelRadius / gear,
                           controlFeedback[1] * wheelRadius / gear);
    kfWheelEncoderSource = true;
    kfUpdateMask |= CAL_KF_WHEEL_ENCODER;
    wheelOdoAccum[0] = 0.0f;
    wheelOdoAccum[1] = 0.0f;
    wheelOdoStartUs = nowUs;
    return;
  }

  applyWheelMeasurements(vLeft, vRight);

  as5VelLast[0] = vLeft;
  as5VelLast[1] = vRight;
  as5Window = window;
  as5Fresh = true;

  wheelOdoAccum[0] = 0.0f;
  wheelOdoAccum[1] = 0.0f;
  wheelOdoStartUs = nowUs;
}

/**
- Stationary estimator update, 1 kHz */
void estimatorUpdate()
{
  uint32_t nowUs = micros();
  float dt = (nowUs - kfLastUs) * 1e-6f;
  kfLastUs = nowUs;
  kfUpdateMask = 0;
  if (dt <= 0.0f || dt > 0.020f)
  {
    for (auto &wheel : wheelMagnetic) wheel.angle = -1;
    as5Fresh = false;
    as5Valid = false;
    posePrevV = estimator.v;
    posePrevOmega = estimator.omega;
    return;
  }
  float aRaw = (imuToBody[0][0] * imu2.acc[0] +
                imuToBody[0][1] * imu2.acc[1] +
                imuToBody[0][2] * imu2.acc[2]) * gravity;
  // Average every fresh IMU read at power-on. The driver's data-ready flag can
  // stay false even while accelUpdate() returns new samples, so use the
  // timestamp rather than imu2.accelFresh.
  static uint32_t lastAccelUs = 0;
  const bool accelNew = imu2.accelTimestampUs != lastAccelUs;
  lastAccelUs = imu2.accelTimestampUs;
  if (!kfAccelCalDone && accelNew &&
      !(calState > 0 && calMode == CAL_TURN_RATE))
  {
    kfAccelCalCnt++;
    kfAccelBias += (aRaw - kfAccelBias) / kfAccelCalCnt;
    if (kfAccelCalCnt >= 2000) kfAccelCalDone = true;
  }
  // The startup average removes the bulk of the accelerometer offset; the
  // filter's b_a tracks the residual.
  kfForwardAccelInput = kfAccelCalDone ? aRaw - kfAccelBias : 0.0f;
  estimator.predict(kfForwardAccelInput, dt);
  if (imu2.gyroFresh && imu2.gyroOffsetDone && imu2.gyroConfigurationValid)
  {
    kfGyroYawInput = (imuToBody[2][0] * imu2.gyro[0] +
                      imuToBody[2][1] * imu2.gyro[1] +
                      imuToBody[2][2] * imu2.gyro[2]) * gyroToRad;
    estimator.updateGyro(kfGyroYawInput);
    if (estimator.lastInnovationValid[KF_M_GYRO])
    {
      kfGyroTimestampUs = imu2.gyroTimestampUs;
      kfUpdateMask |= CAL_KF_GYRO;
    }
  }
  // Wheel speed comes from the AS5X47 absolute angles; the smoothed
  // quadrature feedback is the flagged fallback when the read fails.
  wheelOdometryUpdate();

  // External dead reckoning: heading and position are deliberately outside
  // the filter. Both use the trapezoidal rule over the filter's v and omega.
  const float vNow = estimator.v;
  const float omegaNow = estimator.omega;
  const float ds = 0.5f * (posePrevV + vNow) * dt;
  const float dtheta = 0.5f * (posePrevOmega + omegaNow) * dt;
  const float thetaMid = wrapAngle(poseHeading + 0.5f * dtheta);
  pose[0] += ds * cosf(thetaMid);
  pose[1] += ds * sinf(thetaMid);
  distA += ds;
  poseHeadingTotal += dtheta;
  poseHeading = wrapAngle(poseHeading + dtheta);
  pose[2] = wrapAngle(poseHeading - poseStartTheta);
  pose[3] = imu2.magHeadingValid ? imu2.magHeading : NAN;
  poseVel[0] = vNow;
  poseVel[1] = omegaNow;
  posePrevV = vNow;
  posePrevOmega = omegaNow;
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
    logs[logsCnt].desiredYawRate = desiredYawRate;
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
    logs[logsCnt].kfState[0] = estimator.v;
    logs[logsCnt].kfState[1] = estimator.omega;
    logs[logsCnt].kfState[2] = poseHeading;
    logs[logsCnt].kfState[3] = estimator.b_a;
    logs[logsCnt].kfState[4] = 0.0f; // reserved (former gyro bias)
    logs[logsCnt].vError = controlVCommand - estimator.v;
    logs[logsCnt].yawRateError = yawRatePid.lastError();
    logs[logsCnt].vCommand = controlVCommand;
    logs[logsCnt].yawRateCommand = controlOmegaCommand;
    for (int k = 0; k < 2; k++)
    {
      logs[logsCnt].controlFeedback[k] = controlFeedback[k];
      logs[logsCnt].as5Vel[k] = as5VelLast[k];
      logs[logsCnt].wheelMagnetic[k] = wheelMagnetic[k];
    }
    logs[logsCnt].desiredHeading = desiredHeading;
    logs[logsCnt].headingError = headingPid.lastError();
    logs[logsCnt].omegaRefLimited = controlOmegaRefLimited;
    logs[logsCnt].headingCorrection = controlHeadingCorrection;
    logs[logsCnt].totalAngle = poseHeadingTotal - poseStartHeadingTotal;
    logs[logsCnt].yawRateCorrection = yawRatePid.lastOutput();
    logs[logsCnt].kfAccelInput = kfForwardAccelInput;
    logs[logsCnt].kfGyroInput = kfGyroYawInput;
    for (int k = 0; k < KF_MEAS_COUNT; k++)
      logs[logsCnt].kfInnov[k] = estimator.lastInnovation[k];
    for (int k = 0; k < 3; k++)
    {
      logs[logsCnt].gyroRaw[k] = imu2.gyro[k];
      logs[logsCnt].accRaw[k] = imu2.acc[k];
    }
    logs[logsCnt].kfMask = kfUpdateMask;
    uint16_t diagnosticFlags = 0;
    if (useKfFeedback) diagnosticFlags |= LOG_KF_FEEDBACK;
    if (estimator.healthy) diagnosticFlags |= LOG_KF_HEALTHY;
    if (as5Valid) diagnosticFlags |= LOG_AS5_VALID;
    if (as5Fresh) diagnosticFlags |= LOG_AS5_FRESH;
    if (kfUpdateMask & CAL_KF_WHEEL_ENCODER)
      diagnosticFlags |= LOG_WHEEL_ENCODER_FALLBACK;
    if (fabsf(computedControl[0]) >= 0.99f * controlVoltageLimit)
      diagnosticFlags |= LOG_LEFT_SATURATED;
    if (fabsf(computedControl[1]) >= 0.99f * controlVoltageLimit)
      diagnosticFlags |= LOG_RIGHT_SATURATED;
    if (fabsf(headingPid.lastOutput()) >= 0.99f * tuning::Lim_TU)
      diagnosticFlags |= LOG_HEADING_SATURATED;
    if (fabsf(yawRatePid.lastOutput()) >= 0.99f * tuning::Lim_TU)
      diagnosticFlags |= LOG_YAW_RATE_SATURATED;
    if (imu2.gyroFresh) diagnosticFlags |= LOG_GYRO_FRESH;
    if (imu2.gyroOffsetDone) diagnosticFlags |= LOG_GYRO_OFFSET_DONE;
    if (imu2.gyroConfigurationValid) diagnosticFlags |= LOG_GYRO_CONFIG_VALID;
    if (imu2.accelFresh) diagnosticFlags |= LOG_ACCEL_FRESH;
    for (int k = 0; k < 3; k++)
      if (fabsf(imu2.gyro[k]) >= 0.99f * 1000.0f)
        diagnosticFlags |= LOG_GYRO_SATURATED;
    logs[logsCnt].diagnosticFlags = diagnosticFlags;

    logsCnt++;
  }
}

// Low-rate telemetry for the monitor. It is deliberately a comment line, so
// the format-4 analyzer ignores it, and it never blocks the control loop: the
// row is dropped when the USB transmit buffer does not have room.
void sendLiveLog()
{
  if (!usb.usbIsUp)
    return;
  char buf[192];
  int n = snprintf(buf, sizeof(buf),
      "%% live %.4f %d %.4f %.4f %.4f %.4f %.4f %.1f %.1f %.4f %.4f %u\r\n",
      time_sec, state, estimator.v, estimator.omega, poseHeading,
      controlVCommand, controlOmegaCommand,
      controlSetpoint[0], controlSetpoint[1],
      headingPid.lastError(), yawRatePid.lastError(),
      (unsigned)kfUpdateMask);
  if (n <= 0)
    return;
  if (n >= (int)sizeof(buf))
    n = sizeof(buf) - 1;
  const char *hdr =
      "% live format 1: time state v omega heading vCmd omegaCmd "
      "setL setR headingErr yawErr kfMask\r\n";
  int hn = liveLogHeaderSent ? 0 : (int)strlen(hdr);
  if (Serial.availableForWrite() < hn + n)
    return;
  if (hn > 0)
  {
    Serial.write(hdr, hn);
    liveLogHeaderSent = true;
  }
  Serial.write(buf, n);
}

// ////////////////////////////////////////
// Calibration experiments

const int calTrackSteps = sizeof(tuning::track) / sizeof(tuning::track[0]);
const int calYawSteps = sizeof(tuning::yaw) / sizeof(tuning::yaw[0]);

const float calUpsideDuration[] = {0.5f, 2.5f, 2.5f, 0.5f};
const int calUpsideSteps = sizeof(calUpsideDuration) / sizeof(calUpsideDuration[0]);
const float calUpsideSpeed = 0.20f;
const float calUpsideYawRate = 30.0f * PI / 180.0f;
const float calNoiseSettleTime = 0.5f;

const char * calModeName(CalMode mode)
{
  switch (mode)
  {
    case CAL_MAG: return "mag";
    case CAL_NOISE: return "noise";
    case CAL_TRACK: return "track";
    case CAL_UPSIDE_DOWN: return "upside";
    case CAL_YAW: return "yaw";
    case CAL_TURN_RATE: return "turn-rate-calibration";
    default: return "none";
  }
}

void resetWheelControl()
{
  leftController.reset();
  rightController.reset();
  leftVelFilter.reset(-encoder.encoder[0]);
  rightVelFilter.reset(encoder.encoder[1]);
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
  streamActive = false;
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
  if (logPending)
  {
    usb.send("% calibration blocked: standard log pending; send 'log' or 'log clear' first\r\n");
    return false;
  }
  if (calPending)
  {
    usb.send("% calibration blocked: previous capture pending; send 'cal log' or 'cal clear' first\r\n");
    return false;
  }
  if (mode == CAL_YAW && !gyroFeedbackReady())
  {
    usb.send("% cal yaw refused: gyro configuration/calibration/freshness unavailable\r\n");
    return false;
  }
  liveLogEnabled = false;
  calTruncated = false;
  calMode = mode;
  calControllers = {left_wheel_coeffs, right_wheel_coeffs, yaw_pid_coeffs};
  calCnt = 0;
  calTickCnt = 0;
  calStartUs = micros();
  calDivider = (mode == CAL_MAG) ?
      tuning::logDivider(15.0f, calMax) : 2;
  motor.motorVoltage[0] = 0.0f;
  motor.motorVoltage[1] = 0.0f;
  motor.tick();
  resetWheelControl();

  if (mode == CAL_TRACK or mode == CAL_UPSIDE_DOWN or mode == CAL_YAW or
      mode == CAL_TURN_RATE)
  {
    calState = 0;
    calArmed = true;
    calPhase = 0;
    if (mode == CAL_TRACK)
      usb.send("% cal track armed: clear the floor, then press start\r\n");
    else if (mode == CAL_YAW)
      usb.send("% cal yaw armed: calibrated gyro required, heading disabled; press start\r\n");
    else if (mode == CAL_TURN_RATE)
      usb.send("% Turn-rate calibration sequence: keep still for 4 s, then automatic turns\r\n");
    else
      usb.send("% cal upside armed: secure the robot upside down with wheels clear, then press start\r\n");
    return true;
  }
  calArmed = false;
  calState = 1;
  calEndTime = time_sec + ((mode == CAL_MAG) ? 15.0f : 3.0f);
  if (mode == CAL_MAG)
  {
    imu2.magProbe();
    usb.send("% cal mag: motors off. Rotate 360 deg by hand and back within 15 s.\r\n");
  }
  else
    usb.send("% cal noise: motors off. Keep the robot still for 3 s.\r\n");
  return true;
}

void calBeginMotion()
{
  calArmed = false;
  calState = 1;
  calCnt = 0;
  calTickCnt = 0;
  calDivider = 2;
  calStartUs = micros();
  calTrackIndex = 0;
  calPhase = 10;
  motor.setPWMfrq(80000);
  float total = calMode == CAL_TURN_RATE ? tuning::duration(tuning::turnRateCalibration) :
      calMode == CAL_YAW ? tuning::duration(tuning::yaw) :
      calMode == CAL_TRACK ? tuning::duration(tuning::track) : 6.0f;
  const float firstDuration = calMode == CAL_TURN_RATE ? tuning::turnRateCalibration[0].duration :
      calMode == CAL_YAW ? tuning::yaw[0].duration :
      calMode == CAL_TRACK ? tuning::track[0].duration : calUpsideDuration[0];
  calTrackStepEnd = time_sec + firstDuration;
  calDivider = tuning::logDivider(total, calMax);
  calEndTime = time_sec + total;
  resetWheelControl();
  headingPid.reset();
  yawRatePid.reset();
  velLimiter.reset();
  turnLimiter.reset();
  desiredYawRate = 0.0f;
  controlVCommand = controlOmegaCommand = 0.0f;
  controlOmegaRefLimited = controlHeadingCorrection = 0.0f;
  wheelOdoInit = wheelEncoderInit = false;
  if (calMode == CAL_TURN_RATE)
  {
    imu2.readGyroConfiguration();
    turnRateMetadata = {};
    turnRateMetadata.readable = imu2.gyroConfigurationReadable;
    turnRateMetadata.valid = imu2.gyroConfigurationValid;
    turnRateMetadata.config = imu2.gyroConfigRegister;
    turnRateMetadata.dlpf = imu2.gyroDlpfRegister;
    turnRateMetadata.divider = imu2.gyroDividerRegister;
    imu2.beginGyroCalibration();
    turnRateRun = {};
    turnRateRun.encoder.seed(-encoder.encoder[0], encoder.encoder[1]);
    turnRateRun.transition(0, 0);
    kfAccelBias = 0;
    kfAccelCalCnt = 0;
    kfAccelCalDone = false;
    kfGyroTimestampUs = 0;
    estimator.reset();
    poseHeading = poseStartTheta = 0;
    poseHeadingTotal = poseStartHeadingTotal = 0;
    posePrevV = posePrevOmega = 0;
    pose[0] = pose[1] = pose[2] = distA = 0;
    kfLastUs = micros();
    calStartUs = micros();
    usb.send("% Turn-rate calibration sequence started: 40 s, wheel PI only\r\n");
  }
  else if (calMode == CAL_UPSIDE_DOWN)
    usb.send("% cal upside started; motors stop automatically after 6 s\r\n");
  else if (calMode == CAL_YAW)
    usb.send("% cal yaw started: gyro feedback, no heading correction, 22 s\r\n");
  else
    usb.send("% cal track started: 19 s, +/-0.05 to +/-0.20 m/s\r\n");
}

void calFinish()
{
  if (calState == 0)
    return;
  calState = 0;
  calArmed = false;
  motor.motorVoltage[0] = 0.0f;
  motor.motorVoltage[1] = 0.0f;
  motor.tick();
  resetWheelControl();
  calPending = calCnt > 0;
}

void calStop()
{
  bool wasStreaming = streamActive;
  streamActive = false;
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
  if (wasStreaming)
    usb.send("% stream stopped\r\n");
}

void calUpsideStep()
{
  if (calState == 0 or calMode != CAL_UPSIDE_DOWN)
    return;
  if (time_sec >= calTrackStepEnd)
  {
    calTrackIndex++;
    if (calTrackIndex >= calUpsideSteps)
    {
      calFinish();
      return;
    }
    calTrackStepEnd += calUpsideDuration[calTrackIndex];
  }

  calPhase = calTrackIndex == calUpsideSteps - 1 ? 2 : 10 + calTrackIndex;
  float vRef = 0.0f;
  float omegaRef = 0.0f;
  if (calTrackIndex == 1)
    vRef = calUpsideSpeed;
  else if (calTrackIndex == 2)
    omegaRef = calUpsideYawRate;

  WheelSpeeds ws = polar2Cartesian(vRef, omegaRef);
  const float mpsToMotorRadSec = gear / wheelRadius;
  wheelSpeedControl(ws.left * mpsToMotorRadSec, ws.right * mpsToMotorRadSec);
}

void upsideDownCalibrationSequence()
{
  if (calState > 0)
  {
    if (calMode == CAL_UPSIDE_DOWN)
      calUpsideStep();
    return;
  }
  if (calArmed or calPending)
    return;
  if (!digitalReadFast(PIN_START_BUTTON) and !robot.missionStart)
    return;
  if (!calStart(CAL_UPSIDE_DOWN))
    return;
  calBeginMotion();
  calUpsideStep();
}

void calTick()
{
  if (calMode == CAL_TRACK or calMode == CAL_YAW)
  {
    const tuning::Step *steps = calMode == CAL_YAW ? tuning::yaw : tuning::track;
    const int count = calMode == CAL_YAW ? calYawSteps : calTrackSteps;
    while (time_sec >= calTrackStepEnd)
    {
      calTrackIndex++;
      if (calTrackIndex >= count)
      {
        calFinish();
        return;
      }
      calTrackStepEnd += steps[calTrackIndex].duration;
    }
    calPhase = 10 + calTrackIndex;
    if (calMode == CAL_YAW)
    {
      if (!gyroFeedbackReady())
      {
        usb.send("% cal yaw stopped: gyro unavailable\r\n");
        calFinish();
        return;
      }
      trackingControlUpdate(0.0f, steps[calTrackIndex].reference,
                            poseHeading, false, true);
    }
    else
    {
      float motorSp = steps[calTrackIndex].reference * (gear / wheelRadius);
      wheelSpeedControl(motorSp, motorSp);
    }
  }
  else if (calMode == CAL_UPSIDE_DOWN)
  {
    calUpsideStep();
  }
  else
  {
    calPhase = 0;
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
  if (estimator.healthy) flags |= CAL_KF_HEALTHY;
  if (imu2.magAttempt)
  {
    flags |= CAL_MAG_ATTEMPT;
    if (imu2.magSt1 & 0x01) flags |= CAL_MAG_DRDY;
    if ((imu2.magSt1 | imu2.magSt2) & 0x08) flags |= CAL_MAG_OVERFLOW;
  }
  d.flags = flags;
  d.kfMask = kfUpdateMask;
  d.phase = calPhase;

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
  d.kfV = estimator.v;
  d.kfOmega = estimator.omega;
  d.kfTheta = poseHeading;
  d.kfBA = estimator.b_a;
  d.kfBG = 0.0f; // reserved (former gyro bias)
  for (int i = 0; i < KF_MEAS_COUNT; i++)
  {
    d.kfInnov[i] = estimator.lastInnovation[i];
    // Static gains do not propagate P, so there is no innovation variance S.
    d.kfInnovVar[i] = 0.0f;
  }
  d.magStatus = (uint16_t)imu2.magSt1 | ((uint16_t)imu2.magSt2 << 8);
  uint32_t dtUs = imu2.lastSampleTimeUs();
  d.imuDtUs = dtUs > 0xFFFF ? 0xFFFF : (uint16_t)dtUs;
  d.encoder[0] = (int16_t)(-encoder.encoder[0]);
  d.encoder[1] = (int16_t)(encoder.encoder[1]);
  d.yawReference = calMode == CAL_YAW ? desiredYawRate : 0.0f;
  d.yawReferenceLimited = calMode == CAL_YAW ? controlOmegaRefLimited : 0.0f;
  d.yawCommand = calMode == CAL_YAW ? controlOmegaCommand : 0.0f;
  d.gyroInput = kfGyroYawInput;
  d.wheelVInput = kfWheelVInput;
  d.wheelOmegaInput = kfWheelOmegaInput;
  d.gyroTimestampUs = kfGyroTimestampUs;
  d.wheelTimestampUs = kfWheelTimestampUs;
  d.accelTimestampUs = imu2.accelTimestampUs;
  d.diagnosticFlags = kfWheelEncoderSource ? LOG_WHEEL_ENCODER_FALLBACK : 0;
  if (fabsf(computedControl[0]) >= 0.99f * controlVoltageLimit)
    d.diagnosticFlags |= LOG_LEFT_SATURATED;
  if (fabsf(computedControl[1]) >= 0.99f * controlVoltageLimit)
    d.diagnosticFlags |= LOG_RIGHT_SATURATED;
  if (calMode == CAL_YAW &&
      fabsf(yawRatePid.lastOutput()) >= 0.99f * tuning::Lim_TU)
    d.diagnosticFlags |= LOG_YAW_RATE_SATURATED;
  calCnt++;
}

void printMagProbeSummary()
{
  char s[240];
  snprintf(s, sizeof(s),
           "%% mag probe summary: mpu=0x%02X ak=0x%02X cntl1=0x%02X "
           "bypass=%u mst=%u st1=0x%02X st2=0x%02X asa=%u,%u,%u "
           "available=%u\r\n",
           (unsigned)imu2.mpuWho, (unsigned)imu2.akWho,
           (unsigned)imu2.magCntl1,
           (unsigned)((imu2.magIntCfg >> 1) & 1),
           (unsigned)((imu2.magUserCtrl >> 5) & 1),
           (unsigned)imu2.magSt1, (unsigned)imu2.magSt2,
           (unsigned)imu2.magAsa[0], (unsigned)imu2.magAsa[1],
           (unsigned)imu2.magAsa[2], imu2.magAvailable ? 1u : 0u);
  Serial.print(s);
}

void printCalLog(bool manual = false)
{
  if (calState > 0)
  {
    usb.send("% Calibration capture in progress; request log after it finishes.\r\n");
    return;
  }
  if (manual)
  {
    if (!usb.usbIsUp)
      return;
  }
  else if (!Serial)
    return;
  if (calLogs == nullptr)
  {
    Serial.println("% Calibration buffer allocation failed.");
    return;
  }
  streamActive = false;
  Serial.print("% CAL ");
  Serial.println(calModeName(calMode));
  Serial.print("% max cal samples ");
  Serial.print(calMax);
  Serial.print(" each sized ");
  Serial.print(sizeof(CalData));
  Serial.println(" bytes");
  Serial.print("% Sample time "); Serial.print(sampleTimeUs);
  Serial.print(" us, log divider "); Serial.print(calDivider);
  Serial.print(", gear "); Serial.print(gear, 7);
  Serial.print(", wheel radius "); Serial.print(wheelRadius, 7);
  Serial.print(" m, wheel base "); Serial.print(wheelBase, 7);
  Serial.print(" m, PPR "); Serial.println(encoder.pulsPerRev);
  Serial.print("% Wheel PI L b0,b1,b2,a1,a2 ");
  printLogValue(calControllers.left.b0); printLogValue(calControllers.left.b1);
  printLogValue(calControllers.left.b2); printLogValue(calControllers.left.a1);
  printLogValue(calControllers.left.a2); Serial.println();
  Serial.print("% Wheel PI R b0,b1,b2,a1,a2 ");
  printLogValue(calControllers.right.b0); printLogValue(calControllers.right.b1);
  printLogValue(calControllers.right.b2); printLogValue(calControllers.right.a1);
  printLogValue(calControllers.right.a2); Serial.println();
  Serial.print("% Wheel voltage limit "); Serial.print(controlVoltageLimit, 6);
  Serial.println(" V");
  printPidHeader("Yaw rate", calControllers.yaw, tuning::Lim_TU);
  Serial.print("% Yaw mapping axis,sign,scale ");
  Serial.print(imuYawAxis); printLogValue(imuYawRateSign);
  printLogValue(tuning::yawScale); Serial.println();
  imu2.printGyroConfiguration();
  if (calMode == CAL_UPSIDE_DOWN)
  {
    Serial.print("% Upside schedule: 0.5 s zero, 2.5 s at ");
    Serial.print(calUpsideSpeed, 6); Serial.print(" m/s, 2.5 s at ");
    Serial.print(calUpsideYawRate, 9); Serial.println(" rad/s differential, 0.5 s zero");
    Serial.print("% Noise analysis excludes "); Serial.print(calNoiseSettleTime, 3);
    Serial.println(" s after each transition; phases 10 baseline, 11 forward, 12 turn, 2 settling");
  }
  if (calTruncated)
    Serial.println("% WARNING: cal buffer filled; final samples are missing.");
  Serial.println("% 1 time (s)");
  Serial.println("% 2 mode (1 mag, 2 noise, 3 track, 4 upside, 5 yaw)");
  Serial.println("% 3 flags (bit0 accel fresh, bit1 gyro fresh, bit2 mag fresh, bit3 mag valid, bit4 mag available, bit5 AS5X47 valid, bit6 AS5X47 velocity fresh, bit7 KF healthy, bit8 mag attempt, bit9 mag DRDY, bit10 mag overflow)");
  Serial.println("% 4 kfMask (bit0 gyro, bit1 wheel v, bit2 wheel omega, bits3-4 reserved, bit5 wheel v/omega from encoder fallback)");
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
  Serial.println("% 28-30 estimator v, omega, dead-reckoned heading (m/s, rad/s, rad)");
  Serial.println("% 31-32 estimator b_a, reserved (m/s^2, 0)");
  Serial.println("% 33-36 estimator innovation y (gyro, wheel v, wheel omega, reserved)");
  Serial.println("% 37-40 innovation variance S (0: static gains do not propagate P)");
  Serial.println("% 41 sequence phase (10 baseline, 11 forward, 12 turn, 2 settling; track uses 10 + step index)");
  Serial.println("% 42 mag status (low byte ST1, high byte ST2)");
  Serial.println("% 43 IMU sample interval (us)");
  Serial.println("% 44-45 encoder count low 16 bits (left,right), forward-positive");
  Serial.println("% 46-48 yaw reference, limited reference, command (rad/s; yaw mode only)");
  Serial.println("% 49 mapped/scaled gyro input (rad/s)");
  Serial.println("% 50-51 latest wheel update v,omega (m/s,rad/s; includes encoder fallback)");
  Serial.println("% 52-54 gyro,wheel,accel measurement timestamp (us; held between updates)");
  Serial.println("% 55 diagnostic flags (bit4 latest wheel source encoder, bits5/6 wheel saturation, bit8 yaw saturation)");
  Serial.println("% Decimated capture: timestamps identify distinct measurements; intermediate updates are not captured.");
  Serial.println("% IMU bus 400 kHz; gyro DLPF 92 Hz; accel DLPF 94 Hz; AK8963 continuous 100 Hz");
  Serial.println("% Forward acceleration = -acc[2]; yaw uses the mapping printed above.");
  Serial.print("% AK8963 ASA ");
  Serial.print(imu2.magAsa[0]); Serial.print(",");
  Serial.print(imu2.magAsa[1]); Serial.print(",");
  Serial.print(imu2.magAsa[2]); Serial.print(" offsets ");
  Serial.print(imu2.magOffset[0]); Serial.print(",");
  Serial.print(imu2.magOffset[1]); Serial.print(",");
  Serial.println(imu2.magOffset[2]);
  printMagProbeSummary();

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
    printLogValue(d.kfBA);
    printLogValue(d.kfBG);
    for (int i2 = 0; i2 < KF_MEAS_COUNT; i2++)
      printLogValue(d.kfInnov[i2]);
    for (int i2 = 0; i2 < KF_MEAS_COUNT; i2++)
      printLogValue(d.kfInnovVar[i2]);
    Serial.print(" "); Serial.print(d.phase);
    Serial.print(" "); Serial.print(d.magStatus);
    Serial.print(" "); Serial.print(d.imuDtUs);
    Serial.print(" "); Serial.print(d.encoder[0]);
    Serial.print(" "); Serial.print(d.encoder[1]);
    printLogValue(d.yawReference);
    printLogValue(d.yawReferenceLimited);
    printLogValue(d.yawCommand);
    printLogValue(d.gyroInput);
    printLogValue(d.wheelVInput);
    printLogValue(d.wheelOmegaInput);
    Serial.print(" "); Serial.print(d.gyroTimestampUs);
    Serial.print(" "); Serial.print(d.wheelTimestampUs);
    Serial.print(" "); Serial.print(d.accelTimestampUs);
    Serial.print(" "); Serial.print(d.diagnosticFlags);
    Serial.println("");
  }
  Serial.print("% sent ");
  Serial.print(calCnt);
  Serial.println(" cal lines");
  calPending = false;
  display.setLine("");
}

void clearCalLog()
{
  if (calState > 0)
  {
    usb.send("% cal clear refused: capture active\r\n");
    return;
  }
  calPending = false;
  calCnt = 0;
  calTruncated = false;
  calMode = CAL_NONE;
  calPhase = 0;
  display.setLine("");
  usb.send("% calibration capture cleared\r\n");
}

void streamHeader(int hz)
{
  char s[180];
  snprintf(s, sizeof(s), "%% CAL stream %d Hz (divider %d), cap %lu s\r\n",
           hz, streamDivider, (unsigned long)(streamMaxMs / 1000));
  usb.send(s);
  usb.send("% 1 time (s)\r\n");
  usb.send("% 2 IMU sample interval (us)\r\n");
  usb.send("% 3 flags (same bits as the calibration capture)\r\n");
  usb.send("% 4 AK8963 ST1\r\n");
  usb.send("% 5 AK8963 ST2\r\n");
  usb.send("% 6-8 gyro (x,y,z) (deg/s, offset-corrected)\r\n");
  usb.send("% 9-11 accel (x,y,z) (g)\r\n");
  usb.send("% 12-14 mag (x,y,z) (adjusted counts, 0.15 uT/LSB)\r\n");
  usb.send("% 15-16 AS5X47 angle (left,right) (rad)\r\n");
  usb.send("% 17-18 encoder counts (left,right), forward-positive\r\n");
  usb.send("% 19 MPU temperature (C, updated at 1 Hz)\r\n");
  usb.send("% 20-21 gyro/accel measurement timestamp (us)\r\n");
  Serial.print("% Yaw mapping axis,sign,scale ");
  Serial.print(imuYawAxis); printLogValue(imuYawRateSign);
  printLogValue(tuning::yawScale); Serial.println();
  usb.send("% Forward acceleration = -acc[2]; yaw uses the mapping printed above.\r\n");
  usb.send("% Motors off: keep still for noise, or rotate a measured angle for yaw calibration.\r\n");
  imu2.printGyroConfiguration();
  printMagProbeSummary();
}

void startStream(int hz)
{
  if (state > 0 or calState > 0 or calArmed or calPending or logPending)
  {
    usb.send("% stream: finish the current run or capture first.\r\n");
    return;
  }
  if (!usb.usbIsUp)
  {
    usb.send("% stream: open the monitor first.\r\n");
    return;
  }
  if (hz < 50) hz = 50;
  if (hz > 500) hz = 500;
  streamDivider = 1000 / hz;
  if (streamDivider < 2) streamDivider = 2;
  streamActive = true;
  streamStartMs = millis();
  streamTickCnt = 0;
  streamHeader(hz);
}

void streamTick()
{
  if (!streamActive)
    return;
  if (state > 0 or calState > 0)
  {
    streamActive = false;
    usb.send("% stream stopped: run or capture started.\r\n");
    return;
  }
  if (millis() - streamStartMs > streamMaxMs)
  {
    streamActive = false;
    usb.send("% stream stopped: 20 min cap.\r\n");
    return;
  }
  if ((streamTickCnt++ % streamDivider) != 0)
    return;
  if (!usb.usbIsUp)
    return;
  uint16_t flags = 0;
  if (imu2.accelFresh) flags |= CAL_ACCEL_FRESH;
  if (imu2.gyroFresh) flags |= CAL_GYRO_FRESH;
  if (imu2.magFresh) flags |= CAL_MAG_FRESH;
  if (imu2.magHeadingValid) flags |= CAL_MAG_VALID;
  if (imu2.magAvailable) flags |= CAL_MAG_AVAILABLE;
  if (as5Valid) flags |= CAL_AS5_VALID;
  if (as5Fresh) flags |= CAL_AS5_FRESH;
  if (imu2.magAttempt)
  {
    flags |= CAL_MAG_ATTEMPT;
    if (imu2.magSt1 & 0x01) flags |= CAL_MAG_DRDY;
    if ((imu2.magSt1 | imu2.magSt2) & 0x08) flags |= CAL_MAG_OVERFLOW;
  }
  uint32_t dtUs = imu2.lastSampleTimeUs();
  if (dtUs > 0xFFFF) dtUs = 0xFFFF;
  char buf[280];
  int n = snprintf(buf, sizeof(buf),
      "%.4f %u %u %u %u %.6g %.6g %.6g %.6g %.6g %.6g "
      "%.4f %.4f %.4f %.6f %.6f %d %d %.3f %lu %lu\r\n",
      time_sec, (unsigned)dtUs, (unsigned)flags,
      (unsigned)imu2.magSt1, (unsigned)imu2.magSt2,
      imu2.gyro[0], imu2.gyro[1], imu2.gyro[2],
      imu2.acc[0], imu2.acc[1], imu2.acc[2],
      imu2.mag[0], imu2.mag[1], imu2.mag[2],
      as5AngleLast[0], as5AngleLast[1],
      (int)(-encoder.encoder[0]), (int)(encoder.encoder[1]),
      imu2.temperatureValid ? imu2.temperatureC : NAN,
      (unsigned long)kfGyroTimestampUs, (unsigned long)imu2.accelTimestampUs);
  if (n > 0)
  {
    if (n >= (int)sizeof(buf)) n = sizeof(buf) - 1;
    Serial.write(buf, n);
  }
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
  else if (strncmp(p, "yaw", 3) == 0)
    calStart(CAL_YAW);
  else if (strncmp(p, "imu", 3) == 0)
    imu2.printGyroConfiguration();
  else if (strncmp(p, "gains", 5) == 0)
  {
    if (state > 0 || calState > 0 || calArmed || streamActive)
    {
      usb.send("% cal gains refused: stop the active/armed experiment first\r\n");
      return true;
    }
    char loopName[8], extra;
    float kp, ki;
    if (sscanf(p + 5, "%7s %f %f %c", loopName, &kp, &ki, &extra) != 3 ||
        !isfinite(kp) || !isfinite(ki) || kp < 0 || ki < 0)
    {
      usb.send("% usage: cal gains left|right|yaw Kp Ki (finite, nonnegative)\r\n");
      return true;
    }
    PID_Coefficients *coeffs = nullptr;
    PIDController *pid = nullptr;
    if (strcmp(loopName, "left") == 0) { coeffs = &left_wheel_coeffs; pid = &leftController; }
    if (strcmp(loopName, "right") == 0) { coeffs = &right_wheel_coeffs; pid = &rightController; }
    if (strcmp(loopName, "yaw") == 0) { coeffs = &yaw_pid_coeffs; pid = &yawRatePid; }
    const PID_Coefficients proposed = tuning::pi(kp, ki);
    if (!pid || !isfinite(proposed.b0) || !isfinite(proposed.b1))
      usb.send("% invalid PI loop or coefficients\r\n");
    else
    {
      *coeffs = proposed;
      pid->setCoefficients(proposed);
      printPidHeader(loopName, *coeffs, pid->limit());
      usb.send("% trial gains applied; reset restores tuning.h defaults\r\n");
    }
  }
  else if (strncmp(p, "upside", 6) == 0)
    calStart(CAL_UPSIDE_DOWN);
  else if (strncmp(p, "stream", 6) == 0)
  {
    const char *q = p + 6;
    while (*q == ' ')
      q++;
    int hz = 100;
    if (*q >= '0' && *q <= '9')
      hz = strtol(q, NULL, 10);
    startStream(hz);
  }
  else if (strncmp(p, "stop", 4) == 0)
    calStop();
  else if (strncmp(p, "clear", 5) == 0)
    clearCalLog();
  else if (strncmp(p, "status", 6) == 0)
    printStatus();
  else if (strncmp(p, "log", 3) == 0)
    printCalLog(true);
  else if (strncmp(p, "retrymag", 8) == 0)
    imu2.retryMag();
  else
    usb.send("% cal commands: mag | noise | track | yaw | upside | imu | gains left|right|yaw Kp Ki | stream [Hz] | stop | clear | status | log | retrymag\r\n");
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
  estimator.reset();
  poseHeading = 0.0f;
  poseStartTheta = poseHeading;
  poseHeadingTotal = 0.0f;
  poseStartHeadingTotal = poseHeadingTotal;
  posePrevV = estimator.v;
  posePrevOmega = estimator.omega;
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
        calBeginMotion();
        calTick();
      }
      else
      {
        closedLoopTrajectory();
        //upsideDownCalibrationSequence();
        //sequenceTwoSteps();
        //yawRateStep();
        //forwardThreeSeconds();
        if (state > 0)
        {
          controlUpdate();
        }
      }
      motor.tick();
      if (state > 0) sampleWheelMagnetics();
      if (logTickCnt % logDivider == 0)
        updateLog();
      logTickCnt++;
      if (state > 0 and liveLogEnabled and
          (liveLogTickCnt++ % liveLogDivider) == 0)
        sendLiveLog();
      if (calState > 0 and calTickCnt % calDivider == 0)
        updateCalLog();
      calTickCnt++;
      streamTick();
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
