// Host-side checks for the incremental PID used by the wheel and outer loops.
//
// Build and run from the project root:
//   c++ -std=c++14 -O2 -Isrc/src tests/motion_host/controller_check.cpp \
//       -o /tmp/controller-check
#include "pid.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static bool near(float a, float b, float tol)
{
  return fabsf(a - b) <= tol;
}

static void testIncrementalIntegral()
{
  // b0 = 1, a1 = -1 gives u[k] = u[k-1] + e[k].
  PID_Coefficients c = {1.0f, 0.0f, 0.0f, -1.0f, 0.0f};
  PIDController pid(c, 10.0f);
  assert(near(pid.ComputeCCDE(1.0f), 1.0f, 1e-6f));
  assert(near(pid.ComputeCCDE(1.0f), 2.0f, 1e-6f));
  assert(near(pid.ComputeCCDE(1.0f), 3.0f, 1e-6f));
  printf("PASS incremental form accumulates\n");
}

static void testSaturationAndAntiWindup()
{
  PID_Coefficients c = {1.0f, 0.0f, 0.0f, -1.0f, 0.0f};
  PIDController pid(c, 2.0f);
  assert(near(pid.ComputeCCDE(1.0f), 1.0f, 1e-6f));
  assert(near(pid.ComputeCCDE(1.0f), 2.0f, 1e-6f));
  assert(near(pid.ComputeCCDE(1.0f), 2.0f, 1e-6f)); // clamped and stored
  assert(near(pid.ComputeCCDE(-1.0f), 1.0f, 1e-6f)); // recovers immediately

  PIDController neg(c, 2.0f);
  assert(near(neg.ComputeCCDE(-1.0f), -1.0f, 1e-6f));
  assert(near(neg.ComputeCCDE(-1.0f), -2.0f, 1e-6f));
  assert(near(neg.ComputeCCDE(-1.0f), -2.0f, 1e-6f));
  printf("PASS saturation stores the limited output (no windup)\n");
}

static void testReset()
{
  PID_Coefficients c = {1.0f, 0.0f, 0.0f, -1.0f, 0.0f};
  PIDController pid(c, 10.0f);
  pid.ComputeCCDE(1.0f);
  pid.ComputeCCDE(1.0f);
  pid.reset();
  assert(near(pid.lastOutput(), 0.0f, 1e-6f));
  assert(near(pid.lastError(), 0.0f, 1e-6f));
  assert(near(pid.ComputeCCDE(1.0f), 1.0f, 1e-6f));
  printf("PASS reset clears memory\n");
}

static void testPlaceholderVelocityCoefficients()
{
  // The shipped placeholder is b0 = 0.50, b1 = -0.49, a1 = -1, i.e.
  // u[k] = u[k-1] + 0.50*e[k] - 0.49*e[k-1]. A constant unit error therefore
  // increases the output by 0.01 per sample.
  PID_Coefficients c = {0.50f, -0.49f, 0.0f, -1.0f, 0.0f};
  PIDController pid(c, 10.0f);
  pid.ComputeCCDE(1.0f); // 0.50
  float u = 0.0f;
  for (int i = 0; i < 10; ++i)
    u = pid.ComputeCCDE(1.0f);
  assert(near(u, 0.60f, 1e-5f));
  printf("PASS placeholder velocity coefficients: u=%.4f\n", u);
}

static void testNoLimit()
{
  PID_Coefficients c = {1.0f, 0.0f, 0.0f, -1.0f, 0.0f};
  PIDController pid(c, 0.0f); // limit <= 0 disables clamping
  for (int i = 0; i < 100; ++i)
    pid.ComputeCCDE(1.0f);
  assert(near(pid.lastOutput(), 100.0f, 1e-4f));
  printf("PASS zero limit disables clamping\n");
}

int main()
{
  testIncrementalIntegral();
  testSaturationAndAntiWindup();
  testReset();
  testPlaceholderVelocityCoefficients();
  testNoLimit();
  printf("All PID host checks passed.\n");
  return 0;
}
