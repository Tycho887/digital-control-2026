// c++ -std=c++14 -O2 -Isrc/src tests/motion_host/odometry_check.cpp -o /tmp/odometry-check
#include "odometry.h"
#include <cassert>
#include <cmath>
#include <cstdio>

int main()
{
  PairedSimpson rule;
  // v(t) = t² on two unequal intervals of 1 and 1.5 ms. Scale the
  // rate by using milliseconds in the polynomial, and use omega = -2v.
  auto first = rule.correction(0.001f, 1.0f, -2.0f);
  assert(first.distance == 0 && first.yaw == 0);
  auto second = rule.correction(0.0015f, 5.25f, -10.5f);
  const double trapezoid = 0.0005 + (1.0 + 6.25) * 0.0015 / 2;
  const double exact = 0.001 * 2.5 * 2.5 * 2.5 / 3;
  assert(std::fabs(trapezoid + second.distance - exact) < 1e-9);
  assert(std::fabs(-2 * trapezoid + second.yaw + 2 * exact) < 1e-9);
  assert(rule.distanceDifference == second.distance && rule.yawDifference == second.yaw);
  assert(rule.peakDistanceDifference == -second.distance && rule.peakYawDifference == second.yaw);

  // A fresh run clears pending history and totals. Equal-step Simpson is
  // exact for v(t) = t³ from t=0 to 2, whereas trapezoids give 5.
  rule = {};
  assert(rule.distanceDifference == 0 && rule.peakYawDifference == 0);
  rule.correction(0.001f, 1.0f, 0.0f);
  second = rule.correction(0.001f, 7.0f, 0.0f);
  assert(std::fabs(0.005 + second.distance - 0.004) < 1e-9);

  const double beforeGap = rule.distanceDifference;
  rule.correction(0.001f, 5.0f, -1.0f);
  rule.discardPendingInterval();
  first = rule.correction(0.001f, 2.0f, 1.0f);
  assert(first.distance == 0 && first.yaw == 0);
  second = rule.correction(0.003f, -1.0f, 4.0f);
  assert(second.distance == 0 && second.yaw == 0);
  assert(rule.distanceDifference == beforeGap);
  // Same slopes on both intervals give trapezoids, including signed motion.
  rule.correction(0.001f, -2.0f, 1.0f);
  second = rule.correction(0.002f, -4.0f, 2.0f);
  assert(std::fabs(second.distance) < 1e-9 && std::fabs(second.yaw) < 1e-9);
  std::puts("PASS paired odometry quadrature, gap handling, run statistics");
}
