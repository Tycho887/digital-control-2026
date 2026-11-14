// CSV-in/CSV-out driver for the stationary estimator, used by
// tests/kalman_host/test_parity.py to compare against the NumPy reference.
//
// Build:
//   c++ -std=c++14 -O2 -Isrc/src tests/kalman_host/parity.cpp src/src/kalman.cpp \
//       -o /tmp/kalman-parity
// Run:
//   /tmp/kalman-parity input.csv output.csv
//
// Input header (one row per filter step):
//   step,dt,a,gyro,wheel_v,wheel_omega,use_gyro,use_wheel
// Output header:
//   step,v,b_a,omega,innov_gyro,innov_wheel_v,innov_wheel_omega,healthy
#include "kalman.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int main(int argc, char **argv)
{
  if (argc != 3)
  {
    fprintf(stderr, "usage: %s <input.csv> <output.csv>\n", argv[0]);
    return 2;
  }

  FILE *in = fopen(argv[1], "r");
  if (in == NULL)
  {
    fprintf(stderr, "error: cannot open input file '%s'\n", argv[1]);
    return 1;
  }
  FILE *out = fopen(argv[2], "w");
  if (out == NULL)
  {
    fprintf(stderr, "error: cannot open output file '%s'\n", argv[2]);
    fclose(in);
    return 1;
  }

  StaticDecoupledEstimator f;
  f.reset();

  fprintf(out,
          "step,v,b_a,omega,innov_gyro,innov_wheel_v,"
          "innov_wheel_omega,healthy\n");

  char line[1024];
  int lineNo = 0;
  int status = 0;
  while (fgets(line, sizeof(line), in) != NULL)
  {
    lineNo++;
    if (line[0] == '#' || line[0] == '\n' || line[0] == '\r' || line[0] == '\0')
      continue;
    if (strncmp(line, "step", 4) == 0) // header
      continue;

    int step = 0;
    int useGyro = 0, useWheel = 0;
    double dt = 0.0, a = 0.0, gyro = 0.0;
    double wheelV = 0.0, wheelOmega = 0.0;
    int fields = sscanf(line, "%d,%lf,%lf,%lf,%lf,%lf,%d,%d",
                        &step, &dt, &a, &gyro, &wheelV, &wheelOmega,
                        &useGyro, &useWheel);
    if (fields != 8)
    {
      fprintf(stderr, "error: %s:%d: expected 8 fields, got %d\n",
              argv[1], lineNo, fields);
      status = 1;
      break;
    }

    f.predict((float)a, (float)dt);
    if (useGyro)
      f.updateGyro((float)gyro);
    if (useWheel)
    {
      f.updateWheelV((float)wheelV);
      f.updateWheelOmega((float)wheelOmega);
    }

    fprintf(out, "%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%d\n",
            step, (double)f.v, (double)f.b_a, (double)f.omega,
            (double)f.lastInnovation[KF_M_GYRO],
            (double)f.lastInnovation[KF_M_WHEEL_V],
            (double)f.lastInnovation[KF_M_WHEEL_OMEGA],
            f.healthy ? 1 : 0);
  }

  if (ferror(in))
  {
    fprintf(stderr, "error: reading '%s' failed\n", argv[1]);
    status = 1;
  }
  fclose(in);
  if (fclose(out) != 0)
  {
    fprintf(stderr, "error: writing '%s' failed\n", argv[2]);
    status = 1;
  }
  return status;
}
