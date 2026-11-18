#ifndef PID_H
#define PID_H

/**
 * Incremental (CCDE) PID:
 *
 *   u[k] = b0*e[k] + b1*e[k-1] + b2*e[k-2] - a1*u[k-1] - a2*u[k-2]
 *
 * The output is clamped before it is stored, so the next update starts from
 * the limited value and the integrator cannot wind up.
 *
 * No Arduino headers, so the module also builds on the host.
 */
struct PID_Coefficients
{
  float b0;
  float b1;
  float b2;
  float a1;
  float a2;
};

class PIDController
{
public:
  PIDController(const PID_Coefficients &c, float limit)
      : coeffs(c), outputLimit(limit) {}

  void reset()
  {
    u1 = 0.0f;
    u2 = 0.0f;
    e1 = 0.0f;
    e2 = 0.0f;
    error = 0.0f;
  }

  float ComputeCCDE(float err)
  {
    error = err;
    float u = coeffs.b0 * error + coeffs.b1 * e1 + coeffs.b2 * e2 -
              coeffs.a1 * u1 - coeffs.a2 * u2;

    e2 = e1;
    e1 = error;

    if (outputLimit > 0.0f)
    {
      if (u > outputLimit)
        u = outputLimit;
      else if (u < -outputLimit)
        u = -outputLimit;
    }

    u2 = u1;
    u1 = u;
    return u;
  }

  float lastError() const { return error; }
  float lastOutput() const { return u1; }
  const PID_Coefficients &coefficients() const { return coeffs; }
  float limit() const { return outputLimit; }

  // Firmware permits this only while idle; discard all previous PI memory.
  void setCoefficients(const PID_Coefficients &c)
  {
    coeffs = c;
    reset();
  }

private:
  PID_Coefficients coeffs;
  float outputLimit;
  float u1 = 0.0f;
  float u2 = 0.0f;
  float e1 = 0.0f;
  float e2 = 0.0f;
  float error = 0.0f;
};

#endif
