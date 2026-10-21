#pragma once
#include <BasicLinearAlgebra.h>
#include <stdint.h>
#include <math.h>

// Diagnostic only. SI units; heading remains unwrapped.
class MotionEstimator {
public:
  enum { S, V, A, THETA, W, ALPHA, BA, BG, N };
  BLA::Matrix<N,1> x;
  BLA::Matrix<N,N> P;
  float position[2] = {0,0};
  bool healthy = true;
  uint32_t timestamp = 0;
  // Starting tuning values, not sensor specifications.
  float linearJerk = 1.0f, angularJerk = 4.0f;
  float accelVariance = 0.04f, gyroVariance = 0.0001f;
  float accelBiasWalk = 1e-6f, gyroBiasWalk = 1e-8f;
  void reset(uint32_t now) {
    x.Fill(0); P.Fill(0);
    const float initial[N] = {0,0.1f,1,0,0.1f,1,0.01f,0.001f};
    for(int i=0;i<N;i++) P(i,i)=initial[i];
    position[0]=position[1]=0; timestamp=now; healthy=true;
  }
  bool predict(uint32_t now) {
    float dt = uint32_t(now-timestamp)*1e-6f;
    if(dt<=0) return false;
    if(dt>0.1f) { timestamp=now; healthy=false; return false; }
    timestamp=now;
    BLA::Matrix<N,N> transition, Q; transition.Fill(0); Q.Fill(0);
    for(int i=0;i<N;i++) transition(i,i)=1;
    for(int b : {0,3}) {
      transition(b,b+1)=dt; transition(b,b+2)=0.5f*dt*dt; transition(b+1,b+2)=dt;
      float q=b==0?linearJerk:angularJerk;
      float t2=dt*dt,t3=t2*dt,t4=t3*dt,t5=t4*dt;
      Q(b,b)=q*t5/20; Q(b,b+1)=Q(b+1,b)=q*t4/8;
      Q(b,b+2)=Q(b+2,b)=q*t3/6;
      Q(b+1,b+1)=q*t3/3; Q(b+1,b+2)=Q(b+2,b+1)=q*t2/2;
      Q(b+2,b+2)=q*dt;
    }
    Q(BA,BA)=accelBiasWalk*dt; Q(BG,BG)=gyroBiasWalk*dt;
    x=transition*x; P=transition*P*~transition+Q;
    return true;
  }
  bool measure(float z, const BLA::Matrix<1,N>& h, float variance) {
    if(!isfinite(z)||!isfinite(variance)||variance<=0) return false;
    BLA::Matrix<N,1> ph=P*~h;
    float innovation=z-(h*x)(0,0), innovationVariance=(h*ph)(0,0)+variance;
    if(!isfinite(innovationVariance)||innovationVariance<=0) { healthy=false; return false; }
    BLA::Matrix<N,1> k=ph/innovationVariance;
    BLA::Matrix<N,N> identity; identity.Fill(0);
    for(int i=0;i<N;i++) identity(i,i)=1;
    BLA::Matrix<N,N> m=identity-k*h;
    x=x+k*innovation; P=m*P*~m+(k*~k)*variance;
    for(int i=0;i<N;i++) {
      if(!isfinite(x(i))||!isfinite(P(i,i))||P(i,i)<0) healthy=false;
      for(int j=0;j<i;j++) P(i,j)=P(j,i)=0.5f*(P(i,j)+P(j,i));
    }
    return healthy;
  }
  void integrate(float oldDistance, float oldHeading) {
    float ds=x(S)-oldDistance, midpoint=(x(THETA)+oldHeading)*0.5f;
    position[0]+=ds*cosf(midpoint); position[1]+=ds*sinf(midpoint);
  }
};
