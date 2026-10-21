#include "motion_estimator.h"
#include <cassert>
#include <cstdio>
int main() {
  MotionEstimator e; e.reset(0xffff0000u);
  uint32_t now=e.timestamp;
  for(int tick=0;tick<1000;tick++) {
    float oldS=e.x(0), oldHeading=e.x(3);
    now+=tick%2?11000:9000;
    assert(e.predict(now));
    BLA::Matrix<1,8> h;
    // Forward and reverse, with turning and nonzero sensor biases.
    float v=tick<500?0.2f:-0.2f, w=0.3f;
    for(int side=0;side<2;side++) {
      h.Fill(0); h(0,1)=9.6f/0.0315f; h(0,4)=(side?1:-1)*9.6f*0.14f/(2*0.0315f);
      assert(e.measure(h(0,1)*v+h(0,4)*w,h,1));
    }
    h.Fill(0); h(0,2)=h(0,6)=1; assert(e.measure(0.03f,h,0.04f));
    h.Fill(0); h(0,4)=h(0,7)=1; assert(e.measure(w+0.01f,h,0.0001f));
    e.integrate(oldS,oldHeading);
    for(int i=0;i<8;i++) {
      assert(isfinite(e.x(i)));
      for(int j=0;j<8;j++) assert(fabsf(e.P(i,j)-e.P(j,i))<1e-6);
    }
    // Positive definiteness with a tiny diagonal regularization for known origins.
    double l[8][8]={};
    for(int i=0;i<8;i++) for(int j=0;j<=i;j++) {
      double sum=e.P(i,j)+(i==j?1e-9:0);
      for(int k=0;k<j;k++) sum-=l[i][k]*l[j][k];
      if(i==j) { assert(sum>0); l[i][j]=sqrt(sum); }
      else l[i][j]=sum/l[j][j];
    }
    printf("%u",now);
    for(int i=0;i<8;i++) printf(",%.9g",e.x(i));
    for(int i=0;i<8;i++) for(int j=0;j<8;j++) printf(",%.9g",e.P(i,j));
    printf("\n");
  }
  assert(fabsf(e.x(1)+0.2f)<0.01f);
  assert(fabsf(e.x(4)-0.3f)<0.01f);
  BLA::Matrix<1,8> h; h.Fill(0); h(0,1)=1;
  assert(!e.measure(NAN,h,1)); assert(!e.measure(1,h,-1));
  for(int k=0;k<100;k++) { now+=10000; assert(e.predict(now)); }
  now+=200000; assert(!e.predict(now)); assert(!e.healthy);
  e.reset(now); assert(e.healthy && e.x(0)==0);
}
