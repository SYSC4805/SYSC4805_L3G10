// Time-based heading model. Does not measure slip, drift or actual yaw.
#pragma once
#include <stdint.h>

class SimulatedHeading {
 public:
  SimulatedHeading(uint32_t left90Ms=900, uint32_t right90Ms=900)
    : leftMs(left90Ms),rightMs(right90Ms) {}
  void beginTurn(bool clockwise,uint32_t now){
    update(now); active=false; right=clockwise; segment=0; last=now;
  }
  void setMotion(bool turning,bool clockwise,uint32_t now){
    update(now);active=turning;right=clockwise;
  }
  void update(uint32_t now){
    uint32_t dt=now-last;last=now;
    if(active){
      uint32_t duration=right?rightMs:leftMs;
      if(duration==0)return;
      float degrees=90.0f*dt/duration;
      segment+=degrees;yaw+=right?degrees:-degrees;
    }
  }
  float turnDegrees() const{return segment;}
  float yawDegrees() const{return yaw;}
  const char* source() const{return "SIM_TIME";}
 private:
  uint32_t leftMs,rightMs,last=0;
  bool active=false,right=true;
  float segment=0,yaw=0;
};
using HeadingProvider=SimulatedHeading;
