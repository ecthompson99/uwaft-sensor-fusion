#ifndef __OJBECT_STATE_H__
#define __OBJECT_STATE_H__
#include "ros/ros.h"
class ObjectState {
 public:
  ObjectState(double dx, double vx, double dy, double vy, double timestamp) {
      dx = dx;
      vx = vx;
      dy = dy;
      vy = vy;
      timestamp = timestamp;
      count = 0;
  }

  virtual ~ObjectState();
 private:
  int count;
  uint8_t id; // object ID
  double dx; // longitudinal range
  uint8_t lane; // lane assignment
  double vx; // relaive longitudinal velocity
  double dy; // lateral range
  double ax; // relative longitudinal accel
  bool path; // 1:object in path 2:object not in path
  double vy; // lateral velocity
  double timestamp; //time last object detection
};
#endif  // __OBJECT_STATE_H__