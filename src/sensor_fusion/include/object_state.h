#ifndef __OJBECT_STATE_H__
#define __OBJECT_STATE_H__

#include "ros/ros.h"

class ObjectState {
 public:
  ObjectState();
  virtual ~ObjectState();

 private:
  uint8_t obj_id; // object ID
  float64 obj_dx; // longitudinal range
  uint8 obj_lane; // lane assignment
  float64 obj_vx; // relaive longitudinal velocity
  float64 obj_dy; // lateral range
  float64 obj_ax; // relative longitudinal accel
  bool obj_path; // 1:object in path 2:object not in path
  float64 obj_vy; // lateral velocity
  float64 obj_timestamp; //time last object detection
};

#endif  // __OBJECT_STATE_H__
