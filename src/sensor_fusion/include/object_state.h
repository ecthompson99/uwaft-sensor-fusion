#ifndef __OJBECT_STATE_H__
#define __OBJECT_STATE_H__

#include "ros/ros.h"
#include "sensor_fusion/filtered_object_msg.h"

class ObjectState {
 public:
  ObjectState();
  virtual ~ObjectState();
  uint8_t get_obj_id() const;
  ObjectState create_object(const sensor_fusion::filtered_object_msg& filtered_msg);

 private:
  uint8_t obj_id; // object ID
  double obj_dx; // longitudinal range
  uint8_t obj_lane; // lane assignment
  double obj_vx; // relaive longitudinal velocity
  double obj_dy; // lateral range
  double obj_ax; // relative longitudinal accel
  bool obj_path; // 1:object in path 2:object not in path
  double obj_vy; // lateral velocity
  double obj_timestamp; //time last object detection

};

#endif  // __OBJECT_STATE_H__
