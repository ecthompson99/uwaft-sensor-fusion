#include "object_state.h"
// #include "string.h"

ObjectState::ObjectState(){}
ObjectState::~ObjectState() {}

uint8_t ObjectState::get_obj_id() const { return obj_id; }
uint8_t ObjectState::get_obj_timestamp() const { return obj_timestamp; }
uint8_t ObjectState::get_obj_lane() const { return obj_lane; }
uint8_t ObjectState::get_obj_dx() const { return obj_dx; }

void ObjectState::copy_info(const sensor_fusion::filtered_object_msg& filtered_msg) {
    // cout << "Copy constructor allocating ptr." << endl;
    // ptr = new int;
    // *ptr = *filtered_msg.ptr;
    
    
    obj_id = filtered_msg.obj_id;
    obj_dx = filtered_msg.obj_dx;
    obj_lane = filtered_msg.obj_lane;
    obj_vx = filtered_msg.obj_vx;
    obj_dy = filtered_msg.obj_dy;
    obj_ax = filtered_msg.obj_ax;
    obj_path = filtered_msg.obj_path;
    obj_vy = filtered_msg.obj_vy;
    obj_timestamp = filtered_msg.obj_timestamp;
}

// TODO:
// distinguish 3 tracked objects from vector of 32 objects
// check timestamps, lane of 3 objects
// publish 3 target vehicle objects to CAN topic
