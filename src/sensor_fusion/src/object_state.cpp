#include "object_state.h"

ObjectState::ObjectState(){}
ObjectState::ObjectState(uint8_t set_obj_id, double set_obj_dx, uint8_t set_obj_lane, double set_obj_vx, 
    double set_obj_dy, double set_obj_ax, bool set_obj_path, double set_obj_vy, double set_obj_timestamp, uint8_t set_obj_count) {
    obj_id = set_obj_id;
    obj_dx = set_obj_dx;
    obj_lane = set_obj_lane;
    obj_vx = set_obj_vx;
    obj_dy = set_obj_dy;
    obj_ax = set_obj_ax;
    obj_path = set_obj_path;
    obj_vy = set_obj_vy;
    obj_timestamp = set_obj_timestamp;
		obj_count = set_obj_count;
}
ObjectState::~ObjectState() {}

uint8_t ObjectState::get_obj_id() const { return obj_id; }
double ObjectState::get_obj_dx() const { return obj_dx; }
uint8_t ObjectState::get_obj_lane() const { return obj_lane; }
double ObjectState::get_obj_vx() const {return obj_vx;}
double ObjectState::get_obj_dy() const {return obj_dy;}
double ObjectState::get_obj_ax() const {return obj_ax;}
bool ObjectState::get_obj_path() const {return obj_path;}
double ObjectState::get_obj_vy() const {return obj_vy;}
double ObjectState::get_obj_timestamp() const { return obj_timestamp; }
uint8_t ObjectState::get_obj_count() const { return obj_count; }

void ObjectState::copy_info(const sensor_fusion::filtered_object_msg& filtered_msg) {
    
    obj_id = filtered_msg.obj_id;
    obj_dx = filtered_msg.obj_dx;
    obj_lane = filtered_msg.obj_lane;
    obj_vx = filtered_msg.obj_vx;
    obj_dy = filtered_msg.obj_dy;
    obj_ax = filtered_msg.obj_ax;
    obj_path = filtered_msg.obj_path;
    obj_vy = filtered_msg.obj_vy;
    obj_timestamp = filtered_msg.obj_timestamp;
		obj_count = filtered_msg.obj_count;
}

// TODO:
// distinguish 3 tracked objects from vector of 32 objects
// check timestamps, lane of 3 objects
// publish 3 target vehicle objects to CAN topic
