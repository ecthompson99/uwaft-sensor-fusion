#include "object_state.h"
// #include "string.h"

uint8_t ObjectState::get_obj_id() const { return this.obj_id;}

void ObjectState::copy_info(const sensor_fusion::filtered_object_msg& filtered_msg) {
    // cout << "Copy constructor allocating ptr." << endl;
    // ptr = new int;
    // *ptr = *filtered_msg.ptr;
    
    
    this.obj_id = filtered_msg.obj_id;
    this.obj_dx = filtered_msg.obj_dx;
    this.obj_lane = filtered_msg.obj_lane;
    this.obj_vx = filtered_msg.obj_vx;
    this.obj_dy = filtered_msg.obj_dy;
    this.obj_ax = filtered_msg.obj_ax;
    this.obj_path = filtered_msg.obj_path;
    this.obj_vy = filtered_msg.obj_vy;
    this.obj_timestamp = filtered_msg.obj_timestamp;
}
