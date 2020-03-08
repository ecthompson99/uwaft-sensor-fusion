#include "object_state.h"
// #include "string.h"

uint8_t ObjectState::get_obj_id() const { return obj_id;}

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
}
