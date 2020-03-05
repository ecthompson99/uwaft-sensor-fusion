#include "object_state.h"
#include "string.h"

uint8_t ObjectState::get_obj_id() const { return obj_id;}

ObjectState ObjectState::create_object(const sensor_fusion::filtered_object_msg& filtered_msg) {
    ObjectState new_object = ObjectState();
    new_object.obj_id = filtered_msg.obj_id;
    new_object.obj_dx = filtered_msg.obj_dx;
    new_object.obj_lane = filtered_msg.obj_lane;
    new_object.obj_vx = filtered_msg.obj_vx;
    new_object.obj_dy = filtered_msg.obj_dy;
    new_object.obj_ax = filtered_msg.obj_ax;
    new_object.obj_path = filtered_msg.obj_path;
    new_object.obj_vy = filtered_msg.obj_vy;
    new_object.obj_timestamp = filtered_msg.obj_timestamp;
    return new_object;
}
