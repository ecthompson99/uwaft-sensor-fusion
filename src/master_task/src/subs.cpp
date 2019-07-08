#include <stdio.h>
#include <sstream>

#include "ros/ros.h"

#include "master_task/can_comms_data_msg.h"

static const uint8_t MASTER_MESSAGE_BUFFER_SIZE = 1000;

void can_callback(const master_task::can_comms_data_msg& can_comms_msg){

    bool acc_switch = can_comms_msg.acc_switch_status;
    bool aeb_switch = can_comms_msg.aeb_switch_status;
    bool lc_switch = can_comms_msg.lc_switch_status;
    bool is_acc_valid = can_comms_msg.acc_valid;
    bool is_hold_target_speed = can_comms_msg.hold_target_speed;
    bool is_aeb_valid = can_comms_msg.aeb_valid;
    bool is_aeb_override = can_comms_msg.aeb_override;
    bool is_lc_valid = can_comms_msg.lc_valid;
    int arc = can_comms_msg.alive_rolling_counter;
    

    ROS_INFO_STREAM("\n"
        << "acc_switch_status " << acc_switch << "\n" 
        << "aeb_switch_status " << aeb_switch << "\n"
        << "lc_switch_status " << lc_switch << "\n"
        << "target_accel " << can_comms_msg.target_accel << "\n"
        << "acc_valid " << is_acc_valid << "\n"
        << "hold_target_speed " << is_hold_target_speed << "\n"
        << "speed_setpoint " << can_comms_msg.speed_setpoint << "\n"
        << "aeb_valid " << is_aeb_valid << "\n"
        << "aeb_override " << is_aeb_override << "\n"
        << "lc_valid " << is_lc_valid << "\n"
        << "wheel_angle " << can_comms_msg.wheel_angle << "\n"
        << "alive_rolling_counter " << arc << "\n"
    );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "subs");
    ros::NodeHandle subs_handle;
  
    ros::Subscriber can_sub = subs_handle.subscribe("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE,
        can_callback);
    
    ros::spin();
}
