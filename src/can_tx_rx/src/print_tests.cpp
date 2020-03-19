#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/ext_log_data2_2_32.c"
#include "can_tx_rx/ext_log_data2_2_32.h"

#include "can_tx_rx/mobileye_object_data_msg.h"

void greetingCallback(const can_tx_rx::mobileye_object_data_msg& msg) {
    ROS_INFO_STREAM("Pos x: " << msg.me_dx << "\n");
    ROS_INFO_STREAM("Pos y: " << msg.me_dy << "\n");
    ROS_INFO_STREAM("Vel x: " << msg.me_vx << "\n");
    ROS_INFO_STREAM("Acc x: " << msg.me_ax << "\n");
    ROS_INFO_STREAM("Obj Id: " << uint8_t(msg.me_object_id) << "\n");
    ROS_INFO_STREAM("Obj lane: " << uint8_t(msg.me_object_lane) << "\n");
    ROS_INFO_STREAM("Time: " << msg.me_timestamp << "\n");
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_sub");
    ros::NodeHandle nh;
    ros::Subscriber greeting_sub = nh.subscribe("mobileye_object_data", 1000, greetingCallback);  
  
    ros::spin();
}