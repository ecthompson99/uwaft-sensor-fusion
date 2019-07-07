#include <stdio.h>
#include <sstream>

#include "ros/ros.h"

#include "master_task/drive_ctrl_input_msg.h"
#include "master_task/sensor_diagnostic_flag_msg.h"
#include "master_task/sudo_driver_input_msg.h"

static const uint8_t MASTER_MESSAGE_BUFFER_SIZE = 1000;

int main(int argc, char** argv) {
    ros::init(argc, argv, "pubs");
    ros::NodeHandle pubs_handle;
  
    ros::Publisher drive_ctrl_pub = pubs_handle.advertise<
        master_task::drive_ctrl_input_msg>("drive_control_input", MASTER_MESSAGE_BUFFER_SIZE);
    
    ros::Publisher sensor_diag_pub = pubs_handle.advertise<
        master_task::sensor_diagnostic_flag_msg>("sensor_diagnostic_flag", MASTER_MESSAGE_BUFFER_SIZE);
    
    ros::Publisher sudo_driver_pub = pubs_handle.advertise<
        master_task::sudo_driver_input_msg>("sudo_driver_input", MASTER_MESSAGE_BUFFER_SIZE);

    master_task::drive_ctrl_input_msg drive_input_msg; 
    master_task::sensor_diagnostic_flag_msg sensor_diag_msg; 
    master_task::sudo_driver_input_msg sudo_msg; 

    drive_input_msg.acc_enable = 0;
    drive_input_msg.aeb_enable = 0;
    drive_input_msg.lc_enable = 1;
    drive_input_msg.acc_speed_set_point = 12.65;
    drive_input_msg.acc_dist_set_point = 13.99;

    sensor_diag_msg.radar_reliability = {15,30,50,65,90,105};

    sudo_msg.target_accel = 95.123;
    sudo_msg.wheel_angle = 10.89;
    sudo_msg.aeb_override = 1;

    while (ros::ok()) {    
        drive_ctrl_pub.publish(drive_input_msg);
        sensor_diag_pub.publish(sensor_diag_msg);
        sudo_driver_pub.publish(sudo_msg);
        ros::spinOnce();
    }
}
