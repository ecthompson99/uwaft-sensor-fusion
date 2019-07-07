#include <stdio.h>
#include <sstream>

#include "ros/ros.h"

#include "master_task/can_comms_data_msg.h"
#include "master_task/drive_ctrl_input_msg.h"
#include "master_task/sensor_diagnostic_flag_msg.h"
#include "master_task/sudo_driver_input_msg.h"

static const uint8_t MASTER_MESSAGE_BUFFER_SIZE = 1000;

void drive_ctrl_msg_callback(const master_task::drive_ctrl_input_msg& drive_ctrl_msg){

    bool is_acc_enabled = drive_ctrl_msg.acc_enable;
    bool is_aeb_enabled = drive_ctrl_msg.aeb_enable;
    bool is_lc_enabled = drive_ctrl_msg.lc_enable;

    ROS_INFO_STREAM("\n"
        << "ACC enabled? " << is_acc_enabled << "\n" 
        << "AEB enabled? " << is_aeb_enabled << "\n"
        << "LC enabled? " << is_lc_enabled << "\n"
        << "ACC speed set point " << drive_ctrl_msg.acc_speed_set_point << "\n"
        << "ACC dist set point " << drive_ctrl_msg.acc_dist_set_point << "\n"
    );
}

void sensor_diag_flag_msg_callback(const master_task::sensor_diagnostic_flag_msg& sensor_msg) {

    ROS_INFO_STREAM("\n"
        << "1st sensor " << sensor_msg.radar_reliability[0] << "\n" 
        << "2nd sensor " << sensor_msg.radar_reliability[1] << "\n" 
        << "3rd sensor " << sensor_msg.radar_reliability[2] << "\n" 
        << "4th sensor " << sensor_msg.radar_reliability[3] << "\n" 
        << "5th sensor " << sensor_msg.radar_reliability[4] << "\n" 
        << "6th sensor " << sensor_msg.radar_reliability[5] << "\n"       
    );          
}

void sudo_driver_input_msg_callback(const master_task::sudo_driver_input_msg& input_msg) {

    bool aeb_or = input_msg.aeb_override;

    ROS_INFO_STREAM("\n"
        << "AEB override " << aeb_or << "\n" 
        << "Target Acceleration " << input_msg.target_accel << "\n" 
        << "Wheel angle " << input_msg.wheel_angle << "\n" 
    );
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "master_task");
    ros::NodeHandle master_task_handle;
  
    ros::Subscriber drive_ctrl_sub = master_task_handle.subscribe("drive_control_input", MASTER_MESSAGE_BUFFER_SIZE,
        drive_ctrl_msg_callback);
    
    ros::Subscriber sensor_diag_flag_sub = master_task_handle.subscribe("sensor_diagnostic_flag", 
        MASTER_MESSAGE_BUFFER_SIZE, sensor_diag_flag_msg_callback);

    ros::Subscriber sudo_driver_input_sub = master_task_handle.subscribe("sudo_driver_input",MASTER_MESSAGE_BUFFER_SIZE,
        sudo_driver_input_msg_callback);
  
    ros::Publisher master_task_pub = master_task_handle.advertise<
        master_task::can_comms_data_msg>("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE);
  
    master_task::can_comms_data_msg can_comms_msg; 

    can_comms_msg.acc_switch_status = 1;
    can_comms_msg.aeb_switch_status = 1;
    can_comms_msg.lc_switch_status = 0;
    can_comms_msg.target_accel = 12.5;
    can_comms_msg.acc_valid = 0;
    can_comms_msg.hold_target_speed = 0;
    can_comms_msg.speed_setpoint = 151.45;
    can_comms_msg.aeb_valid = 0;
    can_comms_msg.aeb_override = 0;
    can_comms_msg.lc_valid = 0;
    can_comms_msg.wheel_angle = 16.51;
    can_comms_msg.alive_rolling_counter = 16;

    while (ros::ok()) {    
        master_task_pub.publish(can_comms_msg);
        ros::spinOnce();
    }
}
