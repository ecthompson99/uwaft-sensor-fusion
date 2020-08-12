#include <canlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include "ros/ros.h"
#include "rosbag/bag.h"

#include "acc/ACC.cpp"
#include "acc/ACC.h"

#include "common/drive_ctrl_input_msg.h"
#include "common/target_output_msg.h"

using namespace std; 

int main(int argc, char **argv)
{
    ros::Time = time = ros::Time::now(); 
    common::drive_ctrl_input_msg drive_ctrl; 
    common::target_output_msg target_out; 

    drive_ctrl.acc_speed_set_point = 72; //72 km/h 
    drive_ctrl.acc_gap_level = 2; 

}
