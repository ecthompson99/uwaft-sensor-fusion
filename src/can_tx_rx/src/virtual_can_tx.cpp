#include "ros/ros.h"
#include <canlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"
using namespace std;

static const uint32_t pub_buffer_size = 100;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle test_data_pub_nh;
    
    struct ext_log_data_obstacle_data_a_t a;
    a.obstacle_pos_x = 240;
    a.obstacle_pos_y = 31;
    a.obstacle_vel_x = 100;
    std::cout << a.obstacle_pos_x << std::endl;
    std::cout << a.obstacle_pos_y << std::endl;
    std::cout << a.obstacle_vel_x << std::endl;

    a.obstacle_pos_x = ext_log_data_obstacle_data_a_obstacle_pos_x_encode(a.obstacle_pos_x);
    a.obstacle_pos_y = ext_log_data_obstacle_data_a_obstacle_pos_y_encode(a.obstacle_pos_y);
    a.obstacle_vel_x = ext_log_data_obstacle_data_a_obstacle_vel_x_encode(a.obstacle_vel_x);
    std::cout << a.obstacle_pos_x << std::endl;
    std::cout << a.obstacle_pos_y << std::endl;
    std::cout << a.obstacle_vel_x << std::endl;

    const struct ext_log_data_obstacle_data_a_t *frame_a = &a;

    canHandle hnd;
    canInitializeLibrary();
    hnd = canOpenChannel(0, canOPEN_EXCLUSIVE);
    if (hnd < 0) {
        char msg[64];
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }
    canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);

    // uint8_t can_msg[8] = {0};
    // size_t size = 8u;
    // int pack_return = ext_log_data_obstacle_data_a_pack(can_msg, frame_a, size);
    char *can_msg = "HELLO!";

    while (ros::ok()) 
    {
        canStatus stat = canWrite(hnd, 123, (void *)can_msg, 6, 0);
        canStatus queue_status = canWriteSync(hnd, 1000);
        //if (queue_status == canOK) std::cout << "Queue emptied" << std::endl;
        if (stat < 0)
        {
            std::cout << "Failed, status = " << stat << std::endl;
        }
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
    
    canBusOff(hnd);
    canClose(hnd);

}

