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

    std::cout << "Original data is: " << std::endl;
    std::cout << "Longitudinal Position: " << a.obstacle_pos_x << std::endl;
    std::cout << "Lateral Position: " << a.obstacle_pos_y << std::endl;
    std::cout << "Velocity: " << a.obstacle_vel_x << std::endl;

    a.obstacle_pos_x = ext_log_data_obstacle_data_a_obstacle_pos_x_encode(a.obstacle_pos_x);
    a.obstacle_pos_y = ext_log_data_obstacle_data_a_obstacle_pos_y_encode(a.obstacle_pos_y);
    a.obstacle_vel_x = ext_log_data_obstacle_data_a_obstacle_vel_x_encode(a.obstacle_vel_x);

    std::cout << "Data after encoding is: " << std::endl;
    std::cout << a.obstacle_pos_x << std::endl;
    std::cout << a.obstacle_pos_y << std::endl;
    std::cout << a.obstacle_vel_x << std::endl;

    const struct ext_log_data_obstacle_data_a_t *frame_a = &a;

    canHandle hnd;
    canInitializeLibrary();
    hnd = canOpenChannel(3, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        char msg[64];
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }

    canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);

    uint8_t can_msg[8] = {0};
    uint8_t blank_msg[8] = {0}; 
    size_t size = 8u;
    int pack_return = ext_log_data_obstacle_data_a_pack(&*can_msg, frame_a, size);
    
    int id = 1830;
    //goes through all valid id messages, sending the packed can message when appropriate  
    // starts at lower id messages above 1849, then goes back to sending appropriate messages
    while (ros::ok()) 
    {
         if(id > 1900){
             id = 1849; 
         }
        
        canStatus stat; 
        if(id%3==1){ //A Frame
            stat = canWrite(hnd, id, can_msg, 8, canOPEN_ACCEPT_VIRTUAL); 
        }
        else if (id%3==2){ //B Frame
            stat = canWrite(hnd, id, blank_msg, 8, canOPEN_ACCEPT_VIRTUAL); 
        }
        else if (id%3==0){//C Frame
            stat = canWrite(hnd, id, blank_msg, 8, canOPEN_ACCEPT_VIRTUAL); 
        }
        canStatus queue_status = canWriteSync(hnd, 1000);
        if (stat < 0)
        {
            std::cout << "Failed, status = " << stat << std::endl;
        }
        //if (queue_status == canOK) std::cout << "Queue emptied" << std::endl;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        std::cout << "id = " << id << std::endl; 
        id++; 
    }
    
    canBusOff(hnd);
    canClose(hnd);
}

