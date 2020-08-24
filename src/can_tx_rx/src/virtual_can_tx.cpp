#include "ros/ros.h"
#include <canlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
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
    a.obstacle_pos_y = 32;
    a.obstacle_vel_x = 100;

    struct ext_log_data_lka_left_lane_a_t left_a; 
    left_a.lane_type = 4;
    left_a.quality = 3; 
    left_a.position = 100; 
    left_a.curvature = 1; 
    left_a.curvature_derivative = 1; 
    
    struct ext_log_data_lka_left_lane_b_t left_b;
    left_b.heading_angle = 16; 

    struct ext_log_data_lka_right_lane_a_t right_a; 
    right_a.lane_type = 2; 
    right_a.quality = 0; 
    right_a.position = 50; 
    right_a.curvature = 1; 
    right_a.curvature_derivative = 1;

    struct ext_log_data_lka_right_lane_b_t right_b;
    right_b.heading_angle = 1;

    std::cout << setiosflags(ios::fixed) << std::setprecision(7); 

    std::cout << "Original data is: \n" << std::endl;
    
    std::cout << "Longitudinal Position: " << a.obstacle_pos_x << std::endl;
    std::cout << "Lateral Position: " << a.obstacle_pos_y << std::endl;
    std::cout << "Velocity: " << a.obstacle_vel_x << std::endl;
     
    std::cout << "\nLeft Lane: \n" << std::endl;

    std::cout << "Lane Type: " << left_a.lane_type+0 << std::endl;
    std::cout << "Quality: " << left_a.quality+0 << std::endl;
    std::cout << "Position: " << left_a.position << std::endl;
    std::cout << "Curvature: " << left_a.curvature << std::endl;
    std::cout << "Curvature Derivative: " << left_a.curvature_derivative << std::endl;
    std::cout << "Heading Angle: " << left_b.heading_angle << std::endl;   
    
    std::cout << "\nRight Lane: \n" << std::endl;

    std::cout << "Lane Type: " << right_a.lane_type+0 << std::endl;
    std::cout << "Quality: " << right_a.quality+0 << std::endl;
    std::cout << "Position: " << right_a.position << std::endl;
    std::cout << "Curvature: " << right_a.curvature << std::endl;
    std::cout << "Curvature Derivative: " << right_a.curvature_derivative << std::endl;
    std::cout << "Heading Angle: " << right_b.heading_angle << std::endl;       

    a.obstacle_pos_x = ext_log_data_obstacle_data_a_obstacle_pos_x_encode(a.obstacle_pos_x);
    a.obstacle_pos_y = ext_log_data_obstacle_data_a_obstacle_pos_y_encode(a.obstacle_pos_y);
    a.obstacle_vel_x = ext_log_data_obstacle_data_a_obstacle_vel_x_encode(a.obstacle_vel_x);
    
    left_a.lane_type = ext_log_data_lka_left_lane_a_lane_type_encode(left_a.lane_type); 
    left_a.quality = ext_log_data_lka_left_lane_a_quality_encode(left_a.quality); 
    left_a.position = ext_log_data_lka_left_lane_a_position_encode(left_a.position); 
    left_a.curvature = ext_log_data_lka_left_lane_a_curvature_encode(left_a.curvature);
    left_a.curvature_derivative = ext_log_data_lka_left_lane_a_curvature_derivative_encode(left_a.curvature_derivative);
    left_b.heading_angle = ext_log_data_lka_left_lane_b_heading_angle_encode(left_b.heading_angle);

    right_a.lane_type = ext_log_data_lka_right_lane_a_lane_type_encode(right_a.lane_type); 
    right_a.quality = ext_log_data_lka_right_lane_a_quality_encode(right_a.quality); 
    right_a.position = ext_log_data_lka_right_lane_a_position_encode(right_a.position); 
    right_a.curvature = ext_log_data_lka_right_lane_a_curvature_encode(right_a.curvature);
    right_a.curvature_derivative = ext_log_data_lka_right_lane_a_curvature_derivative_encode(right_a.curvature_derivative);
    right_b.heading_angle = ext_log_data_lka_right_lane_b_heading_angle_encode(right_b.heading_angle);

    std::cout << "\nData after encoding is: \n" << std::endl;
    std::cout << "Longitudinal Position: " << a.obstacle_pos_x << std::endl;
    std::cout << "Lateral Position: " << a.obstacle_pos_y << std::endl;
    std::cout << "Velocity: " << a.obstacle_vel_x << std::endl;
    
    std::cout << "\nLeft Lane: \n" << std::endl;
    std::cout << "Lane Type: " << left_a.lane_type+0 << std::endl;
    std::cout << "Quality: " << left_a.quality+0 << std::endl;
    std::cout << "Position: " << left_a.position << std::endl;
    std::cout << "Curvature: " << left_a.curvature << std::endl;
    std::cout << "Curvature Derivative: " << left_a.curvature_derivative << std::endl;
    std::cout << "Heading Angle: " << left_b.heading_angle << std::endl;   
    
    std::cout << "\nRight Lane: \n" << std::endl;
    std::cout << "Lane Type: " << right_a.lane_type+0 << std::endl;
    std::cout << "Quality: " << right_a.quality+0 << std::endl;
    std::cout << "Position: " << right_a.position << std::endl;
    std::cout << "Curvature: " << right_a.curvature << std::endl;
    std::cout << "Curvature Derivative: " << right_a.curvature_derivative << std::endl;
    std::cout << "Heading Angle: " << right_b.heading_angle << std::endl;  
    
    std::cout << "\n" << std::endl; 

    const struct ext_log_data_obstacle_data_a_t *frame_a = &a;

    const struct ext_log_data_lka_left_lane_a_t *frame_left_a = &left_a;
    const struct ext_log_data_lka_left_lane_b_t *frame_left_b = &left_b;  
    const struct ext_log_data_lka_right_lane_a_t *frame_right_a = &right_a;
    const struct ext_log_data_lka_right_lane_b_t *frame_right_b = &right_b;  

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

    uint8_t can_msg_a[8] = {0};
    uint8_t can_msg_left_a[8] = {0};
    uint8_t can_msg_left_b[8] = {0};
    uint8_t can_msg_right_a[8] = {0};
    uint8_t can_msg_right_b[8] = {0};

    uint8_t blank_msg[8] = {0}; 
    size_t size = 8u;
    /*int pack_return_a = */ ext_log_data_obstacle_data_a_pack(can_msg_a, frame_a, size);
    /*int pack_return_left_a = */ ext_log_data_lka_left_lane_a_pack(can_msg_left_a, frame_left_a, size);
    /*int pack_return_left_b = */ ext_log_data_lka_left_lane_b_pack(can_msg_left_b, frame_left_b, size);
    /*int pack_return_right_a = */ ext_log_data_lka_right_lane_a_pack(can_msg_right_a, frame_right_a, size);
    /*int pack_return_right_b = */ ext_log_data_lka_right_lane_b_pack(can_msg_right_b, frame_right_b, size);

    int id = 1830;
    //goes through all valid id messages, sending the packed can message when appropriate  
    // starts at lower id messages above 1849, then goes back to sending appropriate messages
    while (ros::ok()) 
    {
         
         if(id > 1900){
             id = 1849;
        }
        
        canStatus stat; 
        if(id ==1894){ //LKA Left A 
            stat = canWrite(hnd,id, can_msg_left_a, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id==1895){ //LKA Left B
            stat = canWrite(hnd,id, can_msg_left_b, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id ==1896){ //LKA Right A 
            stat = canWrite(hnd,id, can_msg_right_a, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id==1897){ //LKA Right B
            stat = canWrite(hnd,id, can_msg_right_b, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id%3==1){ //A Frame
            stat = canWrite(hnd, id, can_msg_a, 8, canOPEN_ACCEPT_VIRTUAL); 
        }
        else{ //B or C Frame
            stat = canWrite(hnd, id, blank_msg, 8, canOPEN_ACCEPT_VIRTUAL); 
        }
        if (stat < 0)
        {
            std::cout << "Failed, status = " << stat << std::endl;
        }
        // canStatus queue_status = canWriteSync(hnd, 1000);
        // if (queue_status == canOK) std::cout << "Queue emptied" << std::endl;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        std::cout << "id = " << id << std::endl; 
        id++; 
    }
    
    canBusOff(hnd);
    canClose(hnd);
}

