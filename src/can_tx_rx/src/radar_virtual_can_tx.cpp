#include "ros/ros.h"
#include <canlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <iomanip>
#include "radar.h"

using namespace std;

static const uint32_t pub_buffer_size = 100;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle test_data_pub_nh;
    
    struct radar_radar_diag_response_t diag;
    diag.r_diag_response = 1; 

    struct radar_radar_a_t radar_a; 
    radar_a.radar_dx = 200; 
    radar_a.radar_vx = 100;
    radar_a.radar_dy = 50; 
    radar_a.radar_ax = 10; 
    radar_a.radar_flag_valid = 1;
    radar_a.radar_mess_aconsist_bit = 1;
    
    struct radar_radar_b_t radar_b;
    radar_b.radar_vy = 50; 
    radar_b.radar_d_length = 10; 
    radar_b.radar_dz = 8;
    radar_b.radar_dx_sigma = 32;
    radar_b.radar_vx_sigma = 32;
    radar_b.radar_ax_sigma = 32;
    radar_b.radar_dy_sigma = 32; 
    radar_b.radar_mess_bconsist_bit = 1; 

    struct radar_radar_object_ender_t radar_end; 
    radar_end.radar_tc_counter = 1; 
    radar_end.radar_mess_ender_consist_bit = 1; 
    radar_end.radar_packet_checksum = 0; 

    struct radar_radar_object_starter_t radar_start;
    radar_start.radar_veh_v_ego = 20;
    radar_start.radar_mess_starter_consist_bit = 1; 

    std::cout << "Original Radar data is: \n" << std::endl;
    
    std::cout << "A Frame: " <<std::endl; 
    std::cout << "Longitudinal Position: " << radar_a.radar_dx << std::endl;
    std::cout << "Lateral Position: " << radar_a.radar_dy << std::endl;
    std::cout << "Longitudinal Velocity: " << radar_a.radar_vx << std::endl;
    std::cout << "Longitudinal Acceleration: " << radar_a.radar_ax << std::endl;
     
    std::cout << "\nB Frame: " << std::endl;
    std::cout << "Lateral Velocity: " << radar_b.radar_vy << std::endl; 
    std::cout << "Average Longitudinal Position: " << +radar_b.radar_dx_sigma <<std::endl; 
    std::cout << "Average Longitudinal Velocity: " << +radar_b.radar_vx_sigma <<std::endl; 
    std::cout << "Average Longitudinal Acceleration: " << +radar_b.radar_ax_sigma <<std::endl; 
    std::cout << "Average Lateral Distance: " << +radar_b.radar_dy_sigma <<std::endl; 

    diag.r_diag_response = radar_radar_diag_request_r_diag_request_encode(diag.r_diag_response);

    radar_a.radar_dx = radar_radar_a_radar_dx_encode(radar_a.radar_dx); 
    radar_a.radar_vx = radar_radar_a_radar_vx_encode(radar_a.radar_vx); 
    radar_a.radar_dy = radar_radar_a_radar_dy_encode(radar_a.radar_dy);  
    radar_a.radar_ax = radar_radar_a_radar_ax_encode(radar_a.radar_ax); 
    radar_a.radar_flag_valid = radar_radar_a_radar_flag_valid_encode(radar_a.radar_flag_valid);
    radar_a.radar_mess_aconsist_bit = radar_radar_a_radar_mess_aconsist_bit_encode(radar_a.radar_mess_aconsist_bit);
    
    radar_b.radar_vy = radar_radar_b_radar_vy_encode(radar_b.radar_vy); 
    radar_b.radar_d_length = radar_radar_b_radar_d_length_encode(radar_b.radar_d_length); 
    radar_b.radar_dz = radar_radar_b_radar_dz_encode(radar_b.radar_dz);
    radar_b.radar_dx_sigma = radar_radar_b_radar_dx_sigma_encode(radar_b.radar_dx_sigma);
    radar_b.radar_vx_sigma = radar_radar_b_radar_vx_sigma_encode(radar_b.radar_vx_sigma);
    radar_b.radar_ax_sigma = radar_radar_b_radar_ax_sigma_encode(radar_b.radar_ax_sigma);
    radar_b.radar_dy_sigma = radar_radar_b_radar_dy_sigma_encode(radar_b.radar_dy_sigma); 
    radar_b.radar_mess_bconsist_bit = radar_radar_b_radar_mess_bconsist_bit_encode(radar_b.radar_mess_bconsist_bit); 
    
    radar_end.radar_tc_counter = radar_radar_object_ender_radar_tc_counter_encode(radar_end.radar_tc_counter); 
    radar_end.radar_mess_ender_consist_bit = radar_radar_object_ender_radar_mess_ender_consist_bit_encode(radar_end.radar_mess_ender_consist_bit); 
    radar_end.radar_packet_checksum = radar_radar_object_ender_radar_packet_checksum_encode(radar_end.radar_packet_checksum); 
    
    const struct radar_radar_diag_response_t *radar_diag = &diag;

    const struct radar_radar_a_t *radar_a_frame = &radar_a;
    const struct radar_radar_b_t *radar_b_frame= &radar_b;  
    const struct radar_radar_object_ender_t *radar_obj_ender = &radar_end;
    const struct radar_radar_object_starter_t *radar_obj_starter = &radar_start;  

    canHandle hnd;
    canInitializeLibrary();
    
    canStatus stat; 
    
    hnd = canOpenChannel(1, canOPEN_ACCEPT_VIRTUAL);
    if (hnd < 0) {
        char msg[64];
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }

    canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);

    uint8_t radar_can_msg_a[8] = {0};
    uint8_t radar_can_msg_b[8] = {0};
    uint8_t radar_can_msg_diag[8] = {0};
    uint8_t radar_can_msg_start[8] = {0};
    uint8_t radar_can_msg_end[8] = {0};

    uint8_t blank_msg[8] = {0}; 
    size_t size = 8u;
    radar_radar_a_pack(radar_can_msg_a, radar_a_frame, size);
    radar_radar_b_pack(radar_can_msg_b, radar_b_frame, size);
    radar_radar_diag_response_pack(radar_can_msg_diag, radar_diag, size);
    radar_radar_object_starter_pack(radar_can_msg_start, radar_obj_starter, size);
    radar_radar_object_ender_pack(radar_can_msg_end, radar_obj_ender, size);

    int id = 1280;
    //goes through all valid id messages, sending the packed can message when appropriate  
    // starts at lower id messages above 1849, then goes back to sending appropriate messages
    while (ros::ok()) 
    {
        //radars are hard to test with specific ID's, using this to lock from a specific message
        int idtest = 1388;  
        if(id != idtest){
            id = idtest; 
        }
        
        canStatus stat; 
        if(id == 1280 || id == 1282){ //starters
            stat = canWrite(hnd,id, radar_can_msg_start, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id == 1958 || id == 1985){ //diag responses
            stat = canWrite(hnd,id, radar_can_msg_diag, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id == 1665 || id == 1667){ //enders
            stat = canWrite(hnd,id, radar_can_msg_end, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id >= 1285 && id <= 1657 && id % 2 ==1){ //Radar A Frame
            stat = canWrite(hnd,id, radar_can_msg_a, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id >= 1286 && id <= 1658 && id % 2 ==0){ //Radar B Frame
            stat = canWrite(hnd, id, radar_can_msg_b, 8, canOPEN_ACCEPT_VIRTUAL); 
        }
        else{ //incorrect frame 
            stat = canWrite(hnd, id, blank_msg, 8, canOPEN_ACCEPT_VIRTUAL); 
            std::cout << "Empty message sent" << std::endl; 
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

