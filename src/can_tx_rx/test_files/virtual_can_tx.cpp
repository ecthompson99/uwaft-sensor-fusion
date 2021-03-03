#include "ros/ros.h"
#include <canlib.h>
#include "cav_pcm_csw_struct.h"
#include "cav_pcm_csw.h"
#include <cstdint>

static const uint32_t pub_buffer_size = 100;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle test_data_pub_nh;

    common::can_comms_data_msg commsmsg;
    struct emc_pcm_cav_interface_pcm_to_cav_1_t pcm1;
    struct emc_pcm_cav_interface_pcm_to_cav_2_t pcm2;
    struct emc_pcm_cav_interface_pcm_to_cav_3_t pcm3;

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

    int id = 1072;
    int iter = 0;

    // create publisher to publish comms data
    ros::Publisher publish_comms = test_data_pub_nh.advertise<common::can_comms_data_msg>("can_comms_data", MESSAGE_BUFFER_SIZE);

    while (ros::ok()) 
    {
        // mock data for virtual testing purposes
        if (iter > 5)
            iter = 0;

        // mock comms message from master task
        commsmsg.long_accel = 5*iter;
        commsmsg.lcc_steer = 2*iter;
        commsmsg.acc_valid = 1;
        commsmsg.aeb_valid = 1;
        commsmsg.lcc_valid = 1;
        commsmsg.acc_fault = 0;
        commsmsg.aeb_fault = 0;
        commsmsg.lcc_fault = 0;
        commsmsg.front_radar_fault = 0;
        commsmsg.left_radar_fault = 0;
        commsmsg.right_radar_fault = 0;
        commsmsg.mobileye_fault = 0;
        commsmsg.alive_rolling_counter = iter;

        // Publish comms data
        publish_comms.publish(commsmsg);
        // std::cout << "comms message sent! ";

        // mock PCM data to send to CAVs
        pcm1.pcm_to_cav1_rc = iter;
        pcm1.pcm_lcc_allowed = 1;
        pcm1.pcm_acc_allowed = 1;
        pcm1.pcm_aeb_allowed = 1;
        pcm1.pcm_veh_spd = 5*iter;
        pcm1.pcm_str_ang = 10*iter;
        pcm1.pcm_hsc_alive = 1;
        pcm2.pcm_to_cav2_rc = iter;
        pcm3.pcm_to_cav3_rc = iter;
    
        // encode mock PCM data to send to CAV channel 1 ROS node
        pcm1.pcm_to_cav1_rc = emc_pcm_cav_interface_pcm_to_cav_1_pcm_to_cav1_rc_encode(pcm1.pcm_to_cav1_rc);
        pcm1.pcm_lcc_allowed = emc_pcm_cav_interface_pcm_to_cav_1_pcm_lcc_allowed_encode(pcm1.pcm_lcc_allowed);
        pcm1.pcm_acc_allowed = emc_pcm_cav_interface_pcm_to_cav_1_pcm_acc_allowed_encode(pcm1.pcm_acc_allowed);
        pcm1.pcm_aeb_allowed = emc_pcm_cav_interface_pcm_to_cav_1_pcm_aeb_allowed_encode(pcm1.pcm_aeb_allowed);
        pcm1.pcm_veh_spd = emc_pcm_cav_interface_pcm_to_cav_1_pcm_veh_spd_encode(pcm1.pcm_veh_spd);
        pcm1.pcm_str_ang = emc_pcm_cav_interface_pcm_to_cav_1_pcm_str_ang_encode(pcm1.pcm_str_ang);
        pcm1.pcm_hsc_alive = emc_pcm_cav_interface_pcm_to_cav_1_pcm_hsc_alive_encode(pcm1.pcm_hsc_alive);
        pcm2.pcm_to_cav2_rc = emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_encode(pcm2.pcm_to_cav2_rc);
        pcm3.pcm_to_cav3_rc = emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_encode(pcm3.pcm_to_cav3_rc);

        struct emc_pcm_cav_interface_pcm_to_cav_3_t *pcm_to_cav_3 = &pcm3;
        struct emc_pcm_cav_interface_pcm_to_cav_2_t *pcm_to_cav_2 = &pcm2;
        struct emc_pcm_cav_interface_pcm_to_cav_1_t *pcm_to_cav_1 = &pcm1;

        uint8_t pcm_to_cav_msg_1[8] = {0};
        uint8_t pcm_to_cav_msg_2[8] = {0};
        uint8_t pcm_to_cav_msg_3[8] = {0};
        uint8_t blank_msg[8] = {0}; 

        size_t size = 8u;

        emc_pcm_cav_interface_pcm_to_cav_3_pack(pcm_to_cav_msg_3, pcm_to_cav_3, size);
        emc_pcm_cav_interface_pcm_to_cav_2_pack(pcm_to_cav_msg_2, pcm_to_cav_2, size);
        emc_pcm_cav_interface_pcm_to_cav_1_pack(pcm_to_cav_msg_1, pcm_to_cav_1, size);

        if(id > 1074){
            id = 1072;
        }
        
        canStatus stat; 
        if(id == 1072){ // 1st pcm to cav message
            stat = canWrite(hnd,id, pcm_to_cav_msg_1, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id == 1073){ //2nd cav to pcm message
            stat = canWrite(hnd,id, pcm_to_cav_msg_2, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else if(id ==1074){ //3rd cav to pcm message
            stat = canWrite(hnd,id, pcm_to_cav_msg_3, 8, canOPEN_ACCEPT_VIRTUAL);
        }
        else{ //incorrect frame 
            stat = canWrite(hnd, id, blank_msg, 8, canOPEN_ACCEPT_VIRTUAL); 
            std::cout << "Empty message sent" << std::endl; 
        }
        if (stat < 0)
        {
            std::cout << "Failed, status = " << stat << std::endl;
        }
        
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        std::cout << "id = " << id << std::endl; 
        id++; 
        iter++;
    }
    
    canBusOff(hnd);
    canClose(hnd);
}

