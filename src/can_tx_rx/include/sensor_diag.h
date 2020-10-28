#ifndef __SENSOR_DIAG_H__
#define __SENSOR_DIAG_H__

#include <stdio.h>
#include <sstream>
#include "ros/ros.h"
#include <cstdlib>
// subscriber includes
#include "common/sensor_diagnostic_data_msg.h"

// srv files
#include "common/sensor_diagnostic_flag_CH2.h"  
#include "common/sensor_diagnostic_flag_CH3.h"  
#include "common/sensor_diagnostic_flag_CH4.h"

#define TX_RX_MESSAGE_BUFFER_SIZE 1000

// CAN message constants

const double BLINDNESS_LIMIT = 0.1;
const double MISALIGNMENT_LIMIT = 0.0152;

class SensorDiagnostics {
    public: 
        SensorDiagnostics(ros::NodeHandle* diag_nodehandle) : diag_nodehandle(diag_nodehandle) {
        //    sub_CAN_data = diag_nodehandle->subscribe("SENSOR_DIAGNOSTICS_DATA", TX_RX_MESSAGE_BUFFER_SIZE, &SensorDiagnostics::sub_CAN_data_callback, this);

            client_ch2 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2"); //locate srv file and specify topic
            client_ch3 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3"); //locate srv file and specify topic
            client_ch4 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4"); //locate srv file and specify topic
        };
        
        ros::ServiceClient client_ch2;
        ros::ServiceClient client_ch3;
        ros::ServiceClient client_ch4;

        bool validate_radar(const common::sensor_diagnostic_data_msg& data_msg){
            //-- Radar Signals --//
                // timestamp for R or M not included since global ROS clock will be used
            uint8_t radar_mess_starter_consist_bit = data_msg.radar_mess_starter_consist_bit;  // CAN ID 1280 & 1282
            uint8_t radar_mess_aconsist_bit = data_msg.radar_mess_aconsist_bit;  // CAN ID 1285 -> 1598 (depending on radar and object)
            uint8_t radar_mess_bconsist_bit = data_msg.radar_mess_bconsist_bit;
            uint8_t radar_mess_ender_consist_bit = data_msg.radar_mess_ender_consist_bit;  // CAN ID 1665 & 1667
            uint8_t radar_tc_counter = data_msg.radar_tc_counter;
            uint16_t r_stat_itc_info = data_msg.r_stat_itc_info; // CAN ID 1670 & 1672
            uint8_t r_stat_hw_fail = data_msg.r_stat_hw_fail;
            uint8_t r_stat_sgu_fail = data_msg.r_stat_sgu_fail;
            double r_stat_horizontal_misalignment = data_msg.r_stat_horizontal_misalignment;  // uint16_t
            double r_stat_absorption_blindness = data_msg.r_stat_absorption_blindness;        // uint8_t
            double r_stat_distortion_blindness = data_msg.r_stat_distortion_blindness;
            uint8_t r_stat_mc = data_msg.r_stat_mc;
            uint8_t r_stat_crc = data_msg.r_stat_crc;
            uint8_t tc_check = data_msg.tc_check;
            uint8_t mc_check = data_msg.mc_check;

            uint8_t calculated_checksum = data_msg.radar_packet_checksum;  // Calculated in CAN RX
            uint8_t calculated_crc = crc8bit_calculation(r_stat_itc_info, r_stat_hw_fail, r_stat_sgu_fail, 
                                                        r_stat_horizontal_misalignment,r_stat_absorption_blindness, 
                                                        r_stat_absorption_blindness, r_stat_itc_info);

            return ((radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit == radar_mess_ender_consist_bit) && 
                    radar_tc_counter == tc_check && calculated_checksum == 0 && r_stat_itc_info == 0 &&
                    r_stat_sgu_fail == 0 && r_stat_hw_fail == 0 &&
                    abs(r_stat_horizontal_misalignment) < MISALIGNMENT_LIMIT &&
                    r_stat_absorption_blindness < BLINDNESS_LIMIT && r_stat_distortion_blindness < BLINDNESS_LIMIT &&
                    r_stat_mc == mc_check && r_stat_crc == calculated_crc);
        };
        bool validate_mobileye(const common::sensor_diagnostic_data_msg& data_msg){
            //-- Mobileye Signals --//
            bool headway_valid = data_msg.me_headway_valid;  // CAN ID 1792
            bool maintenance = data_msg.me_maintenance;
            bool failsafe = data_msg.me_failsafe;
            bool flag = false;

            // Quality CAN ID 1894, 1896, 1900 -> 1916

            for (int i = 0; i < 9; i++){
                if (headway_valid && !maintenance && !failsafe && data_msg.me_quality_L[i] > 1 && data_msg.me_quality_R[i] > 1){
                    flag == true;
                }
            }
            return flag; 
        };

    private: 
        ros::NodeHandle* diag_nodehandle;
    //    ros::Subscriber sub_CAN_data;
    //    ros::Subscriber sub_CAN_flag;
        
        uint8_t crc8bit_calculation(uint8_t itc, uint8_t hw, uint8_t sgu, uint8_t hor_misalignment, uint8_t absorption,
                            uint8_t distortion, uint8_t mc) {
            uint8_t crc = 0xFF;

            uint8_t can1670signals[7] = {itc, hw, sgu, hor_misalignment, absorption, distortion, mc};

            for (int index = 0; index < 6; index++) {
                crc ^= can1670signals[index];  // Assign data to CRC

                for (int bitIndex = 0; bitIndex < 8; bitIndex++) {  // LOop through 8 bits
                if ((crc & 0x80 != 0)) {
                    crc = (crc << 1);
                    crc ^= 0x1D;
                } else {
                    crc = (crc << 1);
                }
                }
            }

            return crc;
          };
        };

        // void sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& data_msg){
        //     // temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM

        //     common::sensor_diagnostic_flag_CH2 srv_ch2; // service objects
        //     common::sensor_diagnostic_flag_CH3 srv_ch3; // service objects
        //     common::sensor_diagnostic_flag_CH4 srv_ch4; // service objects

        //     uint8_t frnt_prev_tc_counter = 0; // Front Radar counter trackers
        //     uint8_t frnt_prev_mc = 0;

        //     uint8_t left_prev_tc_counter = 0; // Left Radar counter trackers
        //     uint8_t left_prev_mc = 0;

        //     uint8_t rght_prev_tc_counter = 0; // Right Radar counter trackers
        //     uint8_t rght_prev_mc = 0;

        //     uint8_t channel_number = data_msg.channel_number;
        //     uint8_t radar_number = data_msg.radar_number;

        //     uint8_t radar_tc_counter = data_msg.radar_tc_counter;
        //     uint8_t r_stat_mc = data_msg.r_stat_mc;

        //     uint8_t tc_check = 0;
        //     uint8_t mc_check = 0;

        //     switch (channel_number){
        //     case 2:  // Front Radar
        //         // Account for Radar Counter Reset
        //         if (!(frnt_prev_tc_counter + 0x1 == 256)) {
        //         tc_check = frnt_prev_tc_counter + 0x1;
        //         }

        //         if (!(frnt_prev_mc + 0x1 == 16)) {
        //         mc_check = frnt_prev_mc + 0x1;
        //         }
                
        //         srv_ch2.request.front_radar = validate_radar(data_msg); 

        //         if(srv_ch2.request.front_radar){
        //         std::cout << "Valid Ch2" << std::endl;
        //         }
        //         else{
        //         std::cout << "Invalid Ch2" << std::endl;
        //         }

        //         if (!client_ch2.call(srv_ch2)) {  // CLIENT.CALL ACTUALLY INITIATES THE SERVICE CALL
        //         std::cout << "SERVICE REQUEST CHANNEL 2 FRONT RADAR FAILED" << std::endl;
        //         }

        //         frnt_prev_tc_counter = radar_tc_counter;
        //         frnt_prev_mc = r_stat_mc;

        //         break;
        //     case 3:  // Corner Radars
        //         switch (radar_number) {
        //         case 1:  // Left Radar
        //             // Account for Radar counter reset
        //             if (!(left_prev_tc_counter + 0x1 == 256)) {
        //             tc_check = left_prev_tc_counter + 0x1;
        //             }

        //             if (!(left_prev_mc + 0x1 == 16)) {
        //             mc_check = left_prev_mc + 0x1;
        //             }

        //             srv_ch3.request.left_corner_radar = validate_radar(data_msg); 

        //             if(srv_ch3.request.left_corner_radar){
        //             std::cout << "Valid Ch3 Left" << std::endl;
        //             }
        //             else{
        //             std::cout << "Invalid Ch3 Left" << std::endl;
        //             }

        //             if (!client_ch3.call(srv_ch3)) {  // CLIENT.CALL ACTUALLY INITIATES THE SERVICE CALL
        //             std::cout << "SERVICE REQUEST CHANNEL 3 SIDE RADARS FAILED" << std::endl;
        //             }

        //             left_prev_tc_counter = radar_tc_counter;
        //             left_prev_mc = r_stat_mc;

        //             break;
        //         case 2:  // Right Radar
        //             // Account for Radar counter reset
        //             if (!(rght_prev_tc_counter + 0x1 == 256)) {
        //             tc_check = rght_prev_tc_counter + 0x1;
        //             }

        //             if (!(rght_prev_mc + 0x1 == 16)) {
        //             mc_check = rght_prev_mc + 0x1;
        //             }

        //             srv_ch3.request.right_corner_radar = validate_radar(data_msg); 

        //             if(srv_ch3.request.right_corner_radar){
        //             std::cout << "Valid Ch3 Right" << std::endl;
        //             }
        //             else{
        //             std::cout << "Invalid Ch3 Right" << std::endl;
        //             }

        //             if (!client_ch3.call(srv_ch3)) {  // CLIENT.CALL ACTUALLY INITIATES THE SERVICE CALL
        //             std::cout << "SERVICE REQUEST CHANNEL 3 SIDE RADARS FAILED" << std::endl;
        //             }

        //             rght_prev_tc_counter = radar_tc_counter;
        //             rght_prev_mc = r_stat_mc;

        //             break;
        //         }
        //         break;
        //     case 4:  // Mobileye
        //         srv_ch4.request.mobileye = validate_mobileye(data_msg); 

        //         if(srv_ch4.request.mobileye){
        //         std::cout << "Valid Ch4" << std::endl;
        //         }
        //         else{
        //         std::cout << "Invalid Ch4" << std::endl;
        //         }

        //         if (!client_ch4.call(srv_ch4)) {
        //         std::cout << "SERVICE REQUEST CHANNEL 4 MOBILEYE FAILED" << std::endl;
        //         }
        //         break;
        //     }

        //     std::cout << "AFTER switch cases" << std::endl;
        //     return;
        // };
      



// sub to cantxrx dummy node (later a function)
// srv request to mastertask when something is wrong - look in common for srv files

#endif