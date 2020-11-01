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
            uint8_t calculated_crc =
                crc8bit_calculation(r_stat_itc_info, r_stat_hw_fail, r_stat_sgu_fail, r_stat_horizontal_misalignment,
                                    r_stat_absorption_blindness, r_stat_absorption_blindness, r_stat_itc_info);

            return ((radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
                                radar_mess_ender_consist_bit) &&
                            radar_tc_counter == tc_check && calculated_checksum == 0 && r_stat_itc_info == 0 &&
                            r_stat_sgu_fail == 0 && r_stat_hw_fail == 0 &&
                            abs(r_stat_horizontal_misalignment) < MISALIGNMENT_LIMIT &&
                            r_stat_absorption_blindness < BLINDNESS_LIMIT && r_stat_distortion_blindness < BLINDNESS_LIMIT &&
                            r_stat_mc == mc_check && r_stat_crc == calculated_crc);
        };
        bool validate_mobileye(const common::sensor_diagnostic_data_msg& data_msg){
            //-- Mobileye Signals --//
            bool headway_valid = data_msg.headway_valid;  // CAN ID 1792
            bool maintenance = data_msg.maintenance;
            bool failsafe = data_msg.failsafe;

            uint8_t quality_L = data_msg.quality_L; // CAN ID 1894, 1896, 1900 -> 1916
            uint8_t quality_R = data_msg.quality_R; // CAN ID 1894, 1896, 1900 -> 1916

            return (headway_valid && !maintenance && !failsafe && quality_L > 1 && quality_R > 1); 
        };
    private: 
        ros::NodeHandle* diag_nodehandle;
        
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

#endif