#include "sensor_diag.h"

SensorDiagnostics::SensorDiagnostics(ros::NodeHandle* diag_nodehandle) : diag_nodehandle(diag_nodehandle) {
    sub_CAN_data = diag_nodehandle->subscribe("sensor_diag_data", TX_RX_MESSAGE_BUFFER_SIZE, &SensorDiagnostics::sub_CAN_data_callback, this);

    client_ch2 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2"); //locate srv file and specify topic
    client_ch3 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3"); //locate srv file and specify topic
    client_ch4 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4"); //locate srv file and specify topic
}

uint8_t frnt_prev_tc_counter = 0; // Front Radar counter trackers
uint8_t frnt_prev_mc = 0;

uint8_t left_prev_tc_counter = 0; // Left Radar counter trackers
uint8_t left_prev_mc = 0;

uint8_t rght_prev_tc_counter = 0; // Right Radar counter trackers
uint8_t rght_prev_mc = 0;

/*uint8_t crc8bit_calculation(uint16_t itc, uint8_t hw, uint8_t sgu, uint16_t hor_misalignment, uint8_t abs, uint8_t
dis, uint8_t mc){
    uint8_t crc = 0xFF;

    uint8_t can1670signals[7] = {itc, hw, sgu, hor_misalignment, abs, dis, mc};

    for (int index = 0; index < 6; index++){
        crc ^= can1670signals[index]; // Assign data to CRC

        for (int bitIndex = 0; bitIndex < 8; bitIndex++){ // LOop through 8 bits
            if ((crc & 0x80 != 0)){
                crc = (crc<<1);
                crc ^= 0x1D;
            }
            else{
                crc = (crc<<1);
            }

        }

    }

    return crc;
}*/

/*Pseudo Code for CRC from documentation
uint8_t crc8bit_calculation(const uint8_t* crc_dataPtr, uint32_t crc_length, uint8_t crc_bytePos){
    unsigned char crc = 0xFF;

    for (int byteIndex = 0; byteIndex < crc_length; byteIndex++){ // Loop through length entire message incrementing by
byte
        if (byteIndex != crc_bytePos){ // Skip byte position of CRC

            crc ^= crc_dataPtr[byteIndex]; // Assign data to CRC

            for (int bitIndex = 0; bitIndex < 8; bitIndex++){ // Loop through 8 bits
               if ((crc & 0x80)!= 0){
                   crc = (crc<<1);
                   crc ^= 0x1D;
               }
               else{
                   crc = (crc<<1);
               }
            }
        }
    }
}*/

void SensorDiagnostics::sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& data_msg){
    // temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM

    common::sensor_diagnostic_flag_CH2 srv_ch2; // service objects
    common::sensor_diagnostic_flag_CH3 srv_ch3; // service objects
    common::sensor_diagnostic_flag_CH4 srv_ch4; // service objects

    uint8_t channel_number = data_msg.channel_number;
    uint8_t radar_number = data_msg.radar_number;

    //-- Radar Signals --//
    // timestamp for R or M not included since global ROS clock will be used

    uint8_t radar_mess_starter_consist_bit = data_msg.radar_mess_starter_consist_bit;  // CAN ID 1280 & 1282
    
    uint8_t radar_mess_aconsist_bit = data_msg.radar_mess_aconsist_bit;  // CAN ID 1285 -> 1598 (depending on radar and object)
    uint8_t radar_mess_bconsist_bit = data_msg.radar_mess_bconsist_bit;

    uint8_t radar_mess_ender_cosist_bit = data_msg.radar_mess_ender_cosist_bit;  // CAN ID 1665 & 1667
    uint8_t radar_tc_counter = data_msg.radar_tc_counter;
    uint8_t radar_packet_checksum = data_msg.radar_packet_checksum;

    uint16_t r_stat_itc_info = data_msg.r_stat_itc_info; // CAN ID 1670 & 1672
    uint8_t r_stat_hw_fail = data_msg.r_stat_hw_fail;
    uint8_t r_stat_sgu_fail = data_msg.r_stat_sgu_fail;
    double r_stat_horizontal_misalignment = data_msg.r_stat_horizontal_misalignment;  // uint16_t
    double r_stat_absorption_blindness = data_msg.r_stat_absorption_blindness;        // uint8_t
    double r_stat_distortion_blindness = data_msg.r_stat_distortion_blindness;
    uint8_t r_stat_mc = data_msg.r_stat_mc;
    uint8_t r_stat_crc = data_msg.r_stat_crc;

    // uint8_t calculated_crc = crc8bit_calculation(r_stat_itc_info, r_stat_hw_fail, r_stat_sgu_fail,
    // r_stat_horizontal_misalignment, r_stat_absorption_blindness, r_stat_absorption_blindness, r_stat_itc_info);

    uint8_t calculated_crc = 3;       // Verification Calculation
    uint8_t calculated_checksum = 0;  // data_msg.radar_calculated_checksum;

    //-- Mobileye Signals --//
    uint8_t headway_valid = data_msg.headway_valid; // CAN ID 1792
    uint8_t maintenance = data_msg.maintenance;
    uint8_t failsafe = data_msg.failsafe;

    uint8_t quality = data_msg.quality; // CAN ID 1894, 1896, 1900 -> 1916

    // Outputs to verify for testing
    /*std::cout << "BEFORE switch cases" << std::endl;
    std::cout << "Channel: " << +channel_number << std::endl;
    std::cout << "Radar: " << +radar_number << std::endl;
    std::cout << "St bit: " << +radar_mess_starter_consist_bit << std::endl;
    std::cout << "A bit: " << +radar_mess_aconsist_bit << std::endl;
    std::cout << "B bit: " << +radar_mess_bconsist_bit << std::endl;
    std::cout << "End bit: " << +radar_mess_ender_cosist_bit << std::endl;
    std::cout << "TC: " << +radar_tc_counter << std::endl;
    std::cout << "F Prev TC: " << +frnt_prev_tc_counter << std::endl;
    std::cout << "L Prev TC: " << +left_prev_tc_counter << std::endl;
    std::cout << "R Prev TC: " << +rght_prev_tc_counter << std::endl;
    std::cout << "Calc Check: " << +calculated_checksum << std::endl;
    std::cout << "ITC: " << +r_stat_itc_info << std::endl;
    std::cout << "sgu: " << +r_stat_sgu_fail << std::endl;
    std::cout << "hw: " << +r_stat_hw_fail << std::endl;
    std::cout << "hori mis: " << +r_stat_horizontal_misalignment << std::endl;
    std::cout << "dis: " << +r_stat_distortion_blindness << std::endl;
    std::cout << "abs: " << +r_stat_absorption_blindness << std::endl;
    std::cout << "MC: " << +r_stat_mc << std::endl;
    std::cout << "F Prev MC: " << +frnt_prev_mc << std::endl;
    std::cout << "L Prev MC: " << +left_prev_mc << std::endl;
    std::cout << "R Prev MC: " << +rght_prev_mc << std::endl;
    std::cout << "CRC: " << +r_stat_crc << std::endl;
    std::cout << "Calc CRC: " << +calculated_crc << std::endl;*/

    uint8_t tc_check = 0;
    uint8_t mc_check = 0;

    switch (channel_number){
      case 2:  // Front Radar
        // Account for Radar Counter Reset
        if (!(frnt_prev_tc_counter + 0x1 == 256)) {
          tc_check = frnt_prev_tc_counter + 0x1;
        }

        if (!(frnt_prev_mc + 0x1 == 16)) {
          mc_check = frnt_prev_mc + 0x1;
        }

        if (!(radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
              radar_mess_ender_cosist_bit) ||
            radar_tc_counter != tc_check || calculated_checksum != 0 || r_stat_itc_info != 0 || r_stat_sgu_fail != 0 ||
            r_stat_hw_fail != 0 || abs(r_stat_horizontal_misalignment) > 0.0152 || r_stat_absorption_blindness >= 0.1 ||
            r_stat_distortion_blindness >= 0.1 || r_stat_mc != mc_check || r_stat_crc != calculated_crc) {
          srv_ch2.request.front_radar = false;  // set to invalid
          std::cout << "Invalid Ch2" << std::endl;

        } else if ((radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
                    radar_mess_ender_cosist_bit) &&
                   radar_tc_counter == tc_check && calculated_checksum == 0 && r_stat_itc_info == 0 &&
                   r_stat_sgu_fail == 0 && r_stat_hw_fail == 0 && abs(r_stat_horizontal_misalignment) < 0.0152 &&
                   r_stat_absorption_blindness < 0.1 && r_stat_distortion_blindness < 0.1 && r_stat_mc == mc_check &&
                   r_stat_crc == calculated_crc) {
          srv_ch2.request.front_radar = true;  // set to valid
          std::cout << "Valid Ch2" << std::endl;
        }

        if (!client_ch2.call(srv_ch2)) {  // CLIENT.CALL ACTUALLY INITIATES THE SERVICE CALL
          std::cout << "SERVICE REQUEST CHANNEL 2 FRONT RADAR FAILED" << std::endl;
        }

        frnt_prev_tc_counter = radar_tc_counter;
        frnt_prev_mc = r_stat_mc;

        break;
      case 3:  // Corner Radars
        switch (radar_number) {
          case 1:  // Left Radar
            // Account for Radar counter reset
            if (!(left_prev_tc_counter + 0x1 == 256)) {
              tc_check = left_prev_tc_counter + 0x1;
            }

            if (!(left_prev_mc + 0x1 == 16)) {
              mc_check = left_prev_mc + 0x1;
            }

            if (!(radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
                  radar_mess_ender_cosist_bit) ||
                radar_tc_counter != tc_check || calculated_checksum != 0 || r_stat_itc_info != 0 ||
                r_stat_sgu_fail != 0 || r_stat_hw_fail || abs(r_stat_horizontal_misalignment) > 0.0152 ||
                r_stat_absorption_blindness >= 0.1 || r_stat_distortion_blindness >= 0.1 || r_stat_mc != mc_check ||
                r_stat_crc != calculated_crc) {
              srv_ch3.request.left_corner_radar = false;  // set to invalid
              std::cout << "Invalid Ch3 Left" << std::endl;

            } else if ((radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
                        radar_mess_ender_cosist_bit) &&
                       radar_tc_counter == tc_check && calculated_checksum == 0 && r_stat_itc_info == 0 &&
                       !r_stat_sgu_fail && !r_stat_hw_fail && abs(r_stat_horizontal_misalignment) < 0.0152 &&
                       r_stat_absorption_blindness < 0.1 && r_stat_distortion_blindness < 0.1 &&
                       r_stat_mc == mc_check && r_stat_crc == calculated_crc) {
              srv_ch3.request.left_corner_radar = true;  // set to valid
              std::cout << "Valid Ch3 Left" << std::endl;
            }

            if (!client_ch3.call(srv_ch3)) {  // CLIENT.CALL ACTUALLY INITIATES THE SERVICE CALL
              std::cout << "SERVICE REQUEST CHANNEL 3 SIDE RADARS FAILED" << std::endl;
            }

            left_prev_tc_counter = radar_tc_counter;
            left_prev_mc = r_stat_mc;

            break;
          case 2:  // Right Radar
            // Account for Radar counter reset
            if (!(rght_prev_tc_counter + 0x1 == 256)) {
              tc_check = rght_prev_tc_counter + 0x1;
            }

            if (!(rght_prev_mc + 0x1 == 16)) {
              mc_check = rght_prev_mc + 0x1;
            }

            if (!(radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
                  radar_mess_ender_cosist_bit) ||
                radar_tc_counter != tc_check || calculated_checksum != 0 || r_stat_itc_info != 0 || r_stat_sgu_fail ||
                r_stat_hw_fail || abs(r_stat_horizontal_misalignment) > 0.0152 || r_stat_absorption_blindness >= 0.1 ||
                r_stat_distortion_blindness >= 0.1 || r_stat_mc != mc_check || r_stat_crc != calculated_crc) {
              srv_ch3.request.right_corner_radar = false;  // set to invalid
              std::cout << "Invalid Ch3 Right" << std::endl;

            } else if ((radar_mess_starter_consist_bit == radar_mess_aconsist_bit == radar_mess_bconsist_bit ==
                        radar_mess_ender_cosist_bit) &&
                       radar_tc_counter == tc_check && calculated_checksum == 0 && r_stat_itc_info == 0 &&
                       !r_stat_sgu_fail && !r_stat_hw_fail && abs(r_stat_horizontal_misalignment) < 0.0152 &&
                       r_stat_absorption_blindness < 0.1 && r_stat_distortion_blindness < 0.1 &&
                       r_stat_mc == mc_check && r_stat_crc == calculated_crc) {
              srv_ch3.request.right_corner_radar = true;  // set to valid
              std::cout << "Valid Ch3 Right" << std::endl;
            }

            if (!client_ch3.call(srv_ch3)) {  // CLIENT.CALL ACTUALLY INITIATES THE SERVICE CALL
              std::cout << "SERVICE REQUEST CHANNEL 3 SIDE RADARS FAILED" << std::endl;
            }

            rght_prev_tc_counter = radar_tc_counter;
            rght_prev_mc = r_stat_mc;

            break;
        }
        break;
      case 4:  // Mobileye
        if (!headway_valid || maintenance || failsafe || quality <= 1) {
          srv_ch4.request.mobileye = false;
          std::cout << "Invalid Ch4" << std::endl;

        } else if (headway_valid && !maintenance && !failsafe && quality > 1) {
          srv_ch4.request.mobileye = true;
          std::cout << "Valid Ch4" << std::endl;
        }

        if (!client_ch4.call(srv_ch4)) {
          std::cout << "SERVICE REQUEST CHANNEL 4 REAR RADAR FAILED" << std::endl;
        }
        break;
    }

    std::cout << "AFTER switch cases" << std::endl;
    return;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_diag");
    ros::NodeHandle diag_nodehandle;
    SensorDiagnostics sensDiag = SensorDiagnostics(&diag_nodehandle);
    while (ros::ok()) {
        ros::spinOnce();
    }
}