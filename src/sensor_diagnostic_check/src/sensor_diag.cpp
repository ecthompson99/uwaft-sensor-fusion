#include "sensor_diag.h"

SensorDiagnostics::SensorDiagnostics(ros::NodeHandle* diag_nodehandle) : diag_nodehandle(diag_nodehandle) {
    sub_CAN_data = diag_nodehandle.subscribe("sensor_diag_data", TX_RX_MESSAGE_BUFFER_SIZE, sub_CAN_data_callback);
    sub_CAN_flag = diag_nodehandle.subscribe("sensor_diag_flag", TX_RX_MESSAGE_BUFFER_SIZE, sub_CAN_flag_callback);
    
    client_ch2 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2"); //locate srv file and specify topic
    client_ch3 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3"); //locate srv file and specify topic
    client_ch4 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4"); //locate srv file and specify topic
}

void sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& data_msg){
    // temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM  

    common::sensor_diagnostic_flag_CH2 srv_ch2; // service objects
    common::sensor_diagnostic_flag_CH3 srv_ch3; // service objects
    common::sensor_diagnostic_flag_CH4 srv_ch4; // service objects

    //-- Radar Signals --//
    bool messStarterConistsBit = data_msg.messStarterConsistBit;  // CAN ID 1280 & 1282
    uint32_t veh_psiDt = data_msg.veh_psiDt;

    bool messAConsistBit = data_msg.messAConsistBit;  // CAN ID 1285 -> 1598 (depending on radar and object)
    bool messBConsistBit = data_msg.messBConsistsBit;

    bool messEnderConsistBit = data_msg.messEnderConsistBit;  // CAN ID 1665 & 1667
    uint8_t tcc_counter = data_msg.tcc_counter;
    uint8_t timestamp = data_msg.timestamp;
    // uint8_t packet_checksum = ?

    bool hw_fail = data_msg.hardware_fail;  // CAN ID 1670 & 1672
    bool sgu_fail = data_msg.sgu_fail;
    uint8_t abs_blindness = data_msg.abs_blindness;
    uint8_t dis_blindness = data_msg.dis_blindness;
    uint16_t itc_info = data_msg.itc_info;
    uint16_t hor_misalignment = data_msg.hor_misalignment;
    uint8_t msg_counter = data_msg.message_counter;
    uint32_t msg_crc = data_msg.message_crc;

    //-- Mobileye Signals --//
    bool headway_valid = data_msg.headway_valid;  // CAN ID 1792
    bool maintenance = data_msg.maintenance;
    bool fail_safe = data_msg.failsafe;

    uint8_t timestamp = data_msg.timestamp;  // CAN ID 1848

    uint8_t quality = data_msg.quality;  // CAN ID 1894, 1896, 1900 -> 1916

    // need to add radar tcc_counter, checksum, CRC, and msg counter logic/calculations

    if (!(messStarterConistsBit == messAConsistBit == messBConsistBit == messEnderConsistBit)) {
      srv_ch2.request.front_radar = 0;  // send srv request FAILED with no response needed.
      srv_ch3.request.left_corner_radar = 0;
      srv_ch3.request.right_corner_radar = 0;
    }

    if (veh_psiDt != 0 || timestamp >= 1 || hardware_failure || sgu_failure) {
      srv_ch2.request.front_radar = 0;  // send srv request FAILED with no response needed.
      srv_ch3.request.left_corner_radar = 0;
      srv_ch3.request.right_corner_radar = 0;
    }

    if (0.05 <= (abs_blindness || dis_blindness) < 0.1) {
      // insert Diagnostic level 1 serv request to sensor fusion
    } else if (abs_blindness || dis_blindness)
      >= 0.1() {
        srv_ch2.request.front_radar = 0;  // send srv request FAILED with no response needed.
        srv_ch3.request.left_corner_radar = 0;
        srv_ch3.request.right_corner_radar = 0;
        // insert Diagnostic Level 2 serv requet
      }

    if (itc_info != 0 || abs(hor_misalignment) > 0.0152) {
      srv_ch2.request.front_radar = 0;  // send srv request FAILED with no response needed.
      srv_ch3.request.left_corner_radar = 0;
      srv_ch3.request.right_corner_radar = 0;
    }

    // need to add mobileye timestamp logic
    if (!headway_valid || maintenance || fail_safe) {
      srv_ch4.request.mobileye = 0;
    }

    if (quality == 0 || quality == 1) {
      srv_ch4.request.mobileye = 0;
    }

    bool troubleShootClient =
        client_ch2.call(srv_ch2);  // service request initiated (SENDING data through 'service request')
    bool troubleShootClient =
        client_ch3.call(srv_ch3);  // service request initiated (SENDING data through 'service request')
    bool troubleShootClient =
        client_ch4.call(srv_ch4);  // service request initiated (SENDING data through 'service request')
    cout << troubleShootClient << endl;
}

void sub_CAN_flag_callback(const common::sensor_diagnostic_flag_msg& flag_msg){
    //nothing yet ?
}

int main(int argc, char** argv){
    ros::init(argc, argv, "sensor_diag");
    ros::NodeHandle diag_nodehandle;
    SensorDiagnostics sensDiag = SensorDiagnostics(&diag_nodehandle);
    
    ros::spin();
}
