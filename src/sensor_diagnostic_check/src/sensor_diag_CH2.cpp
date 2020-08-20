#include "sensor_diag.h"

SensorDiagnostics::SensorDiagnostics(ros::NodeHandle* diag_nodehandle) : diag_nodehandle(diag_nodehandle) {
    sub_CAN_data = diag_nodehandle.subscribe("sensor_diag_data", TX_RX_MESSAGE_BUFFER_SIZE, sub_CAN_data_callback);
    sub_CAN_flag = diag_nodehandle.subscribe("sensor_diag_flag", TX_RX_MESSAGE_BUFFER_SIZE, sub_CAN_flag_callback);

    client_ch2 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2"); //locate srv file and specify topic
    client_ch3 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3"); //locate srv file and specify topic
    client_ch4 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4"); //locate srv file and specify topic
}

uint8_t prev_tc_counter = 0; 
uint8_t prev_msg_counter = 0;

void sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& data_msg){
    // temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM

    common::sensor_diagnostic_flag_CH2 srv_ch2; // service objects
    //common::sensor_diagnostic_flag_CH3 srv_ch3; // service objects
    //common::sensor_diagnostic_flag_CH4 srv_ch4; // service objects

    //-- Radar Signals --//
    // timestamp for R or M not included since global ROS clock will be used

    bool messStarterConistsBit = data_msg.messStarterConsistBit;  // CAN ID 1280 & 1282
    uint32_t veh_psiDt = data_msg.veh_psiDt;

    bool messAConsistBit = data_msg.messAConsistBit;  // CAN ID 1285 -> 1598 (depending on radar and object)
    bool messBConsistBit = data_msg.messBConsistsBit;

    bool messEnderConsistBit = data_msg.messEnderConsistBit;  // CAN ID 1665 & 1667
    uint8_t tc_counter = data_msg.tc_counter;
    uint8_t packet_checksum = data_msg.packet_checksum;    

    bool hw_fail = data_msg.hardware_fail; // CAN ID 1670 & 1672
    bool sgu_fail = data_msg.sgu_fail;
    uint8_t abs_blindness = data_msg.abs_blindness;
    uint8_t dis_blindness = data_msg.dis_blindness;
    uint16_t itc_info = data_msg.itc_info;
    uint16_t hor_misalignment = data_msg.hor_misalignment;
    uint8_t msg_counter = data_msg.message_counter;
    uint32_t msg_crc = data_msg.message_crc;

    uint32_t calculated_crc;
    uint8_t calculated_checksum;

    // Checksum Calculaton

    // CRC calculation'
    
    // SAMPLE RADAR DIAGNOSTICS ALGORITHM

    if (srv_ch2.front_radar){
        if (!(messStarterConistsBit == messAConsistBit == messBConsistBit = messEnderConsistBit) 
            || veh_psiDt !=0 || hw_fail || sgu_fail || abs_blindness >= 0.1 || dis_blindness >= 0.1
            || itc_info != 0 || abs(hor_misalignment) > 0.0152 || calculated_checksum != 0 || calculated_crc != msg_crc
            || tc_counter != ++prev_tc_counter || msg_counter != ++prev_msg_counter)

            srv_ch2.request.front_radar = false;
    }
    else{
        if ((messStarterConistsBit == messAConsistBit == messBConsistBit == messEnderConsistBit) 
            && veh_psiDt == 0 && !hw_fail && !sgu_fail && (abs_blindness && dis_blindness) < 0.1 
            && itc_info == 0 && abs(hor_misalignment) < 0.0152 && calculated_checksum == 0 && calculated_crc == msg_crc
            && tc_counter == ++prev_tc_counter && msg_counter == ++prev_msg_counter)

            srv_ch2.request.front_radar = true;
    }

    bool troubleShootClient =
        client_ch2.call(srv_ch2);  // service request initiated (SENDING data through 'service request')
    /*bool troubleShootClient =
        client_ch3.call(srv_ch3);  // service request initiated (SENDING data through 'service request')
    bool troubleShootClient =
        client_ch4.call(srv_ch4);  // service request initiated (SENDING data through 'service request')*/
    cout << troubleShootClient << endl;

    prev_tc_counter = tc_counter; // update counters
    prev_msg_counter = msg_counter;
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
