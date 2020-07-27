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
    
    bool hardware_failure = data_msg.hardware_fail;
    bool sgu_failure = data_msg.sgu_fail;
    uint8_t msg_counter = data_msg.message_counter;
    uint32_t msg_crc = data_msg.message_crc;
    
    //EXAMPLE!
    if(hardware_failure || sgu_failure){
        srv_ch2.request.front_radar = hardware_failure; // send srv request FAILED with no response needed.
        srv_ch3.request.left_corner_radar = 0;
        srv_ch3.request.right_corner_radar = 0;
        srv_ch4.request.mobileye = 0;

        bool troubleShootClient = client_ch2.call(srv_ch2); // service request initiated (SENDING data through 'service request')
        bool troubleShootClient = client_ch3.call(srv_ch3); // service request initiated (SENDING data through 'service request')
        bool troubleShootClient = client_ch4.call(srv_ch4); // service request initiated (SENDING data through 'service request')

        cout << troubleShootClient << endl;
    }
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
