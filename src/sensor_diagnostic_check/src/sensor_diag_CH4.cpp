#include "sensor_diag.h"

SensorDiagnostics::SensorDiagnostics(ros::NodeHandle* diag_nodehandle) : diag_nodehandle(diag_nodehandle) {
    sub_CAN_data = diag_nodehandle.subscribe("sensor_diag_data", TX_RX_MESSAGE_BUFFER_SIZE, sub_CAN_data_callback);
    sub_CAN_flag = diag_nodehandle.subscribe("sensor_diag_flag", TX_RX_MESSAGE_BUFFER_SIZE, sub_CAN_flag_callback);

    //client_ch2 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2"); //locate srv file and specify topic
    //client_ch3 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3"); //locate srv file and specify topic
    client_ch4 = diag_nodehandle->serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4"); //locate srv file and specify topic
}

void sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& data_msg){
    // temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM

    //common::sensor_diagnostic_flag_CH2 srv_ch2; // service objects
    //common::sensor_diagnostic_flag_CH3 srv_ch3; // service objects
    common::sensor_diagnostic_flag_CH4 srv_ch4; // service objects

    //-- Mobileye Signals --//
    bool headway_valid = data_msg.headway_valid;  // CAN ID 1792
    bool maintenance = data_msg.maintenance;
    bool fail_safe = data_msg.failsafe;

    uint8_t quality = data_msg.quality;  // CAN ID 1894, 1896, 1900 -> 1916  

    // SAMPLE MOBILEYE DIAGNSOSTICS ALGORITHM

    if (srv_ch4.mobileye){     
        if (!headway_valid || maintenance || fail_safe || quality == 0 || quality == 1)
            srv_ch4.request.mobileye = false;    
    }
    else{ 
        if (headway_valid && !maintenance && !fail_safe && (quality == 2 || quality == 3))
            srv_ch4.request.mobileye = true;    
    }    

    /*bool troubleShootClient =
        client_ch2.call(srv_ch2);  // service request initiated (SENDING data through 'service request')
    bool troubleShootClient =
        client_ch3.call(srv_ch3);  // service request initiated (SENDING data through 'service request')*/
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
