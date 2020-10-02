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
        SensorDiagnostics(ros::NodeHandle* diag_nodehandle);    //constructor - pointer handle
        ros::ServiceClient client_ch2;
        ros::ServiceClient client_ch3;
        ros::ServiceClient client_ch4;
    private: 
        ros::NodeHandle* diag_nodehandle;
        ros::Subscriber sub_CAN_data;
        ros::Subscriber sub_CAN_flag;

        void sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& diag_data_msg);
        bool validate_radar(const common::sensor_diagnostic_data_msg& diag_data_msg,uint8_t tc_check, uint8_t mc_check);
        bool validate_mobileye(const common::sensor_diagnostic_data_msg& diag_data_msg);
};


// sub to cantxrx dummy node (later a function)
// srv request to mastertask when something is wrong - look in common for srv files

#endif