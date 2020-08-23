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


class SensorDiagnostics {
    public: 
        SensorDiagnostics(ros::NodeHandle* diag_nodehandle);    //constructor - pointer handle
    private: 
        ros::NodeHandle* diag_nodehandle;
        ros::ServiceClient client_ch2;
        ros::ServiceClient client_ch3;
        ros::ServiceClient client_ch4;

        ros::Subscriber sub_CAN_data;
        ros::Subscriber sub_CAN_flag;

        void sub_CAN_data_callback(const common::sensor_diagnostic_data_msg& diag_data_msg);
        void sub_CAN_flag_callback(const common::sensor_diagnostic_flag_msg& diag_flag_msg);

};


// sub to cantxrx dummy node (later a function)
// srv request to mastertask when something is wrong - look in common for srv files

#endif