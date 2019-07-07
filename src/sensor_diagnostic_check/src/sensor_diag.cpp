#include <stdio.h>
#include <sstream>

#include "ros/ros.h"

#include "sensor_diag_dummy/sensor_diagnostic_data_msg.h"
#include "sensor_diag_dummy/sensor_diagnostic_flag_msg.h"

static const uint8_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;

void can_msg_callback(const sensor_diag_dummy::sensor_diagnostic_data_msg& message) {

    //temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM  
    bool hardware_failure = message.hardware_fail;
    bool sgu_failure = message.sgu_fail;
    uint8_t msg_counter = message.message_counter;
    uint32_t msg_crc = message.message_crc;

    ROS_INFO_STREAM("\n"
        << "starterConsistency " << message.starter_consistency << "\n" 
        << "timeStamp " << message.time_stamp << "\n"
        << "enderConsistency " << message.ender_consistency << "\n"
        << "counter " << message.counter << "\n"
        << "checkSum " << message.check_sum << "\n"
        << "horizontalMisalign " << message.horizontal_misalign << "\n"
        << "absorbBlind " << message.absorb_blind << "\n"
        << "distortBlind " << message.distort_blind << "\n"
        << "ITCinfo " << message.itc_info << "\n"
        << "HWfail " << hardware_failure << "\n"
        << "SGUFail " << sgu_failure << "\n"
        << "messageCounter " << msg_counter << "\n"
        << "messageCRC " << msg_crc << "\n");
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "sensor_diag");
    ros::NodeHandle sensor_diag_handle;
  
    ros::Subscriber sensor_diag_sub = sensor_diag_handle.subscribe("sensor_diagnostic_data", TX_RX_MESSAGE_BUFFER_SIZE,
        can_msg_callback);
  
    ros:: Publisher sensor_diag_pub = sensor_diag_handle.advertise<
        sensor_diag_dummy::sensor_diagnostic_flag_msg>("sensor_diagnostic_flags", TX_RX_MESSAGE_BUFFER_SIZE);
  
    sensor_diag_dummy::sensor_diagnostic_flag_msg radar_msg; 
  
    //assign the array random values to test its interface. Will need to be worked on for the actual node.
    radar_msg.radar_reliability = {0,5,15,100,200,255};

    while (ros::ok()) {    
        sensor_diag_pub.publish(radar_msg);
        ros::spinOnce();
    }
}
