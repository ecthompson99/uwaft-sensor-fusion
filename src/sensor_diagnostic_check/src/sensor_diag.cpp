#include <stdio.h>
#include <sstream>

//includes all headers necessary to use for ROS
#include "ros/ros.h"

//includes the header for messages this node receives and sends
#include "sensor_diag_dummy/sensor_diagnostic_data_msg.h"
#include "sensor_diag_dummy/sensor_diagnostic_flag_msg.h"

//ROS will buffer up a max of 1000 messages before beginning to throw away old ones.
int max_messages = 1000;

//callback function for the subscriber
void CAN_callback(const sensor_diag_dummy::sensor_diagnostic_data_msg& message) {

//temporary workaround for int and bool as the values in them do not get properly printed using ROS_INFO_STREAM  
  bool hardware_failure = message.hardware_fail;
  bool SGU_failure = message.SGU_fail;
  int msg_counter = message.message_counter;
  int msg_CRC = message.message_CRC;

//output and log every variable in the message from the topic this node subscribes too. Output messages will need to be worked on.
  ROS_INFO_STREAM("\n"
            << "starterConsistency " << message.starter_consistency << "\n" 
            << "timeStamp " << message.time_stamp << "\n"
            << "enderConsistency " << message.ender_consistency << "\n"
            << "counter " << message.counter << "\n"
            << "checkSum " << message.check_sum << "\n"
            << "horizontalMisalign " << message.horizontal_misalign << "\n"
            << "absorbBlind " << message.absorb_blind << "\n"
            << "distortBlind " << message.distort_blind << "\n"
            << "ITCinfo " << message.ITC_info << "\n"
            << "HWfail " << hardware_failure << "\n"
            << "SGUFail " << SGU_failure << "\n"
            << "messageCounter " << msg_counter << "\n"
            << "messageCRC " << msg_CRC << "\n");
}

int main(int argc, char** argv) {
  //initialize ROS and name the node
  ros::init(argc, argv, "sensor_diag");

  //create a handle to this process' node, initialization of the node
  ros::NodeHandle sensor_diag_handle;

  //sub to the topic CAN_TX_RX
  ros::Subscriber sensor_diag_sub = sensor_diag_handle.subscribe("CANmsg", max_messages, CAN_callback);

  //pub to topic Sensor Diagnostic Flag
  ros:: Publisher sensor_diag_pub = sensor_diag_handle.advertise<
                  sensor_diag_dummy::sensor_diagnostic_flag_msg>("ReliabilityMsg", max_messages);

  //define message object
  sensor_diag_dummy::sensor_diagnostic_flag_msg radar_msg; 

  //assign the array random values to test its interface. Will need to be worked on for the actual node.
  radar_msg.radar_reliability = {0,5,15,100,200,255};

    while (ros::ok()) {
    
    //send the messages to the topic
    sensor_diag_pub.publish(radar_msg);

    //call the callback function from the subscribing node(s)
    ros::spinOnce();
  }
}
