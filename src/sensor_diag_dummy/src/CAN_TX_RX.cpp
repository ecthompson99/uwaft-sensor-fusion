#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticDataMsg.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "CAN_TX_RX");
  ros::NodeHandle CAN_TX_RX_handle;

  ros::Publisher pub = CAN_TX_RX_handle.advertise<
                    sensor_diag_dummy::SensorDiagnosticDataMsg>("CANmsg", 1000);

  sensor_diag_dummy::SensorDiagnosticDataMsg message;

  message.starterConsistency = 16.9856;
  message.timeStamp = 12.4615212;
  message.enderConsistency = 2.56876;
  message.counter = 90.86517;
  message.checkSum = 891.21252363;
  message.horizontalMisalign = 1156.56448917;
  message.absorbBlind = 7.7894138;
  message.distortBlind = 168.1;
  message.ITCinfo = 15;
  message.HWfail = 1;
  message.SGUFail = 0;
  message.messageCounter = 5;
  message.messageCRC = 6;

  while (ros::ok()) {
    pub.publish(message);
    ros::spinOnce(); 
    ros::Duration(0.5).sleep();
  }
}