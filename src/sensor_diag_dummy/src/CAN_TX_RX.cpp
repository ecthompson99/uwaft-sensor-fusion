#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticDataMsg.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "CAN_TX_RX");
  ros::NodeHandle CAN_TX_RX_handle;

  ros::Publisher pub = CAN_TX_RX_handle.advertise<
                    sensor_diag_dummy::SensorDiagnosticDataMsg>("CANmsg", 1000);

  ros::Rate rate(1);

  while (ros::ok()) {
    sensor_diag_dummy::SensorDiagnosticDataMsg message;

    message.starterConsistency = 16.9856;
    message.timeStamp = 12.4615212;
    message.enderConsistency = 2.56876;
    message.counter = 90.86517;
    message.checkSum = 891.21252363;
    message.horizontalMisalign = 1156.56448917;
    message.absorbBlind = 7.7894138;
    message.distortBlind = 16848918.1;
    message.ITCinfo = 1;
  
    message.HWfail = false;
    message.SGUFail = false;

    message.messageCounter = 15;
    message.messageCRC = 90;

    pub.publish(message);
    rate.sleep();
    ros::spinOnce();
  }
}