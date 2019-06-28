#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/msg/SensorDiagnosticDataMsg.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "CAN_TX_RX");
  ros::NodeHandle CAN_TX_RX_handle;

  ros::Publisher pub = CAN_TX_RX_handle.advertise<
                    sensor_diag_dummy::SensorDiagnosticDataMsg>("CANmsg", 1000);

  ros::Rate rate(1);

  double starterConsistency, timeStamp, enderConsistency, counter,
              checkSum, horizontalMisalign, absorbBlind, distortBlind;
  uint16_t ITCinfo;
  bool HWfail, SGUFail;
  uint8_t messageCounter, messageCRC;

  starterConsistency = 16.9856;
  timeStamp = 12.4615212;
  enderConsistency = 2.56876;
  counter = 90.86517;
  checkSum = 891.21252363;
  horizontalMisalign = 1156.56448917;
  absorbBlind = 7.7894138;
  distortBlind = 16848918.1;

  ITCinfo = 1;
  
  HWfail = 0;
  SGUFail = 1;

  messageCounter = 15;
  messageCRC = 90;

  while (ros::ok()) {
    sensor_diag_dummy::SensorDiagnosticDataMsg message;

    message.starterConsistency = starterConsistency;
    message.timeStamp = timeStamp;
    message.enderConsistency = enderConsistency;
    message.counter = counter;
    message.checkSum = checkSum;
    message.horizontalMisalign = horizontalMisalign;
    message.absorbBlind = absorbBlind;
    message.distortBlind = distortBlind;

    pub.publish(message);
    rate.sleep();
    ros::spinOnce();
  }
}