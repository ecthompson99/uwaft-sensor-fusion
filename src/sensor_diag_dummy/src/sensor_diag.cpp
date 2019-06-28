#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticDataMsg.h"
#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros/ros.h"

void CANcallback(const sensor_diag_dummy::SensorDiagnosticDataMsg& message) {
  bool HardwareFail = message.HWfail;
  bool SGUFailure = message.SGUFail;
  int msgCounter = message.messageCounter;
  int msgCRC = message.messageCRC;

  ROS_INFO_STREAM("\n"
            << "starterConsistency " << message.starterConsistency << "\n"
            << "timeStamp " << message.timeStamp << "\n"
            << "enderConsistency " << message.enderConsistency << "\n"
            << "counter " << message.counter << "\n"
            << "checkSum " << message.checkSum << "\n"
            << "horizontalMisalign " << message.horizontalMisalign << "\n"
            << "absorbBlind " << message.absorbBlind << "\n"
            << "distortBlind " << message.distortBlind << "\n"
            << "ITCinfo " << message.ITCinfo << "\n"
            << "HWfail " << HardwareFail << "\n"
            << "SGUFail " << SGUFailure << "\n"
            << "messageCounter " << msgCounter << "\n"
            << "messageCRC " << msgCRC << "\n");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_diag");
  ros::NodeHandle sensor_diag_handle;

  ros::Subscriber sub = sensor_diag_handle.subscribe("CANmsg", 1000, CANcallback);

  ros:: Publisher pub = sensor_diag_handle.advertise<
                  sensor_diag_dummy::SensorDiagnosticFlagMsg>("ReliabilityMsg", 1000);

  sensor_diag_dummy::SensorDiagnosticFlagMsg radarMsg; 
  radarMsg.radarReliability = {0,5,15,100,200,255};

    while (ros::ok()) {  
    pub.publish(radarMsg);
    ros::spinOnce();
  }
}
