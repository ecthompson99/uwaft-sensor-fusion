#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticDataMsg.h"
//#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros/ros.h"

void CANcallback(const sensor_diag_dummy::SensorDiagnosticFlagMsg& message) {
  ROS_INFO_STREAM(
            << "starterConsistency %f" << message.starterConsistency << "\n"
            << "timeStamp %f" << message.timeStamp << "\n"
            << "enderConsistency %f" << message.enderConsistency << "\n"
            << "counter %f" << message.counter << "\n"
            << "checkSum %f" << message.checkSum << "\n"
            << "horizontalMisalign %f" << message.horizontalMisalign << "\n"
            << "absorbBlind %f" << message.absorbBlind << "\n"
            << "distortBlind %f" << message.distortBlind << "\n"
            << "ITCinfo %d" << message.ITCinfo << "\n"
            << "HWfail %d" << message.HWfail << "\n"
            << "SGUFail %d" << message.SGUFail << "\n"
            << "messageCounter %d" << message.messageCounter << "\n"
            << "messageCRC %d" << message.messageCRC << "\n");
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_diag");
  ros::NodeHandle sensor_diag_handle;

  ros::Subscriber sub = sensor_diag_handle.subscribe("CANmsg", 1000, CANcallback);

  ros::spin();
}
