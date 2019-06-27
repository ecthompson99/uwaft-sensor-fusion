#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticDataMsg.h"
//#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros/ros.h"

void CANcallback(const sensor_diag_dummy::SensorDiagnosticFlagMsg& message) {
  ROS_INFO_STREAM(<< "\n"
                  << "The number is %d" << message.messageCounter-1);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_diag");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("CANmsg", 1000, CANcallback);

  ros::spin();
}
