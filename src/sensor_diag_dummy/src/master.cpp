#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros/ros.h"

void diagCallback(const sensor_diag_dummy::SensorDiagnosticFlagMsg& radarMsg) {
  ROS_INFO_STREAM("\n"  << "Radar " << radarMsg.radarReliability[5]);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "master");
  ros::NodeHandle master;
  ros::Subscriber sub = master.subscribe("ReliabilityMsg", 1000, diagCallback);

  ros::spin();
}
