#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros/ros.h"

void diagCallback(const sensor_diag_dummy::SensorDiagnosticFlagMsg& radarMsg) {
  ROS_INFO_STREAM("start" << "\n" << "Radar 1: " << radarMsg.radarReliability[0]
                  << "\n" << "Radar 2: " << radarMsg.radarReliability[1]
                  << "\n" << "Radar 3: " << radarMsg.radarReliability[2]
                  << "\n" << "Radar 4: " << radarMsg.radarReliability[3]
                  << "\n" << "Radar 5: " << radarMsg.radarReliability[4]
                  << "\n" << "Radar 6: " << radarMsg.radarReliability[5]
  );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "master");
  ros::NodeHandle master;
  ros::Subscriber greeting_sub = master.subscribe("ReliabilityMsg", 1000, diagCallback);

  ros::spin();
}
