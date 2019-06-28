#include <stdio.h>
#include <sstream>
#include "sensor_diag_dummy/SensorDiagnosticFlagMsg.h"
#include "ros/ros.h"

void diagCallback(const sensor_diag_dummy::SensorDiagnosticFlagMsg& radarMsg) {
  ROS_INFO_STREAM("\n"  << "Radar 1: %d " << radarMsg.radarReliability[1]
                  << "\n" << "Radar 2: %d " << radarMsg.radarReliability[1]
                  << "\n" << "Radar 3: %d " << radarMsg.radarReliability[2]
                  << "\n" << "Radar 4: %d " << radarMsg.radarReliability[3]
                  << "\n" << "Radar 5: %d " << radarMsg.radarReliability[4]
                  << "\n" << "Radar 6: %d " << radarMsg.radarReliability[5]
                  << "\n"
  );
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "master");
  ros::NodeHandle master;
  ros::Subscriber greeting_sub = master.subscribe("ReliabilityMsg", 1000, diagCallback);

  ros::spin();
}
