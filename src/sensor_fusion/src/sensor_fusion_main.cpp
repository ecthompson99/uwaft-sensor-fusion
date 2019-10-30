#include "sensor_fusion.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "sensor_fusion");
  ros::NodeHandle sensor_fusion_handle;
  SensorFusion sensor_fusion(&sensor_fusion_handle);

  while (ros::ok()) {
    sensor_fusion.publish_fused_data();
    ros::spinOnce();
  }

  return 0;
}
