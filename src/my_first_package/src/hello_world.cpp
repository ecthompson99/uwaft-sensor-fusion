#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello_world");
  ros::NodeHandle nh;

  std::string greeting = "Hello";
  ROS_INFO_STREAM(greeting << ", worssssld!");
  ros::spin();
}
