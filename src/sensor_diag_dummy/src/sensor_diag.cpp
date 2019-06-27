#include "ecmc/GreetingMsg.h"
#include "ros/ros.h"

void greetingCallback(const ecmc::GreetingMsg& msg) {
  ROS_INFO_STREAM(msg.greeting << "\n"
                               << "Greetings sent: " << msg.num_greetings_sent);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "hello_world_sub");
  ros::NodeHandle nh;
  ros::Subscriber greeting_sub = nh.subscribe("Greetings", 1000, greetingCallback);

  ros::spin();
}
