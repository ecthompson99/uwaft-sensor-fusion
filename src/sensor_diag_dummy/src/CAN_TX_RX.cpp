#include "ecmc/GreetingMsg.h"
#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "hello_world_pub");
  ros::NodeHandle nh;

  ros::Publisher greeting_pub = nh.advertise<ecmc::GreetingMsg>("Greetings", 1000);

  ecmc::GreetingMsg msg;
  msg.greeting = "Hello World!";
  int count = 1;

  while (ros::ok()) {
    msg.num_greetings_sent = count;
    greeting_pub.publish(msg);
    count++;
    ros::spinOnce();
  }
}