#include "ros/ros.h"
#include "ecmc/RawSensorObjectData.h" // Where is this
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "hello_world_pub");
    ros::NodeHandle nh;
      
    ros::Publisher greeting_pub = nh.advertise<ecmc::RawSensorObjectData>("Greetings", 1000);  
  
    ecmc::RawSensorObjectData msg;
    msg.accel_x[0] = 20;
    msg.pos_y[0] = 20;
    int count = 1;
 
    // With each iteration change some values of the message (what delays the loop speed)
    while(ros::ok()) {
        msg.num_greetings_sent = count;
        greeting_pub.publish(msg);
        count++;
        ros::spinOnce();
    }
}
