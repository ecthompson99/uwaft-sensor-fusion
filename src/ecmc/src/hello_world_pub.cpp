#include "ros/ros.h"
#include "ecmc/RawSensorObjectDataMsg.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle nh;
      
    // Change name of topic from greetings
    ros::Publisher data_pub = nh.advertise<ecmc::RawSensorObjectDataMsg>("raw_data_test", 1000);  

    // Single message
    ecmc::RawSensorObjectDataMsg msg;

    msg.radarNum = 5;
    msg.numObjects = 3;
    msg.accel_x = {1,2,3};
    msg.vel_x = {1,2,3};
    msg.pos_x = {1,2,3};
    msg.pos_y = {1,2,3};
    msg.existProb = {1,2,3};
    msg.valid = {1,1,1};

    int count = 0;
 
    // With each iteration change some values of the message (what delays the loop speed)
    while(ros::ok()) {
        data_pub.publish(msg);
        count++;
        count = count%5;
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
}
