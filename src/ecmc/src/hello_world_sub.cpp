#include "ros/ros.h"
#include "ecmc/RawSensorObjectDataMsg.h"
 
void dataCallback(const ecmc::RawSensorObjectDataMsg& msg) {
    int radarNum = msg.radarNum;
    int numObjects = msg.numObjects;
    for(int i = 0; i < numObjects; i++){
        ROS_INFO_STREAM(msg.radarNum << "\n");
    }
}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_sub_node");
    ros::NodeHandle nh;
    ros::Subscriber data_sub = nh.subscribe("Greetings", 1000, dataCallback);  
  
    ros::spin();
}
