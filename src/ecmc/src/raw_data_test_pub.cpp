#include "ros/ros.h"

#include "ecmc/raw_sensor_object_data_msg.h"
 
// Constants
static const int PUB_BUFFER_SIZE = 100;

int main(int argc, char **argv)
{
    // Initialize ros and create node handle
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle test_data_pub_nh;
      
    // Create publisher
    ros::Publisher test_data_pub = test_data_pub_nh.advertise<ecmc::raw_sensor_object_data_msg>("raw_data_test", PUB_BUFFER_SIZE);  

    // Create single message
    ecmc::raw_sensor_object_data_msg msg;

    msg.radarNum = 5;
    msg.numObjects = 3;
    msg.accel_x = {1,2,3};
    msg.vel_x = {1,2,3};
    msg.pos_x = {1,2,3};
    msg.pos_y = {1,2,3};
    msg.existProb = {1,2,3};
    msg.valid = {1,1,1};
 
    // Publish 'msg' at each iteration
    while(ros::ok()) {
        test_data_pub.publish(msg);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }
}
