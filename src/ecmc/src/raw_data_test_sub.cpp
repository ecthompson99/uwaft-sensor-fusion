#include "ros/ros.h"

#include "ecmc/raw_sensor_object_data_msg.h"
 
// Constants
const int SUB_BUFFER_SIZE = 100;

// Callback for when raw test data is received from topic
void dataCallback(const ecmc::raw_sensor_object_data_msg& msg) {
    int radarNum = msg.radarNum;
    int numObjects = msg.numObjects;
    
    // Print radar number and number of objects within message
    ROS_INFO_STREAM(numObjects << " OBJECTS FROM RADAR " << radarNum);

    // Iterate through message, print each object
    for(int object_number = 0; object_number < numObjects; object_number++){
        ROS_INFO_STREAM("OBJECT " << object_number << ":" "\n"
            << "Accelerations_x: " << msg.accel_x[object_number] << " "
            << "Velocities_x: " << msg.vel_x[object_number] << " "
            << "Positions_x: " << msg.pos_x[object_number] << " "
            << "Positions_y: " << msg.pos_y[object_number] << " "
            << "Exist Prob: " << msg.existProb[object_number] << " "
            << "Valid: " << msg.valid[object_number] << "\n");
    }
}
 
int main(int argc, char **argv)
{
    // Initialize ros, create subscriber
    ros::init(argc, argv, "data_sub_node");
    ros::NodeHandle test_data_sub_nh;
    ros::Subscriber test_data_sub = test_data_sub_nh.subscribe("raw_data_test", SUB_BUFFER_SIZE, dataCallback);  
  
    // Wait on publisher, call callbacks on messages
    ros::spin();
}
