#include "ros/ros.h"
#include "ecmc/RawSensorObjectDataMsg.h"
 
void dataCallback(const ecmc::RawSensorObjectDataMsg& msg) {
    int radarNum = msg.radarNum;
    int numObjects = msg.numObjects;
    
    ROS_INFO_STREAM(numObjects << " OBJECTS FROM RADAR " << radarNum);

    for(int i = 0; i < numObjects; i++){
        ROS_INFO_STREAM("OBJECT " << i << ":" "\n"
            << "Accelerations_x: " << msg.accel_x[i] << " "
            << "Velocities_x: " << msg.vel_x[i] << " "
            << "Positions_x: " << msg.pos_x[i] << " " 
            << "Positions_y: " << msg.pos_y[i] << " "
            << "Exist Prob: " << msg.existProb[i] << " "
            << "Valid: " << msg.valid[i] << "\n");
    }


}
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_sub_node");
    ros::NodeHandle nh;
    ros::Subscriber data_sub = nh.subscribe("raw_data_test", 1000, dataCallback);  
  
    ros::spin();
}
