#include "ros/ros.h"
#include "ecmc/RawSensorObjectDataMsg.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub");
    ros::NodeHandle nh;
      
    ros::Publisher data_pub = nh.advertise<ecmc::RawSensorObjectDataMsg>("Greetings", 1000);  
  
    ecmc::RawSensorObjectDataMsg msgArray[5];

    for(int i = 0; i < 5; i++){
        msgArray[i].radarNum = i;
        msgArray[i].numObjects = 3;
        for(int j = 0; j < 3; j++){
            msgArray[i].accel_x[j] = 32;
            msgArray[i].vel_x[j] = 32;
            msgArray[i].pos_x[j] = 32;
            msgArray[i].pos_y[j] = 32;
            msgArray[i].existProb[j] = 100;
            msgArray[i].valid = true;
        }
    }

    int count = 0;
 
    while(ros::ok()) {
        data_pub.publish(msgArray[count]);
        count++;
        ros::spinOnce();
    }
}