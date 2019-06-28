#include "ros/ros.h"
#include "ecmc/RawSensorObjectDataMsg.h"
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle nh;
      
    // Change name of topic from greetings
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
            msgArray[i].valid[j] = 1;
        }
    }

    int count = 0;
 
    // With each iteration change some values of the message (what delays the loop speed)
    while(ros::ok()) {
        data_pub.publish(msgArray[count]);
        count++;
        ros::spinOnce();
    }
}
