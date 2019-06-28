#include "ros/ros.h"
<<<<<<< HEAD
#include "ecmc/RawSensorObjectData.h" // Where is this
=======
#include "ecmc/RawSensorObjectDataMsg.h"
>>>>>>> 31f8c93a2a4b1e6905e1d2e101c4f88b1dd4d43c
 
int main(int argc, char **argv)
{
    ros::init(argc, argv, "data_pub");
    ros::NodeHandle nh;
      
<<<<<<< HEAD
    ros::Publisher greeting_pub = nh.advertise<ecmc::RawSensorObjectData>("Greetings", 1000);  
  
    ecmc::RawSensorObjectData msg;
    msg.accel_x[0] = 20;
    msg.pos_y[0] = 20;
    int count = 1;
=======
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
>>>>>>> 31f8c93a2a4b1e6905e1d2e101c4f88b1dd4d43c
 
    // With each iteration change some values of the message (what delays the loop speed)
    while(ros::ok()) {
        data_pub.publish(msgArray[count]);
        count++;
        ros::spinOnce();
    }
}
