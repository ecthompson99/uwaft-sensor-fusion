#include "ros/ros.h"

#include <canlib.h>
#include "ecmc/raw_sensor_object_data_msg.h"

static const uint32_t pub_buffer_size = 100;

int main(int argc, char **argv)
{
    // Initialize ros and create node handle
    ros::init(argc, argv, "data_pub_node");
    ros::NodeHandle test_data_pub_nh;
      
    ros::Publisher test_data_pub = test_data_pub_nh.advertise<ecmc::raw_sensor_object_data_msg>("raw_data_test", pub_buffer_size);  

    ecmc::raw_sensor_object_data_msg msg;

    msg.radar_num = 5;
    msg.num_objects = 3;
    msg.accel_x = {1,2,3};
    msg.vel_x = {1,2,3};
    msg.pos_x = {1,2,3};
    msg.pos_y = {1,2,3};
    msg.exist_prob = {1,2,3};
    msg.valid = {1,1,1};

    canHandle hnd;
    canInitializeLibrary();
    hnd = canOpenChannel(0, canOPEN_EXCLUSIVE);
    if (hnd < 0) {
      char msg[64];
      canGetErrorText((canStatus)hnd, msg, sizeof(msg));
      fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
      exit(1);
    }
    canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);
    char *can_msg = "HELLO!";

    while(ros::ok()) {
        test_data_pub.publish(msg);
        canWrite(hnd, 123, (void *)can_msg, 6, 0);
        canWriteSync(hnd, 500);
        ros::spinOnce();
        ros::Duration(0.5).sleep();
    }

    canBusOff(hnd);
    canClose(hnd);
}
