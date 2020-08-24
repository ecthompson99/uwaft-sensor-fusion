#include "ros/ros.h"
#include <canlib.h>
#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"

static const int sub_buffer_size = 100;

int main(int argc, char** argv) {

  ros::init(argc, argv, "data_sub_node");
  ros::NodeHandle test_data_sub_nh;

  canHandle hnd;
  canInitializeLibrary();
  hnd = canOpenChannel(1, canOPEN_EXCLUSIVE);
  if (hnd < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }
  canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  canBusOn(hnd);

long id;
uint8_t can_msg[8] = {0};
// char can_msg[6] = {0};
unsigned int dlc, flags;
unsigned long timestamp;
size_t size = 8;
while (ros::ok())
{
    canStatus stat = canRead(hnd, &id, can_msg, &dlc, &flags, &timestamp);
    if (stat == canOK) {
      // std::cout << can_msg << std::endl;
      std::cout << "id is " << id << std::endl;
    }
    else {
      std::cout << "no msg" << std::endl;
    }
    // if (stat != canERR_NOMSG) {
    //     std::cout << "Failed, status == " << stat << std::endl;
    // }
    ros::spinOnce();
    ros::Duration(0.5).sleep();
}


struct ext_log_data_obstacle_data_a_t a;
struct ext_log_data_obstacle_data_a_t *frame_a = &a;

/*int unpack_return = */ ext_log_data_obstacle_data_a_unpack(frame_a, can_msg, size);

std::cout << "Data before decoding is: " << std::endl;
std::cout << a.obstacle_pos_x << std::endl;
std::cout << a.obstacle_pos_y << std::endl;
std::cout << a.obstacle_vel_x << std::endl;

std::cout << "Original data is: " << std::endl;
std::cout << ext_log_data_obstacle_data_a_obstacle_pos_x_decode(a.obstacle_pos_x) << std::endl;
std::cout << ext_log_data_obstacle_data_a_obstacle_pos_y_decode(a.obstacle_pos_y) << std::endl;
std::cout << ext_log_data_obstacle_data_a_obstacle_vel_x_decode(a.obstacle_vel_x) << std::endl;

canBusOff(hnd);
canClose(hnd);


return 0;

}

