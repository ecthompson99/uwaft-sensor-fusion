#include <canlib.h>

#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"

#include "can_tx_rx/mobileye_object_data_msg.h"

static const uint16_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;


int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  ros::Publisher raw_obj_data_pub = can_tx_rx_CH4_handle.advertise<can_tx_rx::mobileye_object_data_msg>(
      "mobileye_object_data", TX_RX_MESSAGE_BUFFER_SIZE);

  can_tx_rx::mobileye_object_data_msg obj_data;

  canHandle hnd;

  canInitializeLibrary();

  hnd = canOpenChannel(3, canOPEN_EXCLUSIVE);

  if (hnd < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  canBusOn(hnd);

  long int id;
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  uint8_t size_of_msg = 8;
  uint8_t can_data[8] = {0};

  int unpack_return = -1;  // 0 is successful, negative error code

  while (ros::ok()) {
    canStatus stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);

    if (canOK == stat) {

    }
    canBusOff(hnd);
    canClose(hnd);


    ros::spinOnce();
  }
  return 0;
}

