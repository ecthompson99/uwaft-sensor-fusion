#include "ros/ros.h"

#include <canlib.h>
#include "ecmc/raw_sensor_object_data_msg.h"

static const int sub_buffer_size = 100;

void raw_data_recv_callback(const ecmc::raw_sensor_object_data_msg& msg) {
  uint8_t radar_num = msg.radar_num;
  uint8_t num_objects = msg.num_objects;

  ROS_INFO_STREAM(unsigned(num_objects) << " OBJECTS FROM RADAR " << unsigned(radar_num) << "\n");

  for (uint32_t object_number = 0; object_number < num_objects; object_number++) {
    ROS_INFO_STREAM("OBJECT " << object_number << ":"
                                                  "\n"
                              << "Accelerations_x: " << msg.accel_x[object_number] << " "
                              << "Velocities_x: " << msg.vel_x[object_number] << " "
                              << "Positions_x: " << msg.pos_x[object_number] << " "
                              << "Positions_y: " << msg.pos_y[object_number] << " "
                              << "Exist Prob: " << msg.exist_prob[object_number] << " "
                              << "Valid: " << msg.valid[object_number] << "\n");
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "data_sub_node");
  ros::NodeHandle test_data_sub_nh;
  //ros::Subscriber test_data_sub = test_data_sub_nh.subscribe("raw_data_test", sub_buffer_size, raw_data_recv_callback);

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
  char msg[6] = {0};
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  while (ros::ok()) {
    canStatus status = canRead(hnd, &id, &msg, &dlc, &flag, &time);
    if (canOK == status) {
      ROS_INFO_STREAM(msg);
    }
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
}
