#include "master_task.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "master_task");
  ros::NodeHandle master_task_handle;
  MasterTask master_task(&master_task_handle);

  while (ros::ok()) {
    master_task.publish_can_comms_msg();
    ros::spinOnce();
  }

  return 0;
}
