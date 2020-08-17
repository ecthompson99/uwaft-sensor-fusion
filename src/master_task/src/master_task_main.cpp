#include "master_task.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "master_task");
  ros::NodeHandle master_task_handle;
  MasterTask master_task(&master_task_handle);  

  while (ros::ok()) {
    
    master_task.AEB_24();
    master_task.LCC_10();
    master_task.LCC_3();
    master_task.ACC_15();
    master_task.ACC_17();
    master_task.ACC_20();
    master_task.ACC_4();
    master_task.AEB_13();
    master_task.AEB_22();
    master_task.AEB_26();
    master_task.LCC_11();
    master_task.CAV_1_5();
    master_task.CAV_1_6();
    master_task.ACC_1_1();
    master_task.CAV_2_2();    
    
    master_task.publish_can_comms_msg();
    ros::spinOnce();
    ros::Duration(1).sleep();
  }

  return 0;
}
