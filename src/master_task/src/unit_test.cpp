#include "master_task.h"
#include "ros/ros.h"

int main(int argc, char **argv)
 {
   ros::init(argc, argv, "master_task_unit_test");
 
   ros::NodeHandle n;
   ros::ServiceClient client2 = n.serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2");
   ros::ServiceClient client3 = n.serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3");
   ros::ServiceClient client4 = n.serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4");

   common::sensor_diagnostic_flag_CH2 srv2;
   common::sensor_diagnostic_flag_CH3 srv3;
   common::sensor_diagnostic_flag_CH4 srv4;

   srv2.request.front_radar = true;
   srv3.request.left_corner_radar = true;
   srv3.request.right_corner_radar = true;
   srv4.request.mobileye = true;

   if (client2.call(srv2))
   {
     ROS_INFO("srv2 successfully called!");
   }
   else
   {
     ROS_ERROR("Failed to call service srv2");
     return 1;
   }

   if (client3.call(srv3))
   {
     ROS_INFO("srv3 successfully called!");
   }
   else
   {
     ROS_ERROR("Failed to call service srv3");
     return 1;
   }

   if (client4.call(srv4))
   {
     ROS_INFO("srv4 successfully called!");
   }
   else
   {
     ROS_ERROR("Failed to call service srv4");
     return 1;
   }
 
   return 0;
 }