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


   ros::Publisher drive_crtl_pub = n.advertise<common::drive_ctrl_input_msg>("drive_control_input", 1000);
   ros::Publisher acc_pub = n.advertise<common::acc_output_msg>("acc_output", 1000);
   ros::Publisher aeb_pub = n.advertise<common::aeb_output_msg>("aeb_output", 1000);
   ros::Publisher lcc_pub = n.advertise<common::lcc_output_msg>("lcc_output", 1000);
 
   ros::Rate loop_rate(10);
 
   /**
    * A count of how many messages we have sent. This is used to create
    * a unique string for each message.
    */
   int count = 0;
   while (ros::ok())
   {
     common::drive_ctrl_input_msg drive_ctrl_msg;
     common::acc_output_msg acc_msg;
     common::aeb_output_msg aeb_msg;
     common::lcc_output_msg lcc_msg;

     drive_ctrl_msg.acc_activation = 1;
     drive_ctrl_msg.aeb_activation = 1;
     drive_ctrl_msg.lcc_activation = 1;
     drive_ctrl_msg.acc_allowed = 1;
     drive_ctrl_msg.aeb_allowed = 1;
     drive_ctrl_msg.lcc_allowed = 1;
     drive_ctrl_msg.alive_rolling_counter_MABx = 2;
     drive_ctrl_msg.alive_rolling_counter_Jetson = 2;
     drive_ctrl_msg.acc_gap_level = 2;
     drive_ctrl_msg.str_ang = 10.5;
     drive_ctrl_msg.acc_speed_set_point = 30.5;
     drive_ctrl_msg.veh_spd = 25.5;

     acc_msg.acc_fault = false;
     acc_msg.acc_accel = 4;

     aeb_msg.aeb_fault = false;
     aeb_msg.aeb_engaged = true;
     aeb_msg.aeb_accel = 4;

     lcc_msg.lcc_fault = false;
     lcc_msg.lcc_steer = 2.5;
 
     /**
      * The publish() function is how you send messages. The parameter
      * is the message object. The type of this object must agree with the type
      * given as a template parameter to the advertise<>() call, as was done
      * in the constructor above.
      */
     drive_crtl_pub.publish(drive_ctrl_msg);
     acc_pub.publish(acc_msg);
     aeb_pub.publish(aeb_msg);
     lcc_pub.publish(lcc_msg);
 
     ros::spinOnce();
 
     loop_rate.sleep();
     ++count;
   }
 
   return 0;
 }