#include "ros/ros.h"
#include "can_tx_rx/ext_log_data.h"

void CallBack(const can_tx_rx::ext_log_data& msg){
    ROS_INFO_STREAM("Position X: " << msg.me_dx << "\n" << "Position Y: " << msg.me_dy << "\n" << "Velocity X: "<< msg.me_vx << "\n" << "Acceleration X: " << msg.me_ax << "\n");
}

int main(int argc, char  **argv)
{
    ros::init(argc, argv, "can_tx_rx_CH4_UnitTest_Sub");
    ros::NodeHandle nh; 
    ros::Subscriber sub = nh.subscribe("mobileye_object_data",1000, CallBack); 

    ros::spin(); 
}