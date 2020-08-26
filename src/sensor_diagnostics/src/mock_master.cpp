#include "sensor_diag.h" //message and service file declarations
#include "common/sensor_diagnostic_flag_CH2.h"
#include "common/sensor_diagnostic_flag_CH3.h"
#include "common/sensor_diagnostic_flag_CH4.h"

bool srv_ch2_callback(common::sensor_diagnostic_flag_CH2::Request &req, common::sensor_diagnostic_flag_CH2::Response &res){

  std::cout << "FRONT RADAR STATUS: " << +req.front_radar << std::endl;

  return true;
}

bool srv_ch3_callback(common::sensor_diagnostic_flag_CH3::Request &req, common::sensor_diagnostic_flag_CH3::Response &res){
    std::cout << "LEFT CORNER RADAR STATUS: " << +req.left_corner_radar << std::endl;
    std::cout << "RIGHT CORNER RADAR STATUS: " << +req.right_corner_radar << std::endl;
    return true;
}

bool srv_ch4_callback(common::sensor_diagnostic_flag_CH4::Request &req, common::sensor_diagnostic_flag_CH4::Response &res){
    std::cout << "REAR RADAR STATUS: " << +req.mobileye << std::endl;
    return true;
}

int main(int argc, char **argv){

    ros::init(argc, argv, "mock_master");
    ros::NodeHandle n;
    ros::ServiceServer server_ch2 = n.advertiseService("sensor_diagnostic_CH2", srv_ch2_callback);
    // ros::ServiceServer server_ch3 = n.advertiseService("sensor_diagnostic_CH3", srv_ch3_callback);
    // ros::ServiceServer server_ch4 = n.advertiseService("sensor_diagnostic_CH4", srv_ch4_callback);

    ros::spin();
}  
