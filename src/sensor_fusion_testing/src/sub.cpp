#include "ros/ros.h"
#include "sensor_fusion_testing/radar_object_data_from_matlab.h"
#include "sensor_fusion_testing/mobileye_object_data_from_matlab.h"
 
void radar_callback(const sensor_fusion_testing::radar_object_data_from_matlab& radar_msg) {
    ROS_INFO_STREAM("timestamp" << radar_msg.RadarTimestamp << "  " <<
                        "radar_dx" << radar_msg.RadarDx << "  " <<
                        "radar_dy" << radar_msg.RadarDy << "  " <<
                        "radar_vx" << radar_msg.RadarVx << "  " <<
                        "radar_vy" << radar_msg.RadarVy << "  " <<
                        "radar_ax" << radar_msg.RadarAx << "  " <<
                        "radar_dx_sigma" << radar_msg.RadarDxSigma << "  " <<
                        "radar_dy_sigma" << radar_msg.RadarDySigma << "  " <<
                        "radar_vx_sigma" << radar_msg.RadarVxSigma << "  " <<
                        "radar_ax_sigma" << radar_msg.RadarAxSigma << "\n"
                    );
}

void mobileye_callback(const sensor_fusion_testing::mobileye_object_data_from_matlab& me_msg) {
    ROS_INFO_STREAM("timestamp" << me_msg.MeTimestamp << "  "
                        "me_dx" << me_msg.MeDx << "  " <<
                        "me_dy" << me_msg.MeDy << "  " <<
                        "me_vx" << me_msg.MeVx << "\n"
                    );
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_fusion_sub");
    ros::NodeHandle nh;

    ros::Subscriber sensor_radar_data_obj_sub = nh.subscribe("radar_from_matlab", 1000, radar_callback);


	ros::Subscriber sensor_me_data_obj_sub = nh.subscribe("mobileye_from_matlab", 1000, mobileye_callback);
  

    ros::spin();

}