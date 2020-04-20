#include "ros/ros.h"
#include "sensor_fusion_testing/radar_object_data.h"
#include "sensor_fusion_testing/mobileye_object_data.h"
 
void radar_callback(const sensor_fusion_testing::radar_object_data& radar_msg) {
     ROS_INFO_STREAM("b");
    ROS_INFO_STREAM("timestamp" << radar_msg.RadarTimestamp << "  " <<
                        "radar_dx" << radar_msg.RadarDx << "  " <<
                        "radar_dy" << radar_msg.RadarDy << "  " <<
                        "radar_vx" << radar_msg.RadarVx << "  " <<
                        "radar_vy" << radar_msg.RadarVy << "\n"
                    );
}

// void mobileye_callback(const sensor_fusion_testing::mobileye_object_data& me_msg) {
//     ROS_INFO_STREAM("timestamp" << me_msg.me_timestamp << "  "
//                         "me_dx" << me_msg.me_dx << "  " <<
//                         "me_dy" << me_msg.me_dy << "  " <<
//                         "me_vx" << me_msg.me_vx << "\n"
//                     );
// }


int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_fusion_sub");
    ros::NodeHandle nh;

    ros::Subscriber sensor_radar_data_obj_sub = nh.subscribe("radar_topic", 1000, radar_callback);
    ROS_INFO_STREAM("a");

	// ros::Subscriber sensor_me_data_obj_sub = nh.subscribe("mobileye_topic", 10, mobileye_callback);
  

    ros::spin();

}