#include "ros/ros.h"
#include "sensor_fusion/new_srv.h"

    bool add(sensor_fusion::new_srv::Request &req, sensor_fusion::new_srv::Response &res){
        
        res.data.apple = 3;
        
        return true;
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "server");
        ros::NodeHandle n;

        ros::ServiceServer service = n.advertiseService("service_topic", add);
        
        ROS_INFO("Waiting for my guy, client.");
        ros::spin();

        return 0;
    }