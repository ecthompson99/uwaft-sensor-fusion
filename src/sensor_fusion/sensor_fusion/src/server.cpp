#include "ros/ros.h"
// #include <vector.h>
#include "sensor_fusion/new_srv.h"
#include "sensor_fusion/dummy_msg.h"

    bool add(sensor_fusion::new_srv::Request &req, sensor_fusion::new_srv::Response &res){
        sensor_fusion::dummy_msg test;
	test.apple = 1.1;
	test.bat = 3;
	test.cat = 2.2;
	res.data.push_back(test);
        
        return true;
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "server");
        ros::NodeHandle n;

        ros::ServiceServer service = n.advertiseService("service_topic", add);
        
        ROS_INFO("Ready to add two ints.");
        ros::spin();

        return 0;
    }
