#include "ros/ros.h"
#include "sensor_fusion/new_srv.h"
#include <cstdlib>
   
int main(int argc, char **argv){
    ros::init(argc, argv, "client");
   
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<sensor_fusion::new_srv>("service_topic");
    
    sensor_fusion::new_srv srv;
    
    srv.request.useless = 1;

    if (client.call(srv)){
        ROS_INFO("Sum: %f %d %f", srv.response.data[0].apple, srv.response.data[0].bat, srv.response.data[0].cat);
    }
    else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }
   
    return 0;
}
