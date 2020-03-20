#include "ros/ros.h"
#include "sensor_fusion/new_srv.h"
#include <cstdlib>
#include "sensor_fusion/dummy_msg.h"
   
int main(int argc, char **argv){
    ros::init(argc, argv, "client");
    
    void callback_temp(const sensor_fusion::dummy_msg& msg_obj);

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<sensor_fusion::new_srv>("service_topic");

    ros::Subscriber sub = n.subscribe("tempTopic", 10, callback_temp);

    
    sensor_fusion::new_srv srv;
    
    if (client.call(srv)){
        double lmao = srv.response.data.apple;
        std::cout << lmao << srv.response.data.objstuff << std::endl;
    } else {
        ROS_ERROR("Failed to call service add_two_ints");
        return 1;
    }



    ros::spinOnce();
    return 0;
}

void callback_temp(const sensor_fusion::dummy_msg& msg_obj){
    std::cout << msg_obj.apple << std::endl;
}
