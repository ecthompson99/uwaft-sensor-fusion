#include "ros/ros.h"
#include "sensor_fusion/new_srv.h"
#include <cstdlib>
#include "sensor_fusion/dummy_msg.h"
   

class stateTemp{
    public:

    stateTemp(int id, double dx) : obj_id(id), obj_dx(dx);

    int obj_id;
    double obj_dx;
    // int obj_lane;
    // double obj_vx;
    // double obj_dy;
    // double obj_ax;
    // bool obj_path;
    // double obj_vy;
    // double obj_timestamp;
    // int object_track_num;
};



int main(int argc, char **argv){
    ros::init(argc, argv, "client");
    
    // void callback_temp(const sensor_fusion::dummy_msg& msg_obj);

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<sensor_fusion::new_srv>("service_topic");
    // ros::Subscriber sub = n.subscribe("tempTopic", 10, callback_temp);
    
    sensor_fusion::new_srv srv;
    
    std::vector<stateTemp> stateVector;

    if (client.call(srv)){
        for(int i = 0; i < srv.response.obj_id.size(); i++){
            stateVector.push_back(stateTemp someObj(srv.response.obj_id[i], srv.response.obj_dx[i])); 
        }        
    } else {
        ROS_ERROR("Failed to call service");
        return 1;
    }


    ros::spinOnce();
    return 0;
}


// void callback_temp(const sensor_fusion::dummy_msg& msg_obj){
//     std::cout << msg_obj.apple << std::endl;
// }
