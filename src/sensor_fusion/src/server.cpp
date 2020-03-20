#include "ros/ros.h"
#include "sensor_fusion/new_srv.h"
#include "sensor_fusion/dummy_msg.h"

    class tempClass{
        public:
        int rand;
        std::string bruh;
        
        tempClass(int random, std::string rando) : rand(random), bruh(rando){}        
    };

    class container{
        public:
        std::vector<tempClass> vec;
    };

    bool add(sensor_fusion::new_srv::Request &req, sensor_fusion::new_srv::Response &res){
        container cont;
        tempClass obj(6, "heelo"); 
        // cont.vec.push_back(obj);

        res.data.apple = obj.rand;
        res.data.objstuff = obj.bruh;
        // res.data.objectContainer = container.vec;

        return true;
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "server");
        ros::NodeHandle n;

        ros::Publisher pub = n.advertise<sensor_fusion::dummy_msg>("tempTopic", 10);

        // while(ros::ok()){
        //     pub.publish(msgObj);
        // }
        

        ros::ServiceServer service = n.advertiseService("service_topic", add);
        
        ROS_INFO("Waiting for my guy, client.");
        ros::spin();

        return 0;
    }