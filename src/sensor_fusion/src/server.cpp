#include "ros/ros.h"
#include "sensor_fusion/new_srv.h"
#include "sensor_fusion/dummy_msg.h"

    class tempClass{
        public:
        int intMember;
        double dubMember;
        
        tempClass(int temp, double temp2) :intMember(temp), dubMember(temp2){}        
    };

    class container{
        public:
        std::vector<tempClass> vec;
    };

    bool add(sensor_fusion::new_srv::Request &req, sensor_fusion::new_srv::Response &res){
        
        container cont;

        tempClass obj(6, 6.3);
        tempClass obj2(7, 7.3);
        tempClass obj3(8, 8.3);

        cont.vec.push_back(obj);
        cont.vec.push_back(obj2);
        cont.vec.push_back(obj3);

        // index refers to specific objectNum in the original vector
        for(int i = 0; i < cont.vec.size(); i++){
            res.obj_id.push_back(cont.vec[i].intMember);
            res.obj_dx.push_back(cont.vec[i].dubMember);


            //res.obj_lane.push_back(cont.vec[i].intMember);
            // res.obj_vx[i] = cont.vec[i].intMember;
            // res.obj_dy[i] = cont.vec[i].intMember;
            // res.obj_ax[i] = cont.vec[i].intMember;
            // res.obj_path[i] = cont.vec[i].intMember;
            // res.obj_vy[i] = cont.vec[i].intMember;
            // res.obj_timestamp[i] = cont.vec[i].intMember;
            // res.object_track_num[i] = cont.vec[i].intMember;
        }
        return true;
    }

    int main(int argc, char **argv) {
        ros::init(argc, argv, "server");
        ros::NodeHandle n;

        ros::Publisher pub = n.advertise<sensor_fusion::dummy_msg>("tempTopic", 10);        

        ros::ServiceServer service = n.advertiseService("service_topic", add);
        
        ROS_INFO("Waiting for my guy, client.");
        ros::spin();

        return 0;
    }



// ADD THESE TO THE srv request .srv file
// uint8[] obj_lane
// float64[] obj_vx
// float64[] obj_dy
// float64[] obj_ax
// bool[] obj_path
// float64[] obj_vy
// float64[] obj_timestamp
// uint8[] object_track_num