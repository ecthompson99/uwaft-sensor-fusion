#include "ros/ros.h"
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <simulation_sensor_data/radar1_data.h>
#include <simulation_sensor_data/radar2_data.h>
#include <simulation_sensor_data/radar3_data.h>
#include <simulation_sensor_data/camera_data.h>
#include <gtest/gtest.h>
 
     #include <boost/foreach.hpp>
     #define foreach BOOST_FOREACH

    int main (int argc, char** argv)
    {
        ros::init (argc, argv, "read_bag");
        rosbag::Bag bag;
        bag.open("/home/joannadiao/kaiROS/src/simulation_sensor_data/include/sim_sensor_data.bag", rosbag::bagmode::Read);
    
        std::vector<double> result;
        simulation_sensor_data::radar1_data radar1 = simulation_sensor_data::radar1_data("RADAR_TOPIC");
        
        for (int a = 0; a < 4; a++)
        {
            result.pushback(radar1.radar1_dx_);
        }        
    
        rosbag::View view(bag, rosbag::TopicQuery(result));


    
        foreach(rosbag::MessageInstance const m, view)
        {
            simulation_sensor_data::radar1_data::ConstPtr s = m.instantiate<simulation_sensor_data::radar1_data>();
            if (s != NULL)
                std::cout << s->data << std::endl;
    
            //  std_msgs::Int32::ConstPtr i = m.instantiate<std_msgs::Int32>();
            //  if (i != NULL)
            //      std::cout << i->data << std::endl;
        }
    
        bag.close();
        return 0;
    }
 
     