#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <time.h>
#include <common/drive_ctrl_input_msg.h>
#include <common/target_output_msg.h>

using namespace std;


void read_vehicle_data_csv(string filename, vector<vector<double> >& v2d)
{
    ifstream fin(filename.c_str());
    if (!fin.is_open())
    {
        cout << "File cannot be opened:(" << endl;
        return;
    }

    string line, word;
    int i = -1;

    while (fin)
    {
        i++;
        v2d.push_back(vector<double>());
        std::getline(fin, line, '\n');
        stringstream ss(line);
        for (int x = 0; x < 19; x++)
        {
            std::getline(ss, word, ',');
            v2d[i].push_back(atof(word.c_str()));
        }
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "vehicle_input_matlab");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    bag.open("vehinput.bag", rosbag::bagmode::Write);
    
    std::vector<vector<double> > vehicle_data;
    read_vehicle_data_csv("/home/ryantanary/kaiROS/src/cav_controller_data/src/VehicleInputs.csv", vehicle_data);


    //Output "vehicle_data" vector content
    // for (int a = 0; a < vehicle_data.size(); a++)
    // {
    //     for (int b = 0; b < vehicle_data[a].size(); b++)
    //     {
    //         cout << vehicle_data[a][b] << "  ";
    //     }
    //     cout << endl;
    // }
    
    for (int x = 0; x < vehicle_data.size(); x++)
    {
        ros::Time time(vehicle_data[x][0]);
        common::drive_ctrl_input_msg drive_ctrl;
        common::target_output_msg target_output;

        drive_ctrl.veh_spd = vehicle_data[x][1];
        drive_ctrl.acc_gap_level = 2; 
        drive_ctrl.acc_speed_set_point = 72; 

        target_output.obj_vx = vehicle_data[x][2];
        target_output.obj_dx = vehicle_data[x][3];
        target_output.obj_timestamp = vehicle_data[x][0];   

        if (time.toNSec() == 0) time = ros::TIME_MIN;

        bag.write("DRIVE_CTRL_INPUT", time, drive_ctrl);
        bag.write("TARGET_OBJECT_OUTPUT", time, target_output);
        
    }
    bag.close();

    std::cout << "Conversion complete" << std::endl; 
    
    void close();
    return 0;
}
