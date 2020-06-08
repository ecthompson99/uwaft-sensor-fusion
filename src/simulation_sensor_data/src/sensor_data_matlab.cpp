//#include "rapidcsv.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <simulation_sensor_data/radar1_data.h>
#include <simulation_sensor_data/radar2_data.h>
#include <simulation_sensor_data/radar3_data.h>
#include <simulation_sensor_data/camera_data.h>
using namespace std;


void read_sensor_data_csv(string filename, vector<vector<double> >& v2d)
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
        for (int x = 0; x < 17; x++)
        {
            std::getline(ss, word, ',');
            v2d[i].push_back(atof(word.c_str()));
        }
    }
}




int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_data_matlab");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    bag.open("sim_sensor_data.bag", rosbag::bagmode::Write);
    
    std::vector<vector<double> > sensor_data;
    read_sensor_data_csv("/home/joannadiao/kaiROS/src/simulation_sensor_data/src/sim_sensor_output_matlab.csv", sensor_data);


    //Output "sensor_data" vector content
    // for (int a = 0; a < sensor_data.size(); a++)
    // {
    //     for (int b = 0; b < sensor_data[a].size(); b++)
    //     {
    //         cout << sensor_data[a][b] << "  ";
    //     }
    //     cout << endl;
    // }
    
    double time = 0;

    for (int x = 0; x < sensor_data.size(); x++)
    {
        time = sensor_data[x][0];
        simulation_sensor_data::radar1_data radar1;
        simulation_sensor_data::radar2_data radar2;
        simulation_sensor_data::radar3_data radar3;
        simulation_sensor_data::camera_data camera;

        radar1.radar1_dx_ = sensor_data[x][1];
        radar1.radar1_dy_ = sensor_data[x][2];
        radar1.radar1_vx_ = sensor_data[x][3];
        radar1.radar1_vy_ = sensor_data[x][4];
        radar2.radar2_dx_ = sensor_data[x][5];
        radar2.radar2_dy_ = sensor_data[x][6];
        radar2.radar2_vx_ = sensor_data[x][7];
        radar2.radar2_vy_ = sensor_data[x][8];
        radar3.radar3_dx_ = sensor_data[x][9];
        radar3.radar3_dy_ = sensor_data[x][10];
        radar3.radar3_vx_ = sensor_data[x][11];
        radar3.radar3_vy_ = sensor_data[x][12];
        camera.camera_dx_ = sensor_data[x][13];
        camera.camera_dy_ = sensor_data[x][14];
        camera.camera_vx_ = sensor_data[x][15];
        camera.camera_vy_ = sensor_data[x][16];

        bag.write("radar1_topic", time, radar1);
        bag.write("radar2_topic", time, radar2);
        bag.write("radar3_topic", time, radar3);
        bag.write("camera_topic", time, camera);        
        
    }
    bag.close();

    void close();
    return 0;
}