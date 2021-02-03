#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <time.h>
#include <common/mobileye_object_data.h>
#include <common/radar_object_data.h>
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
    size_t i = 0;

    while (fin)
    {
        v2d.push_back(vector<double>());
        std::getline(fin, line, '\n');
        stringstream ss(line);
        for (int x = 0; x < 19; x++)
        {
            std::getline(ss, word, ',');
            v2d[i].push_back(atof(word.c_str()));
        }
        i++;
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

    for (size_t x = 0; x < sensor_data.size(); x++) {
      ros::Time time(sensor_data[x][0]);
      common::radar_object_data radar1;
      common::radar_object_data radar2;
      common::radar_object_data radar3;
      common::mobileye_object_data camera;

      radar1.radar_dx = sensor_data[x][1];
      radar1.radar_dy = sensor_data[x][2];
      radar1.radar_vx = sensor_data[x][3];
      radar1.radar_vy = sensor_data[x][4];
      radar1.radar_ax = sensor_data[x][5];
      radar1.radar_timestamp = sensor_data[x][0];
      radar1.radar_ax_sigma = 0;
      radar1.radar_dx_sigma = 0;
      radar1.radar_dy_sigma = 0;
      radar1.radar_vx_sigma = 0;

      radar2.radar_dx = sensor_data[x][6];
      radar2.radar_dy = sensor_data[x][7];
      radar2.radar_vx = sensor_data[x][8];
      radar2.radar_vy = sensor_data[x][9];
      radar2.radar_ax = sensor_data[x][10];
      radar2.radar_timestamp = sensor_data[x][0];
      radar2.radar_ax_sigma = 0;
      radar2.radar_dx_sigma = 0;
      radar2.radar_dy_sigma = 0;
      radar2.radar_vx_sigma = 0;

      radar3.radar_dx = sensor_data[x][11];
      radar3.radar_dy = sensor_data[x][12];
      radar3.radar_vx = sensor_data[x][13];
      radar3.radar_vy = sensor_data[x][14];
      radar3.radar_ax = sensor_data[x][15];
      radar3.radar_timestamp = sensor_data[x][0];
      radar3.radar_ax_sigma = 0;
      radar3.radar_dx_sigma = 0;
      radar3.radar_dy_sigma = 0;
      radar3.radar_vx_sigma = 0;

      camera.me_dx = sensor_data[x][16];
      camera.me_dy = sensor_data[x][17];
      camera.me_vx = sensor_data[x][18];
      camera.me_timestamp = sensor_data[x][0];

      if (time.toNSec() == 0) time = ros::TIME_MIN;

      bag.write("FRONT_RADAR_TOPIC", time, radar1);
      bag.write("LEFT_CORNER_RADAR_TOPIC", time, radar2);
      bag.write("RIGHT_CORNER_RADAR_TOPIC", time, radar3);
      bag.write("MOBILEYE_TOPIC", time, camera);        
        
    }
    bag.close();

    void close();
    return 0;
}
