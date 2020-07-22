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
    
    for (int x = 0; x < sensor_data.size(); x++)
    {
        ros::Time time(sensor_data[x][0]);
        common::radar_object_data radar1;
        common::radar_object_data radar2;
        common::radar_object_data radar3;
        common::mobileye_object_data camera;

        radar1.RadarDx = sensor_data[x][1];
        radar1.RadarDy = sensor_data[x][2];
        radar1.RadarVx = sensor_data[x][3];
        radar1.RadarVy = sensor_data[x][4];
        radar1.RadarAx = sensor_data[x][5];
        radar1.RadarTimestamp = sensor_data[x][0];
        radar1.RadarAxSigma = 0;
        radar1.RadarDxSigma = 0;
        radar1.RadarDySigma = 0;
        radar1.RadarVxSigma = 0;

        radar2.RadarDx = sensor_data[x][6];
        radar2.RadarDy = sensor_data[x][7];
        radar2.RadarVx = sensor_data[x][8];
        radar2.RadarVy = sensor_data[x][9];
        radar2.RadarAx = sensor_data[x][10];
        radar2.RadarTimestamp = sensor_data[x][0];
        radar2.RadarAxSigma = 0;
        radar2.RadarDxSigma = 0;
        radar2.RadarDySigma = 0;
        radar2.RadarVxSigma = 0;

        radar3.RadarDx = sensor_data[x][11];
        radar3.RadarDy = sensor_data[x][12];
        radar3.RadarVx = sensor_data[x][13];
        radar3.RadarVy = sensor_data[x][14];
        radar3.RadarAx = sensor_data[x][15];
        radar3.RadarTimestamp = sensor_data[x][0];
        radar3.RadarAxSigma = 0;
        radar3.RadarDxSigma = 0;
        radar3.RadarDySigma = 0;
        radar3.RadarVxSigma = 0;

        camera.MeDx = sensor_data[x][16];
        camera.MeDy = sensor_data[x][17];
        camera.MeVx = sensor_data[x][18];
        camera.MeTimestamp = sensor_data[x][0];
        

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
