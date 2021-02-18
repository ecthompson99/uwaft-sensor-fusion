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

struct radar_data 
{
    float dx[32];
    float dy[32];
    float vx[32];
    float vy[32];
};

struct camera_data 
{
    float dx[10];
    float dy[10];
    float vx[10];
    float vy[10];
};

void radar_data_parser(stringstream& ss, radar_data& radar) 
{
    string word;
    for (int x = 0; x < 128; x++) 
    {
        std::getline(ss, word, ',');
        float sensor_val = atof(word.c_str());
        int idx = x % 32;
        if (x < 32)
            radar.dx[idx] = sensor_val;
        else if (32 <= x && x < 64)
            radar.dy[idx] = sensor_val;
        else if (64 <= x && x < 96)
            radar.vx[idx] = sensor_val;
        else if (96 <= x && x < 128)
            radar.vy[idx] = sensor_val;
    }
}

void camera_data_parser(stringstream& ss, camera_data& camera) 
{
    string word;
    for (int x = 0; x < 30; x++) 
    {
        std::getline(ss, word, ',');
        float sensor_val = atof(word.c_str());
        int idx = x % 10;
        if (x < 10)
            camera.dx[idx] = sensor_val;
        else if (10 <= x && x < 20)
            camera.dy[idx] = sensor_val;
        else if (20 <= x && x < 30)
            camera.vx[idx] = sensor_val;
    }
}

void read_sensor_data_csv(string filename, vector<double>& time, vector<radar_data>& radar1, vector<radar_data>& radar2,
                          vector<radar_data>& radar3, vector<camera_data>& camera) 
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
        std::getline(fin, line, '\n');
        stringstream ss(line);

        std::getline(ss, word, ',');
        time.push_back(atof(word.c_str()));

        radar1.push_back(radar_data());
        radar2.push_back(radar_data());
        radar3.push_back(radar_data());
        camera.push_back(camera_data());

        radar_data_parser(ss, radar1[i]);
        radar_data_parser(ss, radar2[i]);
        radar_data_parser(ss, radar3[i]);
        camera_data_parser(ss, camera[i]);
        
        i++;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "sensor_data_matlab");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    bag.open("sim_sensor_data.bag", rosbag::bagmode::Write);
    
    std::vector<double> time_stamps;
    std::vector<radar_data> radar1;
    std::vector<radar_data> radar2;
    std::vector<radar_data> radar3;
    std::vector<camera_data> camera;
    read_sensor_data_csv("/home/jaiprajapati/kaiROS/src/simulation_sensor_data/src/sim_sensor_output_stationary_approach.csv", time_stamps, radar1, radar2, radar3, camera);

    //Output "sensor_data" vector content
    for (int i = 0; i < time_stamps.size(); i++)
    {
        cout << "Time: " << time_stamps[i] << endl;

        for (int j = 0; j < 32; j++)
        {
            printf("Radar1: %.2f, %.2f, %.2f, %.2f\n", radar1[i].dx[j], radar1[i].dy[j], radar1[i].vx[j], radar1[i].vy[j]);
            printf("Radar2: %.2f, %.2f, %.2f, %.2f\n", radar2[i].dx[j], radar2[i].dy[j], radar2[i].vx[j], radar2[i].vy[j]);
            printf("Radar3: %.2f, %.2f, %.2f, %.2f\n", radar3[i].dx[j], radar3[i].dy[j], radar3[i].vx[j], radar3[i].vy[j]);
            if(j < 10)
                printf("Camera: %.2f, %.2f, %.2f\n", camera[i].dx[j], camera[i].dy[j], camera[i].vx[j]);
        }
        cout << endl;
    }

    for (size_t x = 0; x < time_stamps.size(); x++) 
    {
        ros::Time time(time_stamps[x]);
        common::radar_object_data radar1_data;
        common::radar_object_data radar2_data;
        common::radar_object_data radar3_data;
        common::mobileye_object_data camera_data;

        for(int obj_idx = 0; obj_idx < 32; obj_idx++)
        {
            radar1_data.radar_dx[obj_idx] = radar1[x].dx[obj_idx];
            radar1_data.radar_dy[obj_idx] = radar1[x].dy[obj_idx];
            radar1_data.radar_vx[obj_idx] = radar1[x].vx[obj_idx];
            radar1_data.radar_vy[obj_idx] = radar1[x].vy[obj_idx];
            radar1_data.radar_ax[obj_idx] = 0;
            radar1_data.radar_timestamp = time_stamps[x];
            radar1_data.radar_ax_sigma[obj_idx] = 0;
            radar1_data.radar_dx_sigma[obj_idx] = 0;
            radar1_data.radar_dy_sigma[obj_idx] = 0;
            radar1_data.radar_vx_sigma[obj_idx] = 0;
            radar1_data.radar_w_exist[obj_idx] = 1;
            radar1_data.radar_flag_valid[obj_idx] = 1;
            radar1_data.d_length[obj_idx] = 1;


            radar2_data.radar_dx[obj_idx] = radar2[x].dx[obj_idx];
            radar2_data.radar_dy[obj_idx] = radar2[x].dy[obj_idx];
            radar2_data.radar_vx[obj_idx] = radar2[x].vx[obj_idx];
            radar2_data.radar_vy[obj_idx] = radar2[x].vy[obj_idx];
            radar2_data.radar_ax[obj_idx] = 0;
            radar2_data.radar_timestamp = time_stamps[x];
            radar2_data.radar_ax_sigma[obj_idx] = 0;
            radar2_data.radar_dx_sigma[obj_idx] = 0;
            radar2_data.radar_dy_sigma[obj_idx] = 0;
            radar2_data.radar_vx_sigma[obj_idx] = 0;
            radar2_data.radar_w_exist[obj_idx] = 1;
            radar2_data.radar_flag_valid[obj_idx] = 1;
            radar2_data.d_length[obj_idx] = 1;

            radar3_data.radar_dx[obj_idx] = radar3[x].dx[obj_idx];
            radar3_data.radar_dy[obj_idx] = radar3[x].dy[obj_idx];
            radar3_data.radar_vx[obj_idx] = radar3[x].vx[obj_idx];
            radar3_data.radar_vy[obj_idx] = radar3[x].vy[obj_idx];
            radar3_data.radar_ax[obj_idx] = 0;
            radar3_data.radar_timestamp = time_stamps[x];
            radar3_data.radar_ax_sigma[obj_idx] = 0;
            radar3_data.radar_dx_sigma[obj_idx] = 0;
            radar3_data.radar_dy_sigma[obj_idx] = 0;
            radar3_data.radar_vx_sigma[obj_idx] = 0;
            radar3_data.radar_w_exist[obj_idx] = 1;
            radar3_data.radar_flag_valid[obj_idx] = 1;
            radar3_data.d_length[obj_idx] = 1;
        }

        for(int obj_idx = 0; obj_idx < 10; obj_idx++)
        {
            camera_data.me_dx[obj_idx] = camera[x].dx[obj_idx];
            camera_data.me_dy[obj_idx] = camera[x].dy[obj_idx];
            camera_data.me_vx[obj_idx] = camera[x].vx[obj_idx];
            camera_data.me_timestamp = time_stamps[x];
            camera_data.me_valid[obj_idx] = 1;
        }

        if (time.toNSec() == 0) time = ros::TIME_MIN;

        bag.write("Radar_One_CAN_Rx", time, radar1_data);
        bag.write("Radar_Two_CAN_Rx", time, radar2_data);
        bag.write("Radar_Three_CAN_Rx", time, radar3_data);
        bag.write("Mobileye_CAN_Rx", time, camera_data);        

    }

    bag.close();

    void close();
    return 0;
}
