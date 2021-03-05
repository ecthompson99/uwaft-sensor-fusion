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

struct mobileye_data 
{
    float dx[10];
    float dy[10];
    float vx[10];
    float vy[10];
};

void radar_data_parser(stringstream& ss, radar_data& radar) 
{
    // Iterate comma-seperated entries of radar sensor data and copy to radar_data struct
    // 128 entries correspond to 32 objects in order: dx[32], dy[32], vx[32], vy[32]
    string word;
    for (int x = 0; x < 128; x++) 
    {
        std::getline(ss, word, ',');
        float sensor_val = atof(word.c_str());
        // Index of object (0->31) calculated from csv value index
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

void camera_data_parser(stringstream& ss, mobileye_data& camera) 
{
    // Iterate comma-seperated entries of mobileye sensor data and copy to mobileye_data struct
    // 30 entries correspond to 10 objects in order: dx[10], dy[10], vx[10]
    string word;
    for (int x = 0; x < 30; x++) 
    {
        std::getline(ss, word, ',');
        float sensor_val = atof(word.c_str());
        // Index of object (0->9) calculated from csv value index
        int idx = x % 10;
        if (x < 10)
            camera.dx[idx] = sensor_val;
        else if (10 <= x && x < 20)
            camera.dy[idx] = sensor_val;
        else if (20 <= x && x < 30)
            camera.vx[idx] = sensor_val;
    }
}

void read_sensor_data_csv(string filename, vector<double>& time, vector<radar_data>& front_radar, vector<radar_data>& right_radar,
                          vector<radar_data>& left_radar, vector<mobileye_data>& mobileye) 
{
    ifstream fin(filename.c_str());
    if (!fin.is_open())
    {
        cout << "File cannot be opened:(" << endl;
        return;
    }

    string line, word;
    size_t i = 0;

    // Get radar and mobileye data from driving scenario csv output
    // File Format: Time Stamp, Front Radar Data, Right Radar Data, Left Radar Data, Mobileye Data
    while (fin)
    {
        std::getline(fin, line, '\n');
        stringstream ss(line);

        std::getline(ss, word, ',');
        time.push_back(atof(word.c_str()));

        front_radar.push_back(radar_data());
        right_radar.push_back(radar_data());
        left_radar.push_back(radar_data());
        mobileye.push_back(mobileye_data());

        radar_data_parser(ss, front_radar[i]);
        radar_data_parser(ss, right_radar[i]);
        radar_data_parser(ss, left_radar[i]);
        camera_data_parser(ss, mobileye[i]);
        
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
    std::vector<radar_data> front_radar;
    std::vector<radar_data> right_radar;
    std::vector<radar_data> left_radar;
    std::vector<mobileye_data> mobileye;
    
    string simulation_file_path = getenv("HOME");
    simulation_file_path.append("/kaiROS/src/simulation_sensor_data/src/sim_sensor_output_stationary_approach.csv");
    read_sensor_data_csv(simulation_file_path, time_stamps, front_radar, right_radar, left_radar, mobileye);

    for (size_t x = 0; x < time_stamps.size(); x++) 
    {
        ros::Time time(time_stamps[x]);
        common::radar_object_data front_radar_data;
        common::radar_object_data right_radar_data;
        common::radar_object_data left_radar_data;
        common::mobileye_object_data mobileye_data;

        // Assign radar data to ROS radar_object_data messages
        for(int obj_idx = 0; obj_idx < 32; obj_idx++)
        {
            front_radar_data.radar_dx[obj_idx] = front_radar[x].dx[obj_idx];
            front_radar_data.radar_dy[obj_idx] = front_radar[x].dy[obj_idx];
            front_radar_data.radar_vx[obj_idx] = front_radar[x].vx[obj_idx];
            front_radar_data.radar_vy[obj_idx] = front_radar[x].vy[obj_idx];
            front_radar_data.radar_ax[obj_idx] = 0;
            front_radar_data.radar_timestamp = time_stamps[x];
            front_radar_data.radar_ax_sigma[obj_idx] = 0;
            front_radar_data.radar_dx_sigma[obj_idx] = 0;
            front_radar_data.radar_dy_sigma[obj_idx] = 0;
            front_radar_data.radar_vx_sigma[obj_idx] = 0;
            front_radar_data.radar_w_exist[obj_idx] = 1;
            front_radar_data.radar_flag_valid[obj_idx] = 1;
            front_radar_data.d_length[obj_idx] = 1;

            right_radar_data.radar_dx[obj_idx] = right_radar[x].dx[obj_idx];
            right_radar_data.radar_dy[obj_idx] = right_radar[x].dy[obj_idx];
            right_radar_data.radar_vx[obj_idx] = right_radar[x].vx[obj_idx];
            right_radar_data.radar_vy[obj_idx] = right_radar[x].vy[obj_idx];
            right_radar_data.radar_ax[obj_idx] = 0;
            right_radar_data.radar_timestamp = time_stamps[x];
            right_radar_data.radar_ax_sigma[obj_idx] = 0;
            right_radar_data.radar_dx_sigma[obj_idx] = 0;
            right_radar_data.radar_dy_sigma[obj_idx] = 0;
            right_radar_data.radar_vx_sigma[obj_idx] = 0;
            right_radar_data.radar_w_exist[obj_idx] = 1;
            right_radar_data.radar_flag_valid[obj_idx] = 1;
            right_radar_data.d_length[obj_idx] = 1;

            left_radar_data.radar_dx[obj_idx] = left_radar[x].dx[obj_idx];
            left_radar_data.radar_dy[obj_idx] = left_radar[x].dy[obj_idx];
            left_radar_data.radar_vx[obj_idx] = left_radar[x].vx[obj_idx];
            left_radar_data.radar_vy[obj_idx] = left_radar[x].vy[obj_idx];
            left_radar_data.radar_ax[obj_idx] = 0;
            left_radar_data.radar_timestamp = time_stamps[x];
            left_radar_data.radar_ax_sigma[obj_idx] = 0;
            left_radar_data.radar_dx_sigma[obj_idx] = 0;
            left_radar_data.radar_dy_sigma[obj_idx] = 0;
            left_radar_data.radar_vx_sigma[obj_idx] = 0;
            left_radar_data.radar_w_exist[obj_idx] = 1;
            left_radar_data.radar_flag_valid[obj_idx] = 1;
            left_radar_data.d_length[obj_idx] = 1;
        }

        // Assign mobileye data to ROS mobileye_object_data message
        for(int obj_idx = 0; obj_idx < 10; obj_idx++)
        {
            mobileye_data.me_dx[obj_idx] = mobileye[x].dx[obj_idx];
            mobileye_data.me_dy[obj_idx] = mobileye[x].dy[obj_idx];
            mobileye_data.me_vx[obj_idx] = mobileye[x].vx[obj_idx];
            mobileye_data.me_timestamp = time_stamps[x];
            mobileye_data.me_valid[obj_idx] = 1;
        }

        if (time.toNSec() == 0) time = ros::TIME_MIN;

        // Write radar and mobileye messages along with timestamp to corresponding topics in bag file
        bag.write("Front_Radar_CAN_Rx", time, front_radar_data);
        bag.write("Right_Radar_CAN_Rx", time, right_radar_data);
        bag.write("Left_Radar_CAN_Rx", time, left_radar_data);
        bag.write("Mobileye_CAN_Rx", time, mobileye_data);        
    }

    bag.close();

    void close();
    return 0;
}
