#include <iostream>
#include <fstream>
#include <sstream>
#include "ros/ros.h"
#include <rosbag/bag.h>
#include <time.h>
#include <common/mobileye_object_data.h>
#include <common/radar_object_data.h>
#include <common/drive_ctrl_input_msg.h>

using namespace std;

const char* parse_word(stringstream &ss)
{
    string word;
    std::getline(ss, word, ',');
    return word.c_str();
}

bool to_bool(std::string const& s) {
  return s != "0";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "blf_sensor_data_matlab");
    ros::NodeHandle nh;
    rosbag::Bag bag;
    // Bag file to output combined data to
    bag.open("ScenarioA_15_Sensor_Data.bag", rosbag::bagmode::Write);

    // Radar data CSV file path
    string file_path = getenv("HOME");
    file_path.append("/kaiROS/sensor fusion testing/Excel files/UWAFT_EMC Y3_ScenarioA_15_2021-05-29_17-34-21_radar.csv");
    ifstream fin_front_radar(file_path.c_str());
    if (!fin_front_radar.is_open())
    {
        cout << "Front radar data file cannot be opened." << endl;
        return 0;
    }

    // Mobileye data CSV file path
    file_path = getenv("HOME");
    file_path.append("/kaiROS/sensor fusion testing/Excel files/UWAFT_EMC Y3_ScenarioA_15_2021-05-29_17-34-21_mobileye.csv");
    
    ifstream fin_mobileye(file_path.c_str());
    if (!fin_mobileye.is_open())
    {
        cout << "Mobileye data file cannot be opened." << endl;
        return 0;
    }

    // Vehicle data CSV file path
    file_path = getenv("HOME");
    file_path.append("/kaiROS/sensor fusion testing/Excel files/UWAFT_EMC Y3_ScenarioA_15_2021-05-29_17-34-21_vehicle.csv");
    
    ifstream fin_vehicle(file_path.c_str());
    if (!fin_vehicle.is_open())
    {
        cout << "Vehicle data file cannot be opened." << endl;
        return 0;
    }

    string line, word;

    while (fin_front_radar)
    {
        std::getline(fin_front_radar, line, '\n');
        stringstream ss(line);

        std::getline(ss, word, ',');
        double timestamp = atof(word.c_str());

        ros::Time time(timestamp);

        common::radar_object_data front_radar_data;

        for(int obj_idx = 0; obj_idx < 32; obj_idx++)
        {
            front_radar_data.radar_timestamp = timestamp;
            front_radar_data.radar_dx[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_dy[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_vx[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_vy[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_ax[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_dx_sigma[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_dy_sigma[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_vx_sigma[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_ax_sigma[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_w_exist[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_w_obstacle[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_flag_valid[obj_idx] = to_bool(parse_word(ss));
            front_radar_data.radar_w_non_obstacle[obj_idx] = atof(parse_word(ss)); 
            front_radar_data.flag_meas[obj_idx] = to_bool(parse_word(ss)); 
            front_radar_data.flag_hist[obj_idx] = to_bool(parse_word(ss)); 
            front_radar_data.d_length[obj_idx] = atof(parse_word(ss));
            front_radar_data.radar_dz[obj_idx] = atof(parse_word(ss)); 
            front_radar_data.moving_state[obj_idx] = atof(parse_word(ss)); 
            front_radar_data.radar_w_class[obj_idx] = atof(parse_word(ss)); 
            front_radar_data.radar_obj_class[obj_idx] = atof(parse_word(ss)); 
            front_radar_data.dx_rear_loss[obj_idx] = atof(parse_word(ss)); 
        }
        if (time.toNSec() == 0) time = ros::TIME_MIN;

        // Write radar message along with timestamp to corresponding topic in bag file
        bag.write("Front_Radar_CAN_Rx", time, front_radar_data);
    }

    while (fin_mobileye)
    {
        std::getline(fin_mobileye, line, '\n');
        stringstream ss(line);

        std::getline(ss, word, ',');
        double timestamp = atof(word.c_str());

        ros::Time time(timestamp);

        common::mobileye_object_data mobileye_data;

        // Change to 10 if there are 10 object messages
        for(int obj_idx = 0; obj_idx < 1; obj_idx++)
        {
            mobileye_data.me_timestamp = timestamp;
            mobileye_data.me_dx[obj_idx] = atof(parse_word(ss));
            mobileye_data.me_dy[obj_idx] = atof(parse_word(ss));
            mobileye_data.me_vx[obj_idx] = atof(parse_word(ss));
            mobileye_data.me_ax[obj_idx] = atof(parse_word(ss));
            mobileye_data.me_type[obj_idx] = atoi(parse_word(ss));
            mobileye_data.me_status[obj_idx] = atoi(parse_word(ss));
            mobileye_data.me_valid[obj_idx] = atoi(parse_word(ss));
            mobileye_data.me_cut_in_cut_out[obj_idx] = atoi(parse_word(ss));
            mobileye_data.me_age[obj_idx] = atof(parse_word(ss));
            mobileye_data.me_lane[obj_idx] = atoi(parse_word(ss));
            mobileye_data.me_cipv_flag[obj_idx] = to_bool(parse_word(ss));
        }
        if (time.toNSec() == 0) time = ros::TIME_MIN;

        // Write mobileye message along with timestamp to corresponding topic in bag file
        bag.write("Mobileye_CAN_Rx", time, mobileye_data); 
    }


    while (fin_vehicle)
    {
        std::getline(fin_vehicle, line, '\n');
        stringstream ss(line);

        std::getline(ss, word, ',');
        double timestamp = atof(word.c_str());

        ros::Time time(timestamp);
        common::drive_ctrl_input_msg vehicle_data;

        vehicle_data.veh_spd = atof(parse_word(ss));
        vehicle_data.acc_allowed = atoi(parse_word(ss)); // From PCM, able to run ACC
        vehicle_data.str_ang = atof(parse_word(ss));
        vehicle_data.acc_speed_set_point = 90; //atof(parse_word(ss));
        vehicle_data.acc_activation = true; //atoi(parse_word(ss)); // From CAVs if ACC is active
        vehicle_data.acc_gap_level = 3; //atoi(parse_word(ss));

        if (time.toNSec() == 0) time = ros::TIME_MIN;

        // Write mobileye message along with timestamp to corresponding topic in bag file
        bag.write("drive_ctrl_input", time, vehicle_data);
    }
    

    bag.close();

    void close();
    return 0;
}
