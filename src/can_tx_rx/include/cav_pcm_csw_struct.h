#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "cav_pcm_csw.h"
#include "common/drive_ctrl_input_msg.h"
#include "common/can_comms_data_msg.h"

#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8
#define MESSAGE_BUFFER_SIZE 1000
#define TOPIC_RX "can_comms_data"
#define TOPIC_TX "drive_ctrl_input"

class CAV_PCM_TX_RX{
    public: 
        ros::NodeHandle* node_handle;
        ros::Publisher drive_ctrl_pub;
        ros::Subscriber can_comms_sub;

        CAV_PCM_TX_RX(ros::NodeHandle* node_handle);
        struct cav_pcm_csw_in{            
            double aeb_allowed_decode;
            bool aeb_allowed_is_in_range;
            double acc_allowed_decode;
            bool acc_allowed_is_in_range;
            double lcc_allowed_decode;
            bool lcc_allowed_is_in_range;
            double veh_spd_decode;
            bool veh_spd_is_in_range;
            double hsc_alive_decode;
            bool hsc_alive_is_in_range;
            double str_ang_decode;
            bool str_ang_is_in_range;
            double acc_active_decode;
            bool acc_active_is_in_range;
            double acc_st_speed_decode;
            bool acc_st_speed_is_in_range;

            double pcm_rc1_decode;
            bool pcm_rc1_is_in_range;
            double pcm_rc2_decode;
            bool pcm_rc2_is_in_range;
            double pcm_rc3_decode;
            bool pcm_rc3_is_in_range;

            uint8_t channel_number;
            unsigned long timestamp;
        };
        struct cav_pcm_csw_out{
            double long_accel;
            double lcc_steer; 
            bool acc_valid;
            bool aeb_valid;
            bool lcc_valid;
            bool acc_fault; 
            bool aeb_fault;
            bool lcc_fault; 
            bool front_radar_fault; 
            bool left_radar_fault;
            bool right_radar_fault;
            bool mobileye_fault;
            unsigned int cav_rolling_counter;
            unsigned int lat_rc;
            unsigned int long_rc;
            unsigned int lat_pv;
            unsigned int long_pv;
            unsigned long timestamp;
        };
        CAV_PCM_TX_RX::cav_pcm_csw_out cav_out;
        double signals_in_range(double val, bool cond);
        void get_nums(int id, uint8_t &case_num);
        void can_callback(const common::can_comms_data_msg& recvd_data);
};