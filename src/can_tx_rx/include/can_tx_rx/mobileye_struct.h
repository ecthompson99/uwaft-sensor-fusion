#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"
#include "can_tx_rx/lcc_protocol.c"
#include "can_tx_rx/lcc_protocol.h"

#include "common/mobileye_object_data.h"
#include "common/raw_lane_data.h"

class Mobileye_RX{
    public: 
        Mobileye_RX(ros::NodeHandle* node_handle);
        struct mobileye_object{
            //objection detection
            double obstacle_pos_x_decode;
            bool obstacle_pos_x_is_in_range; 
            double obstacle_pos_y_decode; 
            bool obstacle_pos_y_is_in_range; 

            double obstacle_vel_x_decode;
            bool obstacle_vel_x_is_in_range; 

            //lane detection
            double left_curvature_decode; 
            bool left_curvature_is_in_range; 
            double left_curvature_derivative_decode;
            bool left_curvature_derivative_is_in_range; 
            double left_position_decode; 
            bool left_position_is_in_range; 
            double left_heading_angle_decode; 
            bool left_heading_angle_is_in_range; 
            double left_lane_type_decode; 
            bool left_lane_type_is_in_range; 
            double left_quality_decode; 
            bool left_quality_is_in_range;
            
            double right_curvature_decode; 
            bool right_curvature_is_in_range; 
            double right_curvature_derivative_decode;
            bool right_curvature_derivative_is_in_range; 
            double right_position_decode; 
            bool right_position_is_in_range; 
            double right_heading_angle_decode; 
            bool right_heading_angle_is_in_range; 
            double right_lane_type_decode; 
            bool right_lane_type_is_in_range; 
            double right_quality_decode; 
            bool right_quality_is_in_range; 

            long id; 
            uint8_t can_data[8]; 
            unsigned long time_stamp; 
            unsigned int dlc; 
            unsigned int flag; 

            uint8_t channel_number; 
            uint8_t object_number; 
        } mobileye_obj;

        uint8_t get_nums(mobileye_object mobileye_obj);
        //ros::Subscriber mock_sub_test;
        //void sub_callback(const common::mobileye_object_data& output_obj);
        double signal_in_range(double val, bool cond);

        ext_log_data_tsr_t frame_tsr_unpacked; 
        ext_log_data_obstacle_data_a_t frame_a_unpacked; 
        ext_log_data_obstacle_data_b_t frame_b_unpacked; 
        ext_log_data_obstacle_data_c_t frame_c_unpacked; 

        lcc_protocol_lka_left_lane_a_t left_a_unpacked; 
        lcc_protocol_lka_right_lane_a_t right_a_unpacked;
        lcc_protocol_lka_left_lane_b_t left_b_unpacked; 
        lcc_protocol_lka_right_lane_b_t right_b_unpacked;  

        ros::NodeHandle* node_handle;
        ros::Publisher mob_pub;

    private:
        int unpack_return = -1;  // 0 is successful, negative error code

        ros::Time start = ros::Time::now(); 
};