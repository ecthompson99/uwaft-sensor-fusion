#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "ext_log_data.h"
#include "sensor_diag.h"

#include "common/mobileye_object_data.h"
#include "common/raw_lane_data.h"

#define TX_RX_MESSAGE_BUFFER_SIZE 1000
#define TOPIC_RX "Mobileye_CAN_Rx"
#define TOPIC_DIAG "Mobileye_CAN_Diagnostics"
#define SIZE_OF_MSG 8 

class Mobileye_RX{
    public:
        ros::NodeHandle* node_handle;
        ros::Publisher mob_pub;
        ros::Publisher diag_pub;
        Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
            mob_pub = node_handle->advertise<common::mobileye_object_data>(TOPIC_RX,TX_RX_MESSAGE_BUFFER_SIZE);
            diag_pub = node_handle ->advertise<common::sensor_diagnostic_data_msg>(TOPIC_DIAG,TX_RX_MESSAGE_BUFFER_SIZE);
        };
        struct mobileye_object{
            double obstacle_pos_x_decode;
            bool obstacle_pos_x_is_in_range; 
            double obstacle_pos_y_decode; 
            bool obstacle_pos_y_is_in_range; 
            double obstacle_vel_x_decode;
            bool obstacle_vel_x_is_in_range; 
            double obstacle_accel_x_decode;
            bool obstacle_accel_x_is_in_range;
          
            double obstacle_age_decode;
            bool  obstacle_age_is_in_range;
            double obstacle_width_decode;
            bool obstacle_width_is_in_range;
            double obstacle_length_decode;
            bool obstacle_length_is_in_range;

            double obstacle_id_decode;
            bool obstacle_id_is_in_range;

            double obstacle_cut_in_cut_out_decode;
            bool obstacle_cut_in_cut_out_is_in_range;
            double obstacle_blinker_info_decode;
            bool obstacle_blinker_info_is_in_range;
            double obstacle_valid_decode;
            bool obstacle_valid_is_in_range;
            double obstacle_brake_lights_decode;
            bool obstacle_brake_lights_is_in_range;
            double obstacle_status_decode;
            bool obstacle_status_is_in_range;
            double obstacle_type_decode;
            bool obstacle_type_is_in_range;

            double obstacle_replaced_decode;
            bool obstacle_replaced_is_in_range;
            double obstacle_angle_decode;
            bool obstacle_angle_is_in_range;
            double obstacle_scale_change_decode;
            bool obstacle_scale_change_is_in_range;
            double obstacle_angle_rate_decode;
            bool obstacle_angle_rate_is_in_range;

            uint8_t channel_number; 
            unsigned long time_stamp; 
            
        };

        struct mobileye_lane{
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
            
            double obstacle_lane_decode;
            bool obstacle_lane_is_in_range;

            uint8_t channel_number; 
            unsigned long time_stamp; 
        };

        static void get_nums(int id, int &case_num, int &obj_num){ 
            if(id >=1824 && id <=1830){
                case_num = 1; //Traffic Sensor 
            } else if(id >= 1849 && id <= 1876 && id % 3 == 1){
                case_num = 2; //Obstacle A Frame
            } else if(id >= 1850 && id <= 1877 && id % 3 == 2){
                case)num = 3; //Obstacle B Frame
            } else if(id >= 1851 && id <= 1878 && id % 3 == 0){
                case_num = 4; //Obstacle C Frame
            } else if(id == 1894){
                case_num = 5; //LKA Left Lane Frame A 
            } else if(id == 1895){
                case_num = 6; //LKA Left Lane Frame B 
            } else if(id == 1896){
                case_num = 7; //LKA Right Lane Frame A 
            } else if(id == 1897){
                case_num = 8; //LKA Right Lane Frame B 
            } else{
                case_num = 0; // faulted 
            }

            // deafault values set to -1
            obj_num = -1;

            switch (case_num){
                case 1: // traffic response
                case 2: // obstacle frame A
                case 3: // obstacle frame B
                case 4: // obstacle frame C
                case 5: // left lane frame A
                case 6: // left lane frame B
                case 7: // right lane frame A
                case 8: // right lane frame B
            }
        }; 


        static double signal_in_range(double val, bool cond){
            return (cond) ? (val) : 0; 
        };

    private:
        ros::Time start = ros::Time::now(); 
};