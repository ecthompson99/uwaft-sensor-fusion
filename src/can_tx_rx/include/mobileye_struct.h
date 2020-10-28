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
#define TOPIC_RX_OBJ "Mobileye_CAN_Rx_Object"
#define TOPIC_RX_LANE "Mobileye_CAN_Rx_Lane"
#define TOPIC_DIAG "Mobileye_CAN_Diagnostics"
#define SIZE_OF_MSG 8 

class Mobileye_RX{
    public:
        ros::NodeHandle* node_handle;
        ros::Publisher mob_pub_obj;
        ros::Publisher mob_pub_lane;
        ros::Publisher diag_pub;
        Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
            mob_pub_obj = node_handle->advertise<common::mobileye_object_data>(TOPIC_RX_OBJ,TX_RX_MESSAGE_BUFFER_SIZE);
            diag_pub = node_handle ->advertise<common::sensor_diagnostic_data_msg>(TOPIC_DIAG,TX_RX_MESSAGE_BUFFER_SIZE);
            mob_pub_lane = node_handle->advertise<common::raw_lane_data>(TOPIC_RX_LANE,TX_RX_MESSAGE_BUFFER_SIZE);
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
            double obstacle_cut_in_cut_out_decode;
            bool obstacle_cut_in_cut_out_is_in_range;
            double obstacle_valid_decode;
            bool obstacle_valid_is_in_range;
            double obstacle_status_decode;
            bool obstacle_status_is_in_range;
            double obstacle_type_decode;
            bool obstacle_type_is_in_range;
            double obstacle_cipv_flag_decode;
            bool obstacle_cipv_flag_is_in_range;

            double obstacle_lane_decode; // this is put here for sensor fusion
            bool obstacle_lane_is_in_range;
            double obstacle_id_decode;
            bool obstacle_id_is_in_range;

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

            uint8_t channel_number; 
            unsigned long time_stamp; 
        };

        struct mobileye_diagnostics{
            double headway_valid_decode; 
            bool headway_valid_is_in_range; 
            double failsafe_decode; 
            bool failsafe_is_in_range; 
            double maintenance_decode; 
            bool maintenance_is_in_range; 
        };

        static void get_nums(int id, int &case_num, int &obj_num, int &lk_num){ 
            
            // deafault values set to -1
            obj_num = -1;

            if(id >=1824 && id <=1830){
                case_num = 1; //Traffic Sensor 
            } else if(id >= 1849 && id <= 1876 && id % 3 == 1){
                case_num = 2; //Obstacle A Frame
            } else if(id >= 1850 && id <= 1877 && id % 3 == 2){
                case_num = 3; //Obstacle B Frame
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
            } else if(id == 1792){ // Diagnostics
                case_num == 9;
            }else{
                case_num = 0; // faulted 
            }
            
            if (case_num == 2 || case_num == 3 || case_num == 4){
                obj_num = (id - (1849 + case_num - 2)) / 3; // takes obj number based on object id (start from obj 0)
            }

            if (case_num == 5 || case_num == 6 || case_num == 7 || case_num == 8){
                lk_num = (id - (1894 + case_num - 5)); // Need to confirm this
            }

        }; 


        static double signal_in_range(double val, bool cond){
            return (cond) ? (val) : 0; 
        };

    private:
        ros::Time start = ros::Time::now(); 
};