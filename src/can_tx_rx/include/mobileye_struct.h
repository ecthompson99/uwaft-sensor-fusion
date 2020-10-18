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
            
            double obstacle_lane_decode;
            bool obstacle_lane_decode_is_in_range;

            double obstacle_id_decode;
            bool obstacle_id_is_in_range; 

            long id; 
            uint8_t can_data[8]; 
            unsigned long time_stamp; 
            unsigned int dlc; 
            unsigned int flag; 

            uint8_t channel_number; 
            uint8_t object_number; 
        };

        static uint8_t get_nums(mobileye_object mobileye_obj) {
            if(mobileye_obj.id >=1824 && mobileye_obj.id <=1830){
                return 1; //Traffic Sensor 
            } else if(mobileye_obj.id >= 1849 && mobileye_obj.id <= 1876 && mobileye_obj.id % 3 == 1){
                return 2; //Obstacle A Frame
            } else if(mobileye_obj.id >= 1850 && mobileye_obj.id <= 1877 && mobileye_obj.id % 3 == 2){
                return 3; //Obstacle B Frame
            } else if(mobileye_obj.id >= 1851 && mobileye_obj.id <= 1878 && mobileye_obj.id % 3 == 0){
                return 4; //Obstacle C Frame
            } else if(mobileye_obj.id == 1894){
                return 5; //LKA Left Lane Frame A 
            } else if(mobileye_obj.id == 1895){
                return 6; //LKA Left Lane Frame B 
            } else if(mobileye_obj.id == 1896){
                return 7; //LKA Right Lane Frame A 
            } else if(mobileye_obj.id == 1897){
                return 8; //LKA Right Lane Frame B 
            } else{
                return 0; 
            }
        };

        static double signal_in_range(double val, bool cond){
            return (cond) ? (val) : 0; 
        };

    private:
        ros::Time start = ros::Time::now(); 
};