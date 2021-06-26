#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "ext_log_data.h"
#include "sensor_diag.h"

#include "common/mobileye_object_data.h"
#include "common/raw_lane_data.h"
#include "common/sensor_diagnostic_flag_CH4.h"

#define canDRIVER_SILENT 1
#define SIZE_OF_MSG 8
#define TX_RX_MESSAGE_BUFFER_SIZE 1000
#define TOPIC_RX_OBJ "Mobileye_CAN_Rx"
#define TOPIC_RX_LANE "Mobileye_CAN_Rx_Lane"
#define TOPIC_DIAG "Mobileye_CAN_Diagnostics"
#define CH4_SERVICE "sensor_diagnostic_CH4"

/// Class pertaining to output of Mobileye
class Mobileye_RX{
    public:
        ros::NodeHandle* node_handle;
        ros::Publisher mob_pub_obj;
        ros::Publisher mob_pub_lane;
        ros::Publisher diag_pub;
        ros::ServiceClient client_ch4;

        /// This node advertises Mobileye related output messages
        Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
            mob_pub_obj = node_handle->advertise<common::mobileye_object_data>(TOPIC_RX_OBJ,TX_RX_MESSAGE_BUFFER_SIZE);
            diag_pub = node_handle ->advertise<common::sensor_diagnostic_data_msg>(TOPIC_DIAG,TX_RX_MESSAGE_BUFFER_SIZE);
            mob_pub_lane = node_handle->advertise<common::raw_lane_data>(TOPIC_RX_LANE,TX_RX_MESSAGE_BUFFER_SIZE);
            client_ch4 = node_handle->serviceClient<common::sensor_diagnostic_flag_CH4>(CH4_SERVICE);
        };
        /// Mobileye object parameters. 
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

        /// Mobileye lane parameters.
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

        /// Mobileye diagnostic parameters
        struct mobileye_diagnostics{
            double headway_valid_decode; 
            bool headway_valid_is_in_range; 
            double failsafe_decode; 
            bool failsafe_is_in_range; 
            double maintenance_decode; 
            bool maintenance_is_in_range; 
        };

        static void get_nums(int id, uint8_t& case_num, int& obj_num) {
          // Currently unused due to change from canRead to canReadSpecific
          // deafault values set to -1
          // obj_num = -1;
          obj_num = 1;
          // AWSDisplay 0x700 is 1792; 0x736 is 1846 - Don't need this
          // LaneDetails 0x737 is 1847; 0x765 is 1893 - Don't need this

          // Obstacle_Data_A 0x739 is 1849
          // Obstacle_Data_B 0x73A is 1850
          // Obstacle_Data_C 0x73B is 1851
          // LKA_Left_Lane_A 0x766 is 1894
          // LKA_Left_Lane_B 0x767 is 1895
          // LKA_Right_Lane_A 0x768 is 1896
          // LKA_Right_Lane_B 0x769 is 1897

          // if(id >=1824 && id <=1830){
          //     case_num = 1; //Traffic Sensor
          // }
          if (id == 1849) {
            case_num = 1;  // Obstacle A Frame
          } else if (id == 1850) {
            case_num = 2;  // Obstacle B Frame
          } else if (id >= 1851) {
            case_num = 3;  // Obstacle C Frame
          } else if (id == 1894) {
            case_num = 4;  // LKA Left Lane Frame A
          } else if (id == 1895) {
            case_num = 5;  // LKA Left Lane Frame B
          } else if (id == 1896) {
            case_num = 6;  // LKA Right Lane Frame A
          } else if (id == 1897) {
            case_num = 7;  // LKA Right Lane Frame B
          } else if (id >= 1792 && id <= 1846) {
            case_num = 8;  // Diagnostics
          } else {
            case_num = 0;  // faulted
          }

          // if(id >=1824 && id <=1830){
          //     case_num = 1; //Traffic Sensor
          // } else if(id >= 1849 && id <= 1876 && id % 3 == 1){
          //     case_num = 2; //Obstacle A Frame
          // } else if(id >= 1850 && id <= 1877 && id % 3 == 2){
          //     case_num = 3; //Obstacle B Frame
          // } else if(id >= 1851 && id <= 1878 && id % 3 == 0){
          //     case_num = 4; //Obstacle C Frame
          // } else if(id == 1894){
          //     case_num = 5; //LKA Left Lane Frame A
          // } else if(id == 1895){
          //     case_num = 6; //LKA Left Lane Frame B
          // } else if(id == 1896){
          //     case_num = 7; //LKA Right Lane Frame A
          // } else if(id == 1897){
          //     case_num = 8; //LKA Right Lane Frame B
          // } else if(id == 1792){ // Diagnostics
          //     case_num == 9;
          // }else{
          //     case_num = 0; // faulted
          // }

          // ID's below from ExtLogData2.dbc
          // if (case_num >=2 && case_num <= 4){
          // if (case_num >=1 && case_num <= 3){
          //     /* Obs A frame: 1876, 1873, 1870, 1867, 1864, 1861, 1858, 1855, 1852, 1849
          //     Obs B frame: 1877, 1874, 1871, 1868, 1865, 1862, 1859, 1856, 1853, 1850
          //     Obs C frame: 1878, 1875, 1872, 1869, 1866, 1863,1860, 1857, 1854, 1851
          //     Lowest ID is 1849, increments by 3. Formula allows for calculating object ID from 0-9 given the case
          //     where the IDs are A,B, or C frame
          //     eg. if the id is 1867, (1867 - (1849 + 2-2))/3 = object #6
          //     */
          //     obj_num = (id - (1849 + case_num - 2)) / 3;
          // }
        };

        static double signal_in_range(double val, bool cond) { return (cond) ? (val) : 0; };

       private:
        ros::Time start = ros::Time::now(); 
};