#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"

//#include "common/mobileye_object_data_msg.h"

#define UNIT_TEST_SUBSCRIBER "can_tx_rx_CH4_UnitTest_Sub"
#define TX_RX_MESSAGE_BUFFER_SIZE 1000
//#define canBITRATE_250K -3
#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8 

class Mobileye_RX{
    public: 
        Mobileye_RX(ros::NodeHandle* node_handle);
        void get_nums();
        ros::Subscriber mock_sub_test;
        void sub_callback(const common::mobileye_object_data_msg& output_obj);
        double signal_in_range(double val, bool cond);

        int hnd; 

    private:
        ros::NodeHandle* node_handle; 
        ros::Publisher mobileye;

        int unpack_return = -1;  // 0 is successful, negative error code
        ros::Time start = ros::Time::now(); 
}

struct mobileye_object{
    
    double obstacle_pos_x_decode;
    bool obstacle_pos_x_is_in_range; 
    double obstacle_pos_y_decode; 
    bool obstacle_pos_y_is_in_range; 

    double obstacle_vel_x_decode;
    bool obstacle_vel_x_is_in_range; 

    //double object_accel_x_decode; 
    //bool obstacle_accel_x_is_in_range; 

    //double obstacle_id_decode;
    //bool obstacle_id_is_in_range; 
    //double obstacle_lane_decode; 
    //bool obstacle_lane_is_in_range; 

    uint8_t id; 
    uint8_t can_data[8]; 
    unsigned long time_stamp; 
    unsigned int dlc; 
    unsigned int flag; 

    uint8_t channel_number; 
    uint8_t object_number; 
} mobileye_obj