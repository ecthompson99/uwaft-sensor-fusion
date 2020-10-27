#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "radar.h"
#include "sensor_diag.h"

#include "common/radar_object_data.h"

#define TX_RX_MESSAGE_BUFFER_SIZE 1000
#define TOPIC_RX "Radar_CAN_RX"
#define TOPIC_DIAG "Radar_CAN_Diagnostics"
#define SIZE_OF_MSG 8 

class Radar_RX{
  public:
    ros::NodeHandle* node_handle;
    ros::Publisher rad_pub;
    ros::Publisher diag_pub; 

    Radar_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
      rad_pub = node_handle->advertise<common::radar_object_data>(TOPIC_RX,TX_RX_MESSAGE_BUFFER_SIZE);
      diag_pub = node_handle->advertise<common::sensor_diagnostic_data_msg>(TOPIC_DIAG, TX_RX_MESSAGE_BUFFER_SIZE);
    };
    struct radar_diagnostic_response {
      double diagnostic_decode;
      bool diagnostic_is_in_range;

      unsigned long timestamp;
      uint8_t radar_number;
    };

    struct radar_information {
      double radar_timestamp_decode;
      bool radar_timestamp_is_in_range;
      double tc_counter_decode;
      bool tc_counter_is_in_range;
      double obj_ender_consist_bit_decode;
      bool obj_ender_consist_bit_is_in_range;
      double packet_checksum_decode;
      bool packet_checksum_is_in_range;

      double veh_psi_dt_decode;
      bool veh_psi_dt_is_in_range;
      double veh_v_ego_decode;
      bool veh_v_ego_is_in_range;
      double veh_a_ego_decode;
      bool veh_a_ego_is_in_range;
      double veh_slip_angle_decode;
      bool veh_slip_angle_is_in_range;
      double mess_starter_consist_bit_decode;
      bool mess_starter_consist_bit_is_in_range;

      double itc_info_decode;
      bool itc_info_is_in_range;
      double sgu_fail_decode;
      bool sgu_fail_is_in_range;
      double hw_fail_decode;
      bool hw_fail_is_in_range;
      double horizontal_misalignment_decode;
      bool horizontal_misalignment_is_in_range;
      double absorption_blindness_decode;
      bool absorption_blindness_is_in_range;
      double distortion_blindness_decode;
      bool distortion_blindness_is_in_range;
      double mc_decode;
      bool mc_is_in_range;
      double crc_decode;
      bool crc_is_in_range;

      uint8_t channel_number;
      unsigned long timestamp;
      uint8_t radar_number;
      uint8_t calculated_checksum;
    };

    struct target_tracking_info {
      double target_dx_decode;
      bool target_dx_is_in_range;
      double target_vx_decode;
      bool target_vx_is_in_range;
      double target_dy_decode;
      bool target_dy_is_in_range;
      double target_w_exist_decode;
      bool target_w_exist_is_in_range;
      double target_ax_decode;
      bool target_ax_is_in_range;
      double target_w_obstacle_decode;
      bool target_w_obstacle_is_in_range;
      double target_flag_valid_decode;
      bool target_flag_valid_is_in_range;
      double target_w_non_obstacle_decode;
      bool target_w_non_obstacle_is_in_range;
      double target_flag_meas_decode;
      bool target_flag_meas_is_in_range;
      double target_flag_hist_decode;
      bool target_flag_hist_is_in_range;
      double target_mess_aconsist_bit_decode;
      bool target_mess_aconsist_bit_is_in_range;
      double target_vy_decode;
      bool target_vy_is_in_range;
      double target_d_length_decode;
      bool target_d_length_is_in_range;
      double target_dz_decode;
      bool target_dz_is_in_range;
      double target_moving_state_decode;
      bool target_moving_state_is_in_range;
      double target_dx_sigma_decode;
      bool target_dx_sigma_is_in_range;
      double target_vx_sigma_decode;
      bool target_vx_sigma_is_in_range;
      double target_ax_sigma_decode;
      bool target_ax_sigma_is_in_range;
      double target_dy_sigma_decode;
      bool target_dy_sigma_is_in_range;
      double target_w_class_decode;
      bool target_w_class_is_in_range;
      double target_class_decode;
      bool target_class_is_in_range;
      double target_dx_rear_end_loss_decode;
      bool target_dx_rear_end_loss_is_in_range;
      double target_mess_bconsist_bit_decode;
      bool target_mess_bconsist_bit_is_in_range;

      uint8_t channel_number;
      unsigned long timestamp;
      uint8_t radar_number;
      uint8_t target_object_number;
    };

    struct object_tracking_info {
      double dx_decode;
      bool dx_is_in_range;
      double vx_decode;
      bool vx_is_in_range;
      double dy_decode;
      bool dy_is_in_range;
      double w_exist_decode;
      bool w_exist_is_in_range;
      double ax_decode;
      bool ax_is_in_range;
      double w_obstacle_decode;
      bool w_obstacle_is_in_range;
      double flag_valid_decode;
      bool flag_valid_is_in_range;
      double w_non_obstacle_decode;
      bool w_non_obstacle_is_in_range;
      double flag_meas_decode;
      bool flag_meas_is_in_range;
      double flag_hist_decode;
      bool flag_hist_is_in_range;
      double mess_aconsist_bit_decode;
      bool mess_aconsist_bit_is_in_range;

      double vy_decode;
      bool vy_is_in_range;
      double d_length_decode;
      bool d_length_is_in_range;
      double dz_decode;
      bool dz_is_in_range;
      double moving_state_decode;
      bool moving_state_is_in_range;
      double dx_sigma_decode;
      bool dx_sigma_is_in_range;
      double vx_sigma_decode;
      bool vx_sigma_is_in_range;
      double ax_sigma_decode;
      bool ax_sigma_is_in_range;
      double dy_sigma_decode;
      bool dy_sigma_is_in_range;
      double w_class_decode;
      bool w_class_is_in_range;
      double class_decode;
      bool class_is_in_range;
      double dx_rear_end_loss_decode;
      bool dx_rear_end_loss_is_in_range;
      double mess_bconsist_bit_decode;
      bool mess_bconsist_bit_is_in_range;

      uint8_t channel_number;
      unsigned long timestamp;
      uint8_t radar_number;
      uint8_t object_number;
    };

    static void get_nums(int id, int &case_n, int &radar_n, int &frame_n, int &obj_n, int &target_obj_n) {
      if (id == 1985 || id == 1958 || id == 1879 || id == 1957) {
        case_n = 1; //diag responses and requests
      } else if (id > 1604 && id < 1659) {
        case_n = 2; //target A and B frames (?) the IDs are incorrectly calculated from the dbc 
      } else if (id == 1665 || id == 1667 || id == 1280 || id == 1282 || id == 1670 || id == 1672) {
        case_n = 3; //ender, starter, and statuses messages
      } else if (id > 1284 && id < 1599) {
        case_n = 4; //radar A and B object frames 
      } else {
        case_n = 0; //faulted 
      }
      
      //default these values should be set to -1 (0 is used)
      obj_n = -1;
      target_obj_n = -1; 
      radar_n = -1;
      frame_n = -1; 

      switch (case_n) {
        case 1: //diag responses and requests
          if (id == 1985 || id == 1879) {
            radar_n = 1;//radar 1 in dbc  
          } else if (id == 1958 || id == 1957){
            radar_n = 2;//radar 2 in dbc 
          }
          break;

        case 2: //target A and B frames 
          if (id % 10 == 5 || id % 10 == 6) {
            radar_n = 1;//radar 1 in dbc (all ids for targets end with a 5 or a 6)
          } else if (id %10 == 7 || id % 10 == 8){
            radar_n = 2; //radar 2 in dbc 
          }

          if (id % 10 == 5 || id % 10 == 7) {
            frame_n = 1; //a frame in dbc (all ids for targets end with a 5 if they are radar 1, or 7 if they are  radar 2)
          } else if (id %10 == 6 || id % 10 == 8){
            frame_n = 2; //frame b in dbc 
          }

          target_obj_n = (id - 1600 - (id % 10)) / 10; //takes the target object number based on the defined id 

          break;

        case 3: //ender, starter, and statuses messages
          if (id == 1665 || id == 1280 || id == 1670) {
            radar_n = 1; //radar 1 in dbc 
          } else if (id == 1667 || id == 1282 || id == 1672){
            radar_n = 2; //radar 2 in dbc 
          }
          break;

        case 4://radar A and B object frames 
          if (id % 10 == 5 || id % 10 == 6) {
            radar_n = 1; //radar 1 in dbc (all ids follow the same convention as target messages)
          } else if (id %10 == 7 || id % 10 == 8){
            radar_n = 2; //radar 2 in dbc 
          }

          if (id % 10 == 5 || id % 10 == 7) {
            frame_n = 1; //a frame in dbc (all ids follow the same convention as target messages)
          } else  if(id % 10 == 6 || id % 10 == 8){
            frame_n = 2; //b frame in dbc 
          }

          obj_n = (id - 1280 - (id % 10)) / 10; //takes the tracked object number based on the defined id 

          break;
      }
    };

    static double signals_in_range(double val, bool cond){
        return (cond) ? (val) : 0; 
    };
  
  private:
    ros::Time start = ros::Time::now(); 
};

