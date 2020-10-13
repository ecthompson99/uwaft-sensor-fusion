#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/radar.c"
#include "can_tx_rx/radar.h"

#include "common/radar_object_data.h"

class Radar_RX{
  public:
    Radar_RX(ros::NodeHandle* node_handle);
    struct radar_diagnostic_response {
      double diagnostic_decode;
      bool diagnostic_is_in_range;

      uint8_t channel_number;
      unsigned long timestamp;
      uint8_t radar_number;
    } diag_response;

    struct radar_information {
      double radar_timestamp_decode;
      bool radar_timestamp_is_in_range;
      double tc_counter_decode;
      bool tc_counter_is_in_range;
      double obj_ender_consist_bit_decode;
      bool obj_ender_consist_bit_is_in_range;
      double packet_checksum_encoded;
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
    } radar_info;

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
    } target_info;

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
    } object_info;

    uint8_t get_nums(int id, uint8_t &case_n, uint8_t &radar_n, uint8_t &frame_n, uint8_t &obj_n, uint8_t &target_obj_n);

    double signal_in_range(double val, bool cond);

    radar_radar_diag_response_t diag_response_unpacked; 
    radar_radar_diag_request_t diag_request_unpacked;
    
    radar_radar_a_t radar_a_unpacked; 
    radar_radar_b_t radar_b_unpacked;

    radar_radar_object_ender_t radar_object_ender_unpacked;
    radar_radar_object_starter_t radar_object_starter_unpacked;
    radar_radar_status_t radar_status_unpacked;

    ros::NodeHandle* node_handle;
    ros::Publisher rad_pub;
  
  private:
    int unpack_return = -1; // 0 is successful, negative error code

    ros::Time start = ros::Time::now(); 
};

