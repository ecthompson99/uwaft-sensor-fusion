#include <canlib.h>

#include <stdio.h>
#include <sstream>
#include <vector>
#include "ros/ros.h"
#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"
#include "can_tx_rx/output_structs.h"

#include "can_tx_rx/mobileye_object_data_msg.h"

struct radar_diagnostic_response {
  double diagnostic_decoded;
  bool diagnostic_is_in_range;

  uint8_t channel_number;
  unsigned long timestamp;
  uint8_t radar_number;
} diag_response;

struct radar_information {
  double radar_timestamp_decoded;
  bool radar_timestamp_is_in_range;
  double tc_counter_decoded;
  bool tc_counter_is_in_range;
  double obj_ender_consist_bit_decoded;
  bool obj_ender_consist_bit_is_in_range;
  double packet_checksum_encoded;
  bool packet_checksum_is_in_range;

  double veh_psi_dt_decoded;
  bool veh_psi_dt_is_in_range;
  double veh_v_ego_decoded;
  bool veh_v_ego_is_in_range;
  double veh_a_ego_decoded;
  bool veh_a_ego_is_in_range;
  double veh_slip_angle_decoded;
  bool veh_slip_angle_is_in_range;
  double mess_starter_consist_bit_decoded;
  bool mess_starter_consist_bit_is_in_range;

  double itc_info_decoded;
  bool itc_info_is_in_range;
  double sgu_fail_decoded;
  bool sgu_fail_is_in_range;
  double hw_fail_decoded;
  bool hw_fail_is_in_range;
  double horizontal_misalignment_decoded;
  bool horizontal_misalignment_is_in_range;
  double absorption_blindness_decoded;
  bool absorption_blindness_is_in_range;
  double distortion_blindness_decoded;
  bool distortion_blindness_is_in_range;
  double mc_decoded;
  bool mc_is_in_range;
  double crc_decoded;
  bool crc_is_in_range;

  uint8_t channel_number;
  unsigned long timestamp;
  uint8_t radar_number;
  uint8_t calculated_checksum;
} radar_info;

struct target_tracking_info {
  double target_dx_decoded;
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
  double target_vy_decoded;
  bool target_vy_is_in_range;
  double target_d_length_decoded;
  bool target_d_length_is_in_range;
  double target_dz_decoded;
  bool target_dz_is_in_range;
  double target_moving_state_decoded;
  bool target_moving_state_is_in_range;
  double target_dx_sigma_decoded;
  bool target_dx_sigma_is_in_range;
  double target_vx_sigma_decoded;
  bool target_vx_sigma_is_in_range;
  double target_ax_sigma_decoded;
  bool target_ax_sigma_is_in_range;
  double target_dy_sigma_decoded;
  bool target_dy_sigma_is_in_range;
  double target_w_class_decoded;
  bool target_w_class_is_in_range;
  double target_class_decoded;
  bool target_class_is_in_range;
  double target_dx_rear_end_loss_decoded;
  bool target_dx_rear_end_loss_is_in_range;
  double target_mess_bconsist_bit_decoded;
  bool target_mess_bconsist_bit_is_in_range;

  uint8_t channel_number;
  unsigned long timestamp;
  uint8_t radar_number;
  uint8_t target_object_number;
} target_info;

struct object_tracking_info {
  double dx_decoded;
  bool dx_is_in_range;
  double vx_decoded;
  bool vx_is_in_range;
  double dy_decoded;
  bool dy_is_in_range;
  double w_exist_decoded;
  bool w_exist_is_in_range;
  double ax_decoded;
  bool ax_is_in_range;
  double w_obstacle_decoded;
  bool w_obstacle_is_in_range;
  double flag_valid_decoded;
  bool flag_valid_is_in_range;
  double w_non_obstacle_decoded;
  bool w_non_obstacle_is_in_range;
  double flag_meas_decoded;
  bool flag_meas_is_in_range;
  double flag_hist_decoded;
  bool flag_hist_is_in_range;
  double mess_aconsist_bit_decoded;
  bool mess_aconsist_bit_is_in_range;

  double vy_decoded;
  bool vy_is_in_range;
  double d_length_decoded;
  bool d_length_is_in_range;
  double dz_decoded;
  bool dz_is_in_range;
  double moving_state_decoded;
  bool moving_state_is_in_range;
  double dx_sigma_decoded;
  bool dx_sigma_is_in_range;
  double vx_sigma_decoded;
  bool vx_sigma_is_in_range;
  double ax_sigma_decoded;
  bool ax_sigma_is_in_range;
  double dy_sigma_decoded;
  bool dy_sigma_is_in_range;
  double w_class_decoded;
  bool w_class_is_in_range;
  double class_decoded;
  bool class_is_in_range;
  double dx_rear_end_loss_decoded;
  bool dx_rear_end_loss_is_in_range;
  double mess_bconsist_bit_decoded;
  bool mess_bconsist_bit_is_in_range;

  uint8_t channel_number;
  unsigned long timestamp;
  uint8_t radar_number;
  uint8_t object_number;
} all_object_info;
