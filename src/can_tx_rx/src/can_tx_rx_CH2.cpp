#include <canlib.h>

#include <stdio.h>
#include <sstream>

#include "ros/ros.h"

#include "can_tx_rx/xgu.h"
#include "can_tx_rx/xgu.c"

#include "can_tx_rx/sensor_diagnostic_data_msg.h"
#include "can_tx_rx/raw_sensor_object_data_msg.h"

static const uint16_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;

struct radar_diagnostic_response{
  double diagnostic_decoded;
  bool diagnostic_is_in_range;
}diag_response;

struct radar_information{
  double xgu_radar_timestamp_decoded;
  bool xgu_radar_timestamp_is_in_range;
  double tc_counter_decoded;
  bool tc_counter_is_in_range;
  double obj_ender_consist_bit_decoded;
  bool obj_ender_consist_bit_is_in_range;
  double packet_checksum_encoded;
  bool packet_checksum_is_in_range;
}radar_info;

struct target_tracking_info{
  double target_info.target_dx_decoded;
  bool target_info.target_dx_is_in_range;
  double target_info.target_vx_decode;
  bool target_info.target_vx_is_in_range;
  double target_info.target_dy_decode;
  bool target_info.target_dy_is_in_range;
  double target_info.target_w_exist_decode;
  bool target_info.target_w_exist_is_in_range;
  double target_info.target_ax_decode;
  bool target_info.target_ax_is_in_range;
  double target_info.target_w_obstacle_decode;
  bool target_info.target_w_obstacle_is_in_range;
  double target_info.target_flag_valid_decode;
  bool target_info.target_flag_valid_is_in_range;
  double target_info.target_w_non_obstacle_decode;
  bool target_info.target_w_non_obstacle_is_in_range;
  double target_info.target_flag_meas_decode;
  bool target_info.target_flag_meas_is_in_range;
  double target_info.target_flag_hist_decode;
  bool target_info.target_flag_hist_is_in_range;
  double target_info.target_mess_aconsist_bit_decode;
  bool target_info.target_mess_aconsist_bit_is_in_range;
  double target_info.target_vy_decoded;
  bool target_info.target_vy_is_in_range;
  double target_info.target_d_length_decoded;
  bool target_info.target_d_length_is_in_range;
  double target_info.target_dz_decoded;
  bool target_info.target_dz_is_in_range;
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
}target_info;

void get_nums(int id, uint8_t &case_n, uint8_t &radar_n, uint8_t &frame_n, uint8_t &obj_n, uint8_t &target_obj_n) {
  if (id == 1985 || id == 1958 || id == 1879 || id == 1957) {
    case_n = 1;
  } else if (id > 1604 && id < 1659) {
    case_n = 2;
  } else if (id == 1665 || id == 1667 || id == 1280 || id == 1282 || id == 1670 || id == 1672) {
    case_n = 3;
  } else if (id > 1284 && id < 1599) {
    case_n = 4;
  } else {
    case_n = 0;
  }

  switch (case_n) {
    case 1:
      if (id == 1985 || id == 1879) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }
      break;

    case 2:
      if (id % 10 == 5 || id % 10 == 6) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }

      if (id % 10 == 5 || id % 10 == 7) {
        frame_n = 1;
      } else {
        frame_n = 2;
      }

      target_obj_n = (id - 1600 - (id % 10)) / 10;

      break;

    case 3:
      if (id == 1665 || id == 1280 || id == 1670) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }
      break;

    case 4:
      if (id % 10 == 5 || id % 10 == 6) {
        radar_n = 1;
      } else {
        radar_n = 2;
      }

      if (id % 10 == 5 || id % 10 == 7) {
        frame_n = 1;
      } else {
        frame_n = 2;
      }

      obj_n = (id - 1280 - (id % 10)) / 10;

      break;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "can_tx_rx_CH2");
  ros::NodeHandle can_tx_rx_CH2_handle;

  ros:: Publisher diag_data_pub = can_tx_rx_CH2_handle.advertise<
        can_tx_rx::sensor_diagnostic_data_msg>("sensor_diagnostic_data", TX_RX_MESSAGE_BUFFER_SIZE);

  ros:: Publisher raw_obj_data_pub = can_tx_rx_CH2_handle.advertise<
        can_tx_rx::raw_sensor_object_data_msg>("raw_sensor_object_data", TX_RX_MESSAGE_BUFFER_SIZE); 
        
  can_tx_rx::sensor_diagnostic_data_msg diag_data_msg;
  can_tx_rx::raw_sensor_object_data_msg raw_obj_data_msg;

  canHandle hnd;

  canInitializeLibrary();

  hnd = canOpenChannel(1, canOPEN_EXCLUSIVE);

  if (hnd < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  canBusOn(hnd);

  long int id;
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  uint8_t case_num = 0;
  uint8_t radar_num = 0;           // 1 or 2 = valid
  uint8_t frame_num = 0;           // 1 = Frame_A, 2 = Frame_B, 3 = general, other = error
  uint8_t obj_num = -1;            // 0 to 31 = valid
  uint8_t target_object_num = -1;  // 0 to 5 = valid
  uint8_t size_of_msg = 8;
  uint8_t can_data[8] = {0};

  int target_info.unpack_return = -1;  // 0 is successful, negative error code

  uint8_t serialized_radar_diag_response[sizeof(diag_response)];
  uint8_t serialized_radar_info[sizeof(radar_info)];

  double diag_request = 0;
  uint64_t diag_request_encoded = 0;


  double veh_psi_dt_decoded = 0;
  bool veh_psi_dt_is_in_range = 0;
  double veh_v_ego_decoded = 0;
  bool veh_v_ego_is_in_range = 0;
  double veh_a_ego_decoded = 0;
  bool veh_a_ego_is_in_range = 0;
  double veh_slip_angle_decoded = 0;
  bool veh_slip_angle_is_in_range = 0;
  double mess_starter_consist_bit_decoded = 0;
  bool mess_starter_consist_bit_is_in_range = 0;

  double itc_info_decoded = 0;
  bool itc_info_is_in_range = 0;
  double sgu_fail_decoded = 0;
  bool sgu_fail_is_in_range = 0;
  double hw_fail_decoded = 0;
  bool hw_fail_is_in_range = 0;
  double horizontal_misalignment_decoded = 0;
  bool horizontal_misalignment_is_in_range = 0;
  double absorption_blindness_decoded = 0;
  bool absorption_blindness_is_in_range = 0;
  double distortion_blindness_decoded = 0;
  bool distortion_blindness_is_in_range = 0;
  double mc_decoded = 0;
  bool mc_is_in_range = 0;
  double crc_decoded = 0;
  bool crc_is_in_range = 0;

  double dx_decoded = 0;
  bool dx_is_in_range = 0;
  double vx_decoded = 0;
  bool vx_is_in_range = 0;
  double dy_decoded = 0;
  bool dy_is_in_range = 0;
  double w_exist_decoded = 0;
  bool w_exist_is_in_range = 0;
  double ax_decoded = 0;
  bool ax_is_in_range = 0;
  double w_obstacle_decoded = 0;
  bool w_obstacle_is_in_range = 0;
  double flag_valid_decoded = 0;
  bool flag_valid_is_in_range = 0;
  double w_non_obstacle_decoded = 0;
  bool w_non_obstacle_is_in_range = 0;
  double flag_meas_decoded = 0;
  bool flag_meas_is_in_range = 0;
  double flag_hist_decoded = 0;
  bool flag_hist_is_in_range = 0;
  double mess_aconsist_bit_decoded = 0;
  bool mess_aconsist_bit_is_in_range = 0;

  double vy_decoded = 0;
  bool vy_is_in_range = 0;
  double d_length_decoded = 0;
  bool d_length_is_in_range = 0;
  double dz_decoded = 0; 
  bool dz_is_in_range = 0;
  double moving_state_decoded = 0;
  bool moving_state_is_in_range = 0;
  double dx_sigma_decoded = 0;
  bool dx_sigma_is_in_range = 0;
  double vx_sigma_decoded = 0;
  bool vx_sigma_is_in_range = 0;
  double ax_sigma_decoded = 0;
  bool ax_sigma_is_in_range = 0;
  double dy_sigma_decoded = 0;
  bool dy_sigma_is_in_range = 0;
  double w_class_decoded = 0;
  bool w_class_is_in_range = 0;
  double class_decoded = 0;
  bool class_is_in_range = 0;
  double dx_rear_end_loss_decoded = 0;
  bool dx_rear_end_loss_is_in_range = 0;
  double mess_bconsist_bit_decoded = 0;
  bool mess_bconsist_bit_is_in_range = 0;

  while (ros::ok()) {
    canStatus stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);

    if (canOK == stat) {
      // Front radar = radar_1 and rear radar = radar_2
      get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num);
      switch (case_num) {
        case 1:
          switch (id) {
            case 1985:
              xgu_radar1_diag_response_t r1_diag_response_obj;
              target_info.unpack_return = xgu_radar1_diag_response_unpack(&r1_diag_response_obj, can_data, size_of_msg);
              diag_response.diagnostic_decoded =
                  xgu_radar1_diag_response_r1_diag_response_decode(r1_diag_response_obj.r1_diag_response);
              diag_response.diagnostic_is_in_range =
                  xgu_radar1_diag_response_r1_diag_response_is_in_range(r1_diag_response_obj.r1_diag_response);
              break;
            case 1958:
              xgu_radar2_diag_response_t r2_diag_response_obj;
              target_info.unpack_return = xgu_radar2_diag_response_unpack(&r2_diag_response_obj, can_data, size_of_msg);
              diag_response.diagnostic_decoded =
                  xgu_radar2_diag_response_r2_diag_response_decode(r2_diag_response_obj.r2_diag_response);
              diag_response.diagnostic_is_in_range =
                  xgu_radar2_diag_response_r2_diag_response_is_in_range(r2_diag_response_obj.r2_diag_response);
              break;
          }
        memcpy(serialized_radar_diag_response, &diag_response, sizeof(diag_response));
        break;
        case 2:
          switch (id) {
            case 1605:
              xgu_radar1_target00_a_t r1_target00_a_obj;
              target_info.unpack_return = xgu_radar1_target00_a_unpack(&r1_target00_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar1_target00_a_radar1_target00_dx_decode(r1_target00_a_obj.radar1_target00_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_dx_is_in_range(r1_target00_a_obj.radar1_target00_dx);
              target_info.target_vx_decode = xgu_radar1_target00_a_radar1_target00_vx_decode(r1_target00_a_obj.radar1_target00_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_vx_is_in_range(r1_target00_a_obj.radar1_target00_vx);
              target_info.target_dy_decode = xgu_radar1_target00_a_radar1_target00_dy_decode(r1_target00_a_obj.radar1_target00_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_dy_is_in_range(r1_target00_a_obj.radar1_target00_dy);
              target_info.target_w_exist_decode =
                  xgu_radar1_target00_a_radar1_target00_w_exist_decode(r1_target00_a_obj.radar1_target00_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_w_exist_is_in_range(r1_target00_a_obj.radar1_target00_w_exist);
              target_info.target_ax_decode = xgu_radar1_target00_a_radar1_target00_ax_decode(r1_target00_a_obj.radar1_target00_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_ax_is_in_range(r1_target00_a_obj.radar1_target00_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar1_target00_a_radar1_target00_w_obstacle_decode(r1_target00_a_obj.radar1_target00_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar1_target00_a_radar1_target00_w_obstacle_is_in_range(
                  r1_target00_a_obj.radar1_target00_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar1_target00_a_radar1_target00_flag_valid_decode(r1_target00_a_obj.radar1_target00_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar1_target00_a_radar1_target00_flag_valid_is_in_range(
                  r1_target00_a_obj.radar1_target00_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar1_target00_a_radar1_target00_w_non_obstacle_decode(
                  r1_target00_a_obj.radar1_target00_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar1_target00_a_radar1_target00_w_non_obstacle_is_in_range(
                  r1_target00_a_obj.radar1_target00_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar1_target00_a_radar1_target00_flag_meas_decode(r1_target00_a_obj.radar1_target00_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar1_target00_a_radar1_target00_flag_meas_is_in_range(
                  r1_target00_a_obj.radar1_target00_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar1_target00_a_radar1_target00_flag_hist_decode(r1_target00_a_obj.radar1_target00_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar1_target00_a_radar1_target00_flag_hist_is_in_range(
                  r1_target00_a_obj.radar1_target00_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar1_target00_a_radar1_target00_mess_aconsist_bit_decode(
                  r1_target00_a_obj.radar1_target00_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_mess_aconsist_bit_is_in_range(
                      r1_target00_a_obj.radar1_target00_mess_aconsist_bit);
              break;
            case 1606:
              xgu_radar1_target00_b_t r1_target00_b_obj;
              target_info.unpack_return = xgu_radar1_target00_b_unpack(&r1_target00_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar1_target01_b_radar1_target01_vy_decode(r1_target00_b_obj.radar1_target00_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_vy_is_in_range(r1_target00_b_obj.radar1_target00_vy);
              target_info.target_d_length_decoded =
                  xgu_radar1_target01_b_radar1_target01_d_length_decode(r1_target00_b_obj.radar1_target00_d_length);
              target_info.target_d_length_is_in_range = xgu_radar1_target01_b_radar1_target01_d_length_is_in_range(
                  r1_target00_b_obj.radar1_target00_d_length);
              target_info.target_dz_decoded = xgu_radar1_target01_b_radar1_target01_dz_decode(r1_target00_b_obj.radar1_target00_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_dz_is_in_range(r1_target00_b_obj.radar1_target00_dz);
              target_moving_state_decoded = xgu_radar1_target01_b_radar1_target01_moving_state_decode(
                  r1_target00_b_obj.radar1_target00_moving_state);
              target_moving_state_is_in_range = xgu_radar1_target01_b_radar1_target01_moving_state_is_in_range(
                  r1_target00_b_obj.radar1_target00_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_dx_sigma_decode(r1_target00_b_obj.radar1_target00_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_dx_sigma_is_in_range(
                  r1_target00_b_obj.radar1_target00_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_vx_sigma_decode(r1_target00_b_obj.radar1_target00_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_vx_sigma_is_in_range(
                  r1_target00_b_obj.radar1_target00_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_ax_sigma_decode(r1_target00_b_obj.radar1_target00_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_ax_sigma_is_in_range(
                  r1_target00_b_obj.radar1_target00_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_dy_sigma_decode(r1_target00_b_obj.radar1_target00_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_dy_sigma_is_in_range(
                  r1_target00_b_obj.radar1_target00_dy_sigma);
              target_w_class_decoded =
                  xgu_radar1_target01_b_radar1_target01_w_class_decode(r1_target00_b_obj.radar1_target00_w_class);
              target_w_class_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_w_class_is_in_range(r1_target00_b_obj.radar1_target00_w_class);
              target_class_decoded =
                  xgu_radar1_target01_b_radar1_target01_class_decode(r1_target00_b_obj.radar1_target00_class);
              target_class_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_class_is_in_range(r1_target00_b_obj.radar1_target00_class);
              target_dx_rear_end_loss_decoded = xgu_radar1_target01_b_radar1_target01_dx_rear_end_loss_decode(
                  r1_target00_b_obj.radar1_target00_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar1_target01_b_radar1_target01_dx_rear_end_loss_is_in_range(
                  r1_target00_b_obj.radar1_target00_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar1_target01_b_radar1_target01_mess_bconsist_bit_decode(
                  r1_target00_b_obj.radar1_target00_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_mess_bconsist_bit_is_in_range(
                      r1_target00_b_obj.radar1_target00_mess_bconsist_bit);
              break;
            case 1607:
              xgu_radar2_target00_a_t r2_target00_a_obj;
              target_info.unpack_return = xgu_radar2_target00_a_unpack(&r2_target00_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar2_target00_a_radar2_target00_dx_decode(r2_target00_a_obj.radar2_target00_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_dx_is_in_range(r2_target00_a_obj.radar2_target00_dx);
              target_info.target_vx_decode = xgu_radar2_target00_a_radar2_target00_vx_decode(r2_target00_a_obj.radar2_target00_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_vx_is_in_range(r2_target00_a_obj.radar2_target00_vx);
              target_info.target_dy_decode = xgu_radar2_target00_a_radar2_target00_dy_decode(r2_target00_a_obj.radar2_target00_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_dy_is_in_range(r2_target00_a_obj.radar2_target00_dy);
              target_info.target_w_exist_decode =
                  xgu_radar2_target00_a_radar2_target00_w_exist_decode(r2_target00_a_obj.radar2_target00_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_w_exist_is_in_range(r2_target00_a_obj.radar2_target00_w_exist);
              target_info.target_ax_decode = xgu_radar2_target00_a_radar2_target00_ax_decode(r2_target00_a_obj.radar2_target00_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_ax_is_in_range(r2_target00_a_obj.radar2_target00_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar2_target00_a_radar2_target00_w_obstacle_decode(r2_target00_a_obj.radar2_target00_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar2_target00_a_radar2_target00_w_obstacle_is_in_range(
                  r2_target00_a_obj.radar2_target00_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar2_target00_a_radar2_target00_flag_valid_decode(r2_target00_a_obj.radar2_target00_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar2_target00_a_radar2_target00_flag_valid_is_in_range(
                  r2_target00_a_obj.radar2_target00_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar2_target00_a_radar2_target00_w_non_obstacle_decode(
                  r2_target00_a_obj.radar2_target00_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar2_target00_a_radar2_target00_w_non_obstacle_is_in_range(
                  r2_target00_a_obj.radar2_target00_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar2_target00_a_radar2_target00_flag_meas_decode(r2_target00_a_obj.radar2_target00_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar2_target00_a_radar2_target00_flag_meas_is_in_range(
                  r2_target00_a_obj.radar2_target00_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar2_target00_a_radar2_target00_flag_hist_decode(r2_target00_a_obj.radar2_target00_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar2_target00_a_radar2_target00_flag_hist_is_in_range(
                  r2_target00_a_obj.radar2_target00_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar2_target00_a_radar2_target00_mess_aconsist_bit_decode(
                  r2_target00_a_obj.radar2_target00_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_mess_aconsist_bit_is_in_range(
                      r2_target00_a_obj.radar2_target00_mess_aconsist_bit);
              break;
            case 1608:
              xgu_radar2_target00_b_t r2_target00_b_obj;
              target_info.unpack_return = xgu_radar2_target00_b_unpack(&r2_target00_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar2_target02_b_radar2_target02_vy_decode(r2_target00_b_obj.radar2_target00_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_vy_is_in_range(r2_target00_b_obj.radar2_target00_vy);
              target_info.target_d_length_decoded =
                  xgu_radar2_target02_b_radar2_target02_d_length_decode(r2_target00_b_obj.radar2_target00_d_length);
              target_info.target_d_length_is_in_range = xgu_radar2_target02_b_radar2_target02_d_length_is_in_range(
                  r2_target00_b_obj.radar2_target00_d_length);
              target_info.target_dz_decoded = xgu_radar2_target02_b_radar2_target02_dz_decode(r2_target00_b_obj.radar2_target00_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_dz_is_in_range(r2_target00_b_obj.radar2_target00_dz);
              target_moving_state_decoded = xgu_radar2_target02_b_radar2_target02_moving_state_decode(
                  r2_target00_b_obj.radar2_target00_moving_state);
              target_moving_state_is_in_range = xgu_radar2_target02_b_radar2_target02_moving_state_is_in_range(
                  r2_target00_b_obj.radar2_target00_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_dx_sigma_decode(r2_target00_b_obj.radar2_target00_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_dx_sigma_is_in_range(
                  r2_target00_b_obj.radar2_target00_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_vx_sigma_decode(r2_target00_b_obj.radar2_target00_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_vx_sigma_is_in_range(
                  r2_target00_b_obj.radar2_target00_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_ax_sigma_decode(r2_target00_b_obj.radar2_target00_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_ax_sigma_is_in_range(
                  r2_target00_b_obj.radar2_target00_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_dy_sigma_decode(r2_target00_b_obj.radar2_target00_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_dy_sigma_is_in_range(
                  r2_target00_b_obj.radar2_target00_dy_sigma);
              target_w_class_decoded =
                  xgu_radar2_target02_b_radar2_target02_w_class_decode(r2_target00_b_obj.radar2_target00_w_class);
              target_w_class_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_w_class_is_in_range(r2_target00_b_obj.radar2_target00_w_class);
              target_class_decoded =
                  xgu_radar2_target02_b_radar2_target02_class_decode(r2_target00_b_obj.radar2_target00_class);
              target_class_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_class_is_in_range(r2_target00_b_obj.radar2_target00_class);
              target_dx_rear_end_loss_decoded = xgu_radar2_target02_b_radar2_target02_dx_rear_end_loss_decode(
                  r2_target00_b_obj.radar2_target00_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar2_target02_b_radar2_target02_dx_rear_end_loss_is_in_range(
                  r2_target00_b_obj.radar2_target00_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar2_target02_b_radar2_target02_mess_bconsist_bit_decode(
                  r2_target00_b_obj.radar2_target00_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_mess_bconsist_bit_is_in_range(
                      r2_target00_b_obj.radar2_target00_mess_bconsist_bit);
              break;
            case 1615:
              xgu_radar1_target01_a_t r1_target01_a_obj;
              target_info.unpack_return = xgu_radar1_target01_a_unpack(&r1_target01_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar1_target01_a_radar1_target01_dx_decode(r1_target01_a_obj.radar1_target01_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_dx_is_in_range(r1_target01_a_obj.radar1_target01_dx);
              target_info.target_vx_decode = xgu_radar1_target01_a_radar1_target01_vx_decode(r1_target01_a_obj.radar1_target01_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_vx_is_in_range(r1_target01_a_obj.radar1_target01_vx);
              target_info.target_dy_decode = xgu_radar1_target01_a_radar1_target01_dy_decode(r1_target01_a_obj.radar1_target01_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_dy_is_in_range(r1_target01_a_obj.radar1_target01_dy);
              target_info.target_w_exist_decode =
                  xgu_radar1_target01_a_radar1_target01_w_exist_decode(r1_target01_a_obj.radar1_target01_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_w_exist_is_in_range(r1_target01_a_obj.radar1_target01_w_exist);
              target_info.target_ax_decode = xgu_radar1_target01_a_radar1_target01_ax_decode(r1_target01_a_obj.radar1_target01_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_ax_is_in_range(r1_target01_a_obj.radar1_target01_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar1_target01_a_radar1_target01_w_obstacle_decode(r1_target01_a_obj.radar1_target01_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar1_target01_a_radar1_target01_w_obstacle_is_in_range(
                  r1_target01_a_obj.radar1_target01_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar1_target01_a_radar1_target01_flag_valid_decode(r1_target01_a_obj.radar1_target01_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar1_target01_a_radar1_target01_flag_valid_is_in_range(
                  r1_target01_a_obj.radar1_target01_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar1_target01_a_radar1_target01_w_non_obstacle_decode(
                  r1_target01_a_obj.radar1_target01_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar1_target01_a_radar1_target01_w_non_obstacle_is_in_range(
                  r1_target01_a_obj.radar1_target01_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar1_target01_a_radar1_target01_flag_meas_decode(r1_target01_a_obj.radar1_target01_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar1_target01_a_radar1_target01_flag_meas_is_in_range(
                  r1_target01_a_obj.radar1_target01_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar1_target01_a_radar1_target01_flag_hist_decode(r1_target01_a_obj.radar1_target01_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar1_target01_a_radar1_target01_flag_hist_is_in_range(
                  r1_target01_a_obj.radar1_target01_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar1_target01_a_radar1_target01_mess_aconsist_bit_decode(
                  r1_target01_a_obj.radar1_target01_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_mess_aconsist_bit_is_in_range(
                      r1_target01_a_obj.radar1_target01_mess_aconsist_bit);
              break;
            case 1616:
              xgu_radar1_target01_b_t r1_target01_b_obj;
              target_info.unpack_return = xgu_radar1_target01_b_unpack(&r1_target01_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar1_target01_b_radar1_target01_vy_decode(r1_target01_b_obj.radar1_target01_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_vy_is_in_range(r1_target01_b_obj.radar1_target01_vy);
              target_info.target_d_length_decoded =
                  xgu_radar1_target01_b_radar1_target01_d_length_decode(r1_target01_b_obj.radar1_target01_d_length);
              target_info.target_d_length_is_in_range = xgu_radar1_target01_b_radar1_target01_d_length_is_in_range(
                  r1_target01_b_obj.radar1_target01_d_length);
              target_info.target_dz_decoded = xgu_radar1_target01_b_radar1_target01_dz_decode(r1_target01_b_obj.radar1_target01_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_dz_is_in_range(r1_target01_b_obj.radar1_target01_dz);
              target_moving_state_decoded = xgu_radar1_target01_b_radar1_target01_moving_state_decode(
                  r1_target01_b_obj.radar1_target01_moving_state);
              target_moving_state_is_in_range = xgu_radar1_target01_b_radar1_target01_moving_state_is_in_range(
                  r1_target01_b_obj.radar1_target01_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_dx_sigma_decode(r1_target01_b_obj.radar1_target01_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_dx_sigma_is_in_range(
                  r1_target01_b_obj.radar1_target01_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_vx_sigma_decode(r1_target01_b_obj.radar1_target01_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_vx_sigma_is_in_range(
                  r1_target01_b_obj.radar1_target01_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_ax_sigma_decode(r1_target01_b_obj.radar1_target01_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_ax_sigma_is_in_range(
                  r1_target01_b_obj.radar1_target01_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar1_target01_b_radar1_target01_dy_sigma_decode(r1_target01_b_obj.radar1_target01_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar1_target01_b_radar1_target01_dy_sigma_is_in_range(
                  r1_target01_b_obj.radar1_target01_dy_sigma);
              target_w_class_decoded =
                  xgu_radar1_target01_b_radar1_target01_w_class_decode(r1_target01_b_obj.radar1_target01_w_class);
              target_w_class_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_w_class_is_in_range(r1_target01_b_obj.radar1_target01_w_class);
              target_class_decoded =
                  xgu_radar1_target01_b_radar1_target01_class_decode(r1_target01_b_obj.radar1_target01_class);
              target_class_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_class_is_in_range(r1_target01_b_obj.radar1_target01_class);
              target_dx_rear_end_loss_decoded = xgu_radar1_target01_b_radar1_target01_dx_rear_end_loss_decode(
                  r1_target01_b_obj.radar1_target01_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar1_target01_b_radar1_target01_dx_rear_end_loss_is_in_range(
                  r1_target01_b_obj.radar1_target01_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar1_target01_b_radar1_target01_mess_bconsist_bit_decode(
                  r1_target01_b_obj.radar1_target01_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_mess_bconsist_bit_is_in_range(
                      r1_target01_b_obj.radar1_target01_mess_bconsist_bit);
              break;
            case 1617:
              xgu_radar2_target01_a_t r2_target01_a_obj;
              target_info.unpack_return = xgu_radar2_target01_a_unpack(&r2_target01_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar2_target01_a_radar2_target01_dx_decode(r2_target01_a_obj.radar2_target01_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar2_target01_a_radar2_target01_dx_is_in_range(r2_target01_a_obj.radar2_target01_dx);
              target_info.target_vx_decode = xgu_radar2_target01_a_radar2_target01_vx_decode(r2_target01_a_obj.radar2_target01_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar2_target01_a_radar2_target01_vx_is_in_range(r2_target01_a_obj.radar2_target01_vx);
              target_info.target_dy_decode = xgu_radar2_target01_a_radar2_target01_dy_decode(r2_target01_a_obj.radar2_target01_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar2_target01_a_radar2_target01_dy_is_in_range(r2_target01_a_obj.radar2_target01_dy);
              target_info.target_w_exist_decode =
                  xgu_radar2_target01_a_radar2_target01_w_exist_decode(r2_target01_a_obj.radar2_target01_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar2_target01_a_radar2_target01_w_exist_is_in_range(r2_target01_a_obj.radar2_target01_w_exist);
              target_info.target_ax_decode = xgu_radar2_target01_a_radar2_target01_ax_decode(r2_target01_a_obj.radar2_target01_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar2_target01_a_radar2_target01_ax_is_in_range(r2_target01_a_obj.radar2_target01_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar2_target01_a_radar2_target01_w_obstacle_decode(r2_target01_a_obj.radar2_target01_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar2_target01_a_radar2_target01_w_obstacle_is_in_range(
                  r2_target01_a_obj.radar2_target01_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar2_target01_a_radar2_target01_flag_valid_decode(r2_target01_a_obj.radar2_target01_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar2_target01_a_radar2_target01_flag_valid_is_in_range(
                  r2_target01_a_obj.radar2_target01_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar2_target01_a_radar2_target01_w_non_obstacle_decode(
                  r2_target01_a_obj.radar2_target01_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar2_target01_a_radar2_target01_w_non_obstacle_is_in_range(
                  r2_target01_a_obj.radar2_target01_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar2_target01_a_radar2_target01_flag_meas_decode(r2_target01_a_obj.radar2_target01_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar2_target01_a_radar2_target01_flag_meas_is_in_range(
                  r2_target01_a_obj.radar2_target01_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar2_target01_a_radar2_target01_flag_hist_decode(r2_target01_a_obj.radar2_target01_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar2_target01_a_radar2_target01_flag_hist_is_in_range(
                  r2_target01_a_obj.radar2_target01_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar2_target01_a_radar2_target01_mess_aconsist_bit_decode(
                  r2_target01_a_obj.radar2_target01_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target01_a_radar2_target01_mess_aconsist_bit_is_in_range(
                      r2_target01_a_obj.radar2_target01_mess_aconsist_bit);
              break;
            case 1618:
              xgu_radar2_target01_b_t r2_target01_b_obj;
              target_info.unpack_return = xgu_radar2_target01_b_unpack(&r2_target01_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar2_target02_b_radar2_target02_vy_decode(r2_target01_b_obj.radar2_target01_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_vy_is_in_range(r2_target01_b_obj.radar2_target01_vy);
              target_info.target_d_length_decoded =
                  xgu_radar2_target02_b_radar2_target02_d_length_decode(r2_target01_b_obj.radar2_target01_d_length);
              target_info.target_d_length_is_in_range = xgu_radar2_target02_b_radar2_target02_d_length_is_in_range(
                  r2_target01_b_obj.radar2_target01_d_length);
              target_info.target_dz_decoded = xgu_radar2_target02_b_radar2_target02_dz_decode(r2_target01_b_obj.radar2_target01_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_dz_is_in_range(r2_target01_b_obj.radar2_target01_dz);
              target_moving_state_decoded = xgu_radar2_target02_b_radar2_target02_moving_state_decode(
                  r2_target01_b_obj.radar2_target01_moving_state);
              target_moving_state_is_in_range = xgu_radar2_target02_b_radar2_target02_moving_state_is_in_range(
                  r2_target01_b_obj.radar2_target01_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_dx_sigma_decode(r2_target01_b_obj.radar2_target01_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_dx_sigma_is_in_range(
                  r2_target01_b_obj.radar2_target01_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_vx_sigma_decode(r2_target01_b_obj.radar2_target01_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_vx_sigma_is_in_range(
                  r2_target01_b_obj.radar2_target01_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_ax_sigma_decode(r2_target01_b_obj.radar2_target01_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_ax_sigma_is_in_range(
                  r2_target01_b_obj.radar2_target01_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_dy_sigma_decode(r2_target01_b_obj.radar2_target01_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_dy_sigma_is_in_range(
                  r2_target01_b_obj.radar2_target01_dy_sigma);
              target_w_class_decoded =
                  xgu_radar2_target02_b_radar2_target02_w_class_decode(r2_target01_b_obj.radar2_target01_w_class);
              target_w_class_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_w_class_is_in_range(r2_target01_b_obj.radar2_target01_w_class);
              target_class_decoded =
                  xgu_radar2_target02_b_radar2_target02_class_decode(r2_target01_b_obj.radar2_target01_class);
              target_class_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_class_is_in_range(r2_target01_b_obj.radar2_target01_class);
              target_dx_rear_end_loss_decoded = xgu_radar2_target02_b_radar2_target02_dx_rear_end_loss_decode(
                  r2_target01_b_obj.radar2_target01_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar2_target02_b_radar2_target02_dx_rear_end_loss_is_in_range(
                  r2_target01_b_obj.radar2_target01_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar2_target02_b_radar2_target02_mess_bconsist_bit_decode(
                  r2_target01_b_obj.radar2_target01_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_mess_bconsist_bit_is_in_range(
                      r2_target01_b_obj.radar2_target01_mess_bconsist_bit);
              break;
            case 1625:
              xgu_radar1_target02_a_t r1_target02_a_obj;
              target_info.unpack_return = xgu_radar1_target02_a_unpack(&r1_target02_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar1_target02_a_radar1_target02_dx_decode(r1_target02_a_obj.radar1_target02_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_dx_is_in_range(r1_target02_a_obj.radar1_target02_dx);
              target_info.target_vx_decode = xgu_radar1_target02_a_radar1_target02_vx_decode(r1_target02_a_obj.radar1_target02_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_vx_is_in_range(r1_target02_a_obj.radar1_target02_vx);
              target_info.target_dy_decode = xgu_radar1_target02_a_radar1_target02_dy_decode(r1_target02_a_obj.radar1_target02_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_dy_is_in_range(r1_target02_a_obj.radar1_target02_dy);
              target_info.target_w_exist_decode =
                  xgu_radar1_target02_a_radar1_target02_w_exist_decode(r1_target02_a_obj.radar1_target02_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_w_exist_is_in_range(r1_target02_a_obj.radar1_target02_w_exist);
              target_info.target_ax_decode = xgu_radar1_target02_a_radar1_target02_ax_decode(r1_target02_a_obj.radar1_target02_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_ax_is_in_range(r1_target02_a_obj.radar1_target02_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar1_target02_a_radar1_target02_w_obstacle_decode(r1_target02_a_obj.radar1_target02_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar1_target02_a_radar1_target02_w_obstacle_is_in_range(
                  r1_target02_a_obj.radar1_target02_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar1_target02_a_radar1_target02_flag_valid_decode(r1_target02_a_obj.radar1_target02_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar1_target02_a_radar1_target02_flag_valid_is_in_range(
                  r1_target02_a_obj.radar1_target02_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar1_target02_a_radar1_target02_w_non_obstacle_decode(
                  r1_target02_a_obj.radar1_target02_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar1_target02_a_radar1_target02_w_non_obstacle_is_in_range(
                  r1_target02_a_obj.radar1_target02_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar1_target02_a_radar1_target02_flag_meas_decode(r1_target02_a_obj.radar1_target02_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar1_target02_a_radar1_target02_flag_meas_is_in_range(
                  r1_target02_a_obj.radar1_target02_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar1_target02_a_radar1_target02_flag_hist_decode(r1_target02_a_obj.radar1_target02_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar1_target02_a_radar1_target02_flag_hist_is_in_range(
                  r1_target02_a_obj.radar1_target02_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar1_target02_a_radar1_target02_mess_aconsist_bit_decode(
                  r1_target02_a_obj.radar1_target02_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_mess_aconsist_bit_is_in_range(
                      r1_target02_a_obj.radar1_target02_mess_aconsist_bit);
              break;
            case 1626:
              xgu_radar1_target02_b_t r1_target02_b_obj;
              target_info.unpack_return = xgu_radar1_target02_b_unpack(&r1_target02_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar1_target02_b_radar1_target02_vy_decode(r1_target02_b_obj.radar1_target02_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar1_target02_b_radar1_target02_vy_is_in_range(r1_target02_b_obj.radar1_target02_vy);
              target_info.target_d_length_decoded =
                  xgu_radar1_target02_b_radar1_target02_d_length_decode(r1_target02_b_obj.radar1_target02_d_length);
              target_info.target_d_length_is_in_range = xgu_radar1_target02_b_radar1_target02_d_length_is_in_range(
                  r1_target02_b_obj.radar1_target02_d_length);
              target_info.target_dz_decoded = xgu_radar1_target02_b_radar1_target02_dz_decode(r1_target02_b_obj.radar1_target02_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar1_target02_b_radar1_target02_dz_is_in_range(r1_target02_b_obj.radar1_target02_dz);
              target_moving_state_decoded = xgu_radar1_target02_b_radar1_target02_moving_state_decode(
                  r1_target02_b_obj.radar1_target02_moving_state);
              target_moving_state_is_in_range = xgu_radar1_target02_b_radar1_target02_moving_state_is_in_range(
                  r1_target02_b_obj.radar1_target02_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar1_target02_b_radar1_target02_dx_sigma_decode(r1_target02_b_obj.radar1_target02_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar1_target02_b_radar1_target02_dx_sigma_is_in_range(
                  r1_target02_b_obj.radar1_target02_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar1_target02_b_radar1_target02_vx_sigma_decode(r1_target02_b_obj.radar1_target02_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar1_target02_b_radar1_target02_vx_sigma_is_in_range(
                  r1_target02_b_obj.radar1_target02_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar1_target02_b_radar1_target02_ax_sigma_decode(r1_target02_b_obj.radar1_target02_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar1_target02_b_radar1_target02_ax_sigma_is_in_range(
                  r1_target02_b_obj.radar1_target02_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar1_target02_b_radar1_target02_dy_sigma_decode(r1_target02_b_obj.radar1_target02_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar1_target02_b_radar1_target02_dy_sigma_is_in_range(
                  r1_target02_b_obj.radar1_target02_dy_sigma);
              target_w_class_decoded =
                  xgu_radar1_target02_b_radar1_target02_w_class_decode(r1_target02_b_obj.radar1_target02_w_class);
              target_w_class_is_in_range =
                  xgu_radar1_target02_b_radar1_target02_w_class_is_in_range(r1_target02_b_obj.radar1_target02_w_class);
              target_class_decoded =
                  xgu_radar1_target02_b_radar1_target02_class_decode(r1_target02_b_obj.radar1_target02_class);
              target_class_is_in_range =
                  xgu_radar1_target02_b_radar1_target02_class_is_in_range(r1_target02_b_obj.radar1_target02_class);
              target_dx_rear_end_loss_decoded = xgu_radar1_target02_b_radar1_target02_dx_rear_end_loss_decode(
                  r1_target02_b_obj.radar1_target02_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar1_target02_b_radar1_target02_dx_rear_end_loss_is_in_range(
                  r1_target02_b_obj.radar1_target02_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar1_target02_b_radar1_target02_mess_bconsist_bit_decode(
                  r1_target02_b_obj.radar1_target02_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar1_target02_b_radar1_target02_mess_bconsist_bit_is_in_range(
                      r1_target02_b_obj.radar1_target02_mess_bconsist_bit);
              break;
            case 1627:
              xgu_radar2_target02_a_t r2_target02_a_obj;
              target_info.unpack_return = xgu_radar2_target02_a_unpack(&r2_target02_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar2_target02_a_radar2_target02_dx_decode(r2_target02_a_obj.radar2_target02_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar2_target02_a_radar2_target02_dx_is_in_range(r2_target02_a_obj.radar2_target02_dx);
              target_info.target_vx_decode = xgu_radar2_target02_a_radar2_target02_vx_decode(r2_target02_a_obj.radar2_target02_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar2_target02_a_radar2_target02_vx_is_in_range(r2_target02_a_obj.radar2_target02_vx);
              target_info.target_dy_decode = xgu_radar2_target02_a_radar2_target02_dy_decode(r2_target02_a_obj.radar2_target02_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar2_target02_a_radar2_target02_dy_is_in_range(r2_target02_a_obj.radar2_target02_dy);
              target_info.target_w_exist_decode =
                  xgu_radar2_target02_a_radar2_target02_w_exist_decode(r2_target02_a_obj.radar2_target02_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar2_target02_a_radar2_target02_w_exist_is_in_range(r2_target02_a_obj.radar2_target02_w_exist);
              target_info.target_ax_decode = xgu_radar2_target02_a_radar2_target02_ax_decode(r2_target02_a_obj.radar2_target02_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar2_target02_a_radar2_target02_ax_is_in_range(r2_target02_a_obj.radar2_target02_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar2_target02_a_radar2_target02_w_obstacle_decode(r2_target02_a_obj.radar2_target02_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar2_target02_a_radar2_target02_w_obstacle_is_in_range(
                  r2_target02_a_obj.radar2_target02_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar2_target02_a_radar2_target02_flag_valid_decode(r2_target02_a_obj.radar2_target02_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar2_target02_a_radar2_target02_flag_valid_is_in_range(
                  r2_target02_a_obj.radar2_target02_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar2_target02_a_radar2_target02_w_non_obstacle_decode(
                  r2_target02_a_obj.radar2_target02_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar2_target02_a_radar2_target02_w_non_obstacle_is_in_range(
                  r2_target02_a_obj.radar2_target02_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar2_target02_a_radar2_target02_flag_meas_decode(r2_target02_a_obj.radar2_target02_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar2_target02_a_radar2_target02_flag_meas_is_in_range(
                  r2_target02_a_obj.radar2_target02_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar2_target02_a_radar2_target02_flag_hist_decode(r2_target02_a_obj.radar2_target02_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar2_target02_a_radar2_target02_flag_hist_is_in_range(
                  r2_target02_a_obj.radar2_target02_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar2_target02_a_radar2_target02_mess_aconsist_bit_decode(
                  r2_target02_a_obj.radar2_target02_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target02_a_radar2_target02_mess_aconsist_bit_is_in_range(
                      r2_target02_a_obj.radar2_target02_mess_aconsist_bit);
              break;
            case 1628:
              xgu_radar2_target02_b_t r2_target02_b_obj;
              target_info.unpack_return = xgu_radar2_target02_b_unpack(&r2_target02_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar2_target02_b_radar2_target02_vy_decode(r2_target02_b_obj.radar2_target02_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_vy_is_in_range(r2_target02_b_obj.radar2_target02_vy);
              target_info.target_d_length_decoded =
                  xgu_radar2_target02_b_radar2_target02_d_length_decode(r2_target02_b_obj.radar2_target02_d_length);
              target_info.target_d_length_is_in_range = xgu_radar2_target02_b_radar2_target02_d_length_is_in_range(
                  r2_target02_b_obj.radar2_target02_d_length);
              target_info.target_dz_decoded = xgu_radar2_target02_b_radar2_target02_dz_decode(r2_target02_b_obj.radar2_target02_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_dz_is_in_range(r2_target02_b_obj.radar2_target02_dz);
              target_moving_state_decoded = xgu_radar2_target02_b_radar2_target02_moving_state_decode(
                  r2_target02_b_obj.radar2_target02_moving_state);
              target_moving_state_is_in_range = xgu_radar2_target02_b_radar2_target02_moving_state_is_in_range(
                  r2_target02_b_obj.radar2_target02_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_dx_sigma_decode(r2_target02_b_obj.radar2_target02_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_dx_sigma_is_in_range(
                  r2_target02_b_obj.radar2_target02_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_vx_sigma_decode(r2_target02_b_obj.radar2_target02_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_vx_sigma_is_in_range(
                  r2_target02_b_obj.radar2_target02_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_ax_sigma_decode(r2_target02_b_obj.radar2_target02_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_ax_sigma_is_in_range(
                  r2_target02_b_obj.radar2_target02_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar2_target02_b_radar2_target02_dy_sigma_decode(r2_target02_b_obj.radar2_target02_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar2_target02_b_radar2_target02_dy_sigma_is_in_range(
                  r2_target02_b_obj.radar2_target02_dy_sigma);
              target_w_class_decoded =
                  xgu_radar2_target02_b_radar2_target02_w_class_decode(r2_target02_b_obj.radar2_target02_w_class);
              target_w_class_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_w_class_is_in_range(r2_target02_b_obj.radar2_target02_w_class);
              target_class_decoded =
                  xgu_radar2_target02_b_radar2_target02_class_decode(r2_target02_b_obj.radar2_target02_class);
              target_class_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_class_is_in_range(r2_target02_b_obj.radar2_target02_class);
              target_dx_rear_end_loss_decoded = xgu_radar2_target02_b_radar2_target02_dx_rear_end_loss_decode(
                  r2_target02_b_obj.radar2_target02_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar2_target02_b_radar2_target02_dx_rear_end_loss_is_in_range(
                  r2_target02_b_obj.radar2_target02_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar2_target02_b_radar2_target02_mess_bconsist_bit_decode(
                  r2_target02_b_obj.radar2_target02_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_mess_bconsist_bit_is_in_range(
                      r2_target02_b_obj.radar2_target02_mess_bconsist_bit);
              break;
            case 1635:
              xgu_radar1_target03_a_t r1_target03_a_obj;
              target_info.unpack_return = xgu_radar1_target03_a_unpack(&r1_target03_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar1_target03_a_radar1_target03_dx_decode(r1_target03_a_obj.radar1_target03_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_dx_is_in_range(r1_target03_a_obj.radar1_target03_dx);
              target_info.target_vx_decode = xgu_radar1_target03_a_radar1_target03_vx_decode(r1_target03_a_obj.radar1_target03_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_vx_is_in_range(r1_target03_a_obj.radar1_target03_vx);
              target_info.target_dy_decode = xgu_radar1_target03_a_radar1_target03_dy_decode(r1_target03_a_obj.radar1_target03_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_dy_is_in_range(r1_target03_a_obj.radar1_target03_dy);
              target_info.target_w_exist_decode =
                  xgu_radar1_target03_a_radar1_target03_w_exist_decode(r1_target03_a_obj.radar1_target03_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_w_exist_is_in_range(r1_target03_a_obj.radar1_target03_w_exist);
              target_info.target_ax_decode = xgu_radar1_target03_a_radar1_target03_ax_decode(r1_target03_a_obj.radar1_target03_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_ax_is_in_range(r1_target03_a_obj.radar1_target03_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar1_target03_a_radar1_target03_w_obstacle_decode(r1_target03_a_obj.radar1_target03_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar1_target03_a_radar1_target03_w_obstacle_is_in_range(
                  r1_target03_a_obj.radar1_target03_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar1_target03_a_radar1_target03_flag_valid_decode(r1_target03_a_obj.radar1_target03_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar1_target03_a_radar1_target03_flag_valid_is_in_range(
                  r1_target03_a_obj.radar1_target03_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar1_target03_a_radar1_target03_w_non_obstacle_decode(
                  r1_target03_a_obj.radar1_target03_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar1_target03_a_radar1_target03_w_non_obstacle_is_in_range(
                  r1_target03_a_obj.radar1_target03_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar1_target03_a_radar1_target03_flag_meas_decode(r1_target03_a_obj.radar1_target03_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar1_target03_a_radar1_target03_flag_meas_is_in_range(
                  r1_target03_a_obj.radar1_target03_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar1_target03_a_radar1_target03_flag_hist_decode(r1_target03_a_obj.radar1_target03_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar1_target03_a_radar1_target03_flag_hist_is_in_range(
                  r1_target03_a_obj.radar1_target03_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar1_target03_a_radar1_target03_mess_aconsist_bit_decode(
                  r1_target03_a_obj.radar1_target03_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_mess_aconsist_bit_is_in_range(
                      r1_target03_a_obj.radar1_target03_mess_aconsist_bit);
              break;
            case 1636:
              xgu_radar1_target03_b_t r1_target03_b_obj;
              target_info.unpack_return = xgu_radar1_target03_b_unpack(&r1_target03_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar1_target03_b_radar1_target03_vy_decode(r1_target03_b_obj.radar1_target03_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar1_target03_b_radar1_target03_vy_is_in_range(r1_target03_b_obj.radar1_target03_vy);
              target_info.target_d_length_decoded =
                  xgu_radar1_target03_b_radar1_target03_d_length_decode(r1_target03_b_obj.radar1_target03_d_length);
              target_info.target_d_length_is_in_range = xgu_radar1_target03_b_radar1_target03_d_length_is_in_range(
                  r1_target03_b_obj.radar1_target03_d_length);
              target_info.target_dz_decoded = xgu_radar1_target03_b_radar1_target03_dz_decode(r1_target03_b_obj.radar1_target03_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar1_target03_b_radar1_target03_dz_is_in_range(r1_target03_b_obj.radar1_target03_dz);
              target_moving_state_decoded = xgu_radar1_target03_b_radar1_target03_moving_state_decode(
                  r1_target03_b_obj.radar1_target03_moving_state);
              target_moving_state_is_in_range = xgu_radar1_target03_b_radar1_target03_moving_state_is_in_range(
                  r1_target03_b_obj.radar1_target03_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar1_target03_b_radar1_target03_dx_sigma_decode(r1_target03_b_obj.radar1_target03_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar1_target03_b_radar1_target03_dx_sigma_is_in_range(
                  r1_target03_b_obj.radar1_target03_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar1_target03_b_radar1_target03_vx_sigma_decode(r1_target03_b_obj.radar1_target03_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar1_target03_b_radar1_target03_vx_sigma_is_in_range(
                  r1_target03_b_obj.radar1_target03_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar1_target03_b_radar1_target03_ax_sigma_decode(r1_target03_b_obj.radar1_target03_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar1_target03_b_radar1_target03_ax_sigma_is_in_range(
                  r1_target03_b_obj.radar1_target03_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar1_target03_b_radar1_target03_dy_sigma_decode(r1_target03_b_obj.radar1_target03_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar1_target03_b_radar1_target03_dy_sigma_is_in_range(
                  r1_target03_b_obj.radar1_target03_dy_sigma);
              target_w_class_decoded =
                  xgu_radar1_target03_b_radar1_target03_w_class_decode(r1_target03_b_obj.radar1_target03_w_class);
              target_w_class_is_in_range =
                  xgu_radar1_target03_b_radar1_target03_w_class_is_in_range(r1_target03_b_obj.radar1_target03_w_class);
              target_class_decoded =
                  xgu_radar1_target03_b_radar1_target03_class_decode(r1_target03_b_obj.radar1_target03_class);
              target_class_is_in_range =
                  xgu_radar1_target03_b_radar1_target03_class_is_in_range(r1_target03_b_obj.radar1_target03_class);
              target_dx_rear_end_loss_decoded = xgu_radar1_target03_b_radar1_target03_dx_rear_end_loss_decode(
                  r1_target03_b_obj.radar1_target03_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar1_target03_b_radar1_target03_dx_rear_end_loss_is_in_range(
                  r1_target03_b_obj.radar1_target03_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar1_target03_b_radar1_target03_mess_bconsist_bit_decode(
                  r1_target03_b_obj.radar1_target03_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar1_target03_b_radar1_target03_mess_bconsist_bit_is_in_range(
                      r1_target03_b_obj.radar1_target03_mess_bconsist_bit);
              break;
            case 1637:
              xgu_radar2_target03_a_t r2_target03_a_obj;
              target_info.unpack_return = xgu_radar2_target03_a_unpack(&r2_target03_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar2_target03_a_radar2_target03_dx_decode(r2_target03_a_obj.radar2_target03_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar2_target03_a_radar2_target03_dx_is_in_range(r2_target03_a_obj.radar2_target03_dx);
              target_info.target_vx_decode = xgu_radar2_target03_a_radar2_target03_vx_decode(r2_target03_a_obj.radar2_target03_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar2_target03_a_radar2_target03_vx_is_in_range(r2_target03_a_obj.radar2_target03_vx);
              target_info.target_dy_decode = xgu_radar2_target03_a_radar2_target03_dy_decode(r2_target03_a_obj.radar2_target03_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar2_target03_a_radar2_target03_dy_is_in_range(r2_target03_a_obj.radar2_target03_dy);
              target_info.target_w_exist_decode =
                  xgu_radar2_target03_a_radar2_target03_w_exist_decode(r2_target03_a_obj.radar2_target03_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar2_target03_a_radar2_target03_w_exist_is_in_range(r2_target03_a_obj.radar2_target03_w_exist);
              target_info.target_ax_decode = xgu_radar2_target03_a_radar2_target03_ax_decode(r2_target03_a_obj.radar2_target03_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar2_target03_a_radar2_target03_ax_is_in_range(r2_target03_a_obj.radar2_target03_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar2_target03_a_radar2_target03_w_obstacle_decode(r2_target03_a_obj.radar2_target03_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar2_target03_a_radar2_target03_w_obstacle_is_in_range(
                  r2_target03_a_obj.radar2_target03_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar2_target03_a_radar2_target03_flag_valid_decode(r2_target03_a_obj.radar2_target03_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar2_target03_a_radar2_target03_flag_valid_is_in_range(
                  r2_target03_a_obj.radar2_target03_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar2_target03_a_radar2_target03_w_non_obstacle_decode(
                  r2_target03_a_obj.radar2_target03_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar2_target03_a_radar2_target03_w_non_obstacle_is_in_range(
                  r2_target03_a_obj.radar2_target03_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar2_target03_a_radar2_target03_flag_meas_decode(r2_target03_a_obj.radar2_target03_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar2_target03_a_radar2_target03_flag_meas_is_in_range(
                  r2_target03_a_obj.radar2_target03_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar2_target03_a_radar2_target03_flag_hist_decode(r2_target03_a_obj.radar2_target03_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar2_target03_a_radar2_target03_flag_hist_is_in_range(
                  r2_target03_a_obj.radar2_target03_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar2_target03_a_radar2_target03_mess_aconsist_bit_decode(
                  r2_target03_a_obj.radar2_target03_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target03_a_radar2_target03_mess_aconsist_bit_is_in_range(
                      r2_target03_a_obj.radar2_target03_mess_aconsist_bit);
              break;
            case 1638:
              xgu_radar2_target03_b_t r2_target03_b_obj;
              target_info.unpack_return = xgu_radar2_target03_b_unpack(&r2_target03_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar2_target03_b_radar2_target03_vy_decode(r2_target03_b_obj.radar2_target03_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar2_target03_b_radar2_target03_vy_is_in_range(r2_target03_b_obj.radar2_target03_vy);
              target_info.target_d_length_decoded =
                  xgu_radar2_target03_b_radar2_target03_d_length_decode(r2_target03_b_obj.radar2_target03_d_length);
              target_info.target_d_length_is_in_range = xgu_radar2_target03_b_radar2_target03_d_length_is_in_range(
                  r2_target03_b_obj.radar2_target03_d_length);
              target_info.target_dz_decoded = xgu_radar2_target03_b_radar2_target03_dz_decode(r2_target03_b_obj.radar2_target03_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar2_target03_b_radar2_target03_dz_is_in_range(r2_target03_b_obj.radar2_target03_dz);
              target_moving_state_decoded = xgu_radar2_target03_b_radar2_target03_moving_state_decode(
                  r2_target03_b_obj.radar2_target03_moving_state);
              target_moving_state_is_in_range = xgu_radar2_target03_b_radar2_target03_moving_state_is_in_range(
                  r2_target03_b_obj.radar2_target03_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar2_target03_b_radar2_target03_dx_sigma_decode(r2_target03_b_obj.radar2_target03_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar2_target03_b_radar2_target03_dx_sigma_is_in_range(
                  r2_target03_b_obj.radar2_target03_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar2_target03_b_radar2_target03_vx_sigma_decode(r2_target03_b_obj.radar2_target03_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar2_target03_b_radar2_target03_vx_sigma_is_in_range(
                  r2_target03_b_obj.radar2_target03_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar2_target03_b_radar2_target03_ax_sigma_decode(r2_target03_b_obj.radar2_target03_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar2_target03_b_radar2_target03_ax_sigma_is_in_range(
                  r2_target03_b_obj.radar2_target03_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar2_target03_b_radar2_target03_dy_sigma_decode(r2_target03_b_obj.radar2_target03_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar2_target03_b_radar2_target03_dy_sigma_is_in_range(
                  r2_target03_b_obj.radar2_target03_dy_sigma);
              target_w_class_decoded =
                  xgu_radar2_target03_b_radar2_target03_w_class_decode(r2_target03_b_obj.radar2_target03_w_class);
              target_w_class_is_in_range =
                  xgu_radar2_target03_b_radar2_target03_w_class_is_in_range(r2_target03_b_obj.radar2_target03_w_class);
              target_class_decoded =
                  xgu_radar2_target03_b_radar2_target03_class_decode(r2_target03_b_obj.radar2_target03_class);
              target_class_is_in_range =
                  xgu_radar2_target03_b_radar2_target03_class_is_in_range(r2_target03_b_obj.radar2_target03_class);
              target_dx_rear_end_loss_decoded = xgu_radar2_target03_b_radar2_target03_dx_rear_end_loss_decode(
                  r2_target03_b_obj.radar2_target03_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar2_target03_b_radar2_target03_dx_rear_end_loss_is_in_range(
                  r2_target03_b_obj.radar2_target03_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar2_target03_b_radar2_target03_mess_bconsist_bit_decode(
                  r2_target03_b_obj.radar2_target03_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar2_target03_b_radar2_target03_mess_bconsist_bit_is_in_range(
                      r2_target03_b_obj.radar2_target03_mess_bconsist_bit);
              break;
            case 1645:
              xgu_radar1_target04_a_t r1_target04_a_obj;
              target_info.unpack_return = xgu_radar1_target04_a_unpack(&r1_target04_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar1_target04_a_radar1_target04_dx_decode(r1_target04_a_obj.radar1_target04_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_dx_is_in_range(r1_target04_a_obj.radar1_target04_dx);
              target_info.target_vx_decode = xgu_radar1_target04_a_radar1_target04_vx_decode(r1_target04_a_obj.radar1_target04_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_vx_is_in_range(r1_target04_a_obj.radar1_target04_vx);
              target_info.target_dy_decode = xgu_radar1_target04_a_radar1_target04_dy_decode(r1_target04_a_obj.radar1_target04_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_dy_is_in_range(r1_target04_a_obj.radar1_target04_dy);
              target_info.target_w_exist_decode =
                  xgu_radar1_target04_a_radar1_target04_w_exist_decode(r1_target04_a_obj.radar1_target04_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_w_exist_is_in_range(r1_target04_a_obj.radar1_target04_w_exist);
              target_info.target_ax_decode = xgu_radar1_target04_a_radar1_target04_ax_decode(r1_target04_a_obj.radar1_target04_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_ax_is_in_range(r1_target04_a_obj.radar1_target04_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar1_target04_a_radar1_target04_w_obstacle_decode(r1_target04_a_obj.radar1_target04_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar1_target04_a_radar1_target04_w_obstacle_is_in_range(
                  r1_target04_a_obj.radar1_target04_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar1_target04_a_radar1_target04_flag_valid_decode(r1_target04_a_obj.radar1_target04_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar1_target04_a_radar1_target04_flag_valid_is_in_range(
                  r1_target04_a_obj.radar1_target04_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar1_target04_a_radar1_target04_w_non_obstacle_decode(
                  r1_target04_a_obj.radar1_target04_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar1_target04_a_radar1_target04_w_non_obstacle_is_in_range(
                  r1_target04_a_obj.radar1_target04_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar1_target04_a_radar1_target04_flag_meas_decode(r1_target04_a_obj.radar1_target04_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar1_target04_a_radar1_target04_flag_meas_is_in_range(
                  r1_target04_a_obj.radar1_target04_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar1_target04_a_radar1_target04_flag_hist_decode(r1_target04_a_obj.radar1_target04_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar1_target04_a_radar1_target04_flag_hist_is_in_range(
                  r1_target04_a_obj.radar1_target04_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar1_target04_a_radar1_target04_mess_aconsist_bit_decode(
                  r1_target04_a_obj.radar1_target04_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_mess_aconsist_bit_is_in_range(
                      r1_target04_a_obj.radar1_target04_mess_aconsist_bit);
              break;
            case 1646:
              xgu_radar1_target04_b_t r1_target04_b_obj;
              target_info.unpack_return = xgu_radar1_target04_b_unpack(&r1_target04_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar1_target04_b_radar1_target04_vy_decode(r1_target04_b_obj.radar1_target04_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar1_target04_b_radar1_target04_vy_is_in_range(r1_target04_b_obj.radar1_target04_vy);
              target_info.target_d_length_decoded =
                  xgu_radar1_target04_b_radar1_target04_d_length_decode(r1_target04_b_obj.radar1_target04_d_length);
              target_info.target_d_length_is_in_range = xgu_radar1_target04_b_radar1_target04_d_length_is_in_range(
                  r1_target04_b_obj.radar1_target04_d_length);
              target_info.target_dz_decoded = xgu_radar1_target04_b_radar1_target04_dz_decode(r1_target04_b_obj.radar1_target04_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar1_target04_b_radar1_target04_dz_is_in_range(r1_target04_b_obj.radar1_target04_dz);
              target_moving_state_decoded = xgu_radar1_target04_b_radar1_target04_moving_state_decode(
                  r1_target04_b_obj.radar1_target04_moving_state);
              target_moving_state_is_in_range = xgu_radar1_target04_b_radar1_target04_moving_state_is_in_range(
                  r1_target04_b_obj.radar1_target04_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar1_target04_b_radar1_target04_dx_sigma_decode(r1_target04_b_obj.radar1_target04_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar1_target04_b_radar1_target04_dx_sigma_is_in_range(
                  r1_target04_b_obj.radar1_target04_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar1_target04_b_radar1_target04_vx_sigma_decode(r1_target04_b_obj.radar1_target04_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar1_target04_b_radar1_target04_vx_sigma_is_in_range(
                  r1_target04_b_obj.radar1_target04_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar1_target04_b_radar1_target04_ax_sigma_decode(r1_target04_b_obj.radar1_target04_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar1_target04_b_radar1_target04_ax_sigma_is_in_range(
                  r1_target04_b_obj.radar1_target04_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar1_target04_b_radar1_target04_dy_sigma_decode(r1_target04_b_obj.radar1_target04_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar1_target04_b_radar1_target04_dy_sigma_is_in_range(
                  r1_target04_b_obj.radar1_target04_dy_sigma);
              target_w_class_decoded =
                  xgu_radar1_target04_b_radar1_target04_w_class_decode(r1_target04_b_obj.radar1_target04_w_class);
              target_w_class_is_in_range =
                  xgu_radar1_target04_b_radar1_target04_w_class_is_in_range(r1_target04_b_obj.radar1_target04_w_class);
              target_class_decoded =
                  xgu_radar1_target04_b_radar1_target04_class_decode(r1_target04_b_obj.radar1_target04_class);
              target_class_is_in_range =
                  xgu_radar1_target04_b_radar1_target04_class_is_in_range(r1_target04_b_obj.radar1_target04_class);
              target_dx_rear_end_loss_decoded = xgu_radar1_target04_b_radar1_target04_dx_rear_end_loss_decode(
                  r1_target04_b_obj.radar1_target04_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar1_target04_b_radar1_target04_dx_rear_end_loss_is_in_range(
                  r1_target04_b_obj.radar1_target04_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar1_target04_b_radar1_target04_mess_bconsist_bit_decode(
                  r1_target04_b_obj.radar1_target04_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar1_target04_b_radar1_target04_mess_bconsist_bit_is_in_range(
                      r1_target04_b_obj.radar1_target04_mess_bconsist_bit);
              break;
            case 1647:
              xgu_radar2_target04_a_t r2_target04_a_obj;
              target_info.unpack_return = xgu_radar2_target04_a_unpack(&r2_target04_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar2_target04_a_radar2_target04_dx_decode(r2_target04_a_obj.radar2_target04_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar2_target04_a_radar2_target04_dx_is_in_range(r2_target04_a_obj.radar2_target04_dx);
              target_info.target_vx_decode = xgu_radar2_target04_a_radar2_target04_vx_decode(r2_target04_a_obj.radar2_target04_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar2_target04_a_radar2_target04_vx_is_in_range(r2_target04_a_obj.radar2_target04_vx);
              target_info.target_dy_decode = xgu_radar2_target04_a_radar2_target04_dy_decode(r2_target04_a_obj.radar2_target04_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar2_target04_a_radar2_target04_dy_is_in_range(r2_target04_a_obj.radar2_target04_dy);
              target_info.target_w_exist_decode =
                  xgu_radar2_target04_a_radar2_target04_w_exist_decode(r2_target04_a_obj.radar2_target04_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar2_target04_a_radar2_target04_w_exist_is_in_range(r2_target04_a_obj.radar2_target04_w_exist);
              target_info.target_ax_decode = xgu_radar2_target04_a_radar2_target04_ax_decode(r2_target04_a_obj.radar2_target04_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar2_target04_a_radar2_target04_ax_is_in_range(r2_target04_a_obj.radar2_target04_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar2_target04_a_radar2_target04_w_obstacle_decode(r2_target04_a_obj.radar2_target04_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar2_target04_a_radar2_target04_w_obstacle_is_in_range(
                  r2_target04_a_obj.radar2_target04_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar2_target04_a_radar2_target04_flag_valid_decode(r2_target04_a_obj.radar2_target04_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar2_target04_a_radar2_target04_flag_valid_is_in_range(
                  r2_target04_a_obj.radar2_target04_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar2_target04_a_radar2_target04_w_non_obstacle_decode(
                  r2_target04_a_obj.radar2_target04_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar2_target04_a_radar2_target04_w_non_obstacle_is_in_range(
                  r2_target04_a_obj.radar2_target04_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar2_target04_a_radar2_target04_flag_meas_decode(r2_target04_a_obj.radar2_target04_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar2_target04_a_radar2_target04_flag_meas_is_in_range(
                  r2_target04_a_obj.radar2_target04_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar2_target04_a_radar2_target04_flag_hist_decode(r2_target04_a_obj.radar2_target04_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar2_target04_a_radar2_target04_flag_hist_is_in_range(
                  r2_target04_a_obj.radar2_target04_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar2_target04_a_radar2_target04_mess_aconsist_bit_decode(
                  r2_target04_a_obj.radar2_target04_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target04_a_radar2_target04_mess_aconsist_bit_is_in_range(
                      r2_target04_a_obj.radar2_target04_mess_aconsist_bit);
              break;
            case 1648:
              xgu_radar2_target04_b_t r2_target04_b_obj;
              target_info.unpack_return = xgu_radar2_target04_b_unpack(&r2_target04_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar2_target04_b_radar2_target04_vy_decode(r2_target04_b_obj.radar2_target04_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar2_target04_b_radar2_target04_vy_is_in_range(r2_target04_b_obj.radar2_target04_vy);
              target_info.target_d_length_decoded =
                  xgu_radar2_target04_b_radar2_target04_d_length_decode(r2_target04_b_obj.radar2_target04_d_length);
              target_info.target_d_length_is_in_range = xgu_radar2_target04_b_radar2_target04_d_length_is_in_range(
                  r2_target04_b_obj.radar2_target04_d_length);
              target_info.target_dz_decoded = xgu_radar2_target04_b_radar2_target04_dz_decode(r2_target04_b_obj.radar2_target04_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar2_target04_b_radar2_target04_dz_is_in_range(r2_target04_b_obj.radar2_target04_dz);
              target_moving_state_decoded = xgu_radar2_target04_b_radar2_target04_moving_state_decode(
                  r2_target04_b_obj.radar2_target04_moving_state);
              target_moving_state_is_in_range = xgu_radar2_target04_b_radar2_target04_moving_state_is_in_range(
                  r2_target04_b_obj.radar2_target04_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar2_target04_b_radar2_target04_dx_sigma_decode(r2_target04_b_obj.radar2_target04_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar2_target04_b_radar2_target04_dx_sigma_is_in_range(
                  r2_target04_b_obj.radar2_target04_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar2_target04_b_radar2_target04_vx_sigma_decode(r2_target04_b_obj.radar2_target04_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar2_target04_b_radar2_target04_vx_sigma_is_in_range(
                  r2_target04_b_obj.radar2_target04_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar2_target04_b_radar2_target04_ax_sigma_decode(r2_target04_b_obj.radar2_target04_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar2_target04_b_radar2_target04_ax_sigma_is_in_range(
                  r2_target04_b_obj.radar2_target04_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar2_target04_b_radar2_target04_dy_sigma_decode(r2_target04_b_obj.radar2_target04_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar2_target04_b_radar2_target04_dy_sigma_is_in_range(
                  r2_target04_b_obj.radar2_target04_dy_sigma);
              target_w_class_decoded =
                  xgu_radar2_target04_b_radar2_target04_w_class_decode(r2_target04_b_obj.radar2_target04_w_class);
              target_w_class_is_in_range =
                  xgu_radar2_target04_b_radar2_target04_w_class_is_in_range(r2_target04_b_obj.radar2_target04_w_class);
              target_class_decoded =
                  xgu_radar2_target04_b_radar2_target04_class_decode(r2_target04_b_obj.radar2_target04_class);
              target_class_is_in_range =
                  xgu_radar2_target04_b_radar2_target04_class_is_in_range(r2_target04_b_obj.radar2_target04_class);
              target_dx_rear_end_loss_decoded = xgu_radar2_target04_b_radar2_target04_dx_rear_end_loss_decode(
                  r2_target04_b_obj.radar2_target04_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar2_target04_b_radar2_target04_dx_rear_end_loss_is_in_range(
                  r2_target04_b_obj.radar2_target04_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar2_target04_b_radar2_target04_mess_bconsist_bit_decode(
                  r2_target04_b_obj.radar2_target04_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar2_target04_b_radar2_target04_mess_bconsist_bit_is_in_range(
                      r2_target04_b_obj.radar2_target04_mess_bconsist_bit);
              break;
            case 1655:
              xgu_radar1_target05_a_t r1_target05_a_obj;
              target_info.unpack_return = xgu_radar1_target05_a_unpack(&r1_target05_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar1_target05_a_radar1_target05_dx_decode(r1_target05_a_obj.radar1_target05_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_dx_is_in_range(r1_target05_a_obj.radar1_target05_dx);
              target_info.target_vx_decode = xgu_radar1_target05_a_radar1_target05_vx_decode(r1_target05_a_obj.radar1_target05_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_vx_is_in_range(r1_target05_a_obj.radar1_target05_vx);
              target_info.target_dy_decode = xgu_radar1_target05_a_radar1_target05_dy_decode(r1_target05_a_obj.radar1_target05_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_dy_is_in_range(r1_target05_a_obj.radar1_target05_dy);
              target_info.target_w_exist_decode =
                  xgu_radar1_target05_a_radar1_target05_w_exist_decode(r1_target05_a_obj.radar1_target05_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_w_exist_is_in_range(r1_target05_a_obj.radar1_target05_w_exist);
              target_info.target_ax_decode = xgu_radar1_target05_a_radar1_target05_ax_decode(r1_target05_a_obj.radar1_target05_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_ax_is_in_range(r1_target05_a_obj.radar1_target05_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar1_target05_a_radar1_target05_w_obstacle_decode(r1_target05_a_obj.radar1_target05_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar1_target05_a_radar1_target05_w_obstacle_is_in_range(
                  r1_target05_a_obj.radar1_target05_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar1_target05_a_radar1_target05_flag_valid_decode(r1_target05_a_obj.radar1_target05_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar1_target05_a_radar1_target05_flag_valid_is_in_range(
                  r1_target05_a_obj.radar1_target05_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar1_target05_a_radar1_target05_w_non_obstacle_decode(
                  r1_target05_a_obj.radar1_target05_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar1_target05_a_radar1_target05_w_non_obstacle_is_in_range(
                  r1_target05_a_obj.radar1_target05_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar1_target05_a_radar1_target05_flag_meas_decode(r1_target05_a_obj.radar1_target05_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar1_target05_a_radar1_target05_flag_meas_is_in_range(
                  r1_target05_a_obj.radar1_target05_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar1_target05_a_radar1_target05_flag_hist_decode(r1_target05_a_obj.radar1_target05_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar1_target05_a_radar1_target05_flag_hist_is_in_range(
                  r1_target05_a_obj.radar1_target05_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar1_target05_a_radar1_target05_mess_aconsist_bit_decode(
                  r1_target05_a_obj.radar1_target05_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_mess_aconsist_bit_is_in_range(
                      r1_target05_a_obj.radar1_target05_mess_aconsist_bit);
              break;
            case 1656:
              xgu_radar1_target05_b_t r1_target05_b_obj;
              target_info.unpack_return = xgu_radar1_target05_b_unpack(&r1_target05_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar1_target05_b_radar1_target05_vy_decode(r1_target05_b_obj.radar1_target05_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar1_target05_b_radar1_target05_vy_is_in_range(r1_target05_b_obj.radar1_target05_vy);
              target_info.target_d_length_decoded =
                  xgu_radar1_target05_b_radar1_target05_d_length_decode(r1_target05_b_obj.radar1_target05_d_length);
              target_info.target_d_length_is_in_range = xgu_radar1_target05_b_radar1_target05_d_length_is_in_range(
                  r1_target05_b_obj.radar1_target05_d_length);
              target_info.target_dz_decoded = xgu_radar1_target05_b_radar1_target05_dz_decode(r1_target05_b_obj.radar1_target05_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar1_target05_b_radar1_target05_dz_is_in_range(r1_target05_b_obj.radar1_target05_dz);
              target_moving_state_decoded = xgu_radar1_target05_b_radar1_target05_moving_state_decode(
                  r1_target05_b_obj.radar1_target05_moving_state);
              target_moving_state_is_in_range = xgu_radar1_target05_b_radar1_target05_moving_state_is_in_range(
                  r1_target05_b_obj.radar1_target05_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar1_target05_b_radar1_target05_dx_sigma_decode(r1_target05_b_obj.radar1_target05_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar1_target05_b_radar1_target05_dx_sigma_is_in_range(
                  r1_target05_b_obj.radar1_target05_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar1_target05_b_radar1_target05_vx_sigma_decode(r1_target05_b_obj.radar1_target05_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar1_target05_b_radar1_target05_vx_sigma_is_in_range(
                  r1_target05_b_obj.radar1_target05_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar1_target05_b_radar1_target05_ax_sigma_decode(r1_target05_b_obj.radar1_target05_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar1_target05_b_radar1_target05_ax_sigma_is_in_range(
                  r1_target05_b_obj.radar1_target05_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar1_target05_b_radar1_target05_dy_sigma_decode(r1_target05_b_obj.radar1_target05_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar1_target05_b_radar1_target05_dy_sigma_is_in_range(
                  r1_target05_b_obj.radar1_target05_dy_sigma);
              target_w_class_decoded =
                  xgu_radar1_target05_b_radar1_target05_w_class_decode(r1_target05_b_obj.radar1_target05_w_class);
              target_w_class_is_in_range =
                  xgu_radar1_target05_b_radar1_target05_w_class_is_in_range(r1_target05_b_obj.radar1_target05_w_class);
              target_class_decoded =
                  xgu_radar1_target05_b_radar1_target05_class_decode(r1_target05_b_obj.radar1_target05_class);
              target_class_is_in_range =
                  xgu_radar1_target05_b_radar1_target05_class_is_in_range(r1_target05_b_obj.radar1_target05_class);
              target_dx_rear_end_loss_decoded = xgu_radar1_target05_b_radar1_target05_dx_rear_end_loss_decode(
                  r1_target05_b_obj.radar1_target05_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar1_target05_b_radar1_target05_dx_rear_end_loss_is_in_range(
                  r1_target05_b_obj.radar1_target05_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar1_target05_b_radar1_target05_mess_bconsist_bit_decode(
                  r1_target05_b_obj.radar1_target05_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar1_target05_b_radar1_target05_mess_bconsist_bit_is_in_range(
                      r1_target05_b_obj.radar1_target05_mess_bconsist_bit);

              break;
            case 1657:
              xgu_radar2_target05_a_t r2_target05_a_obj;
              target_info.unpack_return = xgu_radar2_target05_a_unpack(&r2_target05_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = xgu_radar2_target05_a_radar2_target05_dx_decode(r2_target05_a_obj.radar2_target05_dx);
              target_info.target_dx_is_in_range =
                  xgu_radar2_target05_a_radar2_target05_dx_is_in_range(r2_target05_a_obj.radar2_target05_dx);
              target_info.target_vx_decode = xgu_radar2_target05_a_radar2_target05_vx_decode(r2_target05_a_obj.radar2_target05_vx);
              target_info.target_vx_is_in_range =
                  xgu_radar2_target05_a_radar2_target05_vx_is_in_range(r2_target05_a_obj.radar2_target05_vx);
              target_info.target_dy_decode = xgu_radar2_target05_a_radar2_target05_dy_decode(r2_target05_a_obj.radar2_target05_dy);
              target_info.target_dy_is_in_range =
                  xgu_radar2_target05_a_radar2_target05_dy_is_in_range(r2_target05_a_obj.radar2_target05_dy);
              target_info.target_w_exist_decode =
                  xgu_radar2_target05_a_radar2_target05_w_exist_decode(r2_target05_a_obj.radar2_target05_w_exist);
              target_info.target_w_exist_is_in_range =
                  xgu_radar2_target05_a_radar2_target05_w_exist_is_in_range(r2_target05_a_obj.radar2_target05_w_exist);
              target_info.target_ax_decode = xgu_radar2_target05_a_radar2_target05_ax_decode(r2_target05_a_obj.radar2_target05_ax);
              target_info.target_ax_is_in_range =
                  xgu_radar2_target05_a_radar2_target05_ax_is_in_range(r2_target05_a_obj.radar2_target05_ax);
              target_info.target_w_obstacle_decode =
                  xgu_radar2_target05_a_radar2_target05_w_obstacle_decode(r2_target05_a_obj.radar2_target05_w_obstacle);
              target_info.target_w_obstacle_is_in_range = xgu_radar2_target05_a_radar2_target05_w_obstacle_is_in_range(
                  r2_target05_a_obj.radar2_target05_w_obstacle);
              target_info.target_flag_valid_decode =
                  xgu_radar2_target05_a_radar2_target05_flag_valid_decode(r2_target05_a_obj.radar2_target05_flag_valid);
              target_info.target_flag_valid_is_in_range = xgu_radar2_target05_a_radar2_target05_flag_valid_is_in_range(
                  r2_target05_a_obj.radar2_target05_flag_valid);
              target_info.target_w_non_obstacle_decode = xgu_radar2_target05_a_radar2_target05_w_non_obstacle_decode(
                  r2_target05_a_obj.radar2_target05_w_non_obstacle);
              target_info.target_w_non_obstacle_is_in_range = xgu_radar2_target05_a_radar2_target05_w_non_obstacle_is_in_range(
                  r2_target05_a_obj.radar2_target05_w_non_obstacle);
              target_info.target_flag_meas_decode =
                  xgu_radar2_target05_a_radar2_target05_flag_meas_decode(r2_target05_a_obj.radar2_target05_flag_meas);
              target_info.target_flag_meas_is_in_range = xgu_radar2_target05_a_radar2_target05_flag_meas_is_in_range(
                  r2_target05_a_obj.radar2_target05_flag_meas);
              target_info.target_flag_hist_decode =
                  xgu_radar2_target05_a_radar2_target05_flag_hist_decode(r2_target05_a_obj.radar2_target05_flag_hist);
              target_info.target_flag_hist_is_in_range = xgu_radar2_target05_a_radar2_target05_flag_hist_is_in_range(
                  r2_target05_a_obj.radar2_target05_flag_hist);
              target_info.target_mess_aconsist_bit_decode = xgu_radar2_target05_a_radar2_target05_mess_aconsist_bit_decode(
                  r2_target05_a_obj.radar2_target05_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target05_a_radar2_target05_mess_aconsist_bit_is_in_range(
                      r2_target05_a_obj.radar2_target05_mess_aconsist_bit);
              break;
            case 1658:
              xgu_radar2_target05_b_t r2_target05_b_obj;
              target_info.unpack_return = xgu_radar2_target05_b_unpack(&r2_target05_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = xgu_radar2_target05_b_radar2_target05_vy_decode(r2_target05_b_obj.radar2_target05_vy);
              target_info.target_vy_is_in_range =
                  xgu_radar2_target05_b_radar2_target05_vy_is_in_range(r2_target05_b_obj.radar2_target05_vy);
              target_info.target_d_length_decoded =
                  xgu_radar2_target05_b_radar2_target05_d_length_decode(r2_target05_b_obj.radar2_target05_d_length);
              target_info.target_d_length_is_in_range = xgu_radar2_target05_b_radar2_target05_d_length_is_in_range(
                  r2_target05_b_obj.radar2_target05_d_length);
              target_info.target_dz_decoded = xgu_radar2_target05_b_radar2_target05_dz_decode(r2_target05_b_obj.radar2_target05_dz);
              target_info.target_dz_is_in_range =
                  xgu_radar2_target05_b_radar2_target05_dz_is_in_range(r2_target05_b_obj.radar2_target05_dz);
              target_moving_state_decoded = xgu_radar2_target05_b_radar2_target05_moving_state_decode(
                  r2_target05_b_obj.radar2_target05_moving_state);
              target_moving_state_is_in_range = xgu_radar2_target05_b_radar2_target05_moving_state_is_in_range(
                  r2_target05_b_obj.radar2_target05_moving_state);
              target_dx_sigma_decoded =
                  xgu_radar2_target05_b_radar2_target05_dx_sigma_decode(r2_target05_b_obj.radar2_target05_dx_sigma);
              target_dx_sigma_is_in_range = xgu_radar2_target05_b_radar2_target05_dx_sigma_is_in_range(
                  r2_target05_b_obj.radar2_target05_dx_sigma);
              target_vx_sigma_decoded =
                  xgu_radar2_target05_b_radar2_target05_vx_sigma_decode(r2_target05_b_obj.radar2_target05_vx_sigma);
              target_vx_sigma_is_in_range = xgu_radar2_target05_b_radar2_target05_vx_sigma_is_in_range(
                  r2_target05_b_obj.radar2_target05_vx_sigma);
              target_ax_sigma_decoded =
                  xgu_radar2_target05_b_radar2_target05_ax_sigma_decode(r2_target05_b_obj.radar2_target05_ax_sigma);
              target_ax_sigma_is_in_range = xgu_radar2_target05_b_radar2_target05_ax_sigma_is_in_range(
                  r2_target05_b_obj.radar2_target05_ax_sigma);
              target_dy_sigma_decoded =
                  xgu_radar2_target05_b_radar2_target05_dy_sigma_decode(r2_target05_b_obj.radar2_target05_dy_sigma);
              target_dy_sigma_is_in_range = xgu_radar2_target05_b_radar2_target05_dy_sigma_is_in_range(
                  r2_target05_b_obj.radar2_target05_dy_sigma);
              target_w_class_decoded =
                  xgu_radar2_target05_b_radar2_target05_w_class_decode(r2_target05_b_obj.radar2_target05_w_class);
              target_w_class_is_in_range =
                  xgu_radar2_target05_b_radar2_target05_w_class_is_in_range(r2_target05_b_obj.radar2_target05_w_class);
              target_class_decoded =
                  xgu_radar2_target05_b_radar2_target05_class_decode(r2_target05_b_obj.radar2_target05_class);
              target_class_is_in_range =
                  xgu_radar2_target05_b_radar2_target05_class_is_in_range(r2_target05_b_obj.radar2_target05_class);
              target_dx_rear_end_loss_decoded = xgu_radar2_target05_b_radar2_target05_dx_rear_end_loss_decode(
                  r2_target05_b_obj.radar2_target05_dx_rear_end_loss);
              target_dx_rear_end_loss_is_in_range = xgu_radar2_target05_b_radar2_target05_dx_rear_end_loss_is_in_range(
                  r2_target05_b_obj.radar2_target05_dx_rear_end_loss);
              target_mess_bconsist_bit_decoded = xgu_radar2_target05_b_radar2_target05_mess_bconsist_bit_decode(
                  r2_target05_b_obj.radar2_target05_mess_bconsist_bit);
              target_mess_bconsist_bit_is_in_range =
                  xgu_radar2_target05_b_radar2_target05_mess_bconsist_bit_is_in_range(
                      r2_target05_b_obj.radar2_target05_mess_bconsist_bit);
              break;
          }
          break;
        case 3:
          switch (id) {
            case 1665:
              xgu_radar1_object_ender_t r1_obj_ender_obj;
              target_info.unpack_return = xgu_radar1_object_ender_unpack(&r1_obj_ender_obj, can_data, size_of_msg);
              radar_info.xgu_radar_timestamp_decoded =
                  xgu_radar1_object_ender_radar1_timestamp_decode(r1_obj_ender_obj.radar1_timestamp);
              radar_info.xgu_radar_timestamp_is_in_range =
                  xgu_radar1_object_ender_radar1_timestamp_is_in_range(r1_obj_ender_obj.radar1_timestamp);
              radar_info.tc_counter_decoded = xgu_radar1_object_ender_radar1_tc_counter_decode(r1_obj_ender_obj.radar1_tc_counter);
              radar_info.tc_counter_is_in_range =
                  xgu_radar1_object_ender_radar1_tc_counter_is_in_range(r1_obj_ender_obj.radar1_tc_counter);
              radar_info.obj_ender_consist_bit_decoded = xgu_radar1_object_ender_radar1_mess_ender_consist_bit_decode(
                  r1_obj_ender_obj.radar1_mess_ender_consist_bit);
              radar_info.obj_ender_consist_bit_is_in_range = xgu_radar1_object_ender_radar1_mess_ender_consist_bit_is_in_range(
                  r1_obj_ender_obj.radar1_mess_ender_consist_bit);
              radar_info.packet_checksum_encoded =
                  xgu_radar1_object_ender_radar1_packet_checksum_decode(r1_obj_ender_obj.radar1_packet_checksum);
              radar_info.packet_checksum_is_in_range =
                  xgu_radar1_object_ender_radar1_packet_checksum_is_in_range(r1_obj_ender_obj.radar1_packet_checksum);
              break;
            case 1667:
              xgu_radar2_object_ender_t r2_obj_ender_obj;
              target_info.unpack_return = xgu_radar2_object_ender_unpack(&r2_obj_ender_obj, can_data, size_of_msg);
              radar_info.xgu_radar_timestamp_decoded =
                  xgu_radar2_object_ender_radar2_object_timestamp_decode(r2_obj_ender_obj.radar2_object_timestamp);
              radar_info.xgu_radar_timestamp_is_in_range =
                  xgu_radar2_object_ender_radar2_object_timestamp_is_in_range(r2_obj_ender_obj.radar2_object_timestamp);
              radar_info.tc_counter_decoded = xgu_radar2_object_ender_radar2_tc_counter_decode(r2_obj_ender_obj.radar2_tc_counter);
              radar_info.tc_counter_is_in_range =
                  xgu_radar2_object_ender_radar2_tc_counter_is_in_range(r2_obj_ender_obj.radar2_tc_counter);
              radar_info.obj_ender_consist_bit_decoded = xgu_radar2_object_ender_radar2_mess_ender_consist_bit_decode(
                  r2_obj_ender_obj.radar2_mess_ender_consist_bit);
              radar_info.obj_ender_consist_bit_is_in_range = xgu_radar2_object_ender_radar2_mess_ender_consist_bit_is_in_range(
                  r2_obj_ender_obj.radar2_mess_ender_consist_bit);
              radar_info.packet_checksum_encoded =
                  xgu_radar2_object_ender_radar2_packet_checksum_decode(r2_obj_ender_obj.radar2_packet_checksum);
              radar_info.packet_checksum_is_in_range =
                  xgu_radar2_object_ender_radar2_packet_checksum_is_in_range(r2_obj_ender_obj.radar2_packet_checksum);
              break;
            case 1280:
              xgu_radar1_object_starter_t r1_obj_starter_obj;
              target_info.unpack_return = xgu_radar1_object_starter_unpack(&r1_obj_starter_obj, can_data, size_of_msg);
              veh_psi_dt_decoded =
                  xgu_radar1_object_starter_radar1_veh_psi_dt_decode(r1_obj_starter_obj.radar1_veh_psi_dt);
              veh_psi_dt_is_in_range =
                  xgu_radar1_object_starter_radar1_veh_psi_dt_is_in_range(r1_obj_starter_obj.radar1_veh_psi_dt);
              veh_v_ego_decoded =
                  xgu_radar1_object_starter_radar1_veh_v_ego_decode(r1_obj_starter_obj.radar1_veh_v_ego);
              veh_v_ego_is_in_range =
                  xgu_radar1_object_starter_radar1_veh_v_ego_is_in_range(r1_obj_starter_obj.radar1_veh_v_ego);
              veh_a_ego_decoded =
                  xgu_radar1_object_starter_radar1_veh_a_ego_decode(r1_obj_starter_obj.radar1_veh_a_ego);
              veh_a_ego_is_in_range =
                  xgu_radar1_object_starter_radar1_veh_a_ego_is_in_range(r1_obj_starter_obj.radar1_veh_a_ego);
              veh_slip_angle_decoded =
                  xgu_radar1_object_starter_radar1_veh_slip_angle_decode(r1_obj_starter_obj.radar1_veh_slip_angle);
              veh_slip_angle_is_in_range =
                  xgu_radar1_object_starter_radar1_veh_slip_angle_is_in_range(r1_obj_starter_obj.radar1_veh_slip_angle);
              mess_starter_consist_bit_decoded = xgu_radar1_object_starter_radar1_mess_starter_consist_bit_decode(
                  r1_obj_starter_obj.radar1_mess_starter_consist_bit);
              mess_starter_consist_bit_is_in_range =
                  xgu_radar1_object_starter_radar1_mess_starter_consist_bit_is_in_range(
                      r1_obj_starter_obj.radar1_mess_starter_consist_bit);
              break;
            case 1282:
              xgu_radar2_object_starter_t r2_obj_starter_obj;
              target_info.unpack_return = xgu_radar2_object_starter_unpack(&r2_obj_starter_obj, can_data, size_of_msg);
              veh_psi_dt_decoded =
                  xgu_radar2_object_starter_radar2_veh_psi_dt_decode(r2_obj_starter_obj.radar2_veh_psi_dt);
              veh_psi_dt_is_in_range =
                  xgu_radar2_object_starter_radar2_veh_psi_dt_is_in_range(r2_obj_starter_obj.radar2_veh_psi_dt);
              veh_v_ego_decoded =
                  xgu_radar2_object_starter_radar2_veh_v_ego_decode(r2_obj_starter_obj.radar2_veh_v_ego);
              veh_v_ego_is_in_range =
                  xgu_radar2_object_starter_radar2_veh_v_ego_is_in_range(r2_obj_starter_obj.radar2_veh_v_ego);
              veh_a_ego_decoded =
                  xgu_radar2_object_starter_radar2_veh_a_ego_decode(r2_obj_starter_obj.radar2_veh_a_ego);
              veh_a_ego_is_in_range =
                  xgu_radar2_object_starter_radar2_veh_a_ego_is_in_range(r2_obj_starter_obj.radar2_veh_a_ego);
              veh_slip_angle_decoded =
                  xgu_radar2_object_starter_radar2_veh_slip_angle_decode(r2_obj_starter_obj.radar2_veh_slip_angle);
              veh_slip_angle_is_in_range =
                  xgu_radar2_object_starter_radar2_veh_slip_angle_is_in_range(r2_obj_starter_obj.radar2_veh_slip_angle);
              mess_starter_consist_bit_decoded = xgu_radar2_object_starter_radar2_mess_starter_consist_bit_decode(
                  r2_obj_starter_obj.radar2_mess_starter_consist_bit);
              mess_starter_consist_bit_is_in_range =
                  xgu_radar2_object_starter_radar2_mess_starter_consist_bit_is_in_range(
                      r2_obj_starter_obj.radar2_mess_starter_consist_bit);
              break;
            case 1670:
              xgu_radar1_status_t r1_status;
              target_info.unpack_return = xgu_radar1_status_unpack(&r1_status, can_data, size_of_msg);
              itc_info_decoded = xgu_radar1_status_r1_stat_itc_info_decode(r1_status.r1_stat_itc_info);
              itc_info_is_in_range = xgu_radar1_status_r1_stat_itc_info_is_in_range(r1_status.r1_stat_itc_info);
              sgu_fail_decoded = xgu_radar1_status_r1_stat_sgu_fail_decode(r1_status.r1_stat_sgu_fail);
              sgu_fail_is_in_range = xgu_radar1_status_r1_stat_sgu_fail_is_in_range(r1_status.r1_stat_sgu_fail);
              hw_fail_decoded = xgu_radar1_status_r1_stat_hw_fail_decode(r1_status.r1_stat_hw_fail);
              hw_fail_is_in_range = xgu_radar1_status_r1_stat_hw_fail_is_in_range(r1_status.r1_stat_hw_fail);
              horizontal_misalignment_decoded =
                  xgu_radar1_status_r1_stat_horizontal_misalignment_decode(r1_status.r1_stat_horizontal_misalignment);
              horizontal_misalignment_is_in_range = xgu_radar1_status_r1_stat_horizontal_misalignment_is_in_range(
                  r1_status.r1_stat_horizontal_misalignment);
              absorption_blindness_decoded =
                  xgu_radar1_status_r1_stat_absorption_blindness_decode(r1_status.r1_stat_absorption_blindness);
              absorption_blindness_is_in_range =
                  xgu_radar1_status_r1_stat_absorption_blindness_is_in_range(r1_status.r1_stat_absorption_blindness);
              distortion_blindness_decoded =
                  xgu_radar1_status_r1_stat_distortion_blindness_decode(r1_status.r1_stat_distortion_blindness);
              distortion_blindness_is_in_range =
                  xgu_radar1_status_r1_stat_distortion_blindness_is_in_range(r1_status.r1_stat_distortion_blindness);
              mc_decoded = xgu_radar1_status_r1_stat_mc_decode(r1_status.r1_stat_mc);
              mc_is_in_range = xgu_radar1_status_r1_stat_mc_is_in_range(r1_status.r1_stat_mc);
              crc_decoded = xgu_radar1_status_r1_stat_crc_decode(r1_status.r1_stat_crc);
              crc_is_in_range = xgu_radar1_status_r1_stat_crc_is_in_range(r1_status.r1_stat_crc);
              break;
            case 1672:
              xgu_radar2_status_t r2_status;
              target_info.unpack_return = xgu_radar2_status_unpack(&r2_status, can_data, size_of_msg);
              itc_info_decoded = xgu_radar2_status_r2_stat_itc_info_decode(r2_status.r2_stat_itc_info);
              itc_info_is_in_range = xgu_radar2_status_r2_stat_itc_info_is_in_range(r2_status.r2_stat_itc_info);
              sgu_fail_decoded = xgu_radar2_status_r2_stat_sgu_fail_decode(r2_status.r2_stat_sgu_fail);
              sgu_fail_is_in_range = xgu_radar2_status_r2_stat_sgu_fail_is_in_range(r2_status.r2_stat_sgu_fail);
              hw_fail_decoded = xgu_radar2_status_r2_stat_hw_fail_decode(r2_status.r2_stat_hw_fail);
              hw_fail_is_in_range = xgu_radar2_status_r2_stat_hw_fail_is_in_range(r2_status.r2_stat_hw_fail);
              horizontal_misalignment_decoded =
                  xgu_radar2_status_r2_stat_horizontal_misalignment_decode(r2_status.r2_stat_horizontal_misalignment);
              horizontal_misalignment_is_in_range = xgu_radar2_status_r2_stat_horizontal_misalignment_is_in_range(
                  r2_status.r2_stat_horizontal_misalignment);
              absorption_blindness_decoded =
                  xgu_radar2_status_r2_stat_absorption_blindness_decode(r2_status.r2_stat_absorption_blindness);
              absorption_blindness_is_in_range =
                  xgu_radar2_status_r2_stat_absorption_blindness_is_in_range(r2_status.r2_stat_absorption_blindness);
              distortion_blindness_decoded =
                  xgu_radar2_status_r2_stat_distortion_blindness_decode(r2_status.r2_stat_distortion_blindness);
              distortion_blindness_is_in_range =
                  xgu_radar2_status_r2_stat_distortion_blindness_is_in_range(r2_status.r2_stat_distortion_blindness);
              mc_decoded = xgu_radar2_status_r2_stat_mc_decode(r2_status.r2_stat_mc);
              mc_is_in_range = xgu_radar2_status_r2_stat_mc_is_in_range(r2_status.r2_stat_mc);
              crc_decoded = xgu_radar2_status_r2_stat_crc_decode(r2_status.r2_stat_crc);
              crc_is_in_range = xgu_radar2_status_r2_stat_crc_is_in_range(r2_status.r2_stat_crc);
              break;
          }
        memcpy(serialized_radar_info, &radar_info, sizeof(radar_info));
        break;
        case 4:
          switch (id) {
            case 1285:
              xgu_radar1_obj00_a_t r1_obj00_a;
              target_info.unpack_return = xgu_radar1_obj00_a_unpack(&r1_obj00_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj00_a_radar1_obj00_dx_decode(r1_obj00_a.radar1_obj00_dx);
              dx_is_in_range = xgu_radar1_obj00_a_radar1_obj00_dx_is_in_range(r1_obj00_a.radar1_obj00_dx);
              vx_decoded = xgu_radar1_obj00_a_radar1_obj00_vx_decode(r1_obj00_a.radar1_obj00_vx);
              vx_is_in_range = xgu_radar1_obj00_a_radar1_obj00_vx_is_in_range(r1_obj00_a.radar1_obj00_vx);
              dy_decoded = xgu_radar1_obj00_a_radar1_obj00_dy_decode(r1_obj00_a.radar1_obj00_dy);
              dy_is_in_range = xgu_radar1_obj00_a_radar1_obj00_dy_is_in_range(r1_obj00_a.radar1_obj00_dy);
              w_exist_decoded = xgu_radar1_obj00_a_radar1_obj00_w_exist_decode(r1_obj00_a.radar1_obj00_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj00_a_radar1_obj00_w_exist_is_in_range(r1_obj00_a.radar1_obj00_w_exist);
              ax_decoded = xgu_radar1_obj00_a_radar1_obj00_ax_decode(r1_obj00_a.radar1_obj00_ax);
              ax_is_in_range = xgu_radar1_obj00_a_radar1_obj00_ax_is_in_range(r1_obj00_a.radar1_obj00_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj00_a_radar1_obj00_w_obstacle_decode(r1_obj00_a.radar1_obj00_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj00_a_radar1_obj00_w_obstacle_is_in_range(r1_obj00_a.radar1_obj00_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj00_a_radar1_obj00_flag_valid_decode(r1_obj00_a.radar1_obj00_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj00_a_radar1_obj00_flag_valid_is_in_range(r1_obj00_a.radar1_obj00_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj00_a_radar1_obj00_w_non_obstacle_decode(r1_obj00_a.radar1_obj00_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj00_a_radar1_obj00_w_non_obstacle_is_in_range(r1_obj00_a.radar1_obj00_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj00_a_radar1_obj00_flag_meas_decode(r1_obj00_a.radar1_obj00_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj00_a_radar1_obj00_flag_meas_is_in_range(r1_obj00_a.radar1_obj00_flag_meas);
              flag_hist_decoded = xgu_radar1_obj00_a_radar1_obj00_flag_hist_decode(r1_obj00_a.radar1_obj00_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj00_a_radar1_obj00_flag_hist_is_in_range(r1_obj00_a.radar1_obj00_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj00_a_radar1_obj00_mess_aconsist_bit_decode(r1_obj00_a.radar1_obj00_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj00_a_radar1_obj00_mess_aconsist_bit_is_in_range(
                  r1_obj00_a.radar1_obj00_mess_aconsist_bit);
              break;
            case 1295:
              xgu_radar1_obj01_a_t r1_obj01_a;
              target_info.unpack_return = xgu_radar1_obj01_a_unpack(&r1_obj01_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj01_a_radar1_obj01_dx_decode(r1_obj01_a.radar1_obj01_dx);
              dx_is_in_range = xgu_radar1_obj01_a_radar1_obj01_dx_is_in_range(r1_obj01_a.radar1_obj01_dx);
              vx_decoded = xgu_radar1_obj01_a_radar1_obj01_vx_decode(r1_obj01_a.radar1_obj01_vx);
              vx_is_in_range = xgu_radar1_obj01_a_radar1_obj01_vx_is_in_range(r1_obj01_a.radar1_obj01_vx);
              dy_decoded = xgu_radar1_obj01_a_radar1_obj01_dy_decode(r1_obj01_a.radar1_obj01_dy);
              dy_is_in_range = xgu_radar1_obj01_a_radar1_obj01_dy_is_in_range(r1_obj01_a.radar1_obj01_dy);
              w_exist_decoded = xgu_radar1_obj01_a_radar1_obj01_w_exist_decode(r1_obj01_a.radar1_obj01_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj01_a_radar1_obj01_w_exist_is_in_range(r1_obj01_a.radar1_obj01_w_exist);
              ax_decoded = xgu_radar1_obj01_a_radar1_obj01_ax_decode(r1_obj01_a.radar1_obj01_ax);
              ax_is_in_range = xgu_radar1_obj01_a_radar1_obj01_ax_is_in_range(r1_obj01_a.radar1_obj01_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj01_a_radar1_obj01_w_obstacle_decode(r1_obj01_a.radar1_obj01_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj01_a_radar1_obj01_w_obstacle_is_in_range(r1_obj01_a.radar1_obj01_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj01_a_radar1_obj01_flag_valid_decode(r1_obj01_a.radar1_obj01_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj01_a_radar1_obj01_flag_valid_is_in_range(r1_obj01_a.radar1_obj01_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj01_a_radar1_obj01_w_non_obstacle_decode(r1_obj01_a.radar1_obj01_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj01_a_radar1_obj01_w_non_obstacle_is_in_range(r1_obj01_a.radar1_obj01_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj01_a_radar1_obj01_flag_meas_decode(r1_obj01_a.radar1_obj01_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj01_a_radar1_obj01_flag_meas_is_in_range(r1_obj01_a.radar1_obj01_flag_meas);
              flag_hist_decoded = xgu_radar1_obj01_a_radar1_obj01_flag_hist_decode(r1_obj01_a.radar1_obj01_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj01_a_radar1_obj01_flag_hist_is_in_range(r1_obj01_a.radar1_obj01_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj01_a_radar1_obj01_mess_aconsist_bit_decode(r1_obj01_a.radar1_obj01_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj01_a_radar1_obj01_mess_aconsist_bit_is_in_range(
                  r1_obj01_a.radar1_obj01_mess_aconsist_bit);
              break;
            case 1305:
              xgu_radar1_obj02_a_t r1_obj02_a;
              target_info.unpack_return = xgu_radar1_obj02_a_unpack(&r1_obj02_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj02_a_radar1_obj02_dx_decode(r1_obj02_a.radar1_obj02_dx);
              dx_is_in_range = xgu_radar1_obj02_a_radar1_obj02_dx_is_in_range(r1_obj02_a.radar1_obj02_dx);
              vx_decoded = xgu_radar1_obj02_a_radar1_obj02_vx_decode(r1_obj02_a.radar1_obj02_vx);
              vx_is_in_range = xgu_radar1_obj02_a_radar1_obj02_vx_is_in_range(r1_obj02_a.radar1_obj02_vx);
              dy_decoded = xgu_radar1_obj02_a_radar1_obj02_dy_decode(r1_obj02_a.radar1_obj02_dy);
              dy_is_in_range = xgu_radar1_obj02_a_radar1_obj02_dy_is_in_range(r1_obj02_a.radar1_obj02_dy);
              w_exist_decoded = xgu_radar1_obj02_a_radar1_obj02_w_exist_decode(r1_obj02_a.radar1_obj02_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj02_a_radar1_obj02_w_exist_is_in_range(r1_obj02_a.radar1_obj02_w_exist);
              ax_decoded = xgu_radar1_obj02_a_radar1_obj02_ax_decode(r1_obj02_a.radar1_obj02_ax);
              ax_is_in_range = xgu_radar1_obj02_a_radar1_obj02_ax_is_in_range(r1_obj02_a.radar1_obj02_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj02_a_radar1_obj02_w_obstacle_decode(r1_obj02_a.radar1_obj02_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj02_a_radar1_obj02_w_obstacle_is_in_range(r1_obj02_a.radar1_obj02_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj02_a_radar1_obj02_flag_valid_decode(r1_obj02_a.radar1_obj02_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj02_a_radar1_obj02_flag_valid_is_in_range(r1_obj02_a.radar1_obj02_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj02_a_radar1_obj02_w_non_obstacle_decode(r1_obj02_a.radar1_obj02_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj02_a_radar1_obj02_w_non_obstacle_is_in_range(r1_obj02_a.radar1_obj02_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj02_a_radar1_obj02_flag_meas_decode(r1_obj02_a.radar1_obj02_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj02_a_radar1_obj02_flag_meas_is_in_range(r1_obj02_a.radar1_obj02_flag_meas);
              flag_hist_decoded = xgu_radar1_obj02_a_radar1_obj02_flag_hist_decode(r1_obj02_a.radar1_obj02_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj02_a_radar1_obj02_flag_hist_is_in_range(r1_obj02_a.radar1_obj02_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj02_a_radar1_obj02_mess_aconsist_bit_decode(r1_obj02_a.radar1_obj02_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj02_a_radar1_obj02_mess_aconsist_bit_is_in_range(
                  r1_obj02_a.radar1_obj02_mess_aconsist_bit);
              break;
            case 1315:
              xgu_radar1_obj03_a_t r1_obj03_a;
              target_info.unpack_return = xgu_radar1_obj03_a_unpack(&r1_obj03_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj03_a_radar1_obj03_dx_decode(r1_obj03_a.radar1_obj03_dx);
              dx_is_in_range = xgu_radar1_obj03_a_radar1_obj03_dx_is_in_range(r1_obj03_a.radar1_obj03_dx);
              vx_decoded = xgu_radar1_obj03_a_radar1_obj03_vx_decode(r1_obj03_a.radar1_obj03_vx);
              vx_is_in_range = xgu_radar1_obj03_a_radar1_obj03_vx_is_in_range(r1_obj03_a.radar1_obj03_vx);
              dy_decoded = xgu_radar1_obj03_a_radar1_obj03_dy_decode(r1_obj03_a.radar1_obj03_dy);
              dy_is_in_range = xgu_radar1_obj03_a_radar1_obj03_dy_is_in_range(r1_obj03_a.radar1_obj03_dy);
              w_exist_decoded = xgu_radar1_obj03_a_radar1_obj03_w_exist_decode(r1_obj03_a.radar1_obj03_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj03_a_radar1_obj03_w_exist_is_in_range(r1_obj03_a.radar1_obj03_w_exist);
              ax_decoded = xgu_radar1_obj03_a_radar1_obj03_ax_decode(r1_obj03_a.radar1_obj03_ax);
              ax_is_in_range = xgu_radar1_obj03_a_radar1_obj03_ax_is_in_range(r1_obj03_a.radar1_obj03_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj03_a_radar1_obj03_w_obstacle_decode(r1_obj03_a.radar1_obj03_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj03_a_radar1_obj03_w_obstacle_is_in_range(r1_obj03_a.radar1_obj03_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj03_a_radar1_obj03_flag_valid_decode(r1_obj03_a.radar1_obj03_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj03_a_radar1_obj03_flag_valid_is_in_range(r1_obj03_a.radar1_obj03_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj03_a_radar1_obj03_w_non_obstacle_decode(r1_obj03_a.radar1_obj03_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj03_a_radar1_obj03_w_non_obstacle_is_in_range(r1_obj03_a.radar1_obj03_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj03_a_radar1_obj03_flag_meas_decode(r1_obj03_a.radar1_obj03_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj03_a_radar1_obj03_flag_meas_is_in_range(r1_obj03_a.radar1_obj03_flag_meas);
              flag_hist_decoded = xgu_radar1_obj03_a_radar1_obj03_flag_hist_decode(r1_obj03_a.radar1_obj03_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj03_a_radar1_obj03_flag_hist_is_in_range(r1_obj03_a.radar1_obj03_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj03_a_radar1_obj03_mess_aconsist_bit_decode(r1_obj03_a.radar1_obj03_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj03_a_radar1_obj03_mess_aconsist_bit_is_in_range(
                  r1_obj03_a.radar1_obj03_mess_aconsist_bit);
              break;
            case 1325:
              xgu_radar1_obj04_a_t r1_obj04_a;
              target_info.unpack_return = xgu_radar1_obj04_a_unpack(&r1_obj04_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj04_a_radar1_obj04_dx_decode(r1_obj04_a.radar1_obj04_dx);
              dx_is_in_range = xgu_radar1_obj04_a_radar1_obj04_dx_is_in_range(r1_obj04_a.radar1_obj04_dx);
              vx_decoded = xgu_radar1_obj04_a_radar1_obj04_vx_decode(r1_obj04_a.radar1_obj04_vx);
              vx_is_in_range = xgu_radar1_obj04_a_radar1_obj04_vx_is_in_range(r1_obj04_a.radar1_obj04_vx);
              dy_decoded = xgu_radar1_obj04_a_radar1_obj04_dy_decode(r1_obj04_a.radar1_obj04_dy);
              dy_is_in_range = xgu_radar1_obj04_a_radar1_obj04_dy_is_in_range(r1_obj04_a.radar1_obj04_dy);
              w_exist_decoded = xgu_radar1_obj04_a_radar1_obj04_w_exist_decode(r1_obj04_a.radar1_obj04_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj04_a_radar1_obj04_w_exist_is_in_range(r1_obj04_a.radar1_obj04_w_exist);
              ax_decoded = xgu_radar1_obj04_a_radar1_obj04_ax_decode(r1_obj04_a.radar1_obj04_ax);
              ax_is_in_range = xgu_radar1_obj04_a_radar1_obj04_ax_is_in_range(r1_obj04_a.radar1_obj04_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj04_a_radar1_obj04_w_obstacle_decode(r1_obj04_a.radar1_obj04_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj04_a_radar1_obj04_w_obstacle_is_in_range(r1_obj04_a.radar1_obj04_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj04_a_radar1_obj04_flag_valid_decode(r1_obj04_a.radar1_obj04_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj04_a_radar1_obj04_flag_valid_is_in_range(r1_obj04_a.radar1_obj04_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj04_a_radar1_obj04_w_non_obstacle_decode(r1_obj04_a.radar1_obj04_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj04_a_radar1_obj04_w_non_obstacle_is_in_range(r1_obj04_a.radar1_obj04_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj04_a_radar1_obj04_flag_meas_decode(r1_obj04_a.radar1_obj04_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj04_a_radar1_obj04_flag_meas_is_in_range(r1_obj04_a.radar1_obj04_flag_meas);
              flag_hist_decoded = xgu_radar1_obj04_a_radar1_obj04_flag_hist_decode(r1_obj04_a.radar1_obj04_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj04_a_radar1_obj04_flag_hist_is_in_range(r1_obj04_a.radar1_obj04_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj04_a_radar1_obj04_mess_aconsist_bit_decode(r1_obj04_a.radar1_obj04_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj04_a_radar1_obj04_mess_aconsist_bit_is_in_range(
                  r1_obj04_a.radar1_obj04_mess_aconsist_bit);
              break;
            case 1335:
              xgu_radar1_obj05_a_t r1_obj05_a;
              target_info.unpack_return = xgu_radar1_obj05_a_unpack(&r1_obj05_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj05_a_radar1_obj05_dx_decode(r1_obj05_a.radar1_obj05_dx);
              dx_is_in_range = xgu_radar1_obj05_a_radar1_obj05_dx_is_in_range(r1_obj05_a.radar1_obj05_dx);
              vx_decoded = xgu_radar1_obj05_a_radar1_obj05_vx_decode(r1_obj05_a.radar1_obj05_vx);
              vx_is_in_range = xgu_radar1_obj05_a_radar1_obj05_vx_is_in_range(r1_obj05_a.radar1_obj05_vx);
              dy_decoded = xgu_radar1_obj05_a_radar1_obj05_dy_decode(r1_obj05_a.radar1_obj05_dy);
              dy_is_in_range = xgu_radar1_obj05_a_radar1_obj05_dy_is_in_range(r1_obj05_a.radar1_obj05_dy);
              w_exist_decoded = xgu_radar1_obj05_a_radar1_obj05_w_exist_decode(r1_obj05_a.radar1_obj05_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj05_a_radar1_obj05_w_exist_is_in_range(r1_obj05_a.radar1_obj05_w_exist);
              ax_decoded = xgu_radar1_obj05_a_radar1_obj05_ax_decode(r1_obj05_a.radar1_obj05_ax);
              ax_is_in_range = xgu_radar1_obj05_a_radar1_obj05_ax_is_in_range(r1_obj05_a.radar1_obj05_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj05_a_radar1_obj05_w_obstacle_decode(r1_obj05_a.radar1_obj05_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj05_a_radar1_obj05_w_obstacle_is_in_range(r1_obj05_a.radar1_obj05_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj05_a_radar1_obj05_flag_valid_decode(r1_obj05_a.radar1_obj05_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj05_a_radar1_obj05_flag_valid_is_in_range(r1_obj05_a.radar1_obj05_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj05_a_radar1_obj05_w_non_obstacle_decode(r1_obj05_a.radar1_obj05_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj05_a_radar1_obj05_w_non_obstacle_is_in_range(r1_obj05_a.radar1_obj05_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj05_a_radar1_obj05_flag_meas_decode(r1_obj05_a.radar1_obj05_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj05_a_radar1_obj05_flag_meas_is_in_range(r1_obj05_a.radar1_obj05_flag_meas);
              flag_hist_decoded = xgu_radar1_obj05_a_radar1_obj05_flag_hist_decode(r1_obj05_a.radar1_obj05_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj05_a_radar1_obj05_flag_hist_is_in_range(r1_obj05_a.radar1_obj05_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj05_a_radar1_obj05_mess_aconsist_bit_decode(r1_obj05_a.radar1_obj05_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj05_a_radar1_obj05_mess_aconsist_bit_is_in_range(
                  r1_obj05_a.radar1_obj05_mess_aconsist_bit);
              break;
            case 1345:
              xgu_radar1_obj06_a_t r1_obj06_a;
              target_info.unpack_return = xgu_radar1_obj06_a_unpack(&r1_obj06_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj06_a_radar1_obj06_dx_decode(r1_obj06_a.radar1_obj06_dx);
              dx_is_in_range = xgu_radar1_obj06_a_radar1_obj06_dx_is_in_range(r1_obj06_a.radar1_obj06_dx);
              vx_decoded = xgu_radar1_obj06_a_radar1_obj06_vx_decode(r1_obj06_a.radar1_obj06_vx);
              vx_is_in_range = xgu_radar1_obj06_a_radar1_obj06_vx_is_in_range(r1_obj06_a.radar1_obj06_vx);
              dy_decoded = xgu_radar1_obj06_a_radar1_obj06_dy_decode(r1_obj06_a.radar1_obj06_dy);
              dy_is_in_range = xgu_radar1_obj06_a_radar1_obj06_dy_is_in_range(r1_obj06_a.radar1_obj06_dy);
              w_exist_decoded = xgu_radar1_obj06_a_radar1_obj06_w_exist_decode(r1_obj06_a.radar1_obj06_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj06_a_radar1_obj06_w_exist_is_in_range(r1_obj06_a.radar1_obj06_w_exist);
              ax_decoded = xgu_radar1_obj06_a_radar1_obj06_ax_decode(r1_obj06_a.radar1_obj06_ax);
              ax_is_in_range = xgu_radar1_obj06_a_radar1_obj06_ax_is_in_range(r1_obj06_a.radar1_obj06_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj06_a_radar1_obj06_w_obstacle_decode(r1_obj06_a.radar1_obj06_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj06_a_radar1_obj06_w_obstacle_is_in_range(r1_obj06_a.radar1_obj06_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj06_a_radar1_obj06_flag_valid_decode(r1_obj06_a.radar1_obj06_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj06_a_radar1_obj06_flag_valid_is_in_range(r1_obj06_a.radar1_obj06_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj06_a_radar1_obj06_w_non_obstacle_decode(r1_obj06_a.radar1_obj06_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj06_a_radar1_obj06_w_non_obstacle_is_in_range(r1_obj06_a.radar1_obj06_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj06_a_radar1_obj06_flag_meas_decode(r1_obj06_a.radar1_obj06_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj06_a_radar1_obj06_flag_meas_is_in_range(r1_obj06_a.radar1_obj06_flag_meas);
              flag_hist_decoded = xgu_radar1_obj06_a_radar1_obj06_flag_hist_decode(r1_obj06_a.radar1_obj06_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj06_a_radar1_obj06_flag_hist_is_in_range(r1_obj06_a.radar1_obj06_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj06_a_radar1_obj06_mess_aconsist_bit_decode(r1_obj06_a.radar1_obj06_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj06_a_radar1_obj06_mess_aconsist_bit_is_in_range(
                  r1_obj06_a.radar1_obj06_mess_aconsist_bit);
              break;
            case 1355:
              xgu_radar1_obj07_a_t r1_obj07_a;
              target_info.unpack_return = xgu_radar1_obj07_a_unpack(&r1_obj07_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj07_a_radar1_obj07_dx_decode(r1_obj07_a.radar1_obj07_dx);
              dx_is_in_range = xgu_radar1_obj07_a_radar1_obj07_dx_is_in_range(r1_obj07_a.radar1_obj07_dx);
              vx_decoded = xgu_radar1_obj07_a_radar1_obj07_vx_decode(r1_obj07_a.radar1_obj07_vx);
              vx_is_in_range = xgu_radar1_obj07_a_radar1_obj07_vx_is_in_range(r1_obj07_a.radar1_obj07_vx);
              dy_decoded = xgu_radar1_obj07_a_radar1_obj07_dy_decode(r1_obj07_a.radar1_obj07_dy);
              dy_is_in_range = xgu_radar1_obj07_a_radar1_obj07_dy_is_in_range(r1_obj07_a.radar1_obj07_dy);
              w_exist_decoded = xgu_radar1_obj07_a_radar1_obj07_w_exist_decode(r1_obj07_a.radar1_obj07_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj07_a_radar1_obj07_w_exist_is_in_range(r1_obj07_a.radar1_obj07_w_exist);
              ax_decoded = xgu_radar1_obj07_a_radar1_obj07_ax_decode(r1_obj07_a.radar1_obj07_ax);
              ax_is_in_range = xgu_radar1_obj07_a_radar1_obj07_ax_is_in_range(r1_obj07_a.radar1_obj07_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj07_a_radar1_obj07_w_obstacle_decode(r1_obj07_a.radar1_obj07_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj07_a_radar1_obj07_w_obstacle_is_in_range(r1_obj07_a.radar1_obj07_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj07_a_radar1_obj07_flag_valid_decode(r1_obj07_a.radar1_obj07_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj07_a_radar1_obj07_flag_valid_is_in_range(r1_obj07_a.radar1_obj07_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj07_a_radar1_obj07_w_non_obstacle_decode(r1_obj07_a.radar1_obj07_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj07_a_radar1_obj07_w_non_obstacle_is_in_range(r1_obj07_a.radar1_obj07_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj07_a_radar1_obj07_flag_meas_decode(r1_obj07_a.radar1_obj07_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj07_a_radar1_obj07_flag_meas_is_in_range(r1_obj07_a.radar1_obj07_flag_meas);
              flag_hist_decoded = xgu_radar1_obj07_a_radar1_obj07_flag_hist_decode(r1_obj07_a.radar1_obj07_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj07_a_radar1_obj07_flag_hist_is_in_range(r1_obj07_a.radar1_obj07_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj07_a_radar1_obj07_mess_aconsist_bit_decode(r1_obj07_a.radar1_obj07_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj07_a_radar1_obj07_mess_aconsist_bit_is_in_range(
                  r1_obj07_a.radar1_obj07_mess_aconsist_bit);
              break;
            case 1365:
              xgu_radar1_obj08_a_t r1_obj08_a;
              target_info.unpack_return = xgu_radar1_obj08_a_unpack(&r1_obj08_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj08_a_radar1_obj08_dx_decode(r1_obj08_a.radar1_obj08_dx);
              dx_is_in_range = xgu_radar1_obj08_a_radar1_obj08_dx_is_in_range(r1_obj08_a.radar1_obj08_dx);
              vx_decoded = xgu_radar1_obj08_a_radar1_obj08_vx_decode(r1_obj08_a.radar1_obj08_vx);
              vx_is_in_range = xgu_radar1_obj08_a_radar1_obj08_vx_is_in_range(r1_obj08_a.radar1_obj08_vx);
              dy_decoded = xgu_radar1_obj08_a_radar1_obj08_dy_decode(r1_obj08_a.radar1_obj08_dy);
              dy_is_in_range = xgu_radar1_obj08_a_radar1_obj08_dy_is_in_range(r1_obj08_a.radar1_obj08_dy);
              w_exist_decoded = xgu_radar1_obj08_a_radar1_obj08_w_exist_decode(r1_obj08_a.radar1_obj08_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj08_a_radar1_obj08_w_exist_is_in_range(r1_obj08_a.radar1_obj08_w_exist);
              ax_decoded = xgu_radar1_obj08_a_radar1_obj08_ax_decode(r1_obj08_a.radar1_obj08_ax);
              ax_is_in_range = xgu_radar1_obj08_a_radar1_obj08_ax_is_in_range(r1_obj08_a.radar1_obj08_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj08_a_radar1_obj08_w_obstacle_decode(r1_obj08_a.radar1_obj08_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj08_a_radar1_obj08_w_obstacle_is_in_range(r1_obj08_a.radar1_obj08_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj08_a_radar1_obj08_flag_valid_decode(r1_obj08_a.radar1_obj08_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj08_a_radar1_obj08_flag_valid_is_in_range(r1_obj08_a.radar1_obj08_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj08_a_radar1_obj08_w_non_obstacle_decode(r1_obj08_a.radar1_obj08_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj08_a_radar1_obj08_w_non_obstacle_is_in_range(r1_obj08_a.radar1_obj08_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj08_a_radar1_obj08_flag_meas_decode(r1_obj08_a.radar1_obj08_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj08_a_radar1_obj08_flag_meas_is_in_range(r1_obj08_a.radar1_obj08_flag_meas);
              flag_hist_decoded = xgu_radar1_obj08_a_radar1_obj08_flag_hist_decode(r1_obj08_a.radar1_obj08_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj08_a_radar1_obj08_flag_hist_is_in_range(r1_obj08_a.radar1_obj08_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj08_a_radar1_obj08_mess_aconsist_bit_decode(r1_obj08_a.radar1_obj08_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj08_a_radar1_obj08_mess_aconsist_bit_is_in_range(
                  r1_obj08_a.radar1_obj08_mess_aconsist_bit);
              break;
            case 1375:
              xgu_radar1_obj09_a_t r1_obj09_a;
              target_info.unpack_return = xgu_radar1_obj09_a_unpack(&r1_obj09_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj09_a_radar1_obj09_dx_decode(r1_obj09_a.radar1_obj09_dx);
              dx_is_in_range = xgu_radar1_obj09_a_radar1_obj09_dx_is_in_range(r1_obj09_a.radar1_obj09_dx);
              vx_decoded = xgu_radar1_obj09_a_radar1_obj09_vx_decode(r1_obj09_a.radar1_obj09_vx);
              vx_is_in_range = xgu_radar1_obj09_a_radar1_obj09_vx_is_in_range(r1_obj09_a.radar1_obj09_vx);
              dy_decoded = xgu_radar1_obj09_a_radar1_obj09_dy_decode(r1_obj09_a.radar1_obj09_dy);
              dy_is_in_range = xgu_radar1_obj09_a_radar1_obj09_dy_is_in_range(r1_obj09_a.radar1_obj09_dy);
              w_exist_decoded = xgu_radar1_obj09_a_radar1_obj09_w_exist_decode(r1_obj09_a.radar1_obj09_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj09_a_radar1_obj09_w_exist_is_in_range(r1_obj09_a.radar1_obj09_w_exist);
              ax_decoded = xgu_radar1_obj09_a_radar1_obj09_ax_decode(r1_obj09_a.radar1_obj09_ax);
              ax_is_in_range = xgu_radar1_obj09_a_radar1_obj09_ax_is_in_range(r1_obj09_a.radar1_obj09_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj09_a_radar1_obj09_w_obstacle_decode(r1_obj09_a.radar1_obj09_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj09_a_radar1_obj09_w_obstacle_is_in_range(r1_obj09_a.radar1_obj09_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj09_a_radar1_obj09_flag_valid_decode(r1_obj09_a.radar1_obj09_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj09_a_radar1_obj09_flag_valid_is_in_range(r1_obj09_a.radar1_obj09_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj09_a_radar1_obj09_w_non_obstacle_decode(r1_obj09_a.radar1_obj09_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj09_a_radar1_obj09_w_non_obstacle_is_in_range(r1_obj09_a.radar1_obj09_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj09_a_radar1_obj09_flag_meas_decode(r1_obj09_a.radar1_obj09_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj09_a_radar1_obj09_flag_meas_is_in_range(r1_obj09_a.radar1_obj09_flag_meas);
              flag_hist_decoded = xgu_radar1_obj09_a_radar1_obj09_flag_hist_decode(r1_obj09_a.radar1_obj09_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj09_a_radar1_obj09_flag_hist_is_in_range(r1_obj09_a.radar1_obj09_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj09_a_radar1_obj09_mess_aconsist_bit_decode(r1_obj09_a.radar1_obj09_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj09_a_radar1_obj09_mess_aconsist_bit_is_in_range(
                  r1_obj09_a.radar1_obj09_mess_aconsist_bit);
              break;
            case 1385:
              xgu_radar1_obj10_a_t r1_obj10_a;
              target_info.unpack_return = xgu_radar1_obj10_a_unpack(&r1_obj10_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj10_a_radar1_obj10_dx_decode(r1_obj10_a.radar1_obj10_dx);
              dx_is_in_range = xgu_radar1_obj10_a_radar1_obj10_dx_is_in_range(r1_obj10_a.radar1_obj10_dx);
              vx_decoded = xgu_radar1_obj10_a_radar1_obj10_vx_decode(r1_obj10_a.radar1_obj10_vx);
              vx_is_in_range = xgu_radar1_obj10_a_radar1_obj10_vx_is_in_range(r1_obj10_a.radar1_obj10_vx);
              dy_decoded = xgu_radar1_obj10_a_radar1_obj10_dy_decode(r1_obj10_a.radar1_obj10_dy);
              dy_is_in_range = xgu_radar1_obj10_a_radar1_obj10_dy_is_in_range(r1_obj10_a.radar1_obj10_dy);
              w_exist_decoded = xgu_radar1_obj10_a_radar1_obj10_w_exist_decode(r1_obj10_a.radar1_obj10_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj10_a_radar1_obj10_w_exist_is_in_range(r1_obj10_a.radar1_obj10_w_exist);
              ax_decoded = xgu_radar1_obj10_a_radar1_obj10_ax_decode(r1_obj10_a.radar1_obj10_ax);
              ax_is_in_range = xgu_radar1_obj10_a_radar1_obj10_ax_is_in_range(r1_obj10_a.radar1_obj10_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj10_a_radar1_obj10_w_obstacle_decode(r1_obj10_a.radar1_obj10_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj10_a_radar1_obj10_w_obstacle_is_in_range(r1_obj10_a.radar1_obj10_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj10_a_radar1_obj10_flag_valid_decode(r1_obj10_a.radar1_obj10_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj10_a_radar1_obj10_flag_valid_is_in_range(r1_obj10_a.radar1_obj10_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj10_a_radar1_obj10_w_non_obstacle_decode(r1_obj10_a.radar1_obj10_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj10_a_radar1_obj10_w_non_obstacle_is_in_range(r1_obj10_a.radar1_obj10_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj10_a_radar1_obj10_flag_meas_decode(r1_obj10_a.radar1_obj10_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj10_a_radar1_obj10_flag_meas_is_in_range(r1_obj10_a.radar1_obj10_flag_meas);
              flag_hist_decoded = xgu_radar1_obj10_a_radar1_obj10_flag_hist_decode(r1_obj10_a.radar1_obj10_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj10_a_radar1_obj10_flag_hist_is_in_range(r1_obj10_a.radar1_obj10_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj10_a_radar1_obj10_mess_aconsist_bit_decode(r1_obj10_a.radar1_obj10_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj10_a_radar1_obj10_mess_aconsist_bit_is_in_range(
                  r1_obj10_a.radar1_obj10_mess_aconsist_bit);
              break;
            case 1395:
              xgu_radar1_obj11_a_t r1_obj11_a;
              target_info.unpack_return = xgu_radar1_obj11_a_unpack(&r1_obj11_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj11_a_radar1_obj11_dx_decode(r1_obj11_a.radar1_obj11_dx);
              dx_is_in_range = xgu_radar1_obj11_a_radar1_obj11_dx_is_in_range(r1_obj11_a.radar1_obj11_dx);
              vx_decoded = xgu_radar1_obj11_a_radar1_obj11_vx_decode(r1_obj11_a.radar1_obj11_vx);
              vx_is_in_range = xgu_radar1_obj11_a_radar1_obj11_vx_is_in_range(r1_obj11_a.radar1_obj11_vx);
              dy_decoded = xgu_radar1_obj11_a_radar1_obj11_dy_decode(r1_obj11_a.radar1_obj11_dy);
              dy_is_in_range = xgu_radar1_obj11_a_radar1_obj11_dy_is_in_range(r1_obj11_a.radar1_obj11_dy);
              w_exist_decoded = xgu_radar1_obj11_a_radar1_obj11_w_exist_decode(r1_obj11_a.radar1_obj11_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj11_a_radar1_obj11_w_exist_is_in_range(r1_obj11_a.radar1_obj11_w_exist);
              ax_decoded = xgu_radar1_obj11_a_radar1_obj11_ax_decode(r1_obj11_a.radar1_obj11_ax);
              ax_is_in_range = xgu_radar1_obj11_a_radar1_obj11_ax_is_in_range(r1_obj11_a.radar1_obj11_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj11_a_radar1_obj11_w_obstacle_decode(r1_obj11_a.radar1_obj11_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj11_a_radar1_obj11_w_obstacle_is_in_range(r1_obj11_a.radar1_obj11_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj11_a_radar1_obj11_flag_valid_decode(r1_obj11_a.radar1_obj11_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj11_a_radar1_obj11_flag_valid_is_in_range(r1_obj11_a.radar1_obj11_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj11_a_radar1_obj11_w_non_obstacle_decode(r1_obj11_a.radar1_obj11_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj11_a_radar1_obj11_w_non_obstacle_is_in_range(r1_obj11_a.radar1_obj11_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj11_a_radar1_obj11_flag_meas_decode(r1_obj11_a.radar1_obj11_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj11_a_radar1_obj11_flag_meas_is_in_range(r1_obj11_a.radar1_obj11_flag_meas);
              flag_hist_decoded = xgu_radar1_obj11_a_radar1_obj11_flag_hist_decode(r1_obj11_a.radar1_obj11_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj11_a_radar1_obj11_flag_hist_is_in_range(r1_obj11_a.radar1_obj11_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj11_a_radar1_obj11_mess_aconsist_bit_decode(r1_obj11_a.radar1_obj11_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj11_a_radar1_obj11_mess_aconsist_bit_is_in_range(
                  r1_obj11_a.radar1_obj11_mess_aconsist_bit);
              break;
            case 1405:
              xgu_radar1_obj12_a_t r1_obj12_a;
              target_info.unpack_return = xgu_radar1_obj12_a_unpack(&r1_obj12_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj12_a_radar1_obj12_dx_decode(r1_obj12_a.radar1_obj12_dx);
              dx_is_in_range = xgu_radar1_obj12_a_radar1_obj12_dx_is_in_range(r1_obj12_a.radar1_obj12_dx);
              vx_decoded = xgu_radar1_obj12_a_radar1_obj12_vx_decode(r1_obj12_a.radar1_obj12_vx);
              vx_is_in_range = xgu_radar1_obj12_a_radar1_obj12_vx_is_in_range(r1_obj12_a.radar1_obj12_vx);
              dy_decoded = xgu_radar1_obj12_a_radar1_obj12_dy_decode(r1_obj12_a.radar1_obj12_dy);
              dy_is_in_range = xgu_radar1_obj12_a_radar1_obj12_dy_is_in_range(r1_obj12_a.radar1_obj12_dy);
              w_exist_decoded = xgu_radar1_obj12_a_radar1_obj12_w_exist_decode(r1_obj12_a.radar1_obj12_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj12_a_radar1_obj12_w_exist_is_in_range(r1_obj12_a.radar1_obj12_w_exist);
              ax_decoded = xgu_radar1_obj12_a_radar1_obj12_ax_decode(r1_obj12_a.radar1_obj12_ax);
              ax_is_in_range = xgu_radar1_obj12_a_radar1_obj12_ax_is_in_range(r1_obj12_a.radar1_obj12_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj12_a_radar1_obj12_w_obstacle_decode(r1_obj12_a.radar1_obj12_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj12_a_radar1_obj12_w_obstacle_is_in_range(r1_obj12_a.radar1_obj12_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj12_a_radar1_obj12_flag_valid_decode(r1_obj12_a.radar1_obj12_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj12_a_radar1_obj12_flag_valid_is_in_range(r1_obj12_a.radar1_obj12_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj12_a_radar1_obj12_w_non_obstacle_decode(r1_obj12_a.radar1_obj12_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj12_a_radar1_obj12_w_non_obstacle_is_in_range(r1_obj12_a.radar1_obj12_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj12_a_radar1_obj12_flag_meas_decode(r1_obj12_a.radar1_obj12_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj12_a_radar1_obj12_flag_meas_is_in_range(r1_obj12_a.radar1_obj12_flag_meas);
              flag_hist_decoded = xgu_radar1_obj12_a_radar1_obj12_flag_hist_decode(r1_obj12_a.radar1_obj12_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj12_a_radar1_obj12_flag_hist_is_in_range(r1_obj12_a.radar1_obj12_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj12_a_radar1_obj12_mess_aconsist_bit_decode(r1_obj12_a.radar1_obj12_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj12_a_radar1_obj12_mess_aconsist_bit_is_in_range(
                  r1_obj12_a.radar1_obj12_mess_aconsist_bit);
              break;
            case 1415:
              xgu_radar1_obj13_a_t r1_obj13_a;
              target_info.unpack_return = xgu_radar1_obj13_a_unpack(&r1_obj13_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj13_a_radar1_obj13_dx_decode(r1_obj13_a.radar1_obj13_dx);
              dx_is_in_range = xgu_radar1_obj13_a_radar1_obj13_dx_is_in_range(r1_obj13_a.radar1_obj13_dx);
              vx_decoded = xgu_radar1_obj13_a_radar1_obj13_vx_decode(r1_obj13_a.radar1_obj13_vx);
              vx_is_in_range = xgu_radar1_obj13_a_radar1_obj13_vx_is_in_range(r1_obj13_a.radar1_obj13_vx);
              dy_decoded = xgu_radar1_obj13_a_radar1_obj13_dy_decode(r1_obj13_a.radar1_obj13_dy);
              dy_is_in_range = xgu_radar1_obj13_a_radar1_obj13_dy_is_in_range(r1_obj13_a.radar1_obj13_dy);
              w_exist_decoded = xgu_radar1_obj13_a_radar1_obj13_w_exist_decode(r1_obj13_a.radar1_obj13_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj13_a_radar1_obj13_w_exist_is_in_range(r1_obj13_a.radar1_obj13_w_exist);
              ax_decoded = xgu_radar1_obj13_a_radar1_obj13_ax_decode(r1_obj13_a.radar1_obj13_ax);
              ax_is_in_range = xgu_radar1_obj13_a_radar1_obj13_ax_is_in_range(r1_obj13_a.radar1_obj13_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj13_a_radar1_obj13_w_obstacle_decode(r1_obj13_a.radar1_obj13_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj13_a_radar1_obj13_w_obstacle_is_in_range(r1_obj13_a.radar1_obj13_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj13_a_radar1_obj13_flag_valid_decode(r1_obj13_a.radar1_obj13_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj13_a_radar1_obj13_flag_valid_is_in_range(r1_obj13_a.radar1_obj13_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj13_a_radar1_obj13_w_non_obstacle_decode(r1_obj13_a.radar1_obj13_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj13_a_radar1_obj13_w_non_obstacle_is_in_range(r1_obj13_a.radar1_obj13_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj13_a_radar1_obj13_flag_meas_decode(r1_obj13_a.radar1_obj13_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj13_a_radar1_obj13_flag_meas_is_in_range(r1_obj13_a.radar1_obj13_flag_meas);
              flag_hist_decoded = xgu_radar1_obj13_a_radar1_obj13_flag_hist_decode(r1_obj13_a.radar1_obj13_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj13_a_radar1_obj13_flag_hist_is_in_range(r1_obj13_a.radar1_obj13_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj13_a_radar1_obj13_mess_aconsist_bit_decode(r1_obj13_a.radar1_obj13_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj13_a_radar1_obj13_mess_aconsist_bit_is_in_range(
                  r1_obj13_a.radar1_obj13_mess_aconsist_bit);
              break;
            case 1425:
              xgu_radar1_obj14_a_t r1_obj14_a;
              target_info.unpack_return = xgu_radar1_obj14_a_unpack(&r1_obj14_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj14_a_radar1_obj14_dx_decode(r1_obj14_a.radar1_obj14_dx);
              dx_is_in_range = xgu_radar1_obj14_a_radar1_obj14_dx_is_in_range(r1_obj14_a.radar1_obj14_dx);
              vx_decoded = xgu_radar1_obj14_a_radar1_obj14_vx_decode(r1_obj14_a.radar1_obj14_vx);
              vx_is_in_range = xgu_radar1_obj14_a_radar1_obj14_vx_is_in_range(r1_obj14_a.radar1_obj14_vx);
              dy_decoded = xgu_radar1_obj14_a_radar1_obj14_dy_decode(r1_obj14_a.radar1_obj14_dy);
              dy_is_in_range = xgu_radar1_obj14_a_radar1_obj14_dy_is_in_range(r1_obj14_a.radar1_obj14_dy);
              w_exist_decoded = xgu_radar1_obj14_a_radar1_obj14_w_exist_decode(r1_obj14_a.radar1_obj14_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj14_a_radar1_obj14_w_exist_is_in_range(r1_obj14_a.radar1_obj14_w_exist);
              ax_decoded = xgu_radar1_obj14_a_radar1_obj14_ax_decode(r1_obj14_a.radar1_obj14_ax);
              ax_is_in_range = xgu_radar1_obj14_a_radar1_obj14_ax_is_in_range(r1_obj14_a.radar1_obj14_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj14_a_radar1_obj14_w_obstacle_decode(r1_obj14_a.radar1_obj14_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj14_a_radar1_obj14_w_obstacle_is_in_range(r1_obj14_a.radar1_obj14_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj14_a_radar1_obj14_flag_valid_decode(r1_obj14_a.radar1_obj14_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj14_a_radar1_obj14_flag_valid_is_in_range(r1_obj14_a.radar1_obj14_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj14_a_radar1_obj14_w_non_obstacle_decode(r1_obj14_a.radar1_obj14_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj14_a_radar1_obj14_w_non_obstacle_is_in_range(r1_obj14_a.radar1_obj14_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj14_a_radar1_obj14_flag_meas_decode(r1_obj14_a.radar1_obj14_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj14_a_radar1_obj14_flag_meas_is_in_range(r1_obj14_a.radar1_obj14_flag_meas);
              flag_hist_decoded = xgu_radar1_obj14_a_radar1_obj14_flag_hist_decode(r1_obj14_a.radar1_obj14_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj14_a_radar1_obj14_flag_hist_is_in_range(r1_obj14_a.radar1_obj14_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj14_a_radar1_obj14_mess_aconsist_bit_decode(r1_obj14_a.radar1_obj14_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj14_a_radar1_obj14_mess_aconsist_bit_is_in_range(
                  r1_obj14_a.radar1_obj14_mess_aconsist_bit);
              break;
            case 1435:
              xgu_radar1_obj15_a_t r1_obj15_a;
              target_info.unpack_return = xgu_radar1_obj15_a_unpack(&r1_obj15_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj15_a_radar1_obj15_dx_decode(r1_obj15_a.radar1_obj15_dx);
              dx_is_in_range = xgu_radar1_obj15_a_radar1_obj15_dx_is_in_range(r1_obj15_a.radar1_obj15_dx);
              vx_decoded = xgu_radar1_obj15_a_radar1_obj15_vx_decode(r1_obj15_a.radar1_obj15_vx);
              vx_is_in_range = xgu_radar1_obj15_a_radar1_obj15_vx_is_in_range(r1_obj15_a.radar1_obj15_vx);
              dy_decoded = xgu_radar1_obj15_a_radar1_obj15_dy_decode(r1_obj15_a.radar1_obj15_dy);
              dy_is_in_range = xgu_radar1_obj15_a_radar1_obj15_dy_is_in_range(r1_obj15_a.radar1_obj15_dy);
              w_exist_decoded = xgu_radar1_obj15_a_radar1_obj15_w_exist_decode(r1_obj15_a.radar1_obj15_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj15_a_radar1_obj15_w_exist_is_in_range(r1_obj15_a.radar1_obj15_w_exist);
              ax_decoded = xgu_radar1_obj15_a_radar1_obj15_ax_decode(r1_obj15_a.radar1_obj15_ax);
              ax_is_in_range = xgu_radar1_obj15_a_radar1_obj15_ax_is_in_range(r1_obj15_a.radar1_obj15_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj15_a_radar1_obj15_w_obstacle_decode(r1_obj15_a.radar1_obj15_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj15_a_radar1_obj15_w_obstacle_is_in_range(r1_obj15_a.radar1_obj15_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj15_a_radar1_obj15_flag_valid_decode(r1_obj15_a.radar1_obj15_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj15_a_radar1_obj15_flag_valid_is_in_range(r1_obj15_a.radar1_obj15_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj15_a_radar1_obj15_w_non_obstacle_decode(r1_obj15_a.radar1_obj15_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj15_a_radar1_obj15_w_non_obstacle_is_in_range(r1_obj15_a.radar1_obj15_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj15_a_radar1_obj15_flag_meas_decode(r1_obj15_a.radar1_obj15_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj15_a_radar1_obj15_flag_meas_is_in_range(r1_obj15_a.radar1_obj15_flag_meas);
              flag_hist_decoded = xgu_radar1_obj15_a_radar1_obj15_flag_hist_decode(r1_obj15_a.radar1_obj15_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj15_a_radar1_obj15_flag_hist_is_in_range(r1_obj15_a.radar1_obj15_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj15_a_radar1_obj15_mess_aconsist_bit_decode(r1_obj15_a.radar1_obj15_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj15_a_radar1_obj15_mess_aconsist_bit_is_in_range(
                  r1_obj15_a.radar1_obj15_mess_aconsist_bit);
              break;
            case 1445:
              xgu_radar1_obj16_a_t r1_obj16_a;
              target_info.unpack_return = xgu_radar1_obj16_a_unpack(&r1_obj16_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj16_a_radar1_obj16_dx_decode(r1_obj16_a.radar1_obj16_dx);
              dx_is_in_range = xgu_radar1_obj16_a_radar1_obj16_dx_is_in_range(r1_obj16_a.radar1_obj16_dx);
              vx_decoded = xgu_radar1_obj16_a_radar1_obj16_vx_decode(r1_obj16_a.radar1_obj16_vx);
              vx_is_in_range = xgu_radar1_obj16_a_radar1_obj16_vx_is_in_range(r1_obj16_a.radar1_obj16_vx);
              dy_decoded = xgu_radar1_obj16_a_radar1_obj16_dy_decode(r1_obj16_a.radar1_obj16_dy);
              dy_is_in_range = xgu_radar1_obj16_a_radar1_obj16_dy_is_in_range(r1_obj16_a.radar1_obj16_dy);
              w_exist_decoded = xgu_radar1_obj16_a_radar1_obj16_w_exist_decode(r1_obj16_a.radar1_obj16_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj16_a_radar1_obj16_w_exist_is_in_range(r1_obj16_a.radar1_obj16_w_exist);
              ax_decoded = xgu_radar1_obj16_a_radar1_obj16_ax_decode(r1_obj16_a.radar1_obj16_ax);
              ax_is_in_range = xgu_radar1_obj16_a_radar1_obj16_ax_is_in_range(r1_obj16_a.radar1_obj16_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj16_a_radar1_obj16_w_obstacle_decode(r1_obj16_a.radar1_obj16_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj16_a_radar1_obj16_w_obstacle_is_in_range(r1_obj16_a.radar1_obj16_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj16_a_radar1_obj16_flag_valid_decode(r1_obj16_a.radar1_obj16_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj16_a_radar1_obj16_flag_valid_is_in_range(r1_obj16_a.radar1_obj16_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj16_a_radar1_obj16_w_non_obstacle_decode(r1_obj16_a.radar1_obj16_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj16_a_radar1_obj16_w_non_obstacle_is_in_range(r1_obj16_a.radar1_obj16_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj16_a_radar1_obj16_flag_meas_decode(r1_obj16_a.radar1_obj16_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj16_a_radar1_obj16_flag_meas_is_in_range(r1_obj16_a.radar1_obj16_flag_meas);
              flag_hist_decoded = xgu_radar1_obj16_a_radar1_obj16_flag_hist_decode(r1_obj16_a.radar1_obj16_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj16_a_radar1_obj16_flag_hist_is_in_range(r1_obj16_a.radar1_obj16_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj16_a_radar1_obj16_mess_aconsist_bit_decode(r1_obj16_a.radar1_obj16_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj16_a_radar1_obj16_mess_aconsist_bit_is_in_range(
                  r1_obj16_a.radar1_obj16_mess_aconsist_bit);
              break;
            case 1455:
              xgu_radar1_obj17_a_t r1_obj17_a;
              target_info.unpack_return = xgu_radar1_obj17_a_unpack(&r1_obj17_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj17_a_radar1_obj17_dx_decode(r1_obj17_a.radar1_obj17_dx);
              dx_is_in_range = xgu_radar1_obj17_a_radar1_obj17_dx_is_in_range(r1_obj17_a.radar1_obj17_dx);
              vx_decoded = xgu_radar1_obj17_a_radar1_obj17_vx_decode(r1_obj17_a.radar1_obj17_vx);
              vx_is_in_range = xgu_radar1_obj17_a_radar1_obj17_vx_is_in_range(r1_obj17_a.radar1_obj17_vx);
              dy_decoded = xgu_radar1_obj17_a_radar1_obj17_dy_decode(r1_obj17_a.radar1_obj17_dy);
              dy_is_in_range = xgu_radar1_obj17_a_radar1_obj17_dy_is_in_range(r1_obj17_a.radar1_obj17_dy);
              w_exist_decoded = xgu_radar1_obj17_a_radar1_obj17_w_exist_decode(r1_obj17_a.radar1_obj17_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj17_a_radar1_obj17_w_exist_is_in_range(r1_obj17_a.radar1_obj17_w_exist);
              ax_decoded = xgu_radar1_obj17_a_radar1_obj17_ax_decode(r1_obj17_a.radar1_obj17_ax);
              ax_is_in_range = xgu_radar1_obj17_a_radar1_obj17_ax_is_in_range(r1_obj17_a.radar1_obj17_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj17_a_radar1_obj17_w_obstacle_decode(r1_obj17_a.radar1_obj17_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj17_a_radar1_obj17_w_obstacle_is_in_range(r1_obj17_a.radar1_obj17_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj17_a_radar1_obj17_flag_valid_decode(r1_obj17_a.radar1_obj17_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj17_a_radar1_obj17_flag_valid_is_in_range(r1_obj17_a.radar1_obj17_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj17_a_radar1_obj17_w_non_obstacle_decode(r1_obj17_a.radar1_obj17_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj17_a_radar1_obj17_w_non_obstacle_is_in_range(r1_obj17_a.radar1_obj17_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj17_a_radar1_obj17_flag_meas_decode(r1_obj17_a.radar1_obj17_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj17_a_radar1_obj17_flag_meas_is_in_range(r1_obj17_a.radar1_obj17_flag_meas);
              flag_hist_decoded = xgu_radar1_obj17_a_radar1_obj17_flag_hist_decode(r1_obj17_a.radar1_obj17_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj17_a_radar1_obj17_flag_hist_is_in_range(r1_obj17_a.radar1_obj17_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj17_a_radar1_obj17_mess_aconsist_bit_decode(r1_obj17_a.radar1_obj17_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj17_a_radar1_obj17_mess_aconsist_bit_is_in_range(
                  r1_obj17_a.radar1_obj17_mess_aconsist_bit);
              break;
            case 1465:
              xgu_radar1_obj18_a_t r1_obj18_a;
              target_info.unpack_return = xgu_radar1_obj18_a_unpack(&r1_obj18_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj18_a_radar1_obj18_dx_decode(r1_obj18_a.radar1_obj18_dx);
              dx_is_in_range = xgu_radar1_obj18_a_radar1_obj18_dx_is_in_range(r1_obj18_a.radar1_obj18_dx);
              vx_decoded = xgu_radar1_obj18_a_radar1_obj18_vx_decode(r1_obj18_a.radar1_obj18_vx);
              vx_is_in_range = xgu_radar1_obj18_a_radar1_obj18_vx_is_in_range(r1_obj18_a.radar1_obj18_vx);
              dy_decoded = xgu_radar1_obj18_a_radar1_obj18_dy_decode(r1_obj18_a.radar1_obj18_dy);
              dy_is_in_range = xgu_radar1_obj18_a_radar1_obj18_dy_is_in_range(r1_obj18_a.radar1_obj18_dy);
              w_exist_decoded = xgu_radar1_obj18_a_radar1_obj18_w_exist_decode(r1_obj18_a.radar1_obj18_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj18_a_radar1_obj18_w_exist_is_in_range(r1_obj18_a.radar1_obj18_w_exist);
              ax_decoded = xgu_radar1_obj18_a_radar1_obj18_ax_decode(r1_obj18_a.radar1_obj18_ax);
              ax_is_in_range = xgu_radar1_obj18_a_radar1_obj18_ax_is_in_range(r1_obj18_a.radar1_obj18_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj18_a_radar1_obj18_w_obstacle_decode(r1_obj18_a.radar1_obj18_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj18_a_radar1_obj18_w_obstacle_is_in_range(r1_obj18_a.radar1_obj18_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj18_a_radar1_obj18_flag_valid_decode(r1_obj18_a.radar1_obj18_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj18_a_radar1_obj18_flag_valid_is_in_range(r1_obj18_a.radar1_obj18_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj18_a_radar1_obj18_w_non_obstacle_decode(r1_obj18_a.radar1_obj18_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj18_a_radar1_obj18_w_non_obstacle_is_in_range(r1_obj18_a.radar1_obj18_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj18_a_radar1_obj18_flag_meas_decode(r1_obj18_a.radar1_obj18_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj18_a_radar1_obj18_flag_meas_is_in_range(r1_obj18_a.radar1_obj18_flag_meas);
              flag_hist_decoded = xgu_radar1_obj18_a_radar1_obj18_flag_hist_decode(r1_obj18_a.radar1_obj18_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj18_a_radar1_obj18_flag_hist_is_in_range(r1_obj18_a.radar1_obj18_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj18_a_radar1_obj18_mess_aconsist_bit_decode(r1_obj18_a.radar1_obj18_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj18_a_radar1_obj18_mess_aconsist_bit_is_in_range(
                  r1_obj18_a.radar1_obj18_mess_aconsist_bit);
              break;
            case 1475:
              xgu_radar1_obj19_a_t r1_obj19_a;
              target_info.unpack_return = xgu_radar1_obj19_a_unpack(&r1_obj19_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj19_a_radar1_obj19_dx_decode(r1_obj19_a.radar1_obj19_dx);
              dx_is_in_range = xgu_radar1_obj19_a_radar1_obj19_dx_is_in_range(r1_obj19_a.radar1_obj19_dx);
              vx_decoded = xgu_radar1_obj19_a_radar1_obj19_vx_decode(r1_obj19_a.radar1_obj19_vx);
              vx_is_in_range = xgu_radar1_obj19_a_radar1_obj19_vx_is_in_range(r1_obj19_a.radar1_obj19_vx);
              dy_decoded = xgu_radar1_obj19_a_radar1_obj19_dy_decode(r1_obj19_a.radar1_obj19_dy);
              dy_is_in_range = xgu_radar1_obj19_a_radar1_obj19_dy_is_in_range(r1_obj19_a.radar1_obj19_dy);
              w_exist_decoded = xgu_radar1_obj19_a_radar1_obj19_w_exist_decode(r1_obj19_a.radar1_obj19_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj19_a_radar1_obj19_w_exist_is_in_range(r1_obj19_a.radar1_obj19_w_exist);
              ax_decoded = xgu_radar1_obj19_a_radar1_obj19_ax_decode(r1_obj19_a.radar1_obj19_ax);
              ax_is_in_range = xgu_radar1_obj19_a_radar1_obj19_ax_is_in_range(r1_obj19_a.radar1_obj19_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj19_a_radar1_obj19_w_obstacle_decode(r1_obj19_a.radar1_obj19_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj19_a_radar1_obj19_w_obstacle_is_in_range(r1_obj19_a.radar1_obj19_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj19_a_radar1_obj19_flag_valid_decode(r1_obj19_a.radar1_obj19_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj19_a_radar1_obj19_flag_valid_is_in_range(r1_obj19_a.radar1_obj19_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj19_a_radar1_obj19_w_non_obstacle_decode(r1_obj19_a.radar1_obj19_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj19_a_radar1_obj19_w_non_obstacle_is_in_range(r1_obj19_a.radar1_obj19_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj19_a_radar1_obj19_flag_meas_decode(r1_obj19_a.radar1_obj19_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj19_a_radar1_obj19_flag_meas_is_in_range(r1_obj19_a.radar1_obj19_flag_meas);
              flag_hist_decoded = xgu_radar1_obj19_a_radar1_obj19_flag_hist_decode(r1_obj19_a.radar1_obj19_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj19_a_radar1_obj19_flag_hist_is_in_range(r1_obj19_a.radar1_obj19_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj19_a_radar1_obj19_mess_aconsist_bit_decode(r1_obj19_a.radar1_obj19_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj19_a_radar1_obj19_mess_aconsist_bit_is_in_range(
                  r1_obj19_a.radar1_obj19_mess_aconsist_bit);
              break;
            case 1485:
              xgu_radar1_obj20_a_t r1_obj20_a;
              target_info.unpack_return = xgu_radar1_obj20_a_unpack(&r1_obj20_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj20_a_radar1_obj20_dx_decode(r1_obj20_a.radar1_obj20_dx);
              dx_is_in_range = xgu_radar1_obj20_a_radar1_obj20_dx_is_in_range(r1_obj20_a.radar1_obj20_dx);
              vx_decoded = xgu_radar1_obj20_a_radar1_obj20_vx_decode(r1_obj20_a.radar1_obj20_vx);
              vx_is_in_range = xgu_radar1_obj20_a_radar1_obj20_vx_is_in_range(r1_obj20_a.radar1_obj20_vx);
              dy_decoded = xgu_radar1_obj20_a_radar1_obj20_dy_decode(r1_obj20_a.radar1_obj20_dy);
              dy_is_in_range = xgu_radar1_obj20_a_radar1_obj20_dy_is_in_range(r1_obj20_a.radar1_obj20_dy);
              w_exist_decoded = xgu_radar1_obj20_a_radar1_obj20_w_exist_decode(r1_obj20_a.radar1_obj20_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj20_a_radar1_obj20_w_exist_is_in_range(r1_obj20_a.radar1_obj20_w_exist);
              ax_decoded = xgu_radar1_obj20_a_radar1_obj20_ax_decode(r1_obj20_a.radar1_obj20_ax);
              ax_is_in_range = xgu_radar1_obj20_a_radar1_obj20_ax_is_in_range(r1_obj20_a.radar1_obj20_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj20_a_radar1_obj20_w_obstacle_decode(r1_obj20_a.radar1_obj20_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj20_a_radar1_obj20_w_obstacle_is_in_range(r1_obj20_a.radar1_obj20_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj20_a_radar1_obj20_flag_valid_decode(r1_obj20_a.radar1_obj20_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj20_a_radar1_obj20_flag_valid_is_in_range(r1_obj20_a.radar1_obj20_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj20_a_radar1_obj20_w_non_obstacle_decode(r1_obj20_a.radar1_obj20_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj20_a_radar1_obj20_w_non_obstacle_is_in_range(r1_obj20_a.radar1_obj20_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj20_a_radar1_obj20_flag_meas_decode(r1_obj20_a.radar1_obj20_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj20_a_radar1_obj20_flag_meas_is_in_range(r1_obj20_a.radar1_obj20_flag_meas);
              flag_hist_decoded = xgu_radar1_obj20_a_radar1_obj20_flag_hist_decode(r1_obj20_a.radar1_obj20_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj20_a_radar1_obj20_flag_hist_is_in_range(r1_obj20_a.radar1_obj20_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj20_a_radar1_obj20_mess_aconsist_bit_decode(r1_obj20_a.radar1_obj20_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj20_a_radar1_obj20_mess_aconsist_bit_is_in_range(
                  r1_obj20_a.radar1_obj20_mess_aconsist_bit);
              break;
            case 1495:
              xgu_radar1_obj21_a_t r1_obj21_a;
              target_info.unpack_return = xgu_radar1_obj21_a_unpack(&r1_obj21_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj21_a_radar1_obj21_dx_decode(r1_obj21_a.radar1_obj21_dx);
              dx_is_in_range = xgu_radar1_obj21_a_radar1_obj21_dx_is_in_range(r1_obj21_a.radar1_obj21_dx);
              vx_decoded = xgu_radar1_obj21_a_radar1_obj21_vx_decode(r1_obj21_a.radar1_obj21_vx);
              vx_is_in_range = xgu_radar1_obj21_a_radar1_obj21_vx_is_in_range(r1_obj21_a.radar1_obj21_vx);
              dy_decoded = xgu_radar1_obj21_a_radar1_obj21_dy_decode(r1_obj21_a.radar1_obj21_dy);
              dy_is_in_range = xgu_radar1_obj21_a_radar1_obj21_dy_is_in_range(r1_obj21_a.radar1_obj21_dy);
              w_exist_decoded = xgu_radar1_obj21_a_radar1_obj21_w_exist_decode(r1_obj21_a.radar1_obj21_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj21_a_radar1_obj21_w_exist_is_in_range(r1_obj21_a.radar1_obj21_w_exist);
              ax_decoded = xgu_radar1_obj21_a_radar1_obj21_ax_decode(r1_obj21_a.radar1_obj21_ax);
              ax_is_in_range = xgu_radar1_obj21_a_radar1_obj21_ax_is_in_range(r1_obj21_a.radar1_obj21_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj21_a_radar1_obj21_w_obstacle_decode(r1_obj21_a.radar1_obj21_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj21_a_radar1_obj21_w_obstacle_is_in_range(r1_obj21_a.radar1_obj21_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj21_a_radar1_obj21_flag_valid_decode(r1_obj21_a.radar1_obj21_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj21_a_radar1_obj21_flag_valid_is_in_range(r1_obj21_a.radar1_obj21_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj21_a_radar1_obj21_w_non_obstacle_decode(r1_obj21_a.radar1_obj21_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj21_a_radar1_obj21_w_non_obstacle_is_in_range(r1_obj21_a.radar1_obj21_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj21_a_radar1_obj21_flag_meas_decode(r1_obj21_a.radar1_obj21_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj21_a_radar1_obj21_flag_meas_is_in_range(r1_obj21_a.radar1_obj21_flag_meas);
              flag_hist_decoded = xgu_radar1_obj21_a_radar1_obj21_flag_hist_decode(r1_obj21_a.radar1_obj21_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj21_a_radar1_obj21_flag_hist_is_in_range(r1_obj21_a.radar1_obj21_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj21_a_radar1_obj21_mess_aconsist_bit_decode(r1_obj21_a.radar1_obj21_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj21_a_radar1_obj21_mess_aconsist_bit_is_in_range(
                  r1_obj21_a.radar1_obj21_mess_aconsist_bit);
              break;
            case 1505:
              xgu_radar1_obj22_a_t r1_obj22_a;
              target_info.unpack_return = xgu_radar1_obj22_a_unpack(&r1_obj22_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj22_a_radar1_obj22_dx_decode(r1_obj22_a.radar1_obj22_dx);
              dx_is_in_range = xgu_radar1_obj22_a_radar1_obj22_dx_is_in_range(r1_obj22_a.radar1_obj22_dx);
              vx_decoded = xgu_radar1_obj22_a_radar1_obj22_vx_decode(r1_obj22_a.radar1_obj22_vx);
              vx_is_in_range = xgu_radar1_obj22_a_radar1_obj22_vx_is_in_range(r1_obj22_a.radar1_obj22_vx);
              dy_decoded = xgu_radar1_obj22_a_radar1_obj22_dy_decode(r1_obj22_a.radar1_obj22_dy);
              dy_is_in_range = xgu_radar1_obj22_a_radar1_obj22_dy_is_in_range(r1_obj22_a.radar1_obj22_dy);
              w_exist_decoded = xgu_radar1_obj22_a_radar1_obj22_w_exist_decode(r1_obj22_a.radar1_obj22_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj22_a_radar1_obj22_w_exist_is_in_range(r1_obj22_a.radar1_obj22_w_exist);
              ax_decoded = xgu_radar1_obj22_a_radar1_obj22_ax_decode(r1_obj22_a.radar1_obj22_ax);
              ax_is_in_range = xgu_radar1_obj22_a_radar1_obj22_ax_is_in_range(r1_obj22_a.radar1_obj22_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj22_a_radar1_obj22_w_obstacle_decode(r1_obj22_a.radar1_obj22_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj22_a_radar1_obj22_w_obstacle_is_in_range(r1_obj22_a.radar1_obj22_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj22_a_radar1_obj22_flag_valid_decode(r1_obj22_a.radar1_obj22_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj22_a_radar1_obj22_flag_valid_is_in_range(r1_obj22_a.radar1_obj22_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj22_a_radar1_obj22_w_non_obstacle_decode(r1_obj22_a.radar1_obj22_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj22_a_radar1_obj22_w_non_obstacle_is_in_range(r1_obj22_a.radar1_obj22_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj22_a_radar1_obj22_flag_meas_decode(r1_obj22_a.radar1_obj22_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj22_a_radar1_obj22_flag_meas_is_in_range(r1_obj22_a.radar1_obj22_flag_meas);
              flag_hist_decoded = xgu_radar1_obj22_a_radar1_obj22_flag_hist_decode(r1_obj22_a.radar1_obj22_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj22_a_radar1_obj22_flag_hist_is_in_range(r1_obj22_a.radar1_obj22_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj22_a_radar1_obj22_mess_aconsist_bit_decode(r1_obj22_a.radar1_obj22_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj22_a_radar1_obj22_mess_aconsist_bit_is_in_range(
                  r1_obj22_a.radar1_obj22_mess_aconsist_bit);
              break;
            case 1515:
              xgu_radar1_obj23_a_t r1_obj23_a;
              target_info.unpack_return = xgu_radar1_obj23_a_unpack(&r1_obj23_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj23_a_radar1_obj23_dx_decode(r1_obj23_a.radar1_obj23_dx);
              dx_is_in_range = xgu_radar1_obj23_a_radar1_obj23_dx_is_in_range(r1_obj23_a.radar1_obj23_dx);
              vx_decoded = xgu_radar1_obj23_a_radar1_obj23_vx_decode(r1_obj23_a.radar1_obj23_vx);
              vx_is_in_range = xgu_radar1_obj23_a_radar1_obj23_vx_is_in_range(r1_obj23_a.radar1_obj23_vx);
              dy_decoded = xgu_radar1_obj23_a_radar1_obj23_dy_decode(r1_obj23_a.radar1_obj23_dy);
              dy_is_in_range = xgu_radar1_obj23_a_radar1_obj23_dy_is_in_range(r1_obj23_a.radar1_obj23_dy);
              w_exist_decoded = xgu_radar1_obj23_a_radar1_obj23_w_exist_decode(r1_obj23_a.radar1_obj23_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj23_a_radar1_obj23_w_exist_is_in_range(r1_obj23_a.radar1_obj23_w_exist);
              ax_decoded = xgu_radar1_obj23_a_radar1_obj23_ax_decode(r1_obj23_a.radar1_obj23_ax);
              ax_is_in_range = xgu_radar1_obj23_a_radar1_obj23_ax_is_in_range(r1_obj23_a.radar1_obj23_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj23_a_radar1_obj23_w_obstacle_decode(r1_obj23_a.radar1_obj23_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj23_a_radar1_obj23_w_obstacle_is_in_range(r1_obj23_a.radar1_obj23_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj23_a_radar1_obj23_flag_valid_decode(r1_obj23_a.radar1_obj23_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj23_a_radar1_obj23_flag_valid_is_in_range(r1_obj23_a.radar1_obj23_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj23_a_radar1_obj23_w_non_obstacle_decode(r1_obj23_a.radar1_obj23_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj23_a_radar1_obj23_w_non_obstacle_is_in_range(r1_obj23_a.radar1_obj23_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj23_a_radar1_obj23_flag_meas_decode(r1_obj23_a.radar1_obj23_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj23_a_radar1_obj23_flag_meas_is_in_range(r1_obj23_a.radar1_obj23_flag_meas);
              flag_hist_decoded = xgu_radar1_obj23_a_radar1_obj23_flag_hist_decode(r1_obj23_a.radar1_obj23_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj23_a_radar1_obj23_flag_hist_is_in_range(r1_obj23_a.radar1_obj23_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj23_a_radar1_obj23_mess_aconsist_bit_decode(r1_obj23_a.radar1_obj23_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj23_a_radar1_obj23_mess_aconsist_bit_is_in_range(
                  r1_obj23_a.radar1_obj23_mess_aconsist_bit);
              break;
            case 1525:
              xgu_radar1_obj24_a_t r1_obj24_a;
              target_info.unpack_return = xgu_radar1_obj24_a_unpack(&r1_obj24_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj24_a_radar1_obj24_dx_decode(r1_obj24_a.radar1_obj24_dx);
              dx_is_in_range = xgu_radar1_obj24_a_radar1_obj24_dx_is_in_range(r1_obj24_a.radar1_obj24_dx);
              vx_decoded = xgu_radar1_obj24_a_radar1_obj24_vx_decode(r1_obj24_a.radar1_obj24_vx);
              vx_is_in_range = xgu_radar1_obj24_a_radar1_obj24_vx_is_in_range(r1_obj24_a.radar1_obj24_vx);
              dy_decoded = xgu_radar1_obj24_a_radar1_obj24_dy_decode(r1_obj24_a.radar1_obj24_dy);
              dy_is_in_range = xgu_radar1_obj24_a_radar1_obj24_dy_is_in_range(r1_obj24_a.radar1_obj24_dy);
              w_exist_decoded = xgu_radar1_obj24_a_radar1_obj24_w_exist_decode(r1_obj24_a.radar1_obj24_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj24_a_radar1_obj24_w_exist_is_in_range(r1_obj24_a.radar1_obj24_w_exist);
              ax_decoded = xgu_radar1_obj24_a_radar1_obj24_ax_decode(r1_obj24_a.radar1_obj24_ax);
              ax_is_in_range = xgu_radar1_obj24_a_radar1_obj24_ax_is_in_range(r1_obj24_a.radar1_obj24_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj24_a_radar1_obj24_w_obstacle_decode(r1_obj24_a.radar1_obj24_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj24_a_radar1_obj24_w_obstacle_is_in_range(r1_obj24_a.radar1_obj24_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj24_a_radar1_obj24_flag_valid_decode(r1_obj24_a.radar1_obj24_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj24_a_radar1_obj24_flag_valid_is_in_range(r1_obj24_a.radar1_obj24_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj24_a_radar1_obj24_w_non_obstacle_decode(r1_obj24_a.radar1_obj24_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj24_a_radar1_obj24_w_non_obstacle_is_in_range(r1_obj24_a.radar1_obj24_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj24_a_radar1_obj24_flag_meas_decode(r1_obj24_a.radar1_obj24_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj24_a_radar1_obj24_flag_meas_is_in_range(r1_obj24_a.radar1_obj24_flag_meas);
              flag_hist_decoded = xgu_radar1_obj24_a_radar1_obj24_flag_hist_decode(r1_obj24_a.radar1_obj24_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj24_a_radar1_obj24_flag_hist_is_in_range(r1_obj24_a.radar1_obj24_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj24_a_radar1_obj24_mess_aconsist_bit_decode(r1_obj24_a.radar1_obj24_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj24_a_radar1_obj24_mess_aconsist_bit_is_in_range(
                  r1_obj24_a.radar1_obj24_mess_aconsist_bit);
              break;
            case 1535:
              xgu_radar1_obj25_a_t r1_obj25_a;
              target_info.unpack_return = xgu_radar1_obj25_a_unpack(&r1_obj25_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj25_a_radar1_obj25_dx_decode(r1_obj25_a.radar1_obj25_dx);
              dx_is_in_range = xgu_radar1_obj25_a_radar1_obj25_dx_is_in_range(r1_obj25_a.radar1_obj25_dx);
              vx_decoded = xgu_radar1_obj25_a_radar1_obj25_vx_decode(r1_obj25_a.radar1_obj25_vx);
              vx_is_in_range = xgu_radar1_obj25_a_radar1_obj25_vx_is_in_range(r1_obj25_a.radar1_obj25_vx);
              dy_decoded = xgu_radar1_obj25_a_radar1_obj25_dy_decode(r1_obj25_a.radar1_obj25_dy);
              dy_is_in_range = xgu_radar1_obj25_a_radar1_obj25_dy_is_in_range(r1_obj25_a.radar1_obj25_dy);
              w_exist_decoded = xgu_radar1_obj25_a_radar1_obj25_w_exist_decode(r1_obj25_a.radar1_obj25_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj25_a_radar1_obj25_w_exist_is_in_range(r1_obj25_a.radar1_obj25_w_exist);
              ax_decoded = xgu_radar1_obj25_a_radar1_obj25_ax_decode(r1_obj25_a.radar1_obj25_ax);
              ax_is_in_range = xgu_radar1_obj25_a_radar1_obj25_ax_is_in_range(r1_obj25_a.radar1_obj25_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj25_a_radar1_obj25_w_obstacle_decode(r1_obj25_a.radar1_obj25_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj25_a_radar1_obj25_w_obstacle_is_in_range(r1_obj25_a.radar1_obj25_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj25_a_radar1_obj25_flag_valid_decode(r1_obj25_a.radar1_obj25_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj25_a_radar1_obj25_flag_valid_is_in_range(r1_obj25_a.radar1_obj25_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj25_a_radar1_obj25_w_non_obstacle_decode(r1_obj25_a.radar1_obj25_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj25_a_radar1_obj25_w_non_obstacle_is_in_range(r1_obj25_a.radar1_obj25_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj25_a_radar1_obj25_flag_meas_decode(r1_obj25_a.radar1_obj25_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj25_a_radar1_obj25_flag_meas_is_in_range(r1_obj25_a.radar1_obj25_flag_meas);
              flag_hist_decoded = xgu_radar1_obj25_a_radar1_obj25_flag_hist_decode(r1_obj25_a.radar1_obj25_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj25_a_radar1_obj25_flag_hist_is_in_range(r1_obj25_a.radar1_obj25_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj25_a_radar1_obj25_mess_aconsist_bit_decode(r1_obj25_a.radar1_obj25_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj25_a_radar1_obj25_mess_aconsist_bit_is_in_range(
                  r1_obj25_a.radar1_obj25_mess_aconsist_bit);
              break;
            case 1545:
              xgu_radar1_obj26_a_t r1_obj26_a;
              target_info.unpack_return = xgu_radar1_obj26_a_unpack(&r1_obj26_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj26_a_radar1_obj26_dx_decode(r1_obj26_a.radar1_obj26_dx);
              dx_is_in_range = xgu_radar1_obj26_a_radar1_obj26_dx_is_in_range(r1_obj26_a.radar1_obj26_dx);
              vx_decoded = xgu_radar1_obj26_a_radar1_obj26_vx_decode(r1_obj26_a.radar1_obj26_vx);
              vx_is_in_range = xgu_radar1_obj26_a_radar1_obj26_vx_is_in_range(r1_obj26_a.radar1_obj26_vx);
              dy_decoded = xgu_radar1_obj26_a_radar1_obj26_dy_decode(r1_obj26_a.radar1_obj26_dy);
              dy_is_in_range = xgu_radar1_obj26_a_radar1_obj26_dy_is_in_range(r1_obj26_a.radar1_obj26_dy);
              w_exist_decoded = xgu_radar1_obj26_a_radar1_obj26_w_exist_decode(r1_obj26_a.radar1_obj26_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj26_a_radar1_obj26_w_exist_is_in_range(r1_obj26_a.radar1_obj26_w_exist);
              ax_decoded = xgu_radar1_obj26_a_radar1_obj26_ax_decode(r1_obj26_a.radar1_obj26_ax);
              ax_is_in_range = xgu_radar1_obj26_a_radar1_obj26_ax_is_in_range(r1_obj26_a.radar1_obj26_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj26_a_radar1_obj26_w_obstacle_decode(r1_obj26_a.radar1_obj26_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj26_a_radar1_obj26_w_obstacle_is_in_range(r1_obj26_a.radar1_obj26_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj26_a_radar1_obj26_flag_valid_decode(r1_obj26_a.radar1_obj26_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj26_a_radar1_obj26_flag_valid_is_in_range(r1_obj26_a.radar1_obj26_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj26_a_radar1_obj26_w_non_obstacle_decode(r1_obj26_a.radar1_obj26_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj26_a_radar1_obj26_w_non_obstacle_is_in_range(r1_obj26_a.radar1_obj26_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj26_a_radar1_obj26_flag_meas_decode(r1_obj26_a.radar1_obj26_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj26_a_radar1_obj26_flag_meas_is_in_range(r1_obj26_a.radar1_obj26_flag_meas);
              flag_hist_decoded = xgu_radar1_obj26_a_radar1_obj26_flag_hist_decode(r1_obj26_a.radar1_obj26_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj26_a_radar1_obj26_flag_hist_is_in_range(r1_obj26_a.radar1_obj26_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj26_a_radar1_obj26_mess_aconsist_bit_decode(r1_obj26_a.radar1_obj26_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj26_a_radar1_obj26_mess_aconsist_bit_is_in_range(
                  r1_obj26_a.radar1_obj26_mess_aconsist_bit);
              break;
            case 1555:
              xgu_radar1_obj27_a_t r1_obj27_a;
              target_info.unpack_return = xgu_radar1_obj27_a_unpack(&r1_obj27_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj27_a_radar1_obj27_dx_decode(r1_obj27_a.radar1_obj27_dx);
              dx_is_in_range = xgu_radar1_obj27_a_radar1_obj27_dx_is_in_range(r1_obj27_a.radar1_obj27_dx);
              vx_decoded = xgu_radar1_obj27_a_radar1_obj27_vx_decode(r1_obj27_a.radar1_obj27_vx);
              vx_is_in_range = xgu_radar1_obj27_a_radar1_obj27_vx_is_in_range(r1_obj27_a.radar1_obj27_vx);
              dy_decoded = xgu_radar1_obj27_a_radar1_obj27_dy_decode(r1_obj27_a.radar1_obj27_dy);
              dy_is_in_range = xgu_radar1_obj27_a_radar1_obj27_dy_is_in_range(r1_obj27_a.radar1_obj27_dy);
              w_exist_decoded = xgu_radar1_obj27_a_radar1_obj27_w_exist_decode(r1_obj27_a.radar1_obj27_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj27_a_radar1_obj27_w_exist_is_in_range(r1_obj27_a.radar1_obj27_w_exist);
              ax_decoded = xgu_radar1_obj27_a_radar1_obj27_ax_decode(r1_obj27_a.radar1_obj27_ax);
              ax_is_in_range = xgu_radar1_obj27_a_radar1_obj27_ax_is_in_range(r1_obj27_a.radar1_obj27_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj27_a_radar1_obj27_w_obstacle_decode(r1_obj27_a.radar1_obj27_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj27_a_radar1_obj27_w_obstacle_is_in_range(r1_obj27_a.radar1_obj27_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj27_a_radar1_obj27_flag_valid_decode(r1_obj27_a.radar1_obj27_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj27_a_radar1_obj27_flag_valid_is_in_range(r1_obj27_a.radar1_obj27_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj27_a_radar1_obj27_w_non_obstacle_decode(r1_obj27_a.radar1_obj27_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj27_a_radar1_obj27_w_non_obstacle_is_in_range(r1_obj27_a.radar1_obj27_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj27_a_radar1_obj27_flag_meas_decode(r1_obj27_a.radar1_obj27_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj27_a_radar1_obj27_flag_meas_is_in_range(r1_obj27_a.radar1_obj27_flag_meas);
              flag_hist_decoded = xgu_radar1_obj27_a_radar1_obj27_flag_hist_decode(r1_obj27_a.radar1_obj27_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj27_a_radar1_obj27_flag_hist_is_in_range(r1_obj27_a.radar1_obj27_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj27_a_radar1_obj27_mess_aconsist_bit_decode(r1_obj27_a.radar1_obj27_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj27_a_radar1_obj27_mess_aconsist_bit_is_in_range(
                  r1_obj27_a.radar1_obj27_mess_aconsist_bit);
              break;
            case 1565:
              xgu_radar1_obj28_a_t r1_obj28_a;
              target_info.unpack_return = xgu_radar1_obj28_a_unpack(&r1_obj28_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj28_a_radar1_obj28_dx_decode(r1_obj28_a.radar1_obj28_dx);
              dx_is_in_range = xgu_radar1_obj28_a_radar1_obj28_dx_is_in_range(r1_obj28_a.radar1_obj28_dx);
              vx_decoded = xgu_radar1_obj28_a_radar1_obj28_vx_decode(r1_obj28_a.radar1_obj28_vx);
              vx_is_in_range = xgu_radar1_obj28_a_radar1_obj28_vx_is_in_range(r1_obj28_a.radar1_obj28_vx);
              dy_decoded = xgu_radar1_obj28_a_radar1_obj28_dy_decode(r1_obj28_a.radar1_obj28_dy);
              dy_is_in_range = xgu_radar1_obj28_a_radar1_obj28_dy_is_in_range(r1_obj28_a.radar1_obj28_dy);
              w_exist_decoded = xgu_radar1_obj28_a_radar1_obj28_w_exist_decode(r1_obj28_a.radar1_obj28_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj28_a_radar1_obj28_w_exist_is_in_range(r1_obj28_a.radar1_obj28_w_exist);
              ax_decoded = xgu_radar1_obj28_a_radar1_obj28_ax_decode(r1_obj28_a.radar1_obj28_ax);
              ax_is_in_range = xgu_radar1_obj28_a_radar1_obj28_ax_is_in_range(r1_obj28_a.radar1_obj28_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj28_a_radar1_obj28_w_obstacle_decode(r1_obj28_a.radar1_obj28_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj28_a_radar1_obj28_w_obstacle_is_in_range(r1_obj28_a.radar1_obj28_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj28_a_radar1_obj28_flag_valid_decode(r1_obj28_a.radar1_obj28_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj28_a_radar1_obj28_flag_valid_is_in_range(r1_obj28_a.radar1_obj28_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj28_a_radar1_obj28_w_non_obstacle_decode(r1_obj28_a.radar1_obj28_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj28_a_radar1_obj28_w_non_obstacle_is_in_range(r1_obj28_a.radar1_obj28_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj28_a_radar1_obj28_flag_meas_decode(r1_obj28_a.radar1_obj28_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj28_a_radar1_obj28_flag_meas_is_in_range(r1_obj28_a.radar1_obj28_flag_meas);
              flag_hist_decoded = xgu_radar1_obj28_a_radar1_obj28_flag_hist_decode(r1_obj28_a.radar1_obj28_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj28_a_radar1_obj28_flag_hist_is_in_range(r1_obj28_a.radar1_obj28_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj28_a_radar1_obj28_mess_aconsist_bit_decode(r1_obj28_a.radar1_obj28_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj28_a_radar1_obj28_mess_aconsist_bit_is_in_range(
                  r1_obj28_a.radar1_obj28_mess_aconsist_bit);
              break;
            case 1575:
              xgu_radar1_obj29_a_t r1_obj29_a;
              target_info.unpack_return = xgu_radar1_obj29_a_unpack(&r1_obj29_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj29_a_radar1_obj29_dx_decode(r1_obj29_a.radar1_obj29_dx);
              dx_is_in_range = xgu_radar1_obj29_a_radar1_obj29_dx_is_in_range(r1_obj29_a.radar1_obj29_dx);
              vx_decoded = xgu_radar1_obj29_a_radar1_obj29_vx_decode(r1_obj29_a.radar1_obj29_vx);
              vx_is_in_range = xgu_radar1_obj29_a_radar1_obj29_vx_is_in_range(r1_obj29_a.radar1_obj29_vx);
              dy_decoded = xgu_radar1_obj29_a_radar1_obj29_dy_decode(r1_obj29_a.radar1_obj29_dy);
              dy_is_in_range = xgu_radar1_obj29_a_radar1_obj29_dy_is_in_range(r1_obj29_a.radar1_obj29_dy);
              w_exist_decoded = xgu_radar1_obj29_a_radar1_obj29_w_exist_decode(r1_obj29_a.radar1_obj29_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj29_a_radar1_obj29_w_exist_is_in_range(r1_obj29_a.radar1_obj29_w_exist);
              ax_decoded = xgu_radar1_obj29_a_radar1_obj29_ax_decode(r1_obj29_a.radar1_obj29_ax);
              ax_is_in_range = xgu_radar1_obj29_a_radar1_obj29_ax_is_in_range(r1_obj29_a.radar1_obj29_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj29_a_radar1_obj29_w_obstacle_decode(r1_obj29_a.radar1_obj29_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj29_a_radar1_obj29_w_obstacle_is_in_range(r1_obj29_a.radar1_obj29_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj29_a_radar1_obj29_flag_valid_decode(r1_obj29_a.radar1_obj29_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj29_a_radar1_obj29_flag_valid_is_in_range(r1_obj29_a.radar1_obj29_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj29_a_radar1_obj29_w_non_obstacle_decode(r1_obj29_a.radar1_obj29_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj29_a_radar1_obj29_w_non_obstacle_is_in_range(r1_obj29_a.radar1_obj29_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj29_a_radar1_obj29_flag_meas_decode(r1_obj29_a.radar1_obj29_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj29_a_radar1_obj29_flag_meas_is_in_range(r1_obj29_a.radar1_obj29_flag_meas);
              flag_hist_decoded = xgu_radar1_obj29_a_radar1_obj29_flag_hist_decode(r1_obj29_a.radar1_obj29_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj29_a_radar1_obj29_flag_hist_is_in_range(r1_obj29_a.radar1_obj29_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj29_a_radar1_obj29_mess_aconsist_bit_decode(r1_obj29_a.radar1_obj29_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj29_a_radar1_obj29_mess_aconsist_bit_is_in_range(
                  r1_obj29_a.radar1_obj29_mess_aconsist_bit);
              break;
            case 1585:
              xgu_radar1_obj30_a_t r1_obj30_a;
              target_info.unpack_return = xgu_radar1_obj30_a_unpack(&r1_obj30_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj30_a_radar1_obj30_dx_decode(r1_obj30_a.radar1_obj30_dx);
              dx_is_in_range = xgu_radar1_obj30_a_radar1_obj30_dx_is_in_range(r1_obj30_a.radar1_obj30_dx);
              vx_decoded = xgu_radar1_obj30_a_radar1_obj30_vx_decode(r1_obj30_a.radar1_obj30_vx);
              vx_is_in_range = xgu_radar1_obj30_a_radar1_obj30_vx_is_in_range(r1_obj30_a.radar1_obj30_vx);
              dy_decoded = xgu_radar1_obj30_a_radar1_obj30_dy_decode(r1_obj30_a.radar1_obj30_dy);
              dy_is_in_range = xgu_radar1_obj30_a_radar1_obj30_dy_is_in_range(r1_obj30_a.radar1_obj30_dy);
              w_exist_decoded = xgu_radar1_obj30_a_radar1_obj30_w_exist_decode(r1_obj30_a.radar1_obj30_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj30_a_radar1_obj30_w_exist_is_in_range(r1_obj30_a.radar1_obj30_w_exist);
              ax_decoded = xgu_radar1_obj30_a_radar1_obj30_ax_decode(r1_obj30_a.radar1_obj30_ax);
              ax_is_in_range = xgu_radar1_obj30_a_radar1_obj30_ax_is_in_range(r1_obj30_a.radar1_obj30_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj30_a_radar1_obj30_w_obstacle_decode(r1_obj30_a.radar1_obj30_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj30_a_radar1_obj30_w_obstacle_is_in_range(r1_obj30_a.radar1_obj30_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj30_a_radar1_obj30_flag_valid_decode(r1_obj30_a.radar1_obj30_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj30_a_radar1_obj30_flag_valid_is_in_range(r1_obj30_a.radar1_obj30_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj30_a_radar1_obj30_w_non_obstacle_decode(r1_obj30_a.radar1_obj30_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj30_a_radar1_obj30_w_non_obstacle_is_in_range(r1_obj30_a.radar1_obj30_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj30_a_radar1_obj30_flag_meas_decode(r1_obj30_a.radar1_obj30_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj30_a_radar1_obj30_flag_meas_is_in_range(r1_obj30_a.radar1_obj30_flag_meas);
              flag_hist_decoded = xgu_radar1_obj30_a_radar1_obj30_flag_hist_decode(r1_obj30_a.radar1_obj30_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj30_a_radar1_obj30_flag_hist_is_in_range(r1_obj30_a.radar1_obj30_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj30_a_radar1_obj30_mess_aconsist_bit_decode(r1_obj30_a.radar1_obj30_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj30_a_radar1_obj30_mess_aconsist_bit_is_in_range(
                  r1_obj30_a.radar1_obj30_mess_aconsist_bit);
              break;
            case 1595:
              xgu_radar1_obj31_a_t r1_obj31_a;
              target_info.unpack_return = xgu_radar1_obj31_a_unpack(&r1_obj31_a, can_data, size_of_msg);
              dx_decoded = xgu_radar1_obj31_a_radar1_obj31_dx_decode(r1_obj31_a.radar1_obj31_dx);
              dx_is_in_range = xgu_radar1_obj31_a_radar1_obj31_dx_is_in_range(r1_obj31_a.radar1_obj31_dx);
              vx_decoded = xgu_radar1_obj31_a_radar1_obj31_vx_decode(r1_obj31_a.radar1_obj31_vx);
              vx_is_in_range = xgu_radar1_obj31_a_radar1_obj31_vx_is_in_range(r1_obj31_a.radar1_obj31_vx);
              dy_decoded = xgu_radar1_obj31_a_radar1_obj31_dy_decode(r1_obj31_a.radar1_obj31_dy);
              dy_is_in_range = xgu_radar1_obj31_a_radar1_obj31_dy_is_in_range(r1_obj31_a.radar1_obj31_dy);
              w_exist_decoded = xgu_radar1_obj31_a_radar1_obj31_w_exist_decode(r1_obj31_a.radar1_obj31_w_exist);
              w_exist_is_in_range =
                  xgu_radar1_obj31_a_radar1_obj31_w_exist_is_in_range(r1_obj31_a.radar1_obj31_w_exist);
              ax_decoded = xgu_radar1_obj31_a_radar1_obj31_ax_decode(r1_obj31_a.radar1_obj31_ax);
              ax_is_in_range = xgu_radar1_obj31_a_radar1_obj31_ax_is_in_range(r1_obj31_a.radar1_obj31_ax);
              w_obstacle_decoded =
                  xgu_radar1_obj31_a_radar1_obj31_w_obstacle_decode(r1_obj31_a.radar1_obj31_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar1_obj31_a_radar1_obj31_w_obstacle_is_in_range(r1_obj31_a.radar1_obj31_w_obstacle);
              flag_valid_decoded =
                  xgu_radar1_obj31_a_radar1_obj31_flag_valid_decode(r1_obj31_a.radar1_obj31_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar1_obj31_a_radar1_obj31_flag_valid_is_in_range(r1_obj31_a.radar1_obj31_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar1_obj31_a_radar1_obj31_w_non_obstacle_decode(r1_obj31_a.radar1_obj31_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar1_obj31_a_radar1_obj31_w_non_obstacle_is_in_range(r1_obj31_a.radar1_obj31_w_non_obstacle);
              flag_meas_decoded = xgu_radar1_obj31_a_radar1_obj31_flag_meas_decode(r1_obj31_a.radar1_obj31_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar1_obj31_a_radar1_obj31_flag_meas_is_in_range(r1_obj31_a.radar1_obj31_flag_meas);
              flag_hist_decoded = xgu_radar1_obj31_a_radar1_obj31_flag_hist_decode(r1_obj31_a.radar1_obj31_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar1_obj31_a_radar1_obj31_flag_hist_is_in_range(r1_obj31_a.radar1_obj31_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar1_obj31_a_radar1_obj31_mess_aconsist_bit_decode(r1_obj31_a.radar1_obj31_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar1_obj31_a_radar1_obj31_mess_aconsist_bit_is_in_range(
                  r1_obj31_a.radar1_obj31_mess_aconsist_bit);
              break;
            case 1286:
              xgu_radar1_obj00_b_t r1_obj00_b;
              vy_decoded = xgu_radar1_obj00_b_radar1_obj00_vy_decode(r1_obj00_b.radar1_obj00_vy);
              vy_is_in_range = xgu_radar1_obj00_b_radar1_obj00_vy_is_in_range(r1_obj00_b.radar1_obj00_vy);
              d_length_decoded = xgu_radar1_obj00_b_radar1_obj00_d_length_decode(r1_obj00_b.radar1_obj00_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_d_length_is_in_range(r1_obj00_b.radar1_obj00_d_length);
              dz_decoded = xgu_radar1_obj00_b_radar1_obj00_dz_decode(r1_obj00_b.radar1_obj00_dz);
              dz_is_in_range = xgu_radar1_obj00_b_radar1_obj00_dz_is_in_range(r1_obj00_b.radar1_obj00_dz);
              moving_state_decoded =
                  xgu_radar1_obj00_b_radar1_obj00_moving_state_decode(r1_obj00_b.radar1_obj00_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_moving_state_is_in_range(r1_obj00_b.radar1_obj00_moving_state);
              dx_sigma_decoded = xgu_radar1_obj00_b_radar1_obj00_dx_sigma_decode(r1_obj00_b.radar1_obj00_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_dx_sigma_is_in_range(r1_obj00_b.radar1_obj00_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj00_b_radar1_obj00_vx_sigma_decode(r1_obj00_b.radar1_obj00_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_vx_sigma_is_in_range(r1_obj00_b.radar1_obj00_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj00_b_radar1_obj00_ax_sigma_decode(r1_obj00_b.radar1_obj00_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_ax_sigma_is_in_range(r1_obj00_b.radar1_obj00_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj00_b_radar1_obj00_dy_sigma_decode(r1_obj00_b.radar1_obj00_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_dy_sigma_is_in_range(r1_obj00_b.radar1_obj00_dy_sigma);
              w_class_decoded = xgu_radar1_obj00_b_radar1_obj00_w_class_decode(r1_obj00_b.radar1_obj00_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj00_b_radar1_obj00_w_class_is_in_range(r1_obj00_b.radar1_obj00_w_class);
              class_decoded = xgu_radar1_obj00_b_radar1_obj00_class_decode(r1_obj00_b.radar1_obj00_class);
              class_is_in_range = xgu_radar1_obj00_b_radar1_obj00_class_is_in_range(r1_obj00_b.radar1_obj00_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj00_b_radar1_obj00_dx_rear_end_loss_decode(r1_obj00_b.radar1_obj00_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj00_b_radar1_obj00_dx_rear_end_loss_is_in_range(
                  r1_obj00_b.radar1_obj00_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj00_b_radar1_obj00_mess_bconsist_bit_encode(r1_obj00_b.radar1_obj00_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj00_b_radar1_obj00_mess_bconsist_bit_is_in_range(
                  r1_obj00_b.radar1_obj00_mess_bconsist_bit);
              break;
            case 1296:
              xgu_radar1_obj01_b_t r1_obj01_b;
              vy_decoded = xgu_radar1_obj01_b_radar1_obj01_vy_decode(r1_obj01_b.radar1_obj01_vy);
              vy_is_in_range = xgu_radar1_obj01_b_radar1_obj01_vy_is_in_range(r1_obj01_b.radar1_obj01_vy);
              d_length_decoded = xgu_radar1_obj01_b_radar1_obj01_d_length_decode(r1_obj01_b.radar1_obj01_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_d_length_is_in_range(r1_obj01_b.radar1_obj01_d_length);
              dz_decoded = xgu_radar1_obj01_b_radar1_obj01_dz_decode(r1_obj01_b.radar1_obj01_dz);
              dz_is_in_range = xgu_radar1_obj01_b_radar1_obj01_dz_is_in_range(r1_obj01_b.radar1_obj01_dz);
              moving_state_decoded =
                  xgu_radar1_obj01_b_radar1_obj01_moving_state_decode(r1_obj01_b.radar1_obj01_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_moving_state_is_in_range(r1_obj01_b.radar1_obj01_moving_state);
              dx_sigma_decoded = xgu_radar1_obj01_b_radar1_obj01_dx_sigma_decode(r1_obj01_b.radar1_obj01_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_dx_sigma_is_in_range(r1_obj01_b.radar1_obj01_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj01_b_radar1_obj01_vx_sigma_decode(r1_obj01_b.radar1_obj01_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_vx_sigma_is_in_range(r1_obj01_b.radar1_obj01_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj01_b_radar1_obj01_ax_sigma_decode(r1_obj01_b.radar1_obj01_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_ax_sigma_is_in_range(r1_obj01_b.radar1_obj01_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj01_b_radar1_obj01_dy_sigma_decode(r1_obj01_b.radar1_obj01_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_dy_sigma_is_in_range(r1_obj01_b.radar1_obj01_dy_sigma);
              w_class_decoded = xgu_radar1_obj01_b_radar1_obj01_w_class_decode(r1_obj01_b.radar1_obj01_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj01_b_radar1_obj01_w_class_is_in_range(r1_obj01_b.radar1_obj01_w_class);
              class_decoded = xgu_radar1_obj01_b_radar1_obj01_class_decode(r1_obj01_b.radar1_obj01_class);
              class_is_in_range = xgu_radar1_obj01_b_radar1_obj01_class_is_in_range(r1_obj01_b.radar1_obj01_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj01_b_radar1_obj01_dx_rear_end_loss_decode(r1_obj01_b.radar1_obj01_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj01_b_radar1_obj01_dx_rear_end_loss_is_in_range(
                  r1_obj01_b.radar1_obj01_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj01_b_radar1_obj01_mess_bconsist_bit_encode(r1_obj01_b.radar1_obj01_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj01_b_radar1_obj01_mess_bconsist_bit_is_in_range(
                  r1_obj01_b.radar1_obj01_mess_bconsist_bit);
              break;
            case 1306:
              xgu_radar1_obj02_b_t r1_obj02_b;
              vy_decoded = xgu_radar1_obj02_b_radar1_obj02_vy_decode(r1_obj02_b.radar1_obj02_vy);
              vy_is_in_range = xgu_radar1_obj02_b_radar1_obj02_vy_is_in_range(r1_obj02_b.radar1_obj02_vy);
              d_length_decoded = xgu_radar1_obj02_b_radar1_obj02_d_length_decode(r1_obj02_b.radar1_obj02_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_d_length_is_in_range(r1_obj02_b.radar1_obj02_d_length);
              dz_decoded = xgu_radar1_obj02_b_radar1_obj02_dz_decode(r1_obj02_b.radar1_obj02_dz);
              dz_is_in_range = xgu_radar1_obj02_b_radar1_obj02_dz_is_in_range(r1_obj02_b.radar1_obj02_dz);
              moving_state_decoded =
                  xgu_radar1_obj02_b_radar1_obj02_moving_state_decode(r1_obj02_b.radar1_obj02_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_moving_state_is_in_range(r1_obj02_b.radar1_obj02_moving_state);
              dx_sigma_decoded = xgu_radar1_obj02_b_radar1_obj02_dx_sigma_decode(r1_obj02_b.radar1_obj02_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_dx_sigma_is_in_range(r1_obj02_b.radar1_obj02_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj02_b_radar1_obj02_vx_sigma_decode(r1_obj02_b.radar1_obj02_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_vx_sigma_is_in_range(r1_obj02_b.radar1_obj02_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj02_b_radar1_obj02_ax_sigma_decode(r1_obj02_b.radar1_obj02_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_ax_sigma_is_in_range(r1_obj02_b.radar1_obj02_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj02_b_radar1_obj02_dy_sigma_decode(r1_obj02_b.radar1_obj02_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_dy_sigma_is_in_range(r1_obj02_b.radar1_obj02_dy_sigma);
              w_class_decoded = xgu_radar1_obj02_b_radar1_obj02_w_class_decode(r1_obj02_b.radar1_obj02_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj02_b_radar1_obj02_w_class_is_in_range(r1_obj02_b.radar1_obj02_w_class);
              class_decoded = xgu_radar1_obj02_b_radar1_obj02_class_decode(r1_obj02_b.radar1_obj02_class);
              class_is_in_range = xgu_radar1_obj02_b_radar1_obj02_class_is_in_range(r1_obj02_b.radar1_obj02_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj02_b_radar1_obj02_dx_rear_end_loss_decode(r1_obj02_b.radar1_obj02_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj02_b_radar1_obj02_dx_rear_end_loss_is_in_range(
                  r1_obj02_b.radar1_obj02_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj02_b_radar1_obj02_mess_bconsist_bit_encode(r1_obj02_b.radar1_obj02_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj02_b_radar1_obj02_mess_bconsist_bit_is_in_range(
                  r1_obj02_b.radar1_obj02_mess_bconsist_bit);
              break;
            case 1316:
              xgu_radar1_obj03_b_t r1_obj03_b;
              vy_decoded = xgu_radar1_obj03_b_radar1_obj03_vy_decode(r1_obj03_b.radar1_obj03_vy);
              vy_is_in_range = xgu_radar1_obj03_b_radar1_obj03_vy_is_in_range(r1_obj03_b.radar1_obj03_vy);
              d_length_decoded = xgu_radar1_obj03_b_radar1_obj03_d_length_decode(r1_obj03_b.radar1_obj03_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_d_length_is_in_range(r1_obj03_b.radar1_obj03_d_length);
              dz_decoded = xgu_radar1_obj03_b_radar1_obj03_dz_decode(r1_obj03_b.radar1_obj03_dz);
              dz_is_in_range = xgu_radar1_obj03_b_radar1_obj03_dz_is_in_range(r1_obj03_b.radar1_obj03_dz);
              moving_state_decoded =
                  xgu_radar1_obj03_b_radar1_obj03_moving_state_decode(r1_obj03_b.radar1_obj03_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_moving_state_is_in_range(r1_obj03_b.radar1_obj03_moving_state);
              dx_sigma_decoded = xgu_radar1_obj03_b_radar1_obj03_dx_sigma_decode(r1_obj03_b.radar1_obj03_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_dx_sigma_is_in_range(r1_obj03_b.radar1_obj03_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj03_b_radar1_obj03_vx_sigma_decode(r1_obj03_b.radar1_obj03_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_vx_sigma_is_in_range(r1_obj03_b.radar1_obj03_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj03_b_radar1_obj03_ax_sigma_decode(r1_obj03_b.radar1_obj03_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_ax_sigma_is_in_range(r1_obj03_b.radar1_obj03_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj03_b_radar1_obj03_dy_sigma_decode(r1_obj03_b.radar1_obj03_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_dy_sigma_is_in_range(r1_obj03_b.radar1_obj03_dy_sigma);
              w_class_decoded = xgu_radar1_obj03_b_radar1_obj03_w_class_decode(r1_obj03_b.radar1_obj03_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj03_b_radar1_obj03_w_class_is_in_range(r1_obj03_b.radar1_obj03_w_class);
              class_decoded = xgu_radar1_obj03_b_radar1_obj03_class_decode(r1_obj03_b.radar1_obj03_class);
              class_is_in_range = xgu_radar1_obj03_b_radar1_obj03_class_is_in_range(r1_obj03_b.radar1_obj03_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj03_b_radar1_obj03_dx_rear_end_loss_decode(r1_obj03_b.radar1_obj03_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj03_b_radar1_obj03_dx_rear_end_loss_is_in_range(
                  r1_obj03_b.radar1_obj03_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj03_b_radar1_obj03_mess_bconsist_bit_encode(r1_obj03_b.radar1_obj03_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj03_b_radar1_obj03_mess_bconsist_bit_is_in_range(
                  r1_obj03_b.radar1_obj03_mess_bconsist_bit);
              break;
            case 1326:
              xgu_radar1_obj04_b_t r1_obj04_b;
              vy_decoded = xgu_radar1_obj04_b_radar1_obj04_vy_decode(r1_obj04_b.radar1_obj04_vy);
              vy_is_in_range = xgu_radar1_obj04_b_radar1_obj04_vy_is_in_range(r1_obj04_b.radar1_obj04_vy);
              d_length_decoded = xgu_radar1_obj04_b_radar1_obj04_d_length_decode(r1_obj04_b.radar1_obj04_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_d_length_is_in_range(r1_obj04_b.radar1_obj04_d_length);
              dz_decoded = xgu_radar1_obj04_b_radar1_obj04_dz_decode(r1_obj04_b.radar1_obj04_dz);
              dz_is_in_range = xgu_radar1_obj04_b_radar1_obj04_dz_is_in_range(r1_obj04_b.radar1_obj04_dz);
              moving_state_decoded =
                  xgu_radar1_obj04_b_radar1_obj04_moving_state_decode(r1_obj04_b.radar1_obj04_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_moving_state_is_in_range(r1_obj04_b.radar1_obj04_moving_state);
              dx_sigma_decoded = xgu_radar1_obj04_b_radar1_obj04_dx_sigma_decode(r1_obj04_b.radar1_obj04_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_dx_sigma_is_in_range(r1_obj04_b.radar1_obj04_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj04_b_radar1_obj04_vx_sigma_decode(r1_obj04_b.radar1_obj04_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_vx_sigma_is_in_range(r1_obj04_b.radar1_obj04_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj04_b_radar1_obj04_ax_sigma_decode(r1_obj04_b.radar1_obj04_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_ax_sigma_is_in_range(r1_obj04_b.radar1_obj04_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj04_b_radar1_obj04_dy_sigma_decode(r1_obj04_b.radar1_obj04_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_dy_sigma_is_in_range(r1_obj04_b.radar1_obj04_dy_sigma);
              w_class_decoded = xgu_radar1_obj04_b_radar1_obj04_w_class_decode(r1_obj04_b.radar1_obj04_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj04_b_radar1_obj04_w_class_is_in_range(r1_obj04_b.radar1_obj04_w_class);
              class_decoded = xgu_radar1_obj04_b_radar1_obj04_class_decode(r1_obj04_b.radar1_obj04_class);
              class_is_in_range = xgu_radar1_obj04_b_radar1_obj04_class_is_in_range(r1_obj04_b.radar1_obj04_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj04_b_radar1_obj04_dx_rear_end_loss_decode(r1_obj04_b.radar1_obj04_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj04_b_radar1_obj04_dx_rear_end_loss_is_in_range(
                  r1_obj04_b.radar1_obj04_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj04_b_radar1_obj04_mess_bconsist_bit_encode(r1_obj04_b.radar1_obj04_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj04_b_radar1_obj04_mess_bconsist_bit_is_in_range(
                  r1_obj04_b.radar1_obj04_mess_bconsist_bit);
              break;
            case 1336:
              xgu_radar1_obj05_b_t r1_obj05_b;
              vy_decoded = xgu_radar1_obj05_b_radar1_obj05_vy_decode(r1_obj05_b.radar1_obj05_vy);
              vy_is_in_range = xgu_radar1_obj05_b_radar1_obj05_vy_is_in_range(r1_obj05_b.radar1_obj05_vy);
              d_length_decoded = xgu_radar1_obj05_b_radar1_obj05_d_length_decode(r1_obj05_b.radar1_obj05_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_d_length_is_in_range(r1_obj05_b.radar1_obj05_d_length);
              dz_decoded = xgu_radar1_obj05_b_radar1_obj05_dz_decode(r1_obj05_b.radar1_obj05_dz);
              dz_is_in_range = xgu_radar1_obj05_b_radar1_obj05_dz_is_in_range(r1_obj05_b.radar1_obj05_dz);
              moving_state_decoded =
                  xgu_radar1_obj05_b_radar1_obj05_moving_state_decode(r1_obj05_b.radar1_obj05_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_moving_state_is_in_range(r1_obj05_b.radar1_obj05_moving_state);
              dx_sigma_decoded = xgu_radar1_obj05_b_radar1_obj05_dx_sigma_decode(r1_obj05_b.radar1_obj05_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_dx_sigma_is_in_range(r1_obj05_b.radar1_obj05_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj05_b_radar1_obj05_vx_sigma_decode(r1_obj05_b.radar1_obj05_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_vx_sigma_is_in_range(r1_obj05_b.radar1_obj05_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj05_b_radar1_obj05_ax_sigma_decode(r1_obj05_b.radar1_obj05_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_ax_sigma_is_in_range(r1_obj05_b.radar1_obj05_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj05_b_radar1_obj05_dy_sigma_decode(r1_obj05_b.radar1_obj05_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_dy_sigma_is_in_range(r1_obj05_b.radar1_obj05_dy_sigma);
              w_class_decoded = xgu_radar1_obj05_b_radar1_obj05_w_class_decode(r1_obj05_b.radar1_obj05_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj05_b_radar1_obj05_w_class_is_in_range(r1_obj05_b.radar1_obj05_w_class);
              class_decoded = xgu_radar1_obj05_b_radar1_obj05_class_decode(r1_obj05_b.radar1_obj05_class);
              class_is_in_range = xgu_radar1_obj05_b_radar1_obj05_class_is_in_range(r1_obj05_b.radar1_obj05_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj05_b_radar1_obj05_dx_rear_end_loss_decode(r1_obj05_b.radar1_obj05_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj05_b_radar1_obj05_dx_rear_end_loss_is_in_range(
                  r1_obj05_b.radar1_obj05_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj05_b_radar1_obj05_mess_bconsist_bit_encode(r1_obj05_b.radar1_obj05_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj05_b_radar1_obj05_mess_bconsist_bit_is_in_range(
                  r1_obj05_b.radar1_obj05_mess_bconsist_bit);
              break;
            case 1346:
              xgu_radar1_obj06_b_t r1_obj06_b;
              vy_decoded = xgu_radar1_obj06_b_radar1_obj06_vy_decode(r1_obj06_b.radar1_obj06_vy);
              vy_is_in_range = xgu_radar1_obj06_b_radar1_obj06_vy_is_in_range(r1_obj06_b.radar1_obj06_vy);
              d_length_decoded = xgu_radar1_obj06_b_radar1_obj06_d_length_decode(r1_obj06_b.radar1_obj06_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_d_length_is_in_range(r1_obj06_b.radar1_obj06_d_length);
              dz_decoded = xgu_radar1_obj06_b_radar1_obj06_dz_decode(r1_obj06_b.radar1_obj06_dz);
              dz_is_in_range = xgu_radar1_obj06_b_radar1_obj06_dz_is_in_range(r1_obj06_b.radar1_obj06_dz);
              moving_state_decoded =
                  xgu_radar1_obj06_b_radar1_obj06_moving_state_decode(r1_obj06_b.radar1_obj06_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_moving_state_is_in_range(r1_obj06_b.radar1_obj06_moving_state);
              dx_sigma_decoded = xgu_radar1_obj06_b_radar1_obj06_dx_sigma_decode(r1_obj06_b.radar1_obj06_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_dx_sigma_is_in_range(r1_obj06_b.radar1_obj06_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj06_b_radar1_obj06_vx_sigma_decode(r1_obj06_b.radar1_obj06_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_vx_sigma_is_in_range(r1_obj06_b.radar1_obj06_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj06_b_radar1_obj06_ax_sigma_decode(r1_obj06_b.radar1_obj06_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_ax_sigma_is_in_range(r1_obj06_b.radar1_obj06_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj06_b_radar1_obj06_dy_sigma_decode(r1_obj06_b.radar1_obj06_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_dy_sigma_is_in_range(r1_obj06_b.radar1_obj06_dy_sigma);
              w_class_decoded = xgu_radar1_obj06_b_radar1_obj06_w_class_decode(r1_obj06_b.radar1_obj06_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj06_b_radar1_obj06_w_class_is_in_range(r1_obj06_b.radar1_obj06_w_class);
              class_decoded = xgu_radar1_obj06_b_radar1_obj06_class_decode(r1_obj06_b.radar1_obj06_class);
              class_is_in_range = xgu_radar1_obj06_b_radar1_obj06_class_is_in_range(r1_obj06_b.radar1_obj06_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj06_b_radar1_obj06_dx_rear_end_loss_decode(r1_obj06_b.radar1_obj06_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj06_b_radar1_obj06_dx_rear_end_loss_is_in_range(
                  r1_obj06_b.radar1_obj06_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj06_b_radar1_obj06_mess_bconsist_bit_encode(r1_obj06_b.radar1_obj06_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj06_b_radar1_obj06_mess_bconsist_bit_is_in_range(
                  r1_obj06_b.radar1_obj06_mess_bconsist_bit);
              break;
            case 1356:
              xgu_radar1_obj07_b_t r1_obj07_b;
              vy_decoded = xgu_radar1_obj07_b_radar1_obj07_vy_decode(r1_obj07_b.radar1_obj07_vy);
              vy_is_in_range = xgu_radar1_obj07_b_radar1_obj07_vy_is_in_range(r1_obj07_b.radar1_obj07_vy);
              d_length_decoded = xgu_radar1_obj07_b_radar1_obj07_d_length_decode(r1_obj07_b.radar1_obj07_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_d_length_is_in_range(r1_obj07_b.radar1_obj07_d_length);
              dz_decoded = xgu_radar1_obj07_b_radar1_obj07_dz_decode(r1_obj07_b.radar1_obj07_dz);
              dz_is_in_range = xgu_radar1_obj07_b_radar1_obj07_dz_is_in_range(r1_obj07_b.radar1_obj07_dz);
              moving_state_decoded =
                  xgu_radar1_obj07_b_radar1_obj07_moving_state_decode(r1_obj07_b.radar1_obj07_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_moving_state_is_in_range(r1_obj07_b.radar1_obj07_moving_state);
              dx_sigma_decoded = xgu_radar1_obj07_b_radar1_obj07_dx_sigma_decode(r1_obj07_b.radar1_obj07_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_dx_sigma_is_in_range(r1_obj07_b.radar1_obj07_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj07_b_radar1_obj07_vx_sigma_decode(r1_obj07_b.radar1_obj07_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_vx_sigma_is_in_range(r1_obj07_b.radar1_obj07_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj07_b_radar1_obj07_ax_sigma_decode(r1_obj07_b.radar1_obj07_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_ax_sigma_is_in_range(r1_obj07_b.radar1_obj07_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj07_b_radar1_obj07_dy_sigma_decode(r1_obj07_b.radar1_obj07_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_dy_sigma_is_in_range(r1_obj07_b.radar1_obj07_dy_sigma);
              w_class_decoded = xgu_radar1_obj07_b_radar1_obj07_w_class_decode(r1_obj07_b.radar1_obj07_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj07_b_radar1_obj07_w_class_is_in_range(r1_obj07_b.radar1_obj07_w_class);
              class_decoded = xgu_radar1_obj07_b_radar1_obj07_class_decode(r1_obj07_b.radar1_obj07_class);
              class_is_in_range = xgu_radar1_obj07_b_radar1_obj07_class_is_in_range(r1_obj07_b.radar1_obj07_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj07_b_radar1_obj07_dx_rear_end_loss_decode(r1_obj07_b.radar1_obj07_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj07_b_radar1_obj07_dx_rear_end_loss_is_in_range(
                  r1_obj07_b.radar1_obj07_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj07_b_radar1_obj07_mess_bconsist_bit_encode(r1_obj07_b.radar1_obj07_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj07_b_radar1_obj07_mess_bconsist_bit_is_in_range(
                  r1_obj07_b.radar1_obj07_mess_bconsist_bit);
              break;
            case 1366:
              xgu_radar1_obj08_b_t r1_obj08_b;
              vy_decoded = xgu_radar1_obj08_b_radar1_obj08_vy_decode(r1_obj08_b.radar1_obj08_vy);
              vy_is_in_range = xgu_radar1_obj08_b_radar1_obj08_vy_is_in_range(r1_obj08_b.radar1_obj08_vy);
              d_length_decoded = xgu_radar1_obj08_b_radar1_obj08_d_length_decode(r1_obj08_b.radar1_obj08_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_d_length_is_in_range(r1_obj08_b.radar1_obj08_d_length);
              dz_decoded = xgu_radar1_obj08_b_radar1_obj08_dz_decode(r1_obj08_b.radar1_obj08_dz);
              dz_is_in_range = xgu_radar1_obj08_b_radar1_obj08_dz_is_in_range(r1_obj08_b.radar1_obj08_dz);
              moving_state_decoded =
                  xgu_radar1_obj08_b_radar1_obj08_moving_state_decode(r1_obj08_b.radar1_obj08_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_moving_state_is_in_range(r1_obj08_b.radar1_obj08_moving_state);
              dx_sigma_decoded = xgu_radar1_obj08_b_radar1_obj08_dx_sigma_decode(r1_obj08_b.radar1_obj08_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_dx_sigma_is_in_range(r1_obj08_b.radar1_obj08_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj08_b_radar1_obj08_vx_sigma_decode(r1_obj08_b.radar1_obj08_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_vx_sigma_is_in_range(r1_obj08_b.radar1_obj08_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj08_b_radar1_obj08_ax_sigma_decode(r1_obj08_b.radar1_obj08_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_ax_sigma_is_in_range(r1_obj08_b.radar1_obj08_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj08_b_radar1_obj08_dy_sigma_decode(r1_obj08_b.radar1_obj08_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_dy_sigma_is_in_range(r1_obj08_b.radar1_obj08_dy_sigma);
              w_class_decoded = xgu_radar1_obj08_b_radar1_obj08_w_class_decode(r1_obj08_b.radar1_obj08_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj08_b_radar1_obj08_w_class_is_in_range(r1_obj08_b.radar1_obj08_w_class);
              class_decoded = xgu_radar1_obj08_b_radar1_obj08_class_decode(r1_obj08_b.radar1_obj08_class);
              class_is_in_range = xgu_radar1_obj08_b_radar1_obj08_class_is_in_range(r1_obj08_b.radar1_obj08_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj08_b_radar1_obj08_dx_rear_end_loss_decode(r1_obj08_b.radar1_obj08_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj08_b_radar1_obj08_dx_rear_end_loss_is_in_range(
                  r1_obj08_b.radar1_obj08_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj08_b_radar1_obj08_mess_bconsist_bit_encode(r1_obj08_b.radar1_obj08_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj08_b_radar1_obj08_mess_bconsist_bit_is_in_range(
                  r1_obj08_b.radar1_obj08_mess_bconsist_bit);
              break;
            case 1376:
              xgu_radar1_obj09_b_t r1_obj09_b;
              vy_decoded = xgu_radar1_obj09_b_radar1_obj09_vy_decode(r1_obj09_b.radar1_obj09_vy);
              vy_is_in_range = xgu_radar1_obj09_b_radar1_obj09_vy_is_in_range(r1_obj09_b.radar1_obj09_vy);
              d_length_decoded = xgu_radar1_obj09_b_radar1_obj09_d_length_decode(r1_obj09_b.radar1_obj09_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_d_length_is_in_range(r1_obj09_b.radar1_obj09_d_length);
              dz_decoded = xgu_radar1_obj09_b_radar1_obj09_dz_decode(r1_obj09_b.radar1_obj09_dz);
              dz_is_in_range = xgu_radar1_obj09_b_radar1_obj09_dz_is_in_range(r1_obj09_b.radar1_obj09_dz);
              moving_state_decoded =
                  xgu_radar1_obj09_b_radar1_obj09_moving_state_decode(r1_obj09_b.radar1_obj09_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_moving_state_is_in_range(r1_obj09_b.radar1_obj09_moving_state);
              dx_sigma_decoded = xgu_radar1_obj09_b_radar1_obj09_dx_sigma_decode(r1_obj09_b.radar1_obj09_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_dx_sigma_is_in_range(r1_obj09_b.radar1_obj09_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj09_b_radar1_obj09_vx_sigma_decode(r1_obj09_b.radar1_obj09_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_vx_sigma_is_in_range(r1_obj09_b.radar1_obj09_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj09_b_radar1_obj09_ax_sigma_decode(r1_obj09_b.radar1_obj09_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_ax_sigma_is_in_range(r1_obj09_b.radar1_obj09_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj09_b_radar1_obj09_dy_sigma_decode(r1_obj09_b.radar1_obj09_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_dy_sigma_is_in_range(r1_obj09_b.radar1_obj09_dy_sigma);
              w_class_decoded = xgu_radar1_obj09_b_radar1_obj09_w_class_decode(r1_obj09_b.radar1_obj09_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj09_b_radar1_obj09_w_class_is_in_range(r1_obj09_b.radar1_obj09_w_class);
              class_decoded = xgu_radar1_obj09_b_radar1_obj09_class_decode(r1_obj09_b.radar1_obj09_class);
              class_is_in_range = xgu_radar1_obj09_b_radar1_obj09_class_is_in_range(r1_obj09_b.radar1_obj09_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj09_b_radar1_obj09_dx_rear_end_loss_decode(r1_obj09_b.radar1_obj09_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj09_b_radar1_obj09_dx_rear_end_loss_is_in_range(
                  r1_obj09_b.radar1_obj09_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj09_b_radar1_obj09_mess_bconsist_bit_encode(r1_obj09_b.radar1_obj09_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj09_b_radar1_obj09_mess_bconsist_bit_is_in_range(
                  r1_obj09_b.radar1_obj09_mess_bconsist_bit);
              break;
            case 1386:
              xgu_radar1_obj10_b_t r1_obj10_b;
              vy_decoded = xgu_radar1_obj10_b_radar1_obj10_vy_decode(r1_obj10_b.radar1_obj10_vy);
              vy_is_in_range = xgu_radar1_obj10_b_radar1_obj10_vy_is_in_range(r1_obj10_b.radar1_obj10_vy);
              d_length_decoded = xgu_radar1_obj10_b_radar1_obj10_d_length_decode(r1_obj10_b.radar1_obj10_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_d_length_is_in_range(r1_obj10_b.radar1_obj10_d_length);
              dz_decoded = xgu_radar1_obj10_b_radar1_obj10_dz_decode(r1_obj10_b.radar1_obj10_dz);
              dz_is_in_range = xgu_radar1_obj10_b_radar1_obj10_dz_is_in_range(r1_obj10_b.radar1_obj10_dz);
              moving_state_decoded =
                  xgu_radar1_obj10_b_radar1_obj10_moving_state_decode(r1_obj10_b.radar1_obj10_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_moving_state_is_in_range(r1_obj10_b.radar1_obj10_moving_state);
              dx_sigma_decoded = xgu_radar1_obj10_b_radar1_obj10_dx_sigma_decode(r1_obj10_b.radar1_obj10_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_dx_sigma_is_in_range(r1_obj10_b.radar1_obj10_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj10_b_radar1_obj10_vx_sigma_decode(r1_obj10_b.radar1_obj10_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_vx_sigma_is_in_range(r1_obj10_b.radar1_obj10_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj10_b_radar1_obj10_ax_sigma_decode(r1_obj10_b.radar1_obj10_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_ax_sigma_is_in_range(r1_obj10_b.radar1_obj10_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj10_b_radar1_obj10_dy_sigma_decode(r1_obj10_b.radar1_obj10_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_dy_sigma_is_in_range(r1_obj10_b.radar1_obj10_dy_sigma);
              w_class_decoded = xgu_radar1_obj10_b_radar1_obj10_w_class_decode(r1_obj10_b.radar1_obj10_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj10_b_radar1_obj10_w_class_is_in_range(r1_obj10_b.radar1_obj10_w_class);
              class_decoded = xgu_radar1_obj10_b_radar1_obj10_class_decode(r1_obj10_b.radar1_obj10_class);
              class_is_in_range = xgu_radar1_obj10_b_radar1_obj10_class_is_in_range(r1_obj10_b.radar1_obj10_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj10_b_radar1_obj10_dx_rear_end_loss_decode(r1_obj10_b.radar1_obj10_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj10_b_radar1_obj10_dx_rear_end_loss_is_in_range(
                  r1_obj10_b.radar1_obj10_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj10_b_radar1_obj10_mess_bconsist_bit_encode(r1_obj10_b.radar1_obj10_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj10_b_radar1_obj10_mess_bconsist_bit_is_in_range(
                  r1_obj10_b.radar1_obj10_mess_bconsist_bit);
              break;
            case 1396:
              xgu_radar1_obj11_b_t r1_obj11_b;
              vy_decoded = xgu_radar1_obj11_b_radar1_obj11_vy_decode(r1_obj11_b.radar1_obj11_vy);
              vy_is_in_range = xgu_radar1_obj11_b_radar1_obj11_vy_is_in_range(r1_obj11_b.radar1_obj11_vy);
              d_length_decoded = xgu_radar1_obj11_b_radar1_obj11_d_length_decode(r1_obj11_b.radar1_obj11_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_d_length_is_in_range(r1_obj11_b.radar1_obj11_d_length);
              dz_decoded = xgu_radar1_obj11_b_radar1_obj11_dz_decode(r1_obj11_b.radar1_obj11_dz);
              dz_is_in_range = xgu_radar1_obj11_b_radar1_obj11_dz_is_in_range(r1_obj11_b.radar1_obj11_dz);
              moving_state_decoded =
                  xgu_radar1_obj11_b_radar1_obj11_moving_state_decode(r1_obj11_b.radar1_obj11_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_moving_state_is_in_range(r1_obj11_b.radar1_obj11_moving_state);
              dx_sigma_decoded = xgu_radar1_obj11_b_radar1_obj11_dx_sigma_decode(r1_obj11_b.radar1_obj11_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_dx_sigma_is_in_range(r1_obj11_b.radar1_obj11_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj11_b_radar1_obj11_vx_sigma_decode(r1_obj11_b.radar1_obj11_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_vx_sigma_is_in_range(r1_obj11_b.radar1_obj11_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj11_b_radar1_obj11_ax_sigma_decode(r1_obj11_b.radar1_obj11_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_ax_sigma_is_in_range(r1_obj11_b.radar1_obj11_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj11_b_radar1_obj11_dy_sigma_decode(r1_obj11_b.radar1_obj11_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_dy_sigma_is_in_range(r1_obj11_b.radar1_obj11_dy_sigma);
              w_class_decoded = xgu_radar1_obj11_b_radar1_obj11_w_class_decode(r1_obj11_b.radar1_obj11_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj11_b_radar1_obj11_w_class_is_in_range(r1_obj11_b.radar1_obj11_w_class);
              class_decoded = xgu_radar1_obj11_b_radar1_obj11_class_decode(r1_obj11_b.radar1_obj11_class);
              class_is_in_range = xgu_radar1_obj11_b_radar1_obj11_class_is_in_range(r1_obj11_b.radar1_obj11_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj11_b_radar1_obj11_dx_rear_end_loss_decode(r1_obj11_b.radar1_obj11_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj11_b_radar1_obj11_dx_rear_end_loss_is_in_range(
                  r1_obj11_b.radar1_obj11_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj11_b_radar1_obj11_mess_bconsist_bit_encode(r1_obj11_b.radar1_obj11_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj11_b_radar1_obj11_mess_bconsist_bit_is_in_range(
                  r1_obj11_b.radar1_obj11_mess_bconsist_bit);
              break;
            case 1406:
              xgu_radar1_obj12_b_t r1_obj12_b;
              vy_decoded = xgu_radar1_obj12_b_radar1_obj12_vy_decode(r1_obj12_b.radar1_obj12_vy);
              vy_is_in_range = xgu_radar1_obj12_b_radar1_obj12_vy_is_in_range(r1_obj12_b.radar1_obj12_vy);
              d_length_decoded = xgu_radar1_obj12_b_radar1_obj12_d_length_decode(r1_obj12_b.radar1_obj12_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_d_length_is_in_range(r1_obj12_b.radar1_obj12_d_length);
              dz_decoded = xgu_radar1_obj12_b_radar1_obj12_dz_decode(r1_obj12_b.radar1_obj12_dz);
              dz_is_in_range = xgu_radar1_obj12_b_radar1_obj12_dz_is_in_range(r1_obj12_b.radar1_obj12_dz);
              moving_state_decoded =
                  xgu_radar1_obj12_b_radar1_obj12_moving_state_decode(r1_obj12_b.radar1_obj12_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_moving_state_is_in_range(r1_obj12_b.radar1_obj12_moving_state);
              dx_sigma_decoded = xgu_radar1_obj12_b_radar1_obj12_dx_sigma_decode(r1_obj12_b.radar1_obj12_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_dx_sigma_is_in_range(r1_obj12_b.radar1_obj12_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj12_b_radar1_obj12_vx_sigma_decode(r1_obj12_b.radar1_obj12_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_vx_sigma_is_in_range(r1_obj12_b.radar1_obj12_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj12_b_radar1_obj12_ax_sigma_decode(r1_obj12_b.radar1_obj12_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_ax_sigma_is_in_range(r1_obj12_b.radar1_obj12_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj12_b_radar1_obj12_dy_sigma_decode(r1_obj12_b.radar1_obj12_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_dy_sigma_is_in_range(r1_obj12_b.radar1_obj12_dy_sigma);
              w_class_decoded = xgu_radar1_obj12_b_radar1_obj12_w_class_decode(r1_obj12_b.radar1_obj12_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj12_b_radar1_obj12_w_class_is_in_range(r1_obj12_b.radar1_obj12_w_class);
              class_decoded = xgu_radar1_obj12_b_radar1_obj12_class_decode(r1_obj12_b.radar1_obj12_class);
              class_is_in_range = xgu_radar1_obj12_b_radar1_obj12_class_is_in_range(r1_obj12_b.radar1_obj12_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj12_b_radar1_obj12_dx_rear_end_loss_decode(r1_obj12_b.radar1_obj12_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj12_b_radar1_obj12_dx_rear_end_loss_is_in_range(
                  r1_obj12_b.radar1_obj12_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj12_b_radar1_obj12_mess_bconsist_bit_encode(r1_obj12_b.radar1_obj12_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj12_b_radar1_obj12_mess_bconsist_bit_is_in_range(
                  r1_obj12_b.radar1_obj12_mess_bconsist_bit);
              break;
            case 1416:
              xgu_radar1_obj13_b_t r1_obj13_b;
              vy_decoded = xgu_radar1_obj13_b_radar1_obj13_vy_decode(r1_obj13_b.radar1_obj13_vy);
              vy_is_in_range = xgu_radar1_obj13_b_radar1_obj13_vy_is_in_range(r1_obj13_b.radar1_obj13_vy);
              d_length_decoded = xgu_radar1_obj13_b_radar1_obj13_d_length_decode(r1_obj13_b.radar1_obj13_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_d_length_is_in_range(r1_obj13_b.radar1_obj13_d_length);
              dz_decoded = xgu_radar1_obj13_b_radar1_obj13_dz_decode(r1_obj13_b.radar1_obj13_dz);
              dz_is_in_range = xgu_radar1_obj13_b_radar1_obj13_dz_is_in_range(r1_obj13_b.radar1_obj13_dz);
              moving_state_decoded =
                  xgu_radar1_obj13_b_radar1_obj13_moving_state_decode(r1_obj13_b.radar1_obj13_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_moving_state_is_in_range(r1_obj13_b.radar1_obj13_moving_state);
              dx_sigma_decoded = xgu_radar1_obj13_b_radar1_obj13_dx_sigma_decode(r1_obj13_b.radar1_obj13_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_dx_sigma_is_in_range(r1_obj13_b.radar1_obj13_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj13_b_radar1_obj13_vx_sigma_decode(r1_obj13_b.radar1_obj13_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_vx_sigma_is_in_range(r1_obj13_b.radar1_obj13_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj13_b_radar1_obj13_ax_sigma_decode(r1_obj13_b.radar1_obj13_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_ax_sigma_is_in_range(r1_obj13_b.radar1_obj13_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj13_b_radar1_obj13_dy_sigma_decode(r1_obj13_b.radar1_obj13_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_dy_sigma_is_in_range(r1_obj13_b.radar1_obj13_dy_sigma);
              w_class_decoded = xgu_radar1_obj13_b_radar1_obj13_w_class_decode(r1_obj13_b.radar1_obj13_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj13_b_radar1_obj13_w_class_is_in_range(r1_obj13_b.radar1_obj13_w_class);
              class_decoded = xgu_radar1_obj13_b_radar1_obj13_class_decode(r1_obj13_b.radar1_obj13_class);
              class_is_in_range = xgu_radar1_obj13_b_radar1_obj13_class_is_in_range(r1_obj13_b.radar1_obj13_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj13_b_radar1_obj13_dx_rear_end_loss_decode(r1_obj13_b.radar1_obj13_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj13_b_radar1_obj13_dx_rear_end_loss_is_in_range(
                  r1_obj13_b.radar1_obj13_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj13_b_radar1_obj13_mess_bconsist_bit_encode(r1_obj13_b.radar1_obj13_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj13_b_radar1_obj13_mess_bconsist_bit_is_in_range(
                  r1_obj13_b.radar1_obj13_mess_bconsist_bit);
              break;
            case 1426:
              xgu_radar1_obj14_b_t r1_obj14_b;
              vy_decoded = xgu_radar1_obj14_b_radar1_obj14_vy_decode(r1_obj14_b.radar1_obj14_vy);
              vy_is_in_range = xgu_radar1_obj14_b_radar1_obj14_vy_is_in_range(r1_obj14_b.radar1_obj14_vy);
              d_length_decoded = xgu_radar1_obj14_b_radar1_obj14_d_length_decode(r1_obj14_b.radar1_obj14_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_d_length_is_in_range(r1_obj14_b.radar1_obj14_d_length);
              dz_decoded = xgu_radar1_obj14_b_radar1_obj14_dz_decode(r1_obj14_b.radar1_obj14_dz);
              dz_is_in_range = xgu_radar1_obj14_b_radar1_obj14_dz_is_in_range(r1_obj14_b.radar1_obj14_dz);
              moving_state_decoded =
                  xgu_radar1_obj14_b_radar1_obj14_moving_state_decode(r1_obj14_b.radar1_obj14_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_moving_state_is_in_range(r1_obj14_b.radar1_obj14_moving_state);
              dx_sigma_decoded = xgu_radar1_obj14_b_radar1_obj14_dx_sigma_decode(r1_obj14_b.radar1_obj14_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_dx_sigma_is_in_range(r1_obj14_b.radar1_obj14_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj14_b_radar1_obj14_vx_sigma_decode(r1_obj14_b.radar1_obj14_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_vx_sigma_is_in_range(r1_obj14_b.radar1_obj14_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj14_b_radar1_obj14_ax_sigma_decode(r1_obj14_b.radar1_obj14_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_ax_sigma_is_in_range(r1_obj14_b.radar1_obj14_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj14_b_radar1_obj14_dy_sigma_decode(r1_obj14_b.radar1_obj14_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_dy_sigma_is_in_range(r1_obj14_b.radar1_obj14_dy_sigma);
              w_class_decoded = xgu_radar1_obj14_b_radar1_obj14_w_class_decode(r1_obj14_b.radar1_obj14_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj14_b_radar1_obj14_w_class_is_in_range(r1_obj14_b.radar1_obj14_w_class);
              class_decoded = xgu_radar1_obj14_b_radar1_obj14_class_decode(r1_obj14_b.radar1_obj14_class);
              class_is_in_range = xgu_radar1_obj14_b_radar1_obj14_class_is_in_range(r1_obj14_b.radar1_obj14_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj14_b_radar1_obj14_dx_rear_end_loss_decode(r1_obj14_b.radar1_obj14_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj14_b_radar1_obj14_dx_rear_end_loss_is_in_range(
                  r1_obj14_b.radar1_obj14_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj14_b_radar1_obj14_mess_bconsist_bit_encode(r1_obj14_b.radar1_obj14_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj14_b_radar1_obj14_mess_bconsist_bit_is_in_range(
                  r1_obj14_b.radar1_obj14_mess_bconsist_bit);
              break;
            case 1436:
              xgu_radar1_obj15_b_t r1_obj15_b;
              vy_decoded = xgu_radar1_obj15_b_radar1_obj15_vy_decode(r1_obj15_b.radar1_obj15_vy);
              vy_is_in_range = xgu_radar1_obj15_b_radar1_obj15_vy_is_in_range(r1_obj15_b.radar1_obj15_vy);
              d_length_decoded = xgu_radar1_obj15_b_radar1_obj15_d_length_decode(r1_obj15_b.radar1_obj15_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_d_length_is_in_range(r1_obj15_b.radar1_obj15_d_length);
              dz_decoded = xgu_radar1_obj15_b_radar1_obj15_dz_decode(r1_obj15_b.radar1_obj15_dz);
              dz_is_in_range = xgu_radar1_obj15_b_radar1_obj15_dz_is_in_range(r1_obj15_b.radar1_obj15_dz);
              moving_state_decoded =
                  xgu_radar1_obj15_b_radar1_obj15_moving_state_decode(r1_obj15_b.radar1_obj15_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_moving_state_is_in_range(r1_obj15_b.radar1_obj15_moving_state);
              dx_sigma_decoded = xgu_radar1_obj15_b_radar1_obj15_dx_sigma_decode(r1_obj15_b.radar1_obj15_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_dx_sigma_is_in_range(r1_obj15_b.radar1_obj15_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj15_b_radar1_obj15_vx_sigma_decode(r1_obj15_b.radar1_obj15_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_vx_sigma_is_in_range(r1_obj15_b.radar1_obj15_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj15_b_radar1_obj15_ax_sigma_decode(r1_obj15_b.radar1_obj15_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_ax_sigma_is_in_range(r1_obj15_b.radar1_obj15_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj15_b_radar1_obj15_dy_sigma_decode(r1_obj15_b.radar1_obj15_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_dy_sigma_is_in_range(r1_obj15_b.radar1_obj15_dy_sigma);
              w_class_decoded = xgu_radar1_obj15_b_radar1_obj15_w_class_decode(r1_obj15_b.radar1_obj15_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj15_b_radar1_obj15_w_class_is_in_range(r1_obj15_b.radar1_obj15_w_class);
              class_decoded = xgu_radar1_obj15_b_radar1_obj15_class_decode(r1_obj15_b.radar1_obj15_class);
              class_is_in_range = xgu_radar1_obj15_b_radar1_obj15_class_is_in_range(r1_obj15_b.radar1_obj15_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj15_b_radar1_obj15_dx_rear_end_loss_decode(r1_obj15_b.radar1_obj15_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj15_b_radar1_obj15_dx_rear_end_loss_is_in_range(
                  r1_obj15_b.radar1_obj15_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj15_b_radar1_obj15_mess_bconsist_bit_encode(r1_obj15_b.radar1_obj15_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj15_b_radar1_obj15_mess_bconsist_bit_is_in_range(
                  r1_obj15_b.radar1_obj15_mess_bconsist_bit);
              break;
            case 1446:
              xgu_radar1_obj16_b_t r1_obj16_b;
              vy_decoded = xgu_radar1_obj16_b_radar1_obj16_vy_decode(r1_obj16_b.radar1_obj16_vy);
              vy_is_in_range = xgu_radar1_obj16_b_radar1_obj16_vy_is_in_range(r1_obj16_b.radar1_obj16_vy);
              d_length_decoded = xgu_radar1_obj16_b_radar1_obj16_d_length_decode(r1_obj16_b.radar1_obj16_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_d_length_is_in_range(r1_obj16_b.radar1_obj16_d_length);
              dz_decoded = xgu_radar1_obj16_b_radar1_obj16_dz_decode(r1_obj16_b.radar1_obj16_dz);
              dz_is_in_range = xgu_radar1_obj16_b_radar1_obj16_dz_is_in_range(r1_obj16_b.radar1_obj16_dz);
              moving_state_decoded =
                  xgu_radar1_obj16_b_radar1_obj16_moving_state_decode(r1_obj16_b.radar1_obj16_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_moving_state_is_in_range(r1_obj16_b.radar1_obj16_moving_state);
              dx_sigma_decoded = xgu_radar1_obj16_b_radar1_obj16_dx_sigma_decode(r1_obj16_b.radar1_obj16_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_dx_sigma_is_in_range(r1_obj16_b.radar1_obj16_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj16_b_radar1_obj16_vx_sigma_decode(r1_obj16_b.radar1_obj16_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_vx_sigma_is_in_range(r1_obj16_b.radar1_obj16_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj16_b_radar1_obj16_ax_sigma_decode(r1_obj16_b.radar1_obj16_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_ax_sigma_is_in_range(r1_obj16_b.radar1_obj16_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj16_b_radar1_obj16_dy_sigma_decode(r1_obj16_b.radar1_obj16_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_dy_sigma_is_in_range(r1_obj16_b.radar1_obj16_dy_sigma);
              w_class_decoded = xgu_radar1_obj16_b_radar1_obj16_w_class_decode(r1_obj16_b.radar1_obj16_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj16_b_radar1_obj16_w_class_is_in_range(r1_obj16_b.radar1_obj16_w_class);
              class_decoded = xgu_radar1_obj16_b_radar1_obj16_class_decode(r1_obj16_b.radar1_obj16_class);
              class_is_in_range = xgu_radar1_obj16_b_radar1_obj16_class_is_in_range(r1_obj16_b.radar1_obj16_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj16_b_radar1_obj16_dx_rear_end_loss_decode(r1_obj16_b.radar1_obj16_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj16_b_radar1_obj16_dx_rear_end_loss_is_in_range(
                  r1_obj16_b.radar1_obj16_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj16_b_radar1_obj16_mess_bconsist_bit_encode(r1_obj16_b.radar1_obj16_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj16_b_radar1_obj16_mess_bconsist_bit_is_in_range(
                  r1_obj16_b.radar1_obj16_mess_bconsist_bit);
              break;
            case 1456:
              xgu_radar1_obj17_b_t r1_obj17_b;
              vy_decoded = xgu_radar1_obj17_b_radar1_obj17_vy_decode(r1_obj17_b.radar1_obj17_vy);
              vy_is_in_range = xgu_radar1_obj17_b_radar1_obj17_vy_is_in_range(r1_obj17_b.radar1_obj17_vy);
              d_length_decoded = xgu_radar1_obj17_b_radar1_obj17_d_length_decode(r1_obj17_b.radar1_obj17_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_d_length_is_in_range(r1_obj17_b.radar1_obj17_d_length);
              dz_decoded = xgu_radar1_obj17_b_radar1_obj17_dz_decode(r1_obj17_b.radar1_obj17_dz);
              dz_is_in_range = xgu_radar1_obj17_b_radar1_obj17_dz_is_in_range(r1_obj17_b.radar1_obj17_dz);
              moving_state_decoded =
                  xgu_radar1_obj17_b_radar1_obj17_moving_state_decode(r1_obj17_b.radar1_obj17_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_moving_state_is_in_range(r1_obj17_b.radar1_obj17_moving_state);
              dx_sigma_decoded = xgu_radar1_obj17_b_radar1_obj17_dx_sigma_decode(r1_obj17_b.radar1_obj17_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_dx_sigma_is_in_range(r1_obj17_b.radar1_obj17_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj17_b_radar1_obj17_vx_sigma_decode(r1_obj17_b.radar1_obj17_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_vx_sigma_is_in_range(r1_obj17_b.radar1_obj17_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj17_b_radar1_obj17_ax_sigma_decode(r1_obj17_b.radar1_obj17_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_ax_sigma_is_in_range(r1_obj17_b.radar1_obj17_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj17_b_radar1_obj17_dy_sigma_decode(r1_obj17_b.radar1_obj17_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_dy_sigma_is_in_range(r1_obj17_b.radar1_obj17_dy_sigma);
              w_class_decoded = xgu_radar1_obj17_b_radar1_obj17_w_class_decode(r1_obj17_b.radar1_obj17_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj17_b_radar1_obj17_w_class_is_in_range(r1_obj17_b.radar1_obj17_w_class);
              class_decoded = xgu_radar1_obj17_b_radar1_obj17_class_decode(r1_obj17_b.radar1_obj17_class);
              class_is_in_range = xgu_radar1_obj17_b_radar1_obj17_class_is_in_range(r1_obj17_b.radar1_obj17_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj17_b_radar1_obj17_dx_rear_end_loss_decode(r1_obj17_b.radar1_obj17_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj17_b_radar1_obj17_dx_rear_end_loss_is_in_range(
                  r1_obj17_b.radar1_obj17_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj17_b_radar1_obj17_mess_bconsist_bit_encode(r1_obj17_b.radar1_obj17_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj17_b_radar1_obj17_mess_bconsist_bit_is_in_range(
                  r1_obj17_b.radar1_obj17_mess_bconsist_bit);
              break;
            case 1466:
              xgu_radar1_obj18_b_t r1_obj18_b;
              vy_decoded = xgu_radar1_obj18_b_radar1_obj18_vy_decode(r1_obj18_b.radar1_obj18_vy);
              vy_is_in_range = xgu_radar1_obj18_b_radar1_obj18_vy_is_in_range(r1_obj18_b.radar1_obj18_vy);
              d_length_decoded = xgu_radar1_obj18_b_radar1_obj18_d_length_decode(r1_obj18_b.radar1_obj18_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_d_length_is_in_range(r1_obj18_b.radar1_obj18_d_length);
              dz_decoded = xgu_radar1_obj18_b_radar1_obj18_dz_decode(r1_obj18_b.radar1_obj18_dz);
              dz_is_in_range = xgu_radar1_obj18_b_radar1_obj18_dz_is_in_range(r1_obj18_b.radar1_obj18_dz);
              moving_state_decoded =
                  xgu_radar1_obj18_b_radar1_obj18_moving_state_decode(r1_obj18_b.radar1_obj18_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_moving_state_is_in_range(r1_obj18_b.radar1_obj18_moving_state);
              dx_sigma_decoded = xgu_radar1_obj18_b_radar1_obj18_dx_sigma_decode(r1_obj18_b.radar1_obj18_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_dx_sigma_is_in_range(r1_obj18_b.radar1_obj18_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj18_b_radar1_obj18_vx_sigma_decode(r1_obj18_b.radar1_obj18_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_vx_sigma_is_in_range(r1_obj18_b.radar1_obj18_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj18_b_radar1_obj18_ax_sigma_decode(r1_obj18_b.radar1_obj18_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_ax_sigma_is_in_range(r1_obj18_b.radar1_obj18_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj18_b_radar1_obj18_dy_sigma_decode(r1_obj18_b.radar1_obj18_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_dy_sigma_is_in_range(r1_obj18_b.radar1_obj18_dy_sigma);
              w_class_decoded = xgu_radar1_obj18_b_radar1_obj18_w_class_decode(r1_obj18_b.radar1_obj18_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj18_b_radar1_obj18_w_class_is_in_range(r1_obj18_b.radar1_obj18_w_class);
              class_decoded = xgu_radar1_obj18_b_radar1_obj18_class_decode(r1_obj18_b.radar1_obj18_class);
              class_is_in_range = xgu_radar1_obj18_b_radar1_obj18_class_is_in_range(r1_obj18_b.radar1_obj18_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj18_b_radar1_obj18_dx_rear_end_loss_decode(r1_obj18_b.radar1_obj18_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj18_b_radar1_obj18_dx_rear_end_loss_is_in_range(
                  r1_obj18_b.radar1_obj18_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj18_b_radar1_obj18_mess_bconsist_bit_encode(r1_obj18_b.radar1_obj18_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj18_b_radar1_obj18_mess_bconsist_bit_is_in_range(
                  r1_obj18_b.radar1_obj18_mess_bconsist_bit);
              break;
            case 1476:
              xgu_radar1_obj19_b_t r1_obj19_b;
              vy_decoded = xgu_radar1_obj19_b_radar1_obj19_vy_decode(r1_obj19_b.radar1_obj19_vy);
              vy_is_in_range = xgu_radar1_obj19_b_radar1_obj19_vy_is_in_range(r1_obj19_b.radar1_obj19_vy);
              d_length_decoded = xgu_radar1_obj19_b_radar1_obj19_d_length_decode(r1_obj19_b.radar1_obj19_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_d_length_is_in_range(r1_obj19_b.radar1_obj19_d_length);
              dz_decoded = xgu_radar1_obj19_b_radar1_obj19_dz_decode(r1_obj19_b.radar1_obj19_dz);
              dz_is_in_range = xgu_radar1_obj19_b_radar1_obj19_dz_is_in_range(r1_obj19_b.radar1_obj19_dz);
              moving_state_decoded =
                  xgu_radar1_obj19_b_radar1_obj19_moving_state_decode(r1_obj19_b.radar1_obj19_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_moving_state_is_in_range(r1_obj19_b.radar1_obj19_moving_state);
              dx_sigma_decoded = xgu_radar1_obj19_b_radar1_obj19_dx_sigma_decode(r1_obj19_b.radar1_obj19_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_dx_sigma_is_in_range(r1_obj19_b.radar1_obj19_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj19_b_radar1_obj19_vx_sigma_decode(r1_obj19_b.radar1_obj19_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_vx_sigma_is_in_range(r1_obj19_b.radar1_obj19_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj19_b_radar1_obj19_ax_sigma_decode(r1_obj19_b.radar1_obj19_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_ax_sigma_is_in_range(r1_obj19_b.radar1_obj19_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj19_b_radar1_obj19_dy_sigma_decode(r1_obj19_b.radar1_obj19_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_dy_sigma_is_in_range(r1_obj19_b.radar1_obj19_dy_sigma);
              w_class_decoded = xgu_radar1_obj19_b_radar1_obj19_w_class_decode(r1_obj19_b.radar1_obj19_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj19_b_radar1_obj19_w_class_is_in_range(r1_obj19_b.radar1_obj19_w_class);
              class_decoded = xgu_radar1_obj19_b_radar1_obj19_class_decode(r1_obj19_b.radar1_obj19_class);
              class_is_in_range = xgu_radar1_obj19_b_radar1_obj19_class_is_in_range(r1_obj19_b.radar1_obj19_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj19_b_radar1_obj19_dx_rear_end_loss_decode(r1_obj19_b.radar1_obj19_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj19_b_radar1_obj19_dx_rear_end_loss_is_in_range(
                  r1_obj19_b.radar1_obj19_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj19_b_radar1_obj19_mess_bconsist_bit_encode(r1_obj19_b.radar1_obj19_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj19_b_radar1_obj19_mess_bconsist_bit_is_in_range(
                  r1_obj19_b.radar1_obj19_mess_bconsist_bit);
              break;
            case 1486:
              xgu_radar1_obj20_b_t r1_obj20_b;
              vy_decoded = xgu_radar1_obj20_b_radar1_obj20_vy_decode(r1_obj20_b.radar1_obj20_vy);
              vy_is_in_range = xgu_radar1_obj20_b_radar1_obj20_vy_is_in_range(r1_obj20_b.radar1_obj20_vy);
              d_length_decoded = xgu_radar1_obj20_b_radar1_obj20_d_length_decode(r1_obj20_b.radar1_obj20_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_d_length_is_in_range(r1_obj20_b.radar1_obj20_d_length);
              dz_decoded = xgu_radar1_obj20_b_radar1_obj20_dz_decode(r1_obj20_b.radar1_obj20_dz);
              dz_is_in_range = xgu_radar1_obj20_b_radar1_obj20_dz_is_in_range(r1_obj20_b.radar1_obj20_dz);
              moving_state_decoded =
                  xgu_radar1_obj20_b_radar1_obj20_moving_state_decode(r1_obj20_b.radar1_obj20_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_moving_state_is_in_range(r1_obj20_b.radar1_obj20_moving_state);
              dx_sigma_decoded = xgu_radar1_obj20_b_radar1_obj20_dx_sigma_decode(r1_obj20_b.radar1_obj20_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_dx_sigma_is_in_range(r1_obj20_b.radar1_obj20_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj20_b_radar1_obj20_vx_sigma_decode(r1_obj20_b.radar1_obj20_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_vx_sigma_is_in_range(r1_obj20_b.radar1_obj20_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj20_b_radar1_obj20_ax_sigma_decode(r1_obj20_b.radar1_obj20_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_ax_sigma_is_in_range(r1_obj20_b.radar1_obj20_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj20_b_radar1_obj20_dy_sigma_decode(r1_obj20_b.radar1_obj20_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_dy_sigma_is_in_range(r1_obj20_b.radar1_obj20_dy_sigma);
              w_class_decoded = xgu_radar1_obj20_b_radar1_obj20_w_class_decode(r1_obj20_b.radar1_obj20_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj20_b_radar1_obj20_w_class_is_in_range(r1_obj20_b.radar1_obj20_w_class);
              class_decoded = xgu_radar1_obj20_b_radar1_obj20_class_decode(r1_obj20_b.radar1_obj20_class);
              class_is_in_range = xgu_radar1_obj20_b_radar1_obj20_class_is_in_range(r1_obj20_b.radar1_obj20_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj20_b_radar1_obj20_dx_rear_end_loss_decode(r1_obj20_b.radar1_obj20_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj20_b_radar1_obj20_dx_rear_end_loss_is_in_range(
                  r1_obj20_b.radar1_obj20_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj20_b_radar1_obj20_mess_bconsist_bit_encode(r1_obj20_b.radar1_obj20_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj20_b_radar1_obj20_mess_bconsist_bit_is_in_range(
                  r1_obj20_b.radar1_obj20_mess_bconsist_bit);
              break;
            case 1496:
              xgu_radar1_obj21_b_t r1_obj21_b;
              vy_decoded = xgu_radar1_obj21_b_radar1_obj21_vy_decode(r1_obj21_b.radar1_obj21_vy);
              vy_is_in_range = xgu_radar1_obj21_b_radar1_obj21_vy_is_in_range(r1_obj21_b.radar1_obj21_vy);
              d_length_decoded = xgu_radar1_obj21_b_radar1_obj21_d_length_decode(r1_obj21_b.radar1_obj21_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_d_length_is_in_range(r1_obj21_b.radar1_obj21_d_length);
              dz_decoded = xgu_radar1_obj21_b_radar1_obj21_dz_decode(r1_obj21_b.radar1_obj21_dz);
              dz_is_in_range = xgu_radar1_obj21_b_radar1_obj21_dz_is_in_range(r1_obj21_b.radar1_obj21_dz);
              moving_state_decoded =
                  xgu_radar1_obj21_b_radar1_obj21_moving_state_decode(r1_obj21_b.radar1_obj21_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_moving_state_is_in_range(r1_obj21_b.radar1_obj21_moving_state);
              dx_sigma_decoded = xgu_radar1_obj21_b_radar1_obj21_dx_sigma_decode(r1_obj21_b.radar1_obj21_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_dx_sigma_is_in_range(r1_obj21_b.radar1_obj21_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj21_b_radar1_obj21_vx_sigma_decode(r1_obj21_b.radar1_obj21_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_vx_sigma_is_in_range(r1_obj21_b.radar1_obj21_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj21_b_radar1_obj21_ax_sigma_decode(r1_obj21_b.radar1_obj21_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_ax_sigma_is_in_range(r1_obj21_b.radar1_obj21_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj21_b_radar1_obj21_dy_sigma_decode(r1_obj21_b.radar1_obj21_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_dy_sigma_is_in_range(r1_obj21_b.radar1_obj21_dy_sigma);
              w_class_decoded = xgu_radar1_obj21_b_radar1_obj21_w_class_decode(r1_obj21_b.radar1_obj21_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj21_b_radar1_obj21_w_class_is_in_range(r1_obj21_b.radar1_obj21_w_class);
              class_decoded = xgu_radar1_obj21_b_radar1_obj21_class_decode(r1_obj21_b.radar1_obj21_class);
              class_is_in_range = xgu_radar1_obj21_b_radar1_obj21_class_is_in_range(r1_obj21_b.radar1_obj21_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj21_b_radar1_obj21_dx_rear_end_loss_decode(r1_obj21_b.radar1_obj21_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj21_b_radar1_obj21_dx_rear_end_loss_is_in_range(
                  r1_obj21_b.radar1_obj21_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj21_b_radar1_obj21_mess_bconsist_bit_encode(r1_obj21_b.radar1_obj21_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj21_b_radar1_obj21_mess_bconsist_bit_is_in_range(
                  r1_obj21_b.radar1_obj21_mess_bconsist_bit);
              break;
            case 1506:
              xgu_radar1_obj22_b_t r1_obj22_b;
              vy_decoded = xgu_radar1_obj22_b_radar1_obj22_vy_decode(r1_obj22_b.radar1_obj22_vy);
              vy_is_in_range = xgu_radar1_obj22_b_radar1_obj22_vy_is_in_range(r1_obj22_b.radar1_obj22_vy);
              d_length_decoded = xgu_radar1_obj22_b_radar1_obj22_d_length_decode(r1_obj22_b.radar1_obj22_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_d_length_is_in_range(r1_obj22_b.radar1_obj22_d_length);
              dz_decoded = xgu_radar1_obj22_b_radar1_obj22_dz_decode(r1_obj22_b.radar1_obj22_dz);
              dz_is_in_range = xgu_radar1_obj22_b_radar1_obj22_dz_is_in_range(r1_obj22_b.radar1_obj22_dz);
              moving_state_decoded =
                  xgu_radar1_obj22_b_radar1_obj22_moving_state_decode(r1_obj22_b.radar1_obj22_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_moving_state_is_in_range(r1_obj22_b.radar1_obj22_moving_state);
              dx_sigma_decoded = xgu_radar1_obj22_b_radar1_obj22_dx_sigma_decode(r1_obj22_b.radar1_obj22_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_dx_sigma_is_in_range(r1_obj22_b.radar1_obj22_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj22_b_radar1_obj22_vx_sigma_decode(r1_obj22_b.radar1_obj22_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_vx_sigma_is_in_range(r1_obj22_b.radar1_obj22_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj22_b_radar1_obj22_ax_sigma_decode(r1_obj22_b.radar1_obj22_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_ax_sigma_is_in_range(r1_obj22_b.radar1_obj22_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj22_b_radar1_obj22_dy_sigma_decode(r1_obj22_b.radar1_obj22_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_dy_sigma_is_in_range(r1_obj22_b.radar1_obj22_dy_sigma);
              w_class_decoded = xgu_radar1_obj22_b_radar1_obj22_w_class_decode(r1_obj22_b.radar1_obj22_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj22_b_radar1_obj22_w_class_is_in_range(r1_obj22_b.radar1_obj22_w_class);
              class_decoded = xgu_radar1_obj22_b_radar1_obj22_class_decode(r1_obj22_b.radar1_obj22_class);
              class_is_in_range = xgu_radar1_obj22_b_radar1_obj22_class_is_in_range(r1_obj22_b.radar1_obj22_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj22_b_radar1_obj22_dx_rear_end_loss_decode(r1_obj22_b.radar1_obj22_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj22_b_radar1_obj22_dx_rear_end_loss_is_in_range(
                  r1_obj22_b.radar1_obj22_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj22_b_radar1_obj22_mess_bconsist_bit_encode(r1_obj22_b.radar1_obj22_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj22_b_radar1_obj22_mess_bconsist_bit_is_in_range(
                  r1_obj22_b.radar1_obj22_mess_bconsist_bit);
              break;
            case 1516:
              xgu_radar1_obj23_b_t r1_obj23_b;
              vy_decoded = xgu_radar1_obj23_b_radar1_obj23_vy_decode(r1_obj23_b.radar1_obj23_vy);
              vy_is_in_range = xgu_radar1_obj23_b_radar1_obj23_vy_is_in_range(r1_obj23_b.radar1_obj23_vy);
              d_length_decoded = xgu_radar1_obj23_b_radar1_obj23_d_length_decode(r1_obj23_b.radar1_obj23_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_d_length_is_in_range(r1_obj23_b.radar1_obj23_d_length);
              dz_decoded = xgu_radar1_obj23_b_radar1_obj23_dz_decode(r1_obj23_b.radar1_obj23_dz);
              dz_is_in_range = xgu_radar1_obj23_b_radar1_obj23_dz_is_in_range(r1_obj23_b.radar1_obj23_dz);
              moving_state_decoded =
                  xgu_radar1_obj23_b_radar1_obj23_moving_state_decode(r1_obj23_b.radar1_obj23_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_moving_state_is_in_range(r1_obj23_b.radar1_obj23_moving_state);
              dx_sigma_decoded = xgu_radar1_obj23_b_radar1_obj23_dx_sigma_decode(r1_obj23_b.radar1_obj23_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_dx_sigma_is_in_range(r1_obj23_b.radar1_obj23_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj23_b_radar1_obj23_vx_sigma_decode(r1_obj23_b.radar1_obj23_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_vx_sigma_is_in_range(r1_obj23_b.radar1_obj23_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj23_b_radar1_obj23_ax_sigma_decode(r1_obj23_b.radar1_obj23_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_ax_sigma_is_in_range(r1_obj23_b.radar1_obj23_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj23_b_radar1_obj23_dy_sigma_decode(r1_obj23_b.radar1_obj23_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_dy_sigma_is_in_range(r1_obj23_b.radar1_obj23_dy_sigma);
              w_class_decoded = xgu_radar1_obj23_b_radar1_obj23_w_class_decode(r1_obj23_b.radar1_obj23_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj23_b_radar1_obj23_w_class_is_in_range(r1_obj23_b.radar1_obj23_w_class);
              class_decoded = xgu_radar1_obj23_b_radar1_obj23_class_decode(r1_obj23_b.radar1_obj23_class);
              class_is_in_range = xgu_radar1_obj23_b_radar1_obj23_class_is_in_range(r1_obj23_b.radar1_obj23_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj23_b_radar1_obj23_dx_rear_end_loss_decode(r1_obj23_b.radar1_obj23_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj23_b_radar1_obj23_dx_rear_end_loss_is_in_range(
                  r1_obj23_b.radar1_obj23_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj23_b_radar1_obj23_mess_bconsist_bit_encode(r1_obj23_b.radar1_obj23_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj23_b_radar1_obj23_mess_bconsist_bit_is_in_range(
                  r1_obj23_b.radar1_obj23_mess_bconsist_bit);
              break;
            case 1526:
              xgu_radar1_obj24_b_t r1_obj24_b;
              vy_decoded = xgu_radar1_obj24_b_radar1_obj24_vy_decode(r1_obj24_b.radar1_obj24_vy);
              vy_is_in_range = xgu_radar1_obj24_b_radar1_obj24_vy_is_in_range(r1_obj24_b.radar1_obj24_vy);
              d_length_decoded = xgu_radar1_obj24_b_radar1_obj24_d_length_decode(r1_obj24_b.radar1_obj24_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_d_length_is_in_range(r1_obj24_b.radar1_obj24_d_length);
              dz_decoded = xgu_radar1_obj24_b_radar1_obj24_dz_decode(r1_obj24_b.radar1_obj24_dz);
              dz_is_in_range = xgu_radar1_obj24_b_radar1_obj24_dz_is_in_range(r1_obj24_b.radar1_obj24_dz);
              moving_state_decoded =
                  xgu_radar1_obj24_b_radar1_obj24_moving_state_decode(r1_obj24_b.radar1_obj24_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_moving_state_is_in_range(r1_obj24_b.radar1_obj24_moving_state);
              dx_sigma_decoded = xgu_radar1_obj24_b_radar1_obj24_dx_sigma_decode(r1_obj24_b.radar1_obj24_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_dx_sigma_is_in_range(r1_obj24_b.radar1_obj24_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj24_b_radar1_obj24_vx_sigma_decode(r1_obj24_b.radar1_obj24_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_vx_sigma_is_in_range(r1_obj24_b.radar1_obj24_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj24_b_radar1_obj24_ax_sigma_decode(r1_obj24_b.radar1_obj24_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_ax_sigma_is_in_range(r1_obj24_b.radar1_obj24_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj24_b_radar1_obj24_dy_sigma_decode(r1_obj24_b.radar1_obj24_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_dy_sigma_is_in_range(r1_obj24_b.radar1_obj24_dy_sigma);
              w_class_decoded = xgu_radar1_obj24_b_radar1_obj24_w_class_decode(r1_obj24_b.radar1_obj24_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj24_b_radar1_obj24_w_class_is_in_range(r1_obj24_b.radar1_obj24_w_class);
              class_decoded = xgu_radar1_obj24_b_radar1_obj24_class_decode(r1_obj24_b.radar1_obj24_class);
              class_is_in_range = xgu_radar1_obj24_b_radar1_obj24_class_is_in_range(r1_obj24_b.radar1_obj24_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj24_b_radar1_obj24_dx_rear_end_loss_decode(r1_obj24_b.radar1_obj24_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj24_b_radar1_obj24_dx_rear_end_loss_is_in_range(
                  r1_obj24_b.radar1_obj24_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj24_b_radar1_obj24_mess_bconsist_bit_encode(r1_obj24_b.radar1_obj24_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj24_b_radar1_obj24_mess_bconsist_bit_is_in_range(
                  r1_obj24_b.radar1_obj24_mess_bconsist_bit);
              break;
            case 1536:
              xgu_radar1_obj25_b_t r1_obj25_b;
              vy_decoded = xgu_radar1_obj25_b_radar1_obj25_vy_decode(r1_obj25_b.radar1_obj25_vy);
              vy_is_in_range = xgu_radar1_obj25_b_radar1_obj25_vy_is_in_range(r1_obj25_b.radar1_obj25_vy);
              d_length_decoded = xgu_radar1_obj25_b_radar1_obj25_d_length_decode(r1_obj25_b.radar1_obj25_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_d_length_is_in_range(r1_obj25_b.radar1_obj25_d_length);
              dz_decoded = xgu_radar1_obj25_b_radar1_obj25_dz_decode(r1_obj25_b.radar1_obj25_dz);
              dz_is_in_range = xgu_radar1_obj25_b_radar1_obj25_dz_is_in_range(r1_obj25_b.radar1_obj25_dz);
              moving_state_decoded =
                  xgu_radar1_obj25_b_radar1_obj25_moving_state_decode(r1_obj25_b.radar1_obj25_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_moving_state_is_in_range(r1_obj25_b.radar1_obj25_moving_state);
              dx_sigma_decoded = xgu_radar1_obj25_b_radar1_obj25_dx_sigma_decode(r1_obj25_b.radar1_obj25_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_dx_sigma_is_in_range(r1_obj25_b.radar1_obj25_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj25_b_radar1_obj25_vx_sigma_decode(r1_obj25_b.radar1_obj25_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_vx_sigma_is_in_range(r1_obj25_b.radar1_obj25_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj25_b_radar1_obj25_ax_sigma_decode(r1_obj25_b.radar1_obj25_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_ax_sigma_is_in_range(r1_obj25_b.radar1_obj25_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj25_b_radar1_obj25_dy_sigma_decode(r1_obj25_b.radar1_obj25_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_dy_sigma_is_in_range(r1_obj25_b.radar1_obj25_dy_sigma);
              w_class_decoded = xgu_radar1_obj25_b_radar1_obj25_w_class_decode(r1_obj25_b.radar1_obj25_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj25_b_radar1_obj25_w_class_is_in_range(r1_obj25_b.radar1_obj25_w_class);
              class_decoded = xgu_radar1_obj25_b_radar1_obj25_class_decode(r1_obj25_b.radar1_obj25_class);
              class_is_in_range = xgu_radar1_obj25_b_radar1_obj25_class_is_in_range(r1_obj25_b.radar1_obj25_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj25_b_radar1_obj25_dx_rear_end_loss_decode(r1_obj25_b.radar1_obj25_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj25_b_radar1_obj25_dx_rear_end_loss_is_in_range(
                  r1_obj25_b.radar1_obj25_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj25_b_radar1_obj25_mess_bconsist_bit_encode(r1_obj25_b.radar1_obj25_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj25_b_radar1_obj25_mess_bconsist_bit_is_in_range(
                  r1_obj25_b.radar1_obj25_mess_bconsist_bit);
              break;
            case 1546:
              xgu_radar1_obj26_b_t r1_obj26_b;
              vy_decoded = xgu_radar1_obj26_b_radar1_obj26_vy_decode(r1_obj26_b.radar1_obj26_vy);
              vy_is_in_range = xgu_radar1_obj26_b_radar1_obj26_vy_is_in_range(r1_obj26_b.radar1_obj26_vy);
              d_length_decoded = xgu_radar1_obj26_b_radar1_obj26_d_length_decode(r1_obj26_b.radar1_obj26_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_d_length_is_in_range(r1_obj26_b.radar1_obj26_d_length);
              dz_decoded = xgu_radar1_obj26_b_radar1_obj26_dz_decode(r1_obj26_b.radar1_obj26_dz);
              dz_is_in_range = xgu_radar1_obj26_b_radar1_obj26_dz_is_in_range(r1_obj26_b.radar1_obj26_dz);
              moving_state_decoded =
                  xgu_radar1_obj26_b_radar1_obj26_moving_state_decode(r1_obj26_b.radar1_obj26_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_moving_state_is_in_range(r1_obj26_b.radar1_obj26_moving_state);
              dx_sigma_decoded = xgu_radar1_obj26_b_radar1_obj26_dx_sigma_decode(r1_obj26_b.radar1_obj26_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_dx_sigma_is_in_range(r1_obj26_b.radar1_obj26_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj26_b_radar1_obj26_vx_sigma_decode(r1_obj26_b.radar1_obj26_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_vx_sigma_is_in_range(r1_obj26_b.radar1_obj26_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj26_b_radar1_obj26_ax_sigma_decode(r1_obj26_b.radar1_obj26_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_ax_sigma_is_in_range(r1_obj26_b.radar1_obj26_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj26_b_radar1_obj26_dy_sigma_decode(r1_obj26_b.radar1_obj26_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_dy_sigma_is_in_range(r1_obj26_b.radar1_obj26_dy_sigma);
              w_class_decoded = xgu_radar1_obj26_b_radar1_obj26_w_class_decode(r1_obj26_b.radar1_obj26_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj26_b_radar1_obj26_w_class_is_in_range(r1_obj26_b.radar1_obj26_w_class);
              class_decoded = xgu_radar1_obj26_b_radar1_obj26_class_decode(r1_obj26_b.radar1_obj26_class);
              class_is_in_range = xgu_radar1_obj26_b_radar1_obj26_class_is_in_range(r1_obj26_b.radar1_obj26_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj26_b_radar1_obj26_dx_rear_end_loss_decode(r1_obj26_b.radar1_obj26_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj26_b_radar1_obj26_dx_rear_end_loss_is_in_range(
                  r1_obj26_b.radar1_obj26_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj26_b_radar1_obj26_mess_bconsist_bit_encode(r1_obj26_b.radar1_obj26_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj26_b_radar1_obj26_mess_bconsist_bit_is_in_range(
                  r1_obj26_b.radar1_obj26_mess_bconsist_bit);
              break;
            case 1556:
              xgu_radar1_obj27_b_t r1_obj27_b;
              vy_decoded = xgu_radar1_obj27_b_radar1_obj27_vy_decode(r1_obj27_b.radar1_obj27_vy);
              vy_is_in_range = xgu_radar1_obj27_b_radar1_obj27_vy_is_in_range(r1_obj27_b.radar1_obj27_vy);
              d_length_decoded = xgu_radar1_obj27_b_radar1_obj27_d_length_decode(r1_obj27_b.radar1_obj27_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_d_length_is_in_range(r1_obj27_b.radar1_obj27_d_length);
              dz_decoded = xgu_radar1_obj27_b_radar1_obj27_dz_decode(r1_obj27_b.radar1_obj27_dz);
              dz_is_in_range = xgu_radar1_obj27_b_radar1_obj27_dz_is_in_range(r1_obj27_b.radar1_obj27_dz);
              moving_state_decoded =
                  xgu_radar1_obj27_b_radar1_obj27_moving_state_decode(r1_obj27_b.radar1_obj27_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_moving_state_is_in_range(r1_obj27_b.radar1_obj27_moving_state);
              dx_sigma_decoded = xgu_radar1_obj27_b_radar1_obj27_dx_sigma_decode(r1_obj27_b.radar1_obj27_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_dx_sigma_is_in_range(r1_obj27_b.radar1_obj27_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj27_b_radar1_obj27_vx_sigma_decode(r1_obj27_b.radar1_obj27_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_vx_sigma_is_in_range(r1_obj27_b.radar1_obj27_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj27_b_radar1_obj27_ax_sigma_decode(r1_obj27_b.radar1_obj27_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_ax_sigma_is_in_range(r1_obj27_b.radar1_obj27_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj27_b_radar1_obj27_dy_sigma_decode(r1_obj27_b.radar1_obj27_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_dy_sigma_is_in_range(r1_obj27_b.radar1_obj27_dy_sigma);
              w_class_decoded = xgu_radar1_obj27_b_radar1_obj27_w_class_decode(r1_obj27_b.radar1_obj27_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj27_b_radar1_obj27_w_class_is_in_range(r1_obj27_b.radar1_obj27_w_class);
              class_decoded = xgu_radar1_obj27_b_radar1_obj27_class_decode(r1_obj27_b.radar1_obj27_class);
              class_is_in_range = xgu_radar1_obj27_b_radar1_obj27_class_is_in_range(r1_obj27_b.radar1_obj27_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj27_b_radar1_obj27_dx_rear_end_loss_decode(r1_obj27_b.radar1_obj27_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj27_b_radar1_obj27_dx_rear_end_loss_is_in_range(
                  r1_obj27_b.radar1_obj27_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj27_b_radar1_obj27_mess_bconsist_bit_encode(r1_obj27_b.radar1_obj27_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj27_b_radar1_obj27_mess_bconsist_bit_is_in_range(
                  r1_obj27_b.radar1_obj27_mess_bconsist_bit);
              break;
            case 1566:
              xgu_radar1_obj28_b_t r1_obj28_b;
              vy_decoded = xgu_radar1_obj28_b_radar1_obj28_vy_decode(r1_obj28_b.radar1_obj28_vy);
              vy_is_in_range = xgu_radar1_obj28_b_radar1_obj28_vy_is_in_range(r1_obj28_b.radar1_obj28_vy);
              d_length_decoded = xgu_radar1_obj28_b_radar1_obj28_d_length_decode(r1_obj28_b.radar1_obj28_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_d_length_is_in_range(r1_obj28_b.radar1_obj28_d_length);
              dz_decoded = xgu_radar1_obj28_b_radar1_obj28_dz_decode(r1_obj28_b.radar1_obj28_dz);
              dz_is_in_range = xgu_radar1_obj28_b_radar1_obj28_dz_is_in_range(r1_obj28_b.radar1_obj28_dz);
              moving_state_decoded =
                  xgu_radar1_obj28_b_radar1_obj28_moving_state_decode(r1_obj28_b.radar1_obj28_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_moving_state_is_in_range(r1_obj28_b.radar1_obj28_moving_state);
              dx_sigma_decoded = xgu_radar1_obj28_b_radar1_obj28_dx_sigma_decode(r1_obj28_b.radar1_obj28_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_dx_sigma_is_in_range(r1_obj28_b.radar1_obj28_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj28_b_radar1_obj28_vx_sigma_decode(r1_obj28_b.radar1_obj28_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_vx_sigma_is_in_range(r1_obj28_b.radar1_obj28_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj28_b_radar1_obj28_ax_sigma_decode(r1_obj28_b.radar1_obj28_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_ax_sigma_is_in_range(r1_obj28_b.radar1_obj28_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj28_b_radar1_obj28_dy_sigma_decode(r1_obj28_b.radar1_obj28_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_dy_sigma_is_in_range(r1_obj28_b.radar1_obj28_dy_sigma);
              w_class_decoded = xgu_radar1_obj28_b_radar1_obj28_w_class_decode(r1_obj28_b.radar1_obj28_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj28_b_radar1_obj28_w_class_is_in_range(r1_obj28_b.radar1_obj28_w_class);
              class_decoded = xgu_radar1_obj28_b_radar1_obj28_class_decode(r1_obj28_b.radar1_obj28_class);
              class_is_in_range = xgu_radar1_obj28_b_radar1_obj28_class_is_in_range(r1_obj28_b.radar1_obj28_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj28_b_radar1_obj28_dx_rear_end_loss_decode(r1_obj28_b.radar1_obj28_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj28_b_radar1_obj28_dx_rear_end_loss_is_in_range(
                  r1_obj28_b.radar1_obj28_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj28_b_radar1_obj28_mess_bconsist_bit_encode(r1_obj28_b.radar1_obj28_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj28_b_radar1_obj28_mess_bconsist_bit_is_in_range(
                  r1_obj28_b.radar1_obj28_mess_bconsist_bit);
              break;
            case 1576:
              xgu_radar1_obj29_b_t r1_obj29_b;
              vy_decoded = xgu_radar1_obj29_b_radar1_obj29_vy_decode(r1_obj29_b.radar1_obj29_vy);
              vy_is_in_range = xgu_radar1_obj29_b_radar1_obj29_vy_is_in_range(r1_obj29_b.radar1_obj29_vy);
              d_length_decoded = xgu_radar1_obj29_b_radar1_obj29_d_length_decode(r1_obj29_b.radar1_obj29_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_d_length_is_in_range(r1_obj29_b.radar1_obj29_d_length);
              dz_decoded = xgu_radar1_obj29_b_radar1_obj29_dz_decode(r1_obj29_b.radar1_obj29_dz);
              dz_is_in_range = xgu_radar1_obj29_b_radar1_obj29_dz_is_in_range(r1_obj29_b.radar1_obj29_dz);
              moving_state_decoded =
                  xgu_radar1_obj29_b_radar1_obj29_moving_state_decode(r1_obj29_b.radar1_obj29_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_moving_state_is_in_range(r1_obj29_b.radar1_obj29_moving_state);
              dx_sigma_decoded = xgu_radar1_obj29_b_radar1_obj29_dx_sigma_decode(r1_obj29_b.radar1_obj29_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_dx_sigma_is_in_range(r1_obj29_b.radar1_obj29_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj29_b_radar1_obj29_vx_sigma_decode(r1_obj29_b.radar1_obj29_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_vx_sigma_is_in_range(r1_obj29_b.radar1_obj29_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj29_b_radar1_obj29_ax_sigma_decode(r1_obj29_b.radar1_obj29_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_ax_sigma_is_in_range(r1_obj29_b.radar1_obj29_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj29_b_radar1_obj29_dy_sigma_decode(r1_obj29_b.radar1_obj29_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_dy_sigma_is_in_range(r1_obj29_b.radar1_obj29_dy_sigma);
              w_class_decoded = xgu_radar1_obj29_b_radar1_obj29_w_class_decode(r1_obj29_b.radar1_obj29_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj29_b_radar1_obj29_w_class_is_in_range(r1_obj29_b.radar1_obj29_w_class);
              class_decoded = xgu_radar1_obj29_b_radar1_obj29_class_decode(r1_obj29_b.radar1_obj29_class);
              class_is_in_range = xgu_radar1_obj29_b_radar1_obj29_class_is_in_range(r1_obj29_b.radar1_obj29_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj29_b_radar1_obj29_dx_rear_end_loss_decode(r1_obj29_b.radar1_obj29_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj29_b_radar1_obj29_dx_rear_end_loss_is_in_range(
                  r1_obj29_b.radar1_obj29_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj29_b_radar1_obj29_mess_bconsist_bit_encode(r1_obj29_b.radar1_obj29_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj29_b_radar1_obj29_mess_bconsist_bit_is_in_range(
                  r1_obj29_b.radar1_obj29_mess_bconsist_bit);
              break;
            case 1586:
              xgu_radar1_obj30_b_t r1_obj30_b;
              vy_decoded = xgu_radar1_obj30_b_radar1_obj30_vy_decode(r1_obj30_b.radar1_obj30_vy);
              vy_is_in_range = xgu_radar1_obj30_b_radar1_obj30_vy_is_in_range(r1_obj30_b.radar1_obj30_vy);
              d_length_decoded = xgu_radar1_obj30_b_radar1_obj30_d_length_decode(r1_obj30_b.radar1_obj30_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_d_length_is_in_range(r1_obj30_b.radar1_obj30_d_length);
              dz_decoded = xgu_radar1_obj30_b_radar1_obj30_dz_decode(r1_obj30_b.radar1_obj30_dz);
              dz_is_in_range = xgu_radar1_obj30_b_radar1_obj30_dz_is_in_range(r1_obj30_b.radar1_obj30_dz);
              moving_state_decoded =
                  xgu_radar1_obj30_b_radar1_obj30_moving_state_decode(r1_obj30_b.radar1_obj30_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_moving_state_is_in_range(r1_obj30_b.radar1_obj30_moving_state);
              dx_sigma_decoded = xgu_radar1_obj30_b_radar1_obj30_dx_sigma_decode(r1_obj30_b.radar1_obj30_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_dx_sigma_is_in_range(r1_obj30_b.radar1_obj30_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj30_b_radar1_obj30_vx_sigma_decode(r1_obj30_b.radar1_obj30_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_vx_sigma_is_in_range(r1_obj30_b.radar1_obj30_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj30_b_radar1_obj30_ax_sigma_decode(r1_obj30_b.radar1_obj30_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_ax_sigma_is_in_range(r1_obj30_b.radar1_obj30_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj30_b_radar1_obj30_dy_sigma_decode(r1_obj30_b.radar1_obj30_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_dy_sigma_is_in_range(r1_obj30_b.radar1_obj30_dy_sigma);
              w_class_decoded = xgu_radar1_obj30_b_radar1_obj30_w_class_decode(r1_obj30_b.radar1_obj30_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj30_b_radar1_obj30_w_class_is_in_range(r1_obj30_b.radar1_obj30_w_class);
              class_decoded = xgu_radar1_obj30_b_radar1_obj30_class_decode(r1_obj30_b.radar1_obj30_class);
              class_is_in_range = xgu_radar1_obj30_b_radar1_obj30_class_is_in_range(r1_obj30_b.radar1_obj30_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj30_b_radar1_obj30_dx_rear_end_loss_decode(r1_obj30_b.radar1_obj30_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj30_b_radar1_obj30_dx_rear_end_loss_is_in_range(
                  r1_obj30_b.radar1_obj30_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj30_b_radar1_obj30_mess_bconsist_bit_encode(r1_obj30_b.radar1_obj30_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj30_b_radar1_obj30_mess_bconsist_bit_is_in_range(
                  r1_obj30_b.radar1_obj30_mess_bconsist_bit);
              break;
            case 1596:
              xgu_radar1_obj31_b_t r1_obj31_b;
              vy_decoded = xgu_radar1_obj31_b_radar1_obj31_vy_decode(r1_obj31_b.radar1_obj31_vy);
              vy_is_in_range = xgu_radar1_obj31_b_radar1_obj31_vy_is_in_range(r1_obj31_b.radar1_obj31_vy);
              d_length_decoded = xgu_radar1_obj31_b_radar1_obj31_d_length_decode(r1_obj31_b.radar1_obj31_d_length);
              d_length_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_d_length_is_in_range(r1_obj31_b.radar1_obj31_d_length);
              dz_decoded = xgu_radar1_obj31_b_radar1_obj31_dz_decode(r1_obj31_b.radar1_obj31_dz);
              dz_is_in_range = xgu_radar1_obj31_b_radar1_obj31_dz_is_in_range(r1_obj31_b.radar1_obj31_dz);
              moving_state_decoded =
                  xgu_radar1_obj31_b_radar1_obj31_moving_state_decode(r1_obj31_b.radar1_obj31_moving_state);
              moving_state_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_moving_state_is_in_range(r1_obj31_b.radar1_obj31_moving_state);
              dx_sigma_decoded = xgu_radar1_obj31_b_radar1_obj31_dx_sigma_decode(r1_obj31_b.radar1_obj31_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_dx_sigma_is_in_range(r1_obj31_b.radar1_obj31_dx_sigma);
              vx_sigma_decoded = xgu_radar1_obj31_b_radar1_obj31_vx_sigma_decode(r1_obj31_b.radar1_obj31_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_vx_sigma_is_in_range(r1_obj31_b.radar1_obj31_vx_sigma);
              ax_sigma_decoded = xgu_radar1_obj31_b_radar1_obj31_ax_sigma_decode(r1_obj31_b.radar1_obj31_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_ax_sigma_is_in_range(r1_obj31_b.radar1_obj31_ax_sigma);
              dy_sigma_decoded = xgu_radar1_obj31_b_radar1_obj31_dy_sigma_decode(r1_obj31_b.radar1_obj31_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_dy_sigma_is_in_range(r1_obj31_b.radar1_obj31_dy_sigma);
              w_class_decoded = xgu_radar1_obj31_b_radar1_obj31_w_class_decode(r1_obj31_b.radar1_obj31_w_class);
              w_class_is_in_range =
                  xgu_radar1_obj31_b_radar1_obj31_w_class_is_in_range(r1_obj31_b.radar1_obj31_w_class);
              class_decoded = xgu_radar1_obj31_b_radar1_obj31_class_decode(r1_obj31_b.radar1_obj31_class);
              class_is_in_range = xgu_radar1_obj31_b_radar1_obj31_class_is_in_range(r1_obj31_b.radar1_obj31_class);
              dx_rear_end_loss_decoded =
                  xgu_radar1_obj31_b_radar1_obj31_dx_rear_end_loss_decode(r1_obj31_b.radar1_obj31_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar1_obj31_b_radar1_obj31_dx_rear_end_loss_is_in_range(
                  r1_obj31_b.radar1_obj31_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar1_obj31_b_radar1_obj31_mess_bconsist_bit_encode(r1_obj31_b.radar1_obj31_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar1_obj31_b_radar1_obj31_mess_bconsist_bit_is_in_range(
                  r1_obj31_b.radar1_obj31_mess_bconsist_bit);
              break;

            case 1287:
              xgu_radar2_obj00_a_t r2_obj00_a;

              target_info.unpack_return = xgu_radar2_obj00_a_unpack(&r2_obj00_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj00_a_radar2_obj00_dx_decode(r2_obj00_a.radar2_obj00_dx);
              dx_is_in_range = xgu_radar2_obj00_a_radar2_obj00_dx_is_in_range(r2_obj00_a.radar2_obj00_dx);
              vx_decoded = xgu_radar2_obj00_a_radar2_obj00_vx_decode(r2_obj00_a.radar2_obj00_vx);
              vx_is_in_range = xgu_radar2_obj00_a_radar2_obj00_vx_is_in_range(r2_obj00_a.radar2_obj00_vx);
              dy_decoded = xgu_radar2_obj00_a_radar2_obj00_dy_decode(r2_obj00_a.radar2_obj00_dy);
              dy_is_in_range = xgu_radar2_obj00_a_radar2_obj00_dy_is_in_range(r2_obj00_a.radar2_obj00_dy);
              w_exist_decoded = xgu_radar2_obj00_a_radar2_obj00_w_exist_decode(r2_obj00_a.radar2_obj00_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj00_a_radar2_obj00_w_exist_is_in_range(r2_obj00_a.radar2_obj00_w_exist);
              ax_decoded = xgu_radar2_obj00_a_radar2_obj00_ax_decode(r2_obj00_a.radar2_obj00_ax);
              ax_is_in_range = xgu_radar2_obj00_a_radar2_obj00_ax_is_in_range(r2_obj00_a.radar2_obj00_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj00_a_radar2_obj00_w_obstacle_decode(r2_obj00_a.radar2_obj00_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj00_a_radar2_obj00_w_obstacle_is_in_range(r2_obj00_a.radar2_obj00_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj00_a_radar2_obj00_flag_valid_decode(r2_obj00_a.radar2_obj00_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj00_a_radar2_obj00_flag_valid_is_in_range(r2_obj00_a.radar2_obj00_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj00_a_radar2_obj00_w_non_obstacle_decode(r2_obj00_a.radar2_obj00_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj00_a_radar2_obj00_w_non_obstacle_is_in_range(r2_obj00_a.radar2_obj00_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj00_a_radar2_obj00_flag_meas_decode(r2_obj00_a.radar2_obj00_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj00_a_radar2_obj00_flag_meas_is_in_range(r2_obj00_a.radar2_obj00_flag_meas);
              flag_hist_decoded = xgu_radar2_obj00_a_radar2_obj00_flag_hist_decode(r2_obj00_a.radar2_obj00_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj00_a_radar2_obj00_flag_hist_is_in_range(r2_obj00_a.radar2_obj00_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj00_a_radar2_obj00_mess_aconsist_bit_decode(r2_obj00_a.radar2_obj00_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj00_a_radar2_obj00_mess_aconsist_bit_is_in_range(
                  r2_obj00_a.radar2_obj00_mess_aconsist_bit);
              break;

            case 1297:
              xgu_radar2_obj01_a_t r2_obj01_a;

              target_info.unpack_return = xgu_radar2_obj01_a_unpack(&r2_obj01_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj01_a_radar2_obj01_dx_decode(r2_obj01_a.radar2_obj01_dx);
              dx_is_in_range = xgu_radar2_obj01_a_radar2_obj01_dx_is_in_range(r2_obj01_a.radar2_obj01_dx);
              vx_decoded = xgu_radar2_obj01_a_radar2_obj01_vx_decode(r2_obj01_a.radar2_obj01_vx);
              vx_is_in_range = xgu_radar2_obj01_a_radar2_obj01_vx_is_in_range(r2_obj01_a.radar2_obj01_vx);
              dy_decoded = xgu_radar2_obj01_a_radar2_obj01_dy_decode(r2_obj01_a.radar2_obj01_dy);
              dy_is_in_range = xgu_radar2_obj01_a_radar2_obj01_dy_is_in_range(r2_obj01_a.radar2_obj01_dy);
              w_exist_decoded = xgu_radar2_obj01_a_radar2_obj01_w_exist_decode(r2_obj01_a.radar2_obj01_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj01_a_radar2_obj01_w_exist_is_in_range(r2_obj01_a.radar2_obj01_w_exist);
              ax_decoded = xgu_radar2_obj01_a_radar2_obj01_ax_decode(r2_obj01_a.radar2_obj01_ax);
              ax_is_in_range = xgu_radar2_obj01_a_radar2_obj01_ax_is_in_range(r2_obj01_a.radar2_obj01_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj01_a_radar2_obj01_w_obstacle_decode(r2_obj01_a.radar2_obj01_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj01_a_radar2_obj01_w_obstacle_is_in_range(r2_obj01_a.radar2_obj01_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj01_a_radar2_obj01_flag_valid_decode(r2_obj01_a.radar2_obj01_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj01_a_radar2_obj01_flag_valid_is_in_range(r2_obj01_a.radar2_obj01_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj01_a_radar2_obj01_w_non_obstacle_decode(r2_obj01_a.radar2_obj01_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj01_a_radar2_obj01_w_non_obstacle_is_in_range(r2_obj01_a.radar2_obj01_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj01_a_radar2_obj01_flag_meas_decode(r2_obj01_a.radar2_obj01_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj01_a_radar2_obj01_flag_meas_is_in_range(r2_obj01_a.radar2_obj01_flag_meas);
              flag_hist_decoded = xgu_radar2_obj01_a_radar2_obj01_flag_hist_decode(r2_obj01_a.radar2_obj01_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj01_a_radar2_obj01_flag_hist_is_in_range(r2_obj01_a.radar2_obj01_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj01_a_radar2_obj01_mess_aconsist_bit_decode(r2_obj01_a.radar2_obj01_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj01_a_radar2_obj01_mess_aconsist_bit_is_in_range(
                  r2_obj01_a.radar2_obj01_mess_aconsist_bit);
              break;

            case 1307:
              xgu_radar2_obj02_a_t r2_obj02_a;

              target_info.unpack_return = xgu_radar2_obj02_a_unpack(&r2_obj02_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj02_a_radar2_obj02_dx_decode(r2_obj02_a.radar2_obj02_dx);
              dx_is_in_range = xgu_radar2_obj02_a_radar2_obj02_dx_is_in_range(r2_obj02_a.radar2_obj02_dx);
              vx_decoded = xgu_radar2_obj02_a_radar2_obj02_vx_decode(r2_obj02_a.radar2_obj02_vx);
              vx_is_in_range = xgu_radar2_obj02_a_radar2_obj02_vx_is_in_range(r2_obj02_a.radar2_obj02_vx);
              dy_decoded = xgu_radar2_obj02_a_radar2_obj02_dy_decode(r2_obj02_a.radar2_obj02_dy);
              dy_is_in_range = xgu_radar2_obj02_a_radar2_obj02_dy_is_in_range(r2_obj02_a.radar2_obj02_dy);
              w_exist_decoded = xgu_radar2_obj02_a_radar2_obj02_w_exist_decode(r2_obj02_a.radar2_obj02_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj02_a_radar2_obj02_w_exist_is_in_range(r2_obj02_a.radar2_obj02_w_exist);
              ax_decoded = xgu_radar2_obj02_a_radar2_obj02_ax_decode(r2_obj02_a.radar2_obj02_ax);
              ax_is_in_range = xgu_radar2_obj02_a_radar2_obj02_ax_is_in_range(r2_obj02_a.radar2_obj02_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj02_a_radar2_obj02_w_obstacle_decode(r2_obj02_a.radar2_obj02_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj02_a_radar2_obj02_w_obstacle_is_in_range(r2_obj02_a.radar2_obj02_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj02_a_radar2_obj02_flag_valid_decode(r2_obj02_a.radar2_obj02_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj02_a_radar2_obj02_flag_valid_is_in_range(r2_obj02_a.radar2_obj02_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj02_a_radar2_obj02_w_non_obstacle_decode(r2_obj02_a.radar2_obj02_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj02_a_radar2_obj02_w_non_obstacle_is_in_range(r2_obj02_a.radar2_obj02_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj02_a_radar2_obj02_flag_meas_decode(r2_obj02_a.radar2_obj02_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj02_a_radar2_obj02_flag_meas_is_in_range(r2_obj02_a.radar2_obj02_flag_meas);
              flag_hist_decoded = xgu_radar2_obj02_a_radar2_obj02_flag_hist_decode(r2_obj02_a.radar2_obj02_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj02_a_radar2_obj02_flag_hist_is_in_range(r2_obj02_a.radar2_obj02_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj02_a_radar2_obj02_mess_aconsist_bit_decode(r2_obj02_a.radar2_obj02_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj02_a_radar2_obj02_mess_aconsist_bit_is_in_range(
                  r2_obj02_a.radar2_obj02_mess_aconsist_bit);
              break;

            case 1317:
              xgu_radar2_obj03_a_t r2_obj03_a;

              target_info.unpack_return = xgu_radar2_obj03_a_unpack(&r2_obj03_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj03_a_radar2_obj03_dx_decode(r2_obj03_a.radar2_obj03_dx);
              dx_is_in_range = xgu_radar2_obj03_a_radar2_obj03_dx_is_in_range(r2_obj03_a.radar2_obj03_dx);
              vx_decoded = xgu_radar2_obj03_a_radar2_obj03_vx_decode(r2_obj03_a.radar2_obj03_vx);
              vx_is_in_range = xgu_radar2_obj03_a_radar2_obj03_vx_is_in_range(r2_obj03_a.radar2_obj03_vx);
              dy_decoded = xgu_radar2_obj03_a_radar2_obj03_dy_decode(r2_obj03_a.radar2_obj03_dy);
              dy_is_in_range = xgu_radar2_obj03_a_radar2_obj03_dy_is_in_range(r2_obj03_a.radar2_obj03_dy);
              w_exist_decoded = xgu_radar2_obj03_a_radar2_obj03_w_exist_decode(r2_obj03_a.radar2_obj03_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj03_a_radar2_obj03_w_exist_is_in_range(r2_obj03_a.radar2_obj03_w_exist);
              ax_decoded = xgu_radar2_obj03_a_radar2_obj03_ax_decode(r2_obj03_a.radar2_obj03_ax);
              ax_is_in_range = xgu_radar2_obj03_a_radar2_obj03_ax_is_in_range(r2_obj03_a.radar2_obj03_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj03_a_radar2_obj03_w_obstacle_decode(r2_obj03_a.radar2_obj03_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj03_a_radar2_obj03_w_obstacle_is_in_range(r2_obj03_a.radar2_obj03_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj03_a_radar2_obj03_flag_valid_decode(r2_obj03_a.radar2_obj03_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj03_a_radar2_obj03_flag_valid_is_in_range(r2_obj03_a.radar2_obj03_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj03_a_radar2_obj03_w_non_obstacle_decode(r2_obj03_a.radar2_obj03_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj03_a_radar2_obj03_w_non_obstacle_is_in_range(r2_obj03_a.radar2_obj03_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj03_a_radar2_obj03_flag_meas_decode(r2_obj03_a.radar2_obj03_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj03_a_radar2_obj03_flag_meas_is_in_range(r2_obj03_a.radar2_obj03_flag_meas);
              flag_hist_decoded = xgu_radar2_obj03_a_radar2_obj03_flag_hist_decode(r2_obj03_a.radar2_obj03_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj03_a_radar2_obj03_flag_hist_is_in_range(r2_obj03_a.radar2_obj03_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj03_a_radar2_obj03_mess_aconsist_bit_decode(r2_obj03_a.radar2_obj03_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj03_a_radar2_obj03_mess_aconsist_bit_is_in_range(
                  r2_obj03_a.radar2_obj03_mess_aconsist_bit);
              break;

            case 1327:
              xgu_radar2_obj04_a_t r2_obj04_a;

              target_info.unpack_return = xgu_radar2_obj04_a_unpack(&r2_obj04_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj04_a_radar2_obj04_dx_decode(r2_obj04_a.radar2_obj04_dx);
              dx_is_in_range = xgu_radar2_obj04_a_radar2_obj04_dx_is_in_range(r2_obj04_a.radar2_obj04_dx);
              vx_decoded = xgu_radar2_obj04_a_radar2_obj04_vx_decode(r2_obj04_a.radar2_obj04_vx);
              vx_is_in_range = xgu_radar2_obj04_a_radar2_obj04_vx_is_in_range(r2_obj04_a.radar2_obj04_vx);
              dy_decoded = xgu_radar2_obj04_a_radar2_obj04_dy_decode(r2_obj04_a.radar2_obj04_dy);
              dy_is_in_range = xgu_radar2_obj04_a_radar2_obj04_dy_is_in_range(r2_obj04_a.radar2_obj04_dy);
              w_exist_decoded = xgu_radar2_obj04_a_radar2_obj04_w_exist_decode(r2_obj04_a.radar2_obj04_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj04_a_radar2_obj04_w_exist_is_in_range(r2_obj04_a.radar2_obj04_w_exist);
              ax_decoded = xgu_radar2_obj04_a_radar2_obj04_ax_decode(r2_obj04_a.radar2_obj04_ax);
              ax_is_in_range = xgu_radar2_obj04_a_radar2_obj04_ax_is_in_range(r2_obj04_a.radar2_obj04_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj04_a_radar2_obj04_w_obstacle_decode(r2_obj04_a.radar2_obj04_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj04_a_radar2_obj04_w_obstacle_is_in_range(r2_obj04_a.radar2_obj04_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj04_a_radar2_obj04_flag_valid_decode(r2_obj04_a.radar2_obj04_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj04_a_radar2_obj04_flag_valid_is_in_range(r2_obj04_a.radar2_obj04_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj04_a_radar2_obj04_w_non_obstacle_decode(r2_obj04_a.radar2_obj04_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj04_a_radar2_obj04_w_non_obstacle_is_in_range(r2_obj04_a.radar2_obj04_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj04_a_radar2_obj04_flag_meas_decode(r2_obj04_a.radar2_obj04_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj04_a_radar2_obj04_flag_meas_is_in_range(r2_obj04_a.radar2_obj04_flag_meas);
              flag_hist_decoded = xgu_radar2_obj04_a_radar2_obj04_flag_hist_decode(r2_obj04_a.radar2_obj04_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj04_a_radar2_obj04_flag_hist_is_in_range(r2_obj04_a.radar2_obj04_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj04_a_radar2_obj04_mess_aconsist_bit_decode(r2_obj04_a.radar2_obj04_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj04_a_radar2_obj04_mess_aconsist_bit_is_in_range(
                  r2_obj04_a.radar2_obj04_mess_aconsist_bit);
              break;

            case 1337:
              xgu_radar2_obj05_a_t r2_obj05_a;

              target_info.unpack_return = xgu_radar2_obj05_a_unpack(&r2_obj05_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj05_a_radar2_obj05_dx_decode(r2_obj05_a.radar2_obj05_dx);
              dx_is_in_range = xgu_radar2_obj05_a_radar2_obj05_dx_is_in_range(r2_obj05_a.radar2_obj05_dx);
              vx_decoded = xgu_radar2_obj05_a_radar2_obj05_vx_decode(r2_obj05_a.radar2_obj05_vx);
              vx_is_in_range = xgu_radar2_obj05_a_radar2_obj05_vx_is_in_range(r2_obj05_a.radar2_obj05_vx);
              dy_decoded = xgu_radar2_obj05_a_radar2_obj05_dy_decode(r2_obj05_a.radar2_obj05_dy);
              dy_is_in_range = xgu_radar2_obj05_a_radar2_obj05_dy_is_in_range(r2_obj05_a.radar2_obj05_dy);
              w_exist_decoded = xgu_radar2_obj05_a_radar2_obj05_w_exist_decode(r2_obj05_a.radar2_obj05_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj05_a_radar2_obj05_w_exist_is_in_range(r2_obj05_a.radar2_obj05_w_exist);
              ax_decoded = xgu_radar2_obj05_a_radar2_obj05_ax_decode(r2_obj05_a.radar2_obj05_ax);
              ax_is_in_range = xgu_radar2_obj05_a_radar2_obj05_ax_is_in_range(r2_obj05_a.radar2_obj05_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj05_a_radar2_obj05_w_obstacle_decode(r2_obj05_a.radar2_obj05_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj05_a_radar2_obj05_w_obstacle_is_in_range(r2_obj05_a.radar2_obj05_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj05_a_radar2_obj05_flag_valid_decode(r2_obj05_a.radar2_obj05_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj05_a_radar2_obj05_flag_valid_is_in_range(r2_obj05_a.radar2_obj05_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj05_a_radar2_obj05_w_non_obstacle_decode(r2_obj05_a.radar2_obj05_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj05_a_radar2_obj05_w_non_obstacle_is_in_range(r2_obj05_a.radar2_obj05_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj05_a_radar2_obj05_flag_meas_decode(r2_obj05_a.radar2_obj05_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj05_a_radar2_obj05_flag_meas_is_in_range(r2_obj05_a.radar2_obj05_flag_meas);
              flag_hist_decoded = xgu_radar2_obj05_a_radar2_obj05_flag_hist_decode(r2_obj05_a.radar2_obj05_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj05_a_radar2_obj05_flag_hist_is_in_range(r2_obj05_a.radar2_obj05_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj05_a_radar2_obj05_mess_aconsist_bit_decode(r2_obj05_a.radar2_obj05_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj05_a_radar2_obj05_mess_aconsist_bit_is_in_range(
                  r2_obj05_a.radar2_obj05_mess_aconsist_bit);
              break;

            case 1347:
              xgu_radar2_obj06_a_t r2_obj06_a;

              target_info.unpack_return = xgu_radar2_obj06_a_unpack(&r2_obj06_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj06_a_radar2_obj06_dx_decode(r2_obj06_a.radar2_obj06_dx);
              dx_is_in_range = xgu_radar2_obj06_a_radar2_obj06_dx_is_in_range(r2_obj06_a.radar2_obj06_dx);
              vx_decoded = xgu_radar2_obj06_a_radar2_obj06_vx_decode(r2_obj06_a.radar2_obj06_vx);
              vx_is_in_range = xgu_radar2_obj06_a_radar2_obj06_vx_is_in_range(r2_obj06_a.radar2_obj06_vx);
              dy_decoded = xgu_radar2_obj06_a_radar2_obj06_dy_decode(r2_obj06_a.radar2_obj06_dy);
              dy_is_in_range = xgu_radar2_obj06_a_radar2_obj06_dy_is_in_range(r2_obj06_a.radar2_obj06_dy);
              w_exist_decoded = xgu_radar2_obj06_a_radar2_obj06_w_exist_decode(r2_obj06_a.radar2_obj06_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj06_a_radar2_obj06_w_exist_is_in_range(r2_obj06_a.radar2_obj06_w_exist);
              ax_decoded = xgu_radar2_obj06_a_radar2_obj06_ax_decode(r2_obj06_a.radar2_obj06_ax);
              ax_is_in_range = xgu_radar2_obj06_a_radar2_obj06_ax_is_in_range(r2_obj06_a.radar2_obj06_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj06_a_radar2_obj06_w_obstacle_decode(r2_obj06_a.radar2_obj06_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj06_a_radar2_obj06_w_obstacle_is_in_range(r2_obj06_a.radar2_obj06_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj06_a_radar2_obj06_flag_valid_decode(r2_obj06_a.radar2_obj06_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj06_a_radar2_obj06_flag_valid_is_in_range(r2_obj06_a.radar2_obj06_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj06_a_radar2_obj06_w_non_obstacle_decode(r2_obj06_a.radar2_obj06_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj06_a_radar2_obj06_w_non_obstacle_is_in_range(r2_obj06_a.radar2_obj06_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj06_a_radar2_obj06_flag_meas_decode(r2_obj06_a.radar2_obj06_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj06_a_radar2_obj06_flag_meas_is_in_range(r2_obj06_a.radar2_obj06_flag_meas);
              flag_hist_decoded = xgu_radar2_obj06_a_radar2_obj06_flag_hist_decode(r2_obj06_a.radar2_obj06_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj06_a_radar2_obj06_flag_hist_is_in_range(r2_obj06_a.radar2_obj06_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj06_a_radar2_obj06_mess_aconsist_bit_decode(r2_obj06_a.radar2_obj06_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj06_a_radar2_obj06_mess_aconsist_bit_is_in_range(
                  r2_obj06_a.radar2_obj06_mess_aconsist_bit);
              break;

            case 1357:
              xgu_radar2_obj07_a_t r2_obj07_a;

              target_info.unpack_return = xgu_radar2_obj07_a_unpack(&r2_obj07_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj07_a_radar2_obj07_dx_decode(r2_obj07_a.radar2_obj07_dx);
              dx_is_in_range = xgu_radar2_obj07_a_radar2_obj07_dx_is_in_range(r2_obj07_a.radar2_obj07_dx);
              vx_decoded = xgu_radar2_obj07_a_radar2_obj07_vx_decode(r2_obj07_a.radar2_obj07_vx);
              vx_is_in_range = xgu_radar2_obj07_a_radar2_obj07_vx_is_in_range(r2_obj07_a.radar2_obj07_vx);
              dy_decoded = xgu_radar2_obj07_a_radar2_obj07_dy_decode(r2_obj07_a.radar2_obj07_dy);
              dy_is_in_range = xgu_radar2_obj07_a_radar2_obj07_dy_is_in_range(r2_obj07_a.radar2_obj07_dy);
              w_exist_decoded = xgu_radar2_obj07_a_radar2_obj07_w_exist_decode(r2_obj07_a.radar2_obj07_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj07_a_radar2_obj07_w_exist_is_in_range(r2_obj07_a.radar2_obj07_w_exist);
              ax_decoded = xgu_radar2_obj07_a_radar2_obj07_ax_decode(r2_obj07_a.radar2_obj07_ax);
              ax_is_in_range = xgu_radar2_obj07_a_radar2_obj07_ax_is_in_range(r2_obj07_a.radar2_obj07_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj07_a_radar2_obj07_w_obstacle_decode(r2_obj07_a.radar2_obj07_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj07_a_radar2_obj07_w_obstacle_is_in_range(r2_obj07_a.radar2_obj07_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj07_a_radar2_obj07_flag_valid_decode(r2_obj07_a.radar2_obj07_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj07_a_radar2_obj07_flag_valid_is_in_range(r2_obj07_a.radar2_obj07_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj07_a_radar2_obj07_w_non_obstacle_decode(r2_obj07_a.radar2_obj07_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj07_a_radar2_obj07_w_non_obstacle_is_in_range(r2_obj07_a.radar2_obj07_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj07_a_radar2_obj07_flag_meas_decode(r2_obj07_a.radar2_obj07_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj07_a_radar2_obj07_flag_meas_is_in_range(r2_obj07_a.radar2_obj07_flag_meas);
              flag_hist_decoded = xgu_radar2_obj07_a_radar2_obj07_flag_hist_decode(r2_obj07_a.radar2_obj07_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj07_a_radar2_obj07_flag_hist_is_in_range(r2_obj07_a.radar2_obj07_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj07_a_radar2_obj07_mess_aconsist_bit_decode(r2_obj07_a.radar2_obj07_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj07_a_radar2_obj07_mess_aconsist_bit_is_in_range(
                  r2_obj07_a.radar2_obj07_mess_aconsist_bit);
              break;

            case 1367:
              xgu_radar2_obj08_a_t r2_obj08_a;

              target_info.unpack_return = xgu_radar2_obj08_a_unpack(&r2_obj08_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj08_a_radar2_obj08_dx_decode(r2_obj08_a.radar2_obj08_dx);
              dx_is_in_range = xgu_radar2_obj08_a_radar2_obj08_dx_is_in_range(r2_obj08_a.radar2_obj08_dx);
              vx_decoded = xgu_radar2_obj08_a_radar2_obj08_vx_decode(r2_obj08_a.radar2_obj08_vx);
              vx_is_in_range = xgu_radar2_obj08_a_radar2_obj08_vx_is_in_range(r2_obj08_a.radar2_obj08_vx);
              dy_decoded = xgu_radar2_obj08_a_radar2_obj08_dy_decode(r2_obj08_a.radar2_obj08_dy);
              dy_is_in_range = xgu_radar2_obj08_a_radar2_obj08_dy_is_in_range(r2_obj08_a.radar2_obj08_dy);
              w_exist_decoded = xgu_radar2_obj08_a_radar2_obj08_w_exist_decode(r2_obj08_a.radar2_obj08_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj08_a_radar2_obj08_w_exist_is_in_range(r2_obj08_a.radar2_obj08_w_exist);
              ax_decoded = xgu_radar2_obj08_a_radar2_obj08_ax_decode(r2_obj08_a.radar2_obj08_ax);
              ax_is_in_range = xgu_radar2_obj08_a_radar2_obj08_ax_is_in_range(r2_obj08_a.radar2_obj08_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj08_a_radar2_obj08_w_obstacle_decode(r2_obj08_a.radar2_obj08_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj08_a_radar2_obj08_w_obstacle_is_in_range(r2_obj08_a.radar2_obj08_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj08_a_radar2_obj08_flag_valid_decode(r2_obj08_a.radar2_obj08_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj08_a_radar2_obj08_flag_valid_is_in_range(r2_obj08_a.radar2_obj08_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj08_a_radar2_obj08_w_non_obstacle_decode(r2_obj08_a.radar2_obj08_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj08_a_radar2_obj08_w_non_obstacle_is_in_range(r2_obj08_a.radar2_obj08_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj08_a_radar2_obj08_flag_meas_decode(r2_obj08_a.radar2_obj08_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj08_a_radar2_obj08_flag_meas_is_in_range(r2_obj08_a.radar2_obj08_flag_meas);
              flag_hist_decoded = xgu_radar2_obj08_a_radar2_obj08_flag_hist_decode(r2_obj08_a.radar2_obj08_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj08_a_radar2_obj08_flag_hist_is_in_range(r2_obj08_a.radar2_obj08_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj08_a_radar2_obj08_mess_aconsist_bit_decode(r2_obj08_a.radar2_obj08_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj08_a_radar2_obj08_mess_aconsist_bit_is_in_range(
                  r2_obj08_a.radar2_obj08_mess_aconsist_bit);
              break;

            case 1377:
              xgu_radar2_obj09_a_t r2_obj09_a;

              target_info.unpack_return = xgu_radar2_obj09_a_unpack(&r2_obj09_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj09_a_radar2_obj09_dx_decode(r2_obj09_a.radar2_obj09_dx);
              dx_is_in_range = xgu_radar2_obj09_a_radar2_obj09_dx_is_in_range(r2_obj09_a.radar2_obj09_dx);
              vx_decoded = xgu_radar2_obj09_a_radar2_obj09_vx_decode(r2_obj09_a.radar2_obj09_vx);
              vx_is_in_range = xgu_radar2_obj09_a_radar2_obj09_vx_is_in_range(r2_obj09_a.radar2_obj09_vx);
              dy_decoded = xgu_radar2_obj09_a_radar2_obj09_dy_decode(r2_obj09_a.radar2_obj09_dy);
              dy_is_in_range = xgu_radar2_obj09_a_radar2_obj09_dy_is_in_range(r2_obj09_a.radar2_obj09_dy);
              w_exist_decoded = xgu_radar2_obj09_a_radar2_obj09_w_exist_decode(r2_obj09_a.radar2_obj09_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj09_a_radar2_obj09_w_exist_is_in_range(r2_obj09_a.radar2_obj09_w_exist);
              ax_decoded = xgu_radar2_obj09_a_radar2_obj09_ax_decode(r2_obj09_a.radar2_obj09_ax);
              ax_is_in_range = xgu_radar2_obj09_a_radar2_obj09_ax_is_in_range(r2_obj09_a.radar2_obj09_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj09_a_radar2_obj09_w_obstacle_decode(r2_obj09_a.radar2_obj09_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj09_a_radar2_obj09_w_obstacle_is_in_range(r2_obj09_a.radar2_obj09_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj09_a_radar2_obj09_flag_valid_decode(r2_obj09_a.radar2_obj09_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj09_a_radar2_obj09_flag_valid_is_in_range(r2_obj09_a.radar2_obj09_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj09_a_radar2_obj09_w_non_obstacle_decode(r2_obj09_a.radar2_obj09_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj09_a_radar2_obj09_w_non_obstacle_is_in_range(r2_obj09_a.radar2_obj09_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj09_a_radar2_obj09_flag_meas_decode(r2_obj09_a.radar2_obj09_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj09_a_radar2_obj09_flag_meas_is_in_range(r2_obj09_a.radar2_obj09_flag_meas);
              flag_hist_decoded = xgu_radar2_obj09_a_radar2_obj09_flag_hist_decode(r2_obj09_a.radar2_obj09_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj09_a_radar2_obj09_flag_hist_is_in_range(r2_obj09_a.radar2_obj09_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj09_a_radar2_obj09_mess_aconsist_bit_decode(r2_obj09_a.radar2_obj09_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj09_a_radar2_obj09_mess_aconsist_bit_is_in_range(
                  r2_obj09_a.radar2_obj09_mess_aconsist_bit);
              break;

            case 1387:
              xgu_radar2_obj10_a_t r2_obj10_a;

              target_info.unpack_return = xgu_radar2_obj10_a_unpack(&r2_obj10_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj10_a_radar2_obj10_dx_decode(r2_obj10_a.radar2_obj10_dx);
              dx_is_in_range = xgu_radar2_obj10_a_radar2_obj10_dx_is_in_range(r2_obj10_a.radar2_obj10_dx);
              vx_decoded = xgu_radar2_obj10_a_radar2_obj10_vx_decode(r2_obj10_a.radar2_obj10_vx);
              vx_is_in_range = xgu_radar2_obj10_a_radar2_obj10_vx_is_in_range(r2_obj10_a.radar2_obj10_vx);
              dy_decoded = xgu_radar2_obj10_a_radar2_obj10_dy_decode(r2_obj10_a.radar2_obj10_dy);
              dy_is_in_range = xgu_radar2_obj10_a_radar2_obj10_dy_is_in_range(r2_obj10_a.radar2_obj10_dy);
              w_exist_decoded = xgu_radar2_obj10_a_radar2_obj10_w_exist_decode(r2_obj10_a.radar2_obj10_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj10_a_radar2_obj10_w_exist_is_in_range(r2_obj10_a.radar2_obj10_w_exist);
              ax_decoded = xgu_radar2_obj10_a_radar2_obj10_ax_decode(r2_obj10_a.radar2_obj10_ax);
              ax_is_in_range = xgu_radar2_obj10_a_radar2_obj10_ax_is_in_range(r2_obj10_a.radar2_obj10_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj10_a_radar2_obj10_w_obstacle_decode(r2_obj10_a.radar2_obj10_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj10_a_radar2_obj10_w_obstacle_is_in_range(r2_obj10_a.radar2_obj10_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj10_a_radar2_obj10_flag_valid_decode(r2_obj10_a.radar2_obj10_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj10_a_radar2_obj10_flag_valid_is_in_range(r2_obj10_a.radar2_obj10_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj10_a_radar2_obj10_w_non_obstacle_decode(r2_obj10_a.radar2_obj10_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj10_a_radar2_obj10_w_non_obstacle_is_in_range(r2_obj10_a.radar2_obj10_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj10_a_radar2_obj10_flag_meas_decode(r2_obj10_a.radar2_obj10_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj10_a_radar2_obj10_flag_meas_is_in_range(r2_obj10_a.radar2_obj10_flag_meas);
              flag_hist_decoded = xgu_radar2_obj10_a_radar2_obj10_flag_hist_decode(r2_obj10_a.radar2_obj10_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj10_a_radar2_obj10_flag_hist_is_in_range(r2_obj10_a.radar2_obj10_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj10_a_radar2_obj10_mess_aconsist_bit_decode(r2_obj10_a.radar2_obj10_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj10_a_radar2_obj10_mess_aconsist_bit_is_in_range(
                  r2_obj10_a.radar2_obj10_mess_aconsist_bit);
              break;

            case 1397:
              xgu_radar2_obj11_a_t r2_obj11_a;

              target_info.unpack_return = xgu_radar2_obj11_a_unpack(&r2_obj11_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj11_a_radar2_obj11_dx_decode(r2_obj11_a.radar2_obj11_dx);
              dx_is_in_range = xgu_radar2_obj11_a_radar2_obj11_dx_is_in_range(r2_obj11_a.radar2_obj11_dx);
              vx_decoded = xgu_radar2_obj11_a_radar2_obj11_vx_decode(r2_obj11_a.radar2_obj11_vx);
              vx_is_in_range = xgu_radar2_obj11_a_radar2_obj11_vx_is_in_range(r2_obj11_a.radar2_obj11_vx);
              dy_decoded = xgu_radar2_obj11_a_radar2_obj11_dy_decode(r2_obj11_a.radar2_obj11_dy);
              dy_is_in_range = xgu_radar2_obj11_a_radar2_obj11_dy_is_in_range(r2_obj11_a.radar2_obj11_dy);
              w_exist_decoded = xgu_radar2_obj11_a_radar2_obj11_w_exist_decode(r2_obj11_a.radar2_obj11_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj11_a_radar2_obj11_w_exist_is_in_range(r2_obj11_a.radar2_obj11_w_exist);
              ax_decoded = xgu_radar2_obj11_a_radar2_obj11_ax_decode(r2_obj11_a.radar2_obj11_ax);
              ax_is_in_range = xgu_radar2_obj11_a_radar2_obj11_ax_is_in_range(r2_obj11_a.radar2_obj11_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj11_a_radar2_obj11_w_obstacle_decode(r2_obj11_a.radar2_obj11_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj11_a_radar2_obj11_w_obstacle_is_in_range(r2_obj11_a.radar2_obj11_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj11_a_radar2_obj11_flag_valid_decode(r2_obj11_a.radar2_obj11_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj11_a_radar2_obj11_flag_valid_is_in_range(r2_obj11_a.radar2_obj11_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj11_a_radar2_obj11_w_non_obstacle_decode(r2_obj11_a.radar2_obj11_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj11_a_radar2_obj11_w_non_obstacle_is_in_range(r2_obj11_a.radar2_obj11_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj11_a_radar2_obj11_flag_meas_decode(r2_obj11_a.radar2_obj11_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj11_a_radar2_obj11_flag_meas_is_in_range(r2_obj11_a.radar2_obj11_flag_meas);
              flag_hist_decoded = xgu_radar2_obj11_a_radar2_obj11_flag_hist_decode(r2_obj11_a.radar2_obj11_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj11_a_radar2_obj11_flag_hist_is_in_range(r2_obj11_a.radar2_obj11_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj11_a_radar2_obj11_mess_aconsist_bit_decode(r2_obj11_a.radar2_obj11_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj11_a_radar2_obj11_mess_aconsist_bit_is_in_range(
                  r2_obj11_a.radar2_obj11_mess_aconsist_bit);
              break;

            case 1407:
              xgu_radar2_obj12_a_t r2_obj12_a;

              target_info.unpack_return = xgu_radar2_obj12_a_unpack(&r2_obj12_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj12_a_radar2_obj12_dx_decode(r2_obj12_a.radar2_obj12_dx);
              dx_is_in_range = xgu_radar2_obj12_a_radar2_obj12_dx_is_in_range(r2_obj12_a.radar2_obj12_dx);
              vx_decoded = xgu_radar2_obj12_a_radar2_obj12_vx_decode(r2_obj12_a.radar2_obj12_vx);
              vx_is_in_range = xgu_radar2_obj12_a_radar2_obj12_vx_is_in_range(r2_obj12_a.radar2_obj12_vx);
              dy_decoded = xgu_radar2_obj12_a_radar2_obj12_dy_decode(r2_obj12_a.radar2_obj12_dy);
              dy_is_in_range = xgu_radar2_obj12_a_radar2_obj12_dy_is_in_range(r2_obj12_a.radar2_obj12_dy);
              w_exist_decoded = xgu_radar2_obj12_a_radar2_obj12_w_exist_decode(r2_obj12_a.radar2_obj12_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj12_a_radar2_obj12_w_exist_is_in_range(r2_obj12_a.radar2_obj12_w_exist);
              ax_decoded = xgu_radar2_obj12_a_radar2_obj12_ax_decode(r2_obj12_a.radar2_obj12_ax);
              ax_is_in_range = xgu_radar2_obj12_a_radar2_obj12_ax_is_in_range(r2_obj12_a.radar2_obj12_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj12_a_radar2_obj12_w_obstacle_decode(r2_obj12_a.radar2_obj12_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj12_a_radar2_obj12_w_obstacle_is_in_range(r2_obj12_a.radar2_obj12_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj12_a_radar2_obj12_flag_valid_decode(r2_obj12_a.radar2_obj12_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj12_a_radar2_obj12_flag_valid_is_in_range(r2_obj12_a.radar2_obj12_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj12_a_radar2_obj12_w_non_obstacle_decode(r2_obj12_a.radar2_obj12_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj12_a_radar2_obj12_w_non_obstacle_is_in_range(r2_obj12_a.radar2_obj12_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj12_a_radar2_obj12_flag_meas_decode(r2_obj12_a.radar2_obj12_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj12_a_radar2_obj12_flag_meas_is_in_range(r2_obj12_a.radar2_obj12_flag_meas);
              flag_hist_decoded = xgu_radar2_obj12_a_radar2_obj12_flag_hist_decode(r2_obj12_a.radar2_obj12_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj12_a_radar2_obj12_flag_hist_is_in_range(r2_obj12_a.radar2_obj12_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj12_a_radar2_obj12_mess_aconsist_bit_decode(r2_obj12_a.radar2_obj12_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj12_a_radar2_obj12_mess_aconsist_bit_is_in_range(
                  r2_obj12_a.radar2_obj12_mess_aconsist_bit);
              break;

            case 1417:
              xgu_radar2_obj13_a_t r2_obj13_a;

              target_info.unpack_return = xgu_radar2_obj13_a_unpack(&r2_obj13_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj13_a_radar2_obj13_dx_decode(r2_obj13_a.radar2_obj13_dx);
              dx_is_in_range = xgu_radar2_obj13_a_radar2_obj13_dx_is_in_range(r2_obj13_a.radar2_obj13_dx);
              vx_decoded = xgu_radar2_obj13_a_radar2_obj13_vx_decode(r2_obj13_a.radar2_obj13_vx);
              vx_is_in_range = xgu_radar2_obj13_a_radar2_obj13_vx_is_in_range(r2_obj13_a.radar2_obj13_vx);
              dy_decoded = xgu_radar2_obj13_a_radar2_obj13_dy_decode(r2_obj13_a.radar2_obj13_dy);
              dy_is_in_range = xgu_radar2_obj13_a_radar2_obj13_dy_is_in_range(r2_obj13_a.radar2_obj13_dy);
              w_exist_decoded = xgu_radar2_obj13_a_radar2_obj13_w_exist_decode(r2_obj13_a.radar2_obj13_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj13_a_radar2_obj13_w_exist_is_in_range(r2_obj13_a.radar2_obj13_w_exist);
              ax_decoded = xgu_radar2_obj13_a_radar2_obj13_ax_decode(r2_obj13_a.radar2_obj13_ax);
              ax_is_in_range = xgu_radar2_obj13_a_radar2_obj13_ax_is_in_range(r2_obj13_a.radar2_obj13_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj13_a_radar2_obj13_w_obstacle_decode(r2_obj13_a.radar2_obj13_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj13_a_radar2_obj13_w_obstacle_is_in_range(r2_obj13_a.radar2_obj13_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj13_a_radar2_obj13_flag_valid_decode(r2_obj13_a.radar2_obj13_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj13_a_radar2_obj13_flag_valid_is_in_range(r2_obj13_a.radar2_obj13_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj13_a_radar2_obj13_w_non_obstacle_decode(r2_obj13_a.radar2_obj13_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj13_a_radar2_obj13_w_non_obstacle_is_in_range(r2_obj13_a.radar2_obj13_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj13_a_radar2_obj13_flag_meas_decode(r2_obj13_a.radar2_obj13_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj13_a_radar2_obj13_flag_meas_is_in_range(r2_obj13_a.radar2_obj13_flag_meas);
              flag_hist_decoded = xgu_radar2_obj13_a_radar2_obj13_flag_hist_decode(r2_obj13_a.radar2_obj13_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj13_a_radar2_obj13_flag_hist_is_in_range(r2_obj13_a.radar2_obj13_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj13_a_radar2_obj13_mess_aconsist_bit_decode(r2_obj13_a.radar2_obj13_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj13_a_radar2_obj13_mess_aconsist_bit_is_in_range(
                  r2_obj13_a.radar2_obj13_mess_aconsist_bit);
              break;

            case 1427:
              xgu_radar2_obj14_a_t r2_obj14_a;

              target_info.unpack_return = xgu_radar2_obj14_a_unpack(&r2_obj14_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj14_a_radar2_obj14_dx_decode(r2_obj14_a.radar2_obj14_dx);
              dx_is_in_range = xgu_radar2_obj14_a_radar2_obj14_dx_is_in_range(r2_obj14_a.radar2_obj14_dx);
              vx_decoded = xgu_radar2_obj14_a_radar2_obj14_vx_decode(r2_obj14_a.radar2_obj14_vx);
              vx_is_in_range = xgu_radar2_obj14_a_radar2_obj14_vx_is_in_range(r2_obj14_a.radar2_obj14_vx);
              dy_decoded = xgu_radar2_obj14_a_radar2_obj14_dy_decode(r2_obj14_a.radar2_obj14_dy);
              dy_is_in_range = xgu_radar2_obj14_a_radar2_obj14_dy_is_in_range(r2_obj14_a.radar2_obj14_dy);
              w_exist_decoded = xgu_radar2_obj14_a_radar2_obj14_w_exist_decode(r2_obj14_a.radar2_obj14_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj14_a_radar2_obj14_w_exist_is_in_range(r2_obj14_a.radar2_obj14_w_exist);
              ax_decoded = xgu_radar2_obj14_a_radar2_obj14_ax_decode(r2_obj14_a.radar2_obj14_ax);
              ax_is_in_range = xgu_radar2_obj14_a_radar2_obj14_ax_is_in_range(r2_obj14_a.radar2_obj14_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj14_a_radar2_obj14_w_obstacle_decode(r2_obj14_a.radar2_obj14_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj14_a_radar2_obj14_w_obstacle_is_in_range(r2_obj14_a.radar2_obj14_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj14_a_radar2_obj14_flag_valid_decode(r2_obj14_a.radar2_obj14_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj14_a_radar2_obj14_flag_valid_is_in_range(r2_obj14_a.radar2_obj14_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj14_a_radar2_obj14_w_non_obstacle_decode(r2_obj14_a.radar2_obj14_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj14_a_radar2_obj14_w_non_obstacle_is_in_range(r2_obj14_a.radar2_obj14_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj14_a_radar2_obj14_flag_meas_decode(r2_obj14_a.radar2_obj14_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj14_a_radar2_obj14_flag_meas_is_in_range(r2_obj14_a.radar2_obj14_flag_meas);
              flag_hist_decoded = xgu_radar2_obj14_a_radar2_obj14_flag_hist_decode(r2_obj14_a.radar2_obj14_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj14_a_radar2_obj14_flag_hist_is_in_range(r2_obj14_a.radar2_obj14_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj14_a_radar2_obj14_mess_aconsist_bit_decode(r2_obj14_a.radar2_obj14_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj14_a_radar2_obj14_mess_aconsist_bit_is_in_range(
                  r2_obj14_a.radar2_obj14_mess_aconsist_bit);
              break;

            case 1437:
              xgu_radar2_obj15_a_t r2_obj15_a;

              target_info.unpack_return = xgu_radar2_obj15_a_unpack(&r2_obj15_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj15_a_radar2_obj15_dx_decode(r2_obj15_a.radar2_obj15_dx);
              dx_is_in_range = xgu_radar2_obj15_a_radar2_obj15_dx_is_in_range(r2_obj15_a.radar2_obj15_dx);
              vx_decoded = xgu_radar2_obj15_a_radar2_obj15_vx_decode(r2_obj15_a.radar2_obj15_vx);
              vx_is_in_range = xgu_radar2_obj15_a_radar2_obj15_vx_is_in_range(r2_obj15_a.radar2_obj15_vx);
              dy_decoded = xgu_radar2_obj15_a_radar2_obj15_dy_decode(r2_obj15_a.radar2_obj15_dy);
              dy_is_in_range = xgu_radar2_obj15_a_radar2_obj15_dy_is_in_range(r2_obj15_a.radar2_obj15_dy);
              w_exist_decoded = xgu_radar2_obj15_a_radar2_obj15_w_exist_decode(r2_obj15_a.radar2_obj15_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj15_a_radar2_obj15_w_exist_is_in_range(r2_obj15_a.radar2_obj15_w_exist);
              ax_decoded = xgu_radar2_obj15_a_radar2_obj15_ax_decode(r2_obj15_a.radar2_obj15_ax);
              ax_is_in_range = xgu_radar2_obj15_a_radar2_obj15_ax_is_in_range(r2_obj15_a.radar2_obj15_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj15_a_radar2_obj15_w_obstacle_decode(r2_obj15_a.radar2_obj15_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj15_a_radar2_obj15_w_obstacle_is_in_range(r2_obj15_a.radar2_obj15_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj15_a_radar2_obj15_flag_valid_decode(r2_obj15_a.radar2_obj15_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj15_a_radar2_obj15_flag_valid_is_in_range(r2_obj15_a.radar2_obj15_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj15_a_radar2_obj15_w_non_obstacle_decode(r2_obj15_a.radar2_obj15_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj15_a_radar2_obj15_w_non_obstacle_is_in_range(r2_obj15_a.radar2_obj15_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj15_a_radar2_obj15_flag_meas_decode(r2_obj15_a.radar2_obj15_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj15_a_radar2_obj15_flag_meas_is_in_range(r2_obj15_a.radar2_obj15_flag_meas);
              flag_hist_decoded = xgu_radar2_obj15_a_radar2_obj15_flag_hist_decode(r2_obj15_a.radar2_obj15_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj15_a_radar2_obj15_flag_hist_is_in_range(r2_obj15_a.radar2_obj15_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj15_a_radar2_obj15_mess_aconsist_bit_decode(r2_obj15_a.radar2_obj15_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj15_a_radar2_obj15_mess_aconsist_bit_is_in_range(
                  r2_obj15_a.radar2_obj15_mess_aconsist_bit);
              break;

            case 1447:
              xgu_radar2_obj16_a_t r2_obj16_a;

              target_info.unpack_return = xgu_radar2_obj16_a_unpack(&r2_obj16_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj16_a_radar2_obj16_dx_decode(r2_obj16_a.radar2_obj16_dx);
              dx_is_in_range = xgu_radar2_obj16_a_radar2_obj16_dx_is_in_range(r2_obj16_a.radar2_obj16_dx);
              vx_decoded = xgu_radar2_obj16_a_radar2_obj16_vx_decode(r2_obj16_a.radar2_obj16_vx);
              vx_is_in_range = xgu_radar2_obj16_a_radar2_obj16_vx_is_in_range(r2_obj16_a.radar2_obj16_vx);
              dy_decoded = xgu_radar2_obj16_a_radar2_obj16_dy_decode(r2_obj16_a.radar2_obj16_dy);
              dy_is_in_range = xgu_radar2_obj16_a_radar2_obj16_dy_is_in_range(r2_obj16_a.radar2_obj16_dy);
              w_exist_decoded = xgu_radar2_obj16_a_radar2_obj16_w_exist_decode(r2_obj16_a.radar2_obj16_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj16_a_radar2_obj16_w_exist_is_in_range(r2_obj16_a.radar2_obj16_w_exist);
              ax_decoded = xgu_radar2_obj16_a_radar2_obj16_ax_decode(r2_obj16_a.radar2_obj16_ax);
              ax_is_in_range = xgu_radar2_obj16_a_radar2_obj16_ax_is_in_range(r2_obj16_a.radar2_obj16_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj16_a_radar2_obj16_w_obstacle_decode(r2_obj16_a.radar2_obj16_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj16_a_radar2_obj16_w_obstacle_is_in_range(r2_obj16_a.radar2_obj16_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj16_a_radar2_obj16_flag_valid_decode(r2_obj16_a.radar2_obj16_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj16_a_radar2_obj16_flag_valid_is_in_range(r2_obj16_a.radar2_obj16_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj16_a_radar2_obj16_w_non_obstacle_decode(r2_obj16_a.radar2_obj16_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj16_a_radar2_obj16_w_non_obstacle_is_in_range(r2_obj16_a.radar2_obj16_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj16_a_radar2_obj16_flag_meas_decode(r2_obj16_a.radar2_obj16_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj16_a_radar2_obj16_flag_meas_is_in_range(r2_obj16_a.radar2_obj16_flag_meas);
              flag_hist_decoded = xgu_radar2_obj16_a_radar2_obj16_flag_hist_decode(r2_obj16_a.radar2_obj16_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj16_a_radar2_obj16_flag_hist_is_in_range(r2_obj16_a.radar2_obj16_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj16_a_radar2_obj16_mess_aconsist_bit_decode(r2_obj16_a.radar2_obj16_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj16_a_radar2_obj16_mess_aconsist_bit_is_in_range(
                  r2_obj16_a.radar2_obj16_mess_aconsist_bit);
              break;

            case 1457:
              xgu_radar2_obj17_a_t r2_obj17_a;

              target_info.unpack_return = xgu_radar2_obj17_a_unpack(&r2_obj17_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj17_a_radar2_obj17_dx_decode(r2_obj17_a.radar2_obj17_dx);
              dx_is_in_range = xgu_radar2_obj17_a_radar2_obj17_dx_is_in_range(r2_obj17_a.radar2_obj17_dx);
              vx_decoded = xgu_radar2_obj17_a_radar2_obj17_vx_decode(r2_obj17_a.radar2_obj17_vx);
              vx_is_in_range = xgu_radar2_obj17_a_radar2_obj17_vx_is_in_range(r2_obj17_a.radar2_obj17_vx);
              dy_decoded = xgu_radar2_obj17_a_radar2_obj17_dy_decode(r2_obj17_a.radar2_obj17_dy);
              dy_is_in_range = xgu_radar2_obj17_a_radar2_obj17_dy_is_in_range(r2_obj17_a.radar2_obj17_dy);
              w_exist_decoded = xgu_radar2_obj17_a_radar2_obj17_w_exist_decode(r2_obj17_a.radar2_obj17_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj17_a_radar2_obj17_w_exist_is_in_range(r2_obj17_a.radar2_obj17_w_exist);
              ax_decoded = xgu_radar2_obj17_a_radar2_obj17_ax_decode(r2_obj17_a.radar2_obj17_ax);
              ax_is_in_range = xgu_radar2_obj17_a_radar2_obj17_ax_is_in_range(r2_obj17_a.radar2_obj17_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj17_a_radar2_obj17_w_obstacle_decode(r2_obj17_a.radar2_obj17_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj17_a_radar2_obj17_w_obstacle_is_in_range(r2_obj17_a.radar2_obj17_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj17_a_radar2_obj17_flag_valid_decode(r2_obj17_a.radar2_obj17_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj17_a_radar2_obj17_flag_valid_is_in_range(r2_obj17_a.radar2_obj17_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj17_a_radar2_obj17_w_non_obstacle_decode(r2_obj17_a.radar2_obj17_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj17_a_radar2_obj17_w_non_obstacle_is_in_range(r2_obj17_a.radar2_obj17_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj17_a_radar2_obj17_flag_meas_decode(r2_obj17_a.radar2_obj17_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj17_a_radar2_obj17_flag_meas_is_in_range(r2_obj17_a.radar2_obj17_flag_meas);
              flag_hist_decoded = xgu_radar2_obj17_a_radar2_obj17_flag_hist_decode(r2_obj17_a.radar2_obj17_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj17_a_radar2_obj17_flag_hist_is_in_range(r2_obj17_a.radar2_obj17_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj17_a_radar2_obj17_mess_aconsist_bit_decode(r2_obj17_a.radar2_obj17_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj17_a_radar2_obj17_mess_aconsist_bit_is_in_range(
                  r2_obj17_a.radar2_obj17_mess_aconsist_bit);
              break;

            case 1467:
              xgu_radar2_obj18_a_t r2_obj18_a;

              target_info.unpack_return = xgu_radar2_obj18_a_unpack(&r2_obj18_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj18_a_radar2_obj18_dx_decode(r2_obj18_a.radar2_obj18_dx);
              dx_is_in_range = xgu_radar2_obj18_a_radar2_obj18_dx_is_in_range(r2_obj18_a.radar2_obj18_dx);
              vx_decoded = xgu_radar2_obj18_a_radar2_obj18_vx_decode(r2_obj18_a.radar2_obj18_vx);
              vx_is_in_range = xgu_radar2_obj18_a_radar2_obj18_vx_is_in_range(r2_obj18_a.radar2_obj18_vx);
              dy_decoded = xgu_radar2_obj18_a_radar2_obj18_dy_decode(r2_obj18_a.radar2_obj18_dy);
              dy_is_in_range = xgu_radar2_obj18_a_radar2_obj18_dy_is_in_range(r2_obj18_a.radar2_obj18_dy);
              w_exist_decoded = xgu_radar2_obj18_a_radar2_obj18_w_exist_decode(r2_obj18_a.radar2_obj18_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj18_a_radar2_obj18_w_exist_is_in_range(r2_obj18_a.radar2_obj18_w_exist);
              ax_decoded = xgu_radar2_obj18_a_radar2_obj18_ax_decode(r2_obj18_a.radar2_obj18_ax);
              ax_is_in_range = xgu_radar2_obj18_a_radar2_obj18_ax_is_in_range(r2_obj18_a.radar2_obj18_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj18_a_radar2_obj18_w_obstacle_decode(r2_obj18_a.radar2_obj18_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj18_a_radar2_obj18_w_obstacle_is_in_range(r2_obj18_a.radar2_obj18_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj18_a_radar2_obj18_flag_valid_decode(r2_obj18_a.radar2_obj18_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj18_a_radar2_obj18_flag_valid_is_in_range(r2_obj18_a.radar2_obj18_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj18_a_radar2_obj18_w_non_obstacle_decode(r2_obj18_a.radar2_obj18_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj18_a_radar2_obj18_w_non_obstacle_is_in_range(r2_obj18_a.radar2_obj18_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj18_a_radar2_obj18_flag_meas_decode(r2_obj18_a.radar2_obj18_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj18_a_radar2_obj18_flag_meas_is_in_range(r2_obj18_a.radar2_obj18_flag_meas);
              flag_hist_decoded = xgu_radar2_obj18_a_radar2_obj18_flag_hist_decode(r2_obj18_a.radar2_obj18_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj18_a_radar2_obj18_flag_hist_is_in_range(r2_obj18_a.radar2_obj18_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj18_a_radar2_obj18_mess_aconsist_bit_decode(r2_obj18_a.radar2_obj18_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj18_a_radar2_obj18_mess_aconsist_bit_is_in_range(
                  r2_obj18_a.radar2_obj18_mess_aconsist_bit);
              break;

            case 1477:
              xgu_radar2_obj19_a_t r2_obj19_a;

              target_info.unpack_return = xgu_radar2_obj19_a_unpack(&r2_obj19_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj19_a_radar2_obj19_dx_decode(r2_obj19_a.radar2_obj19_dx);
              dx_is_in_range = xgu_radar2_obj19_a_radar2_obj19_dx_is_in_range(r2_obj19_a.radar2_obj19_dx);
              vx_decoded = xgu_radar2_obj19_a_radar2_obj19_vx_decode(r2_obj19_a.radar2_obj19_vx);
              vx_is_in_range = xgu_radar2_obj19_a_radar2_obj19_vx_is_in_range(r2_obj19_a.radar2_obj19_vx);
              dy_decoded = xgu_radar2_obj19_a_radar2_obj19_dy_decode(r2_obj19_a.radar2_obj19_dy);
              dy_is_in_range = xgu_radar2_obj19_a_radar2_obj19_dy_is_in_range(r2_obj19_a.radar2_obj19_dy);
              w_exist_decoded = xgu_radar2_obj19_a_radar2_obj19_w_exist_decode(r2_obj19_a.radar2_obj19_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj19_a_radar2_obj19_w_exist_is_in_range(r2_obj19_a.radar2_obj19_w_exist);
              ax_decoded = xgu_radar2_obj19_a_radar2_obj19_ax_decode(r2_obj19_a.radar2_obj19_ax);
              ax_is_in_range = xgu_radar2_obj19_a_radar2_obj19_ax_is_in_range(r2_obj19_a.radar2_obj19_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj19_a_radar2_obj19_w_obstacle_decode(r2_obj19_a.radar2_obj19_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj19_a_radar2_obj19_w_obstacle_is_in_range(r2_obj19_a.radar2_obj19_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj19_a_radar2_obj19_flag_valid_decode(r2_obj19_a.radar2_obj19_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj19_a_radar2_obj19_flag_valid_is_in_range(r2_obj19_a.radar2_obj19_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj19_a_radar2_obj19_w_non_obstacle_decode(r2_obj19_a.radar2_obj19_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj19_a_radar2_obj19_w_non_obstacle_is_in_range(r2_obj19_a.radar2_obj19_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj19_a_radar2_obj19_flag_meas_decode(r2_obj19_a.radar2_obj19_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj19_a_radar2_obj19_flag_meas_is_in_range(r2_obj19_a.radar2_obj19_flag_meas);
              flag_hist_decoded = xgu_radar2_obj19_a_radar2_obj19_flag_hist_decode(r2_obj19_a.radar2_obj19_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj19_a_radar2_obj19_flag_hist_is_in_range(r2_obj19_a.radar2_obj19_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj19_a_radar2_obj19_mess_aconsist_bit_decode(r2_obj19_a.radar2_obj19_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj19_a_radar2_obj19_mess_aconsist_bit_is_in_range(
                  r2_obj19_a.radar2_obj19_mess_aconsist_bit);
              break;

            case 1487:
              xgu_radar2_obj20_a_t r2_obj20_a;

              target_info.unpack_return = xgu_radar2_obj20_a_unpack(&r2_obj20_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj20_a_radar2_obj20_dx_decode(r2_obj20_a.radar2_obj20_dx);
              dx_is_in_range = xgu_radar2_obj20_a_radar2_obj20_dx_is_in_range(r2_obj20_a.radar2_obj20_dx);
              vx_decoded = xgu_radar2_obj20_a_radar2_obj20_vx_decode(r2_obj20_a.radar2_obj20_vx);
              vx_is_in_range = xgu_radar2_obj20_a_radar2_obj20_vx_is_in_range(r2_obj20_a.radar2_obj20_vx);
              dy_decoded = xgu_radar2_obj20_a_radar2_obj20_dy_decode(r2_obj20_a.radar2_obj20_dy);
              dy_is_in_range = xgu_radar2_obj20_a_radar2_obj20_dy_is_in_range(r2_obj20_a.radar2_obj20_dy);
              w_exist_decoded = xgu_radar2_obj20_a_radar2_obj20_w_exist_decode(r2_obj20_a.radar2_obj20_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj20_a_radar2_obj20_w_exist_is_in_range(r2_obj20_a.radar2_obj20_w_exist);
              ax_decoded = xgu_radar2_obj20_a_radar2_obj20_ax_decode(r2_obj20_a.radar2_obj20_ax);
              ax_is_in_range = xgu_radar2_obj20_a_radar2_obj20_ax_is_in_range(r2_obj20_a.radar2_obj20_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj20_a_radar2_obj20_w_obstacle_decode(r2_obj20_a.radar2_obj20_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj20_a_radar2_obj20_w_obstacle_is_in_range(r2_obj20_a.radar2_obj20_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj20_a_radar2_obj20_flag_valid_decode(r2_obj20_a.radar2_obj20_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj20_a_radar2_obj20_flag_valid_is_in_range(r2_obj20_a.radar2_obj20_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj20_a_radar2_obj20_w_non_obstacle_decode(r2_obj20_a.radar2_obj20_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj20_a_radar2_obj20_w_non_obstacle_is_in_range(r2_obj20_a.radar2_obj20_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj20_a_radar2_obj20_flag_meas_decode(r2_obj20_a.radar2_obj20_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj20_a_radar2_obj20_flag_meas_is_in_range(r2_obj20_a.radar2_obj20_flag_meas);
              flag_hist_decoded = xgu_radar2_obj20_a_radar2_obj20_flag_hist_decode(r2_obj20_a.radar2_obj20_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj20_a_radar2_obj20_flag_hist_is_in_range(r2_obj20_a.radar2_obj20_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj20_a_radar2_obj20_mess_aconsist_bit_decode(r2_obj20_a.radar2_obj20_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj20_a_radar2_obj20_mess_aconsist_bit_is_in_range(
                  r2_obj20_a.radar2_obj20_mess_aconsist_bit);
              break;

            case 1497:
              xgu_radar2_obj21_a_t r2_obj21_a;

              target_info.unpack_return = xgu_radar2_obj21_a_unpack(&r2_obj21_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj21_a_radar2_obj21_dx_decode(r2_obj21_a.radar2_obj21_dx);
              dx_is_in_range = xgu_radar2_obj21_a_radar2_obj21_dx_is_in_range(r2_obj21_a.radar2_obj21_dx);
              vx_decoded = xgu_radar2_obj21_a_radar2_obj21_vx_decode(r2_obj21_a.radar2_obj21_vx);
              vx_is_in_range = xgu_radar2_obj21_a_radar2_obj21_vx_is_in_range(r2_obj21_a.radar2_obj21_vx);
              dy_decoded = xgu_radar2_obj21_a_radar2_obj21_dy_decode(r2_obj21_a.radar2_obj21_dy);
              dy_is_in_range = xgu_radar2_obj21_a_radar2_obj21_dy_is_in_range(r2_obj21_a.radar2_obj21_dy);
              w_exist_decoded = xgu_radar2_obj21_a_radar2_obj21_w_exist_decode(r2_obj21_a.radar2_obj21_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj21_a_radar2_obj21_w_exist_is_in_range(r2_obj21_a.radar2_obj21_w_exist);
              ax_decoded = xgu_radar2_obj21_a_radar2_obj21_ax_decode(r2_obj21_a.radar2_obj21_ax);
              ax_is_in_range = xgu_radar2_obj21_a_radar2_obj21_ax_is_in_range(r2_obj21_a.radar2_obj21_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj21_a_radar2_obj21_w_obstacle_decode(r2_obj21_a.radar2_obj21_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj21_a_radar2_obj21_w_obstacle_is_in_range(r2_obj21_a.radar2_obj21_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj21_a_radar2_obj21_flag_valid_decode(r2_obj21_a.radar2_obj21_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj21_a_radar2_obj21_flag_valid_is_in_range(r2_obj21_a.radar2_obj21_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj21_a_radar2_obj21_w_non_obstacle_decode(r2_obj21_a.radar2_obj21_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj21_a_radar2_obj21_w_non_obstacle_is_in_range(r2_obj21_a.radar2_obj21_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj21_a_radar2_obj21_flag_meas_decode(r2_obj21_a.radar2_obj21_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj21_a_radar2_obj21_flag_meas_is_in_range(r2_obj21_a.radar2_obj21_flag_meas);
              flag_hist_decoded = xgu_radar2_obj21_a_radar2_obj21_flag_hist_decode(r2_obj21_a.radar2_obj21_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj21_a_radar2_obj21_flag_hist_is_in_range(r2_obj21_a.radar2_obj21_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj21_a_radar2_obj21_mess_aconsist_bit_decode(r2_obj21_a.radar2_obj21_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj21_a_radar2_obj21_mess_aconsist_bit_is_in_range(
                  r2_obj21_a.radar2_obj21_mess_aconsist_bit);
              break;

            case 1507:
              xgu_radar2_obj22_a_t r2_obj22_a;

              target_info.unpack_return = xgu_radar2_obj22_a_unpack(&r2_obj22_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj22_a_radar2_obj22_dx_decode(r2_obj22_a.radar2_obj22_dx);
              dx_is_in_range = xgu_radar2_obj22_a_radar2_obj22_dx_is_in_range(r2_obj22_a.radar2_obj22_dx);
              vx_decoded = xgu_radar2_obj22_a_radar2_obj22_vx_decode(r2_obj22_a.radar2_obj22_vx);
              vx_is_in_range = xgu_radar2_obj22_a_radar2_obj22_vx_is_in_range(r2_obj22_a.radar2_obj22_vx);
              dy_decoded = xgu_radar2_obj22_a_radar2_obj22_dy_decode(r2_obj22_a.radar2_obj22_dy);
              dy_is_in_range = xgu_radar2_obj22_a_radar2_obj22_dy_is_in_range(r2_obj22_a.radar2_obj22_dy);
              w_exist_decoded = xgu_radar2_obj22_a_radar2_obj22_w_exist_decode(r2_obj22_a.radar2_obj22_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj22_a_radar2_obj22_w_exist_is_in_range(r2_obj22_a.radar2_obj22_w_exist);
              ax_decoded = xgu_radar2_obj22_a_radar2_obj22_ax_decode(r2_obj22_a.radar2_obj22_ax);
              ax_is_in_range = xgu_radar2_obj22_a_radar2_obj22_ax_is_in_range(r2_obj22_a.radar2_obj22_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj22_a_radar2_obj22_w_obstacle_decode(r2_obj22_a.radar2_obj22_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj22_a_radar2_obj22_w_obstacle_is_in_range(r2_obj22_a.radar2_obj22_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj22_a_radar2_obj22_flag_valid_decode(r2_obj22_a.radar2_obj22_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj22_a_radar2_obj22_flag_valid_is_in_range(r2_obj22_a.radar2_obj22_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj22_a_radar2_obj22_w_non_obstacle_decode(r2_obj22_a.radar2_obj22_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj22_a_radar2_obj22_w_non_obstacle_is_in_range(r2_obj22_a.radar2_obj22_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj22_a_radar2_obj22_flag_meas_decode(r2_obj22_a.radar2_obj22_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj22_a_radar2_obj22_flag_meas_is_in_range(r2_obj22_a.radar2_obj22_flag_meas);
              flag_hist_decoded = xgu_radar2_obj22_a_radar2_obj22_flag_hist_decode(r2_obj22_a.radar2_obj22_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj22_a_radar2_obj22_flag_hist_is_in_range(r2_obj22_a.radar2_obj22_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj22_a_radar2_obj22_mess_aconsist_bit_decode(r2_obj22_a.radar2_obj22_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj22_a_radar2_obj22_mess_aconsist_bit_is_in_range(
                  r2_obj22_a.radar2_obj22_mess_aconsist_bit);
              break;

            case 1517:
              xgu_radar2_obj23_a_t r2_obj23_a;

              target_info.unpack_return = xgu_radar2_obj23_a_unpack(&r2_obj23_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj23_a_radar2_obj23_dx_decode(r2_obj23_a.radar2_obj23_dx);
              dx_is_in_range = xgu_radar2_obj23_a_radar2_obj23_dx_is_in_range(r2_obj23_a.radar2_obj23_dx);
              vx_decoded = xgu_radar2_obj23_a_radar2_obj23_vx_decode(r2_obj23_a.radar2_obj23_vx);
              vx_is_in_range = xgu_radar2_obj23_a_radar2_obj23_vx_is_in_range(r2_obj23_a.radar2_obj23_vx);
              dy_decoded = xgu_radar2_obj23_a_radar2_obj23_dy_decode(r2_obj23_a.radar2_obj23_dy);
              dy_is_in_range = xgu_radar2_obj23_a_radar2_obj23_dy_is_in_range(r2_obj23_a.radar2_obj23_dy);
              w_exist_decoded = xgu_radar2_obj23_a_radar2_obj23_w_exist_decode(r2_obj23_a.radar2_obj23_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj23_a_radar2_obj23_w_exist_is_in_range(r2_obj23_a.radar2_obj23_w_exist);
              ax_decoded = xgu_radar2_obj23_a_radar2_obj23_ax_decode(r2_obj23_a.radar2_obj23_ax);
              ax_is_in_range = xgu_radar2_obj23_a_radar2_obj23_ax_is_in_range(r2_obj23_a.radar2_obj23_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj23_a_radar2_obj23_w_obstacle_decode(r2_obj23_a.radar2_obj23_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj23_a_radar2_obj23_w_obstacle_is_in_range(r2_obj23_a.radar2_obj23_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj23_a_radar2_obj23_flag_valid_decode(r2_obj23_a.radar2_obj23_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj23_a_radar2_obj23_flag_valid_is_in_range(r2_obj23_a.radar2_obj23_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj23_a_radar2_obj23_w_non_obstacle_decode(r2_obj23_a.radar2_obj23_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj23_a_radar2_obj23_w_non_obstacle_is_in_range(r2_obj23_a.radar2_obj23_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj23_a_radar2_obj23_flag_meas_decode(r2_obj23_a.radar2_obj23_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj23_a_radar2_obj23_flag_meas_is_in_range(r2_obj23_a.radar2_obj23_flag_meas);
              flag_hist_decoded = xgu_radar2_obj23_a_radar2_obj23_flag_hist_decode(r2_obj23_a.radar2_obj23_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj23_a_radar2_obj23_flag_hist_is_in_range(r2_obj23_a.radar2_obj23_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj23_a_radar2_obj23_mess_aconsist_bit_decode(r2_obj23_a.radar2_obj23_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj23_a_radar2_obj23_mess_aconsist_bit_is_in_range(
                  r2_obj23_a.radar2_obj23_mess_aconsist_bit);
              break;

            case 1527:
              xgu_radar2_obj24_a_t r2_obj24_a;

              target_info.unpack_return = xgu_radar2_obj24_a_unpack(&r2_obj24_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj24_a_radar2_obj24_dx_decode(r2_obj24_a.radar2_obj24_dx);
              dx_is_in_range = xgu_radar2_obj24_a_radar2_obj24_dx_is_in_range(r2_obj24_a.radar2_obj24_dx);
              vx_decoded = xgu_radar2_obj24_a_radar2_obj24_vx_decode(r2_obj24_a.radar2_obj24_vx);
              vx_is_in_range = xgu_radar2_obj24_a_radar2_obj24_vx_is_in_range(r2_obj24_a.radar2_obj24_vx);
              dy_decoded = xgu_radar2_obj24_a_radar2_obj24_dy_decode(r2_obj24_a.radar2_obj24_dy);
              dy_is_in_range = xgu_radar2_obj24_a_radar2_obj24_dy_is_in_range(r2_obj24_a.radar2_obj24_dy);
              w_exist_decoded = xgu_radar2_obj24_a_radar2_obj24_w_exist_decode(r2_obj24_a.radar2_obj24_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj24_a_radar2_obj24_w_exist_is_in_range(r2_obj24_a.radar2_obj24_w_exist);
              ax_decoded = xgu_radar2_obj24_a_radar2_obj24_ax_decode(r2_obj24_a.radar2_obj24_ax);
              ax_is_in_range = xgu_radar2_obj24_a_radar2_obj24_ax_is_in_range(r2_obj24_a.radar2_obj24_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj24_a_radar2_obj24_w_obstacle_decode(r2_obj24_a.radar2_obj24_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj24_a_radar2_obj24_w_obstacle_is_in_range(r2_obj24_a.radar2_obj24_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj24_a_radar2_obj24_flag_valid_decode(r2_obj24_a.radar2_obj24_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj24_a_radar2_obj24_flag_valid_is_in_range(r2_obj24_a.radar2_obj24_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj24_a_radar2_obj24_w_non_obstacle_decode(r2_obj24_a.radar2_obj24_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj24_a_radar2_obj24_w_non_obstacle_is_in_range(r2_obj24_a.radar2_obj24_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj24_a_radar2_obj24_flag_meas_decode(r2_obj24_a.radar2_obj24_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj24_a_radar2_obj24_flag_meas_is_in_range(r2_obj24_a.radar2_obj24_flag_meas);
              flag_hist_decoded = xgu_radar2_obj24_a_radar2_obj24_flag_hist_decode(r2_obj24_a.radar2_obj24_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj24_a_radar2_obj24_flag_hist_is_in_range(r2_obj24_a.radar2_obj24_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj24_a_radar2_obj24_mess_aconsist_bit_decode(r2_obj24_a.radar2_obj24_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj24_a_radar2_obj24_mess_aconsist_bit_is_in_range(
                  r2_obj24_a.radar2_obj24_mess_aconsist_bit);
              break;

            case 1537:
              xgu_radar2_obj25_a_t r2_obj25_a;

              target_info.unpack_return = xgu_radar2_obj25_a_unpack(&r2_obj25_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj25_a_radar2_obj25_dx_decode(r2_obj25_a.radar2_obj25_dx);
              dx_is_in_range = xgu_radar2_obj25_a_radar2_obj25_dx_is_in_range(r2_obj25_a.radar2_obj25_dx);
              vx_decoded = xgu_radar2_obj25_a_radar2_obj25_vx_decode(r2_obj25_a.radar2_obj25_vx);
              vx_is_in_range = xgu_radar2_obj25_a_radar2_obj25_vx_is_in_range(r2_obj25_a.radar2_obj25_vx);
              dy_decoded = xgu_radar2_obj25_a_radar2_obj25_dy_decode(r2_obj25_a.radar2_obj25_dy);
              dy_is_in_range = xgu_radar2_obj25_a_radar2_obj25_dy_is_in_range(r2_obj25_a.radar2_obj25_dy);
              w_exist_decoded = xgu_radar2_obj25_a_radar2_obj25_w_exist_decode(r2_obj25_a.radar2_obj25_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj25_a_radar2_obj25_w_exist_is_in_range(r2_obj25_a.radar2_obj25_w_exist);
              ax_decoded = xgu_radar2_obj25_a_radar2_obj25_ax_decode(r2_obj25_a.radar2_obj25_ax);
              ax_is_in_range = xgu_radar2_obj25_a_radar2_obj25_ax_is_in_range(r2_obj25_a.radar2_obj25_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj25_a_radar2_obj25_w_obstacle_decode(r2_obj25_a.radar2_obj25_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj25_a_radar2_obj25_w_obstacle_is_in_range(r2_obj25_a.radar2_obj25_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj25_a_radar2_obj25_flag_valid_decode(r2_obj25_a.radar2_obj25_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj25_a_radar2_obj25_flag_valid_is_in_range(r2_obj25_a.radar2_obj25_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj25_a_radar2_obj25_w_non_obstacle_decode(r2_obj25_a.radar2_obj25_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj25_a_radar2_obj25_w_non_obstacle_is_in_range(r2_obj25_a.radar2_obj25_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj25_a_radar2_obj25_flag_meas_decode(r2_obj25_a.radar2_obj25_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj25_a_radar2_obj25_flag_meas_is_in_range(r2_obj25_a.radar2_obj25_flag_meas);
              flag_hist_decoded = xgu_radar2_obj25_a_radar2_obj25_flag_hist_decode(r2_obj25_a.radar2_obj25_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj25_a_radar2_obj25_flag_hist_is_in_range(r2_obj25_a.radar2_obj25_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj25_a_radar2_obj25_mess_aconsist_bit_decode(r2_obj25_a.radar2_obj25_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj25_a_radar2_obj25_mess_aconsist_bit_is_in_range(
                  r2_obj25_a.radar2_obj25_mess_aconsist_bit);
              break;

            case 1547:
              xgu_radar2_obj26_a_t r2_obj26_a;

              target_info.unpack_return = xgu_radar2_obj26_a_unpack(&r2_obj26_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj26_a_radar2_obj26_dx_decode(r2_obj26_a.radar2_obj26_dx);
              dx_is_in_range = xgu_radar2_obj26_a_radar2_obj26_dx_is_in_range(r2_obj26_a.radar2_obj26_dx);
              vx_decoded = xgu_radar2_obj26_a_radar2_obj26_vx_decode(r2_obj26_a.radar2_obj26_vx);
              vx_is_in_range = xgu_radar2_obj26_a_radar2_obj26_vx_is_in_range(r2_obj26_a.radar2_obj26_vx);
              dy_decoded = xgu_radar2_obj26_a_radar2_obj26_dy_decode(r2_obj26_a.radar2_obj26_dy);
              dy_is_in_range = xgu_radar2_obj26_a_radar2_obj26_dy_is_in_range(r2_obj26_a.radar2_obj26_dy);
              w_exist_decoded = xgu_radar2_obj26_a_radar2_obj26_w_exist_decode(r2_obj26_a.radar2_obj26_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj26_a_radar2_obj26_w_exist_is_in_range(r2_obj26_a.radar2_obj26_w_exist);
              ax_decoded = xgu_radar2_obj26_a_radar2_obj26_ax_decode(r2_obj26_a.radar2_obj26_ax);
              ax_is_in_range = xgu_radar2_obj26_a_radar2_obj26_ax_is_in_range(r2_obj26_a.radar2_obj26_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj26_a_radar2_obj26_w_obstacle_decode(r2_obj26_a.radar2_obj26_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj26_a_radar2_obj26_w_obstacle_is_in_range(r2_obj26_a.radar2_obj26_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj26_a_radar2_obj26_flag_valid_decode(r2_obj26_a.radar2_obj26_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj26_a_radar2_obj26_flag_valid_is_in_range(r2_obj26_a.radar2_obj26_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj26_a_radar2_obj26_w_non_obstacle_decode(r2_obj26_a.radar2_obj26_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj26_a_radar2_obj26_w_non_obstacle_is_in_range(r2_obj26_a.radar2_obj26_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj26_a_radar2_obj26_flag_meas_decode(r2_obj26_a.radar2_obj26_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj26_a_radar2_obj26_flag_meas_is_in_range(r2_obj26_a.radar2_obj26_flag_meas);
              flag_hist_decoded = xgu_radar2_obj26_a_radar2_obj26_flag_hist_decode(r2_obj26_a.radar2_obj26_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj26_a_radar2_obj26_flag_hist_is_in_range(r2_obj26_a.radar2_obj26_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj26_a_radar2_obj26_mess_aconsist_bit_decode(r2_obj26_a.radar2_obj26_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj26_a_radar2_obj26_mess_aconsist_bit_is_in_range(
                  r2_obj26_a.radar2_obj26_mess_aconsist_bit);
              break;

            case 1557:
              xgu_radar2_obj27_a_t r2_obj27_a;

              target_info.unpack_return = xgu_radar2_obj27_a_unpack(&r2_obj27_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj27_a_radar2_obj27_dx_decode(r2_obj27_a.radar2_obj27_dx);
              dx_is_in_range = xgu_radar2_obj27_a_radar2_obj27_dx_is_in_range(r2_obj27_a.radar2_obj27_dx);
              vx_decoded = xgu_radar2_obj27_a_radar2_obj27_vx_decode(r2_obj27_a.radar2_obj27_vx);
              vx_is_in_range = xgu_radar2_obj27_a_radar2_obj27_vx_is_in_range(r2_obj27_a.radar2_obj27_vx);
              dy_decoded = xgu_radar2_obj27_a_radar2_obj27_dy_decode(r2_obj27_a.radar2_obj27_dy);
              dy_is_in_range = xgu_radar2_obj27_a_radar2_obj27_dy_is_in_range(r2_obj27_a.radar2_obj27_dy);
              w_exist_decoded = xgu_radar2_obj27_a_radar2_obj27_w_exist_decode(r2_obj27_a.radar2_obj27_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj27_a_radar2_obj27_w_exist_is_in_range(r2_obj27_a.radar2_obj27_w_exist);
              ax_decoded = xgu_radar2_obj27_a_radar2_obj27_ax_decode(r2_obj27_a.radar2_obj27_ax);
              ax_is_in_range = xgu_radar2_obj27_a_radar2_obj27_ax_is_in_range(r2_obj27_a.radar2_obj27_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj27_a_radar2_obj27_w_obstacle_decode(r2_obj27_a.radar2_obj27_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj27_a_radar2_obj27_w_obstacle_is_in_range(r2_obj27_a.radar2_obj27_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj27_a_radar2_obj27_flag_valid_decode(r2_obj27_a.radar2_obj27_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj27_a_radar2_obj27_flag_valid_is_in_range(r2_obj27_a.radar2_obj27_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj27_a_radar2_obj27_w_non_obstacle_decode(r2_obj27_a.radar2_obj27_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj27_a_radar2_obj27_w_non_obstacle_is_in_range(r2_obj27_a.radar2_obj27_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj27_a_radar2_obj27_flag_meas_decode(r2_obj27_a.radar2_obj27_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj27_a_radar2_obj27_flag_meas_is_in_range(r2_obj27_a.radar2_obj27_flag_meas);
              flag_hist_decoded = xgu_radar2_obj27_a_radar2_obj27_flag_hist_decode(r2_obj27_a.radar2_obj27_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj27_a_radar2_obj27_flag_hist_is_in_range(r2_obj27_a.radar2_obj27_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj27_a_radar2_obj27_mess_aconsist_bit_decode(r2_obj27_a.radar2_obj27_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj27_a_radar2_obj27_mess_aconsist_bit_is_in_range(
                  r2_obj27_a.radar2_obj27_mess_aconsist_bit);
              break;

            case 1567:
              xgu_radar2_obj28_a_t r2_obj28_a;

              target_info.unpack_return = xgu_radar2_obj28_a_unpack(&r2_obj28_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj28_a_radar2_obj28_dx_decode(r2_obj28_a.radar2_obj28_dx);
              dx_is_in_range = xgu_radar2_obj28_a_radar2_obj28_dx_is_in_range(r2_obj28_a.radar2_obj28_dx);
              vx_decoded = xgu_radar2_obj28_a_radar2_obj28_vx_decode(r2_obj28_a.radar2_obj28_vx);
              vx_is_in_range = xgu_radar2_obj28_a_radar2_obj28_vx_is_in_range(r2_obj28_a.radar2_obj28_vx);
              dy_decoded = xgu_radar2_obj28_a_radar2_obj28_dy_decode(r2_obj28_a.radar2_obj28_dy);
              dy_is_in_range = xgu_radar2_obj28_a_radar2_obj28_dy_is_in_range(r2_obj28_a.radar2_obj28_dy);
              w_exist_decoded = xgu_radar2_obj28_a_radar2_obj28_w_exist_decode(r2_obj28_a.radar2_obj28_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj28_a_radar2_obj28_w_exist_is_in_range(r2_obj28_a.radar2_obj28_w_exist);
              ax_decoded = xgu_radar2_obj28_a_radar2_obj28_ax_decode(r2_obj28_a.radar2_obj28_ax);
              ax_is_in_range = xgu_radar2_obj28_a_radar2_obj28_ax_is_in_range(r2_obj28_a.radar2_obj28_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj28_a_radar2_obj28_w_obstacle_decode(r2_obj28_a.radar2_obj28_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj28_a_radar2_obj28_w_obstacle_is_in_range(r2_obj28_a.radar2_obj28_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj28_a_radar2_obj28_flag_valid_decode(r2_obj28_a.radar2_obj28_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj28_a_radar2_obj28_flag_valid_is_in_range(r2_obj28_a.radar2_obj28_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj28_a_radar2_obj28_w_non_obstacle_decode(r2_obj28_a.radar2_obj28_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj28_a_radar2_obj28_w_non_obstacle_is_in_range(r2_obj28_a.radar2_obj28_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj28_a_radar2_obj28_flag_meas_decode(r2_obj28_a.radar2_obj28_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj28_a_radar2_obj28_flag_meas_is_in_range(r2_obj28_a.radar2_obj28_flag_meas);
              flag_hist_decoded = xgu_radar2_obj28_a_radar2_obj28_flag_hist_decode(r2_obj28_a.radar2_obj28_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj28_a_radar2_obj28_flag_hist_is_in_range(r2_obj28_a.radar2_obj28_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj28_a_radar2_obj28_mess_aconsist_bit_decode(r2_obj28_a.radar2_obj28_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj28_a_radar2_obj28_mess_aconsist_bit_is_in_range(
                  r2_obj28_a.radar2_obj28_mess_aconsist_bit);
              break;

            case 1577:
              xgu_radar2_obj29_a_t r2_obj29_a;

              target_info.unpack_return = xgu_radar2_obj29_a_unpack(&r2_obj29_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj29_a_radar2_obj29_dx_decode(r2_obj29_a.radar2_obj29_dx);
              dx_is_in_range = xgu_radar2_obj29_a_radar2_obj29_dx_is_in_range(r2_obj29_a.radar2_obj29_dx);
              vx_decoded = xgu_radar2_obj29_a_radar2_obj29_vx_decode(r2_obj29_a.radar2_obj29_vx);
              vx_is_in_range = xgu_radar2_obj29_a_radar2_obj29_vx_is_in_range(r2_obj29_a.radar2_obj29_vx);
              dy_decoded = xgu_radar2_obj29_a_radar2_obj29_dy_decode(r2_obj29_a.radar2_obj29_dy);
              dy_is_in_range = xgu_radar2_obj29_a_radar2_obj29_dy_is_in_range(r2_obj29_a.radar2_obj29_dy);
              w_exist_decoded = xgu_radar2_obj29_a_radar2_obj29_w_exist_decode(r2_obj29_a.radar2_obj29_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj29_a_radar2_obj29_w_exist_is_in_range(r2_obj29_a.radar2_obj29_w_exist);
              ax_decoded = xgu_radar2_obj29_a_radar2_obj29_ax_decode(r2_obj29_a.radar2_obj29_ax);
              ax_is_in_range = xgu_radar2_obj29_a_radar2_obj29_ax_is_in_range(r2_obj29_a.radar2_obj29_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj29_a_radar2_obj29_w_obstacle_decode(r2_obj29_a.radar2_obj29_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj29_a_radar2_obj29_w_obstacle_is_in_range(r2_obj29_a.radar2_obj29_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj29_a_radar2_obj29_flag_valid_decode(r2_obj29_a.radar2_obj29_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj29_a_radar2_obj29_flag_valid_is_in_range(r2_obj29_a.radar2_obj29_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj29_a_radar2_obj29_w_non_obstacle_decode(r2_obj29_a.radar2_obj29_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj29_a_radar2_obj29_w_non_obstacle_is_in_range(r2_obj29_a.radar2_obj29_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj29_a_radar2_obj29_flag_meas_decode(r2_obj29_a.radar2_obj29_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj29_a_radar2_obj29_flag_meas_is_in_range(r2_obj29_a.radar2_obj29_flag_meas);
              flag_hist_decoded = xgu_radar2_obj29_a_radar2_obj29_flag_hist_decode(r2_obj29_a.radar2_obj29_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj29_a_radar2_obj29_flag_hist_is_in_range(r2_obj29_a.radar2_obj29_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj29_a_radar2_obj29_mess_aconsist_bit_decode(r2_obj29_a.radar2_obj29_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj29_a_radar2_obj29_mess_aconsist_bit_is_in_range(
                  r2_obj29_a.radar2_obj29_mess_aconsist_bit);
              break;

            case 1587:
              xgu_radar2_obj30_a_t r2_obj30_a;

              target_info.unpack_return = xgu_radar2_obj30_a_unpack(&r2_obj30_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj30_a_radar2_obj30_dx_decode(r2_obj30_a.radar2_obj30_dx);
              dx_is_in_range = xgu_radar2_obj30_a_radar2_obj30_dx_is_in_range(r2_obj30_a.radar2_obj30_dx);
              vx_decoded = xgu_radar2_obj30_a_radar2_obj30_vx_decode(r2_obj30_a.radar2_obj30_vx);
              vx_is_in_range = xgu_radar2_obj30_a_radar2_obj30_vx_is_in_range(r2_obj30_a.radar2_obj30_vx);
              dy_decoded = xgu_radar2_obj30_a_radar2_obj30_dy_decode(r2_obj30_a.radar2_obj30_dy);
              dy_is_in_range = xgu_radar2_obj30_a_radar2_obj30_dy_is_in_range(r2_obj30_a.radar2_obj30_dy);
              w_exist_decoded = xgu_radar2_obj30_a_radar2_obj30_w_exist_decode(r2_obj30_a.radar2_obj30_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj30_a_radar2_obj30_w_exist_is_in_range(r2_obj30_a.radar2_obj30_w_exist);
              ax_decoded = xgu_radar2_obj30_a_radar2_obj30_ax_decode(r2_obj30_a.radar2_obj30_ax);
              ax_is_in_range = xgu_radar2_obj30_a_radar2_obj30_ax_is_in_range(r2_obj30_a.radar2_obj30_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj30_a_radar2_obj30_w_obstacle_decode(r2_obj30_a.radar2_obj30_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj30_a_radar2_obj30_w_obstacle_is_in_range(r2_obj30_a.radar2_obj30_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj30_a_radar2_obj30_flag_valid_decode(r2_obj30_a.radar2_obj30_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj30_a_radar2_obj30_flag_valid_is_in_range(r2_obj30_a.radar2_obj30_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj30_a_radar2_obj30_w_non_obstacle_decode(r2_obj30_a.radar2_obj30_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj30_a_radar2_obj30_w_non_obstacle_is_in_range(r2_obj30_a.radar2_obj30_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj30_a_radar2_obj30_flag_meas_decode(r2_obj30_a.radar2_obj30_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj30_a_radar2_obj30_flag_meas_is_in_range(r2_obj30_a.radar2_obj30_flag_meas);
              flag_hist_decoded = xgu_radar2_obj30_a_radar2_obj30_flag_hist_decode(r2_obj30_a.radar2_obj30_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj30_a_radar2_obj30_flag_hist_is_in_range(r2_obj30_a.radar2_obj30_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj30_a_radar2_obj30_mess_aconsist_bit_decode(r2_obj30_a.radar2_obj30_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj30_a_radar2_obj30_mess_aconsist_bit_is_in_range(
                  r2_obj30_a.radar2_obj30_mess_aconsist_bit);
              break;

            case 1597:
              xgu_radar2_obj31_a_t r2_obj31_a;

              target_info.unpack_return = xgu_radar2_obj31_a_unpack(&r2_obj31_a, can_data, size_of_msg);
              dx_decoded = xgu_radar2_obj31_a_radar2_obj31_dx_decode(r2_obj31_a.radar2_obj31_dx);
              dx_is_in_range = xgu_radar2_obj31_a_radar2_obj31_dx_is_in_range(r2_obj31_a.radar2_obj31_dx);
              vx_decoded = xgu_radar2_obj31_a_radar2_obj31_vx_decode(r2_obj31_a.radar2_obj31_vx);
              vx_is_in_range = xgu_radar2_obj31_a_radar2_obj31_vx_is_in_range(r2_obj31_a.radar2_obj31_vx);
              dy_decoded = xgu_radar2_obj31_a_radar2_obj31_dy_decode(r2_obj31_a.radar2_obj31_dy);
              dy_is_in_range = xgu_radar2_obj31_a_radar2_obj31_dy_is_in_range(r2_obj31_a.radar2_obj31_dy);
              w_exist_decoded = xgu_radar2_obj31_a_radar2_obj31_w_exist_decode(r2_obj31_a.radar2_obj31_w_exist);
              w_exist_is_in_range =
                  xgu_radar2_obj31_a_radar2_obj31_w_exist_is_in_range(r2_obj31_a.radar2_obj31_w_exist);
              ax_decoded = xgu_radar2_obj31_a_radar2_obj31_ax_decode(r2_obj31_a.radar2_obj31_ax);
              ax_is_in_range = xgu_radar2_obj31_a_radar2_obj31_ax_is_in_range(r2_obj31_a.radar2_obj31_ax);
              w_obstacle_decoded =
                  xgu_radar2_obj31_a_radar2_obj31_w_obstacle_decode(r2_obj31_a.radar2_obj31_w_obstacle);
              w_obstacle_is_in_range =
                  xgu_radar2_obj31_a_radar2_obj31_w_obstacle_is_in_range(r2_obj31_a.radar2_obj31_w_obstacle);
              flag_valid_decoded =
                  xgu_radar2_obj31_a_radar2_obj31_flag_valid_decode(r2_obj31_a.radar2_obj31_flag_valid);
              flag_valid_is_in_range =
                  xgu_radar2_obj31_a_radar2_obj31_flag_valid_is_in_range(r2_obj31_a.radar2_obj31_flag_valid);
              w_non_obstacle_decoded =
                  xgu_radar2_obj31_a_radar2_obj31_w_non_obstacle_decode(r2_obj31_a.radar2_obj31_w_non_obstacle);
              w_non_obstacle_is_in_range =
                  xgu_radar2_obj31_a_radar2_obj31_w_non_obstacle_is_in_range(r2_obj31_a.radar2_obj31_w_non_obstacle);
              flag_meas_decoded = xgu_radar2_obj31_a_radar2_obj31_flag_meas_decode(r2_obj31_a.radar2_obj31_flag_meas);
              flag_meas_is_in_range =
                  xgu_radar2_obj31_a_radar2_obj31_flag_meas_is_in_range(r2_obj31_a.radar2_obj31_flag_meas);
              flag_hist_decoded = xgu_radar2_obj31_a_radar2_obj31_flag_hist_decode(r2_obj31_a.radar2_obj31_flag_hist);
              flag_hist_is_in_range =
                  xgu_radar2_obj31_a_radar2_obj31_flag_hist_is_in_range(r2_obj31_a.radar2_obj31_flag_hist);
              mess_aconsist_bit_decoded =
                  xgu_radar2_obj31_a_radar2_obj31_mess_aconsist_bit_decode(r2_obj31_a.radar2_obj31_mess_aconsist_bit);
              mess_aconsist_bit_is_in_range = xgu_radar2_obj31_a_radar2_obj31_mess_aconsist_bit_is_in_range(
                  r2_obj31_a.radar2_obj31_mess_aconsist_bit);
              break;

            case 1288:
              xgu_radar2_obj00_b_t r2_obj00_b;

              vy_decoded = xgu_radar2_obj00_b_radar2_obj00_vy_decode(r2_obj00_b.radar2_obj00_vy);
              vy_is_in_range = xgu_radar2_obj00_b_radar2_obj00_vy_is_in_range(r2_obj00_b.radar2_obj00_vy);
              d_length_decoded = xgu_radar2_obj00_b_radar2_obj00_d_length_decode(r2_obj00_b.radar2_obj00_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_d_length_is_in_range(r2_obj00_b.radar2_obj00_d_length);
              dz_decoded = xgu_radar2_obj00_b_radar2_obj00_dz_decode(r2_obj00_b.radar2_obj00_dz);
              dz_is_in_range = xgu_radar2_obj00_b_radar2_obj00_dz_is_in_range(r2_obj00_b.radar2_obj00_dz);
              moving_state_decoded =
                  xgu_radar2_obj00_b_radar2_obj00_moving_state_decode(r2_obj00_b.radar2_obj00_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_moving_state_is_in_range(r2_obj00_b.radar2_obj00_moving_state);
              dx_sigma_decoded = xgu_radar2_obj00_b_radar2_obj00_dx_sigma_decode(r2_obj00_b.radar2_obj00_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_dx_sigma_is_in_range(r2_obj00_b.radar2_obj00_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj00_b_radar2_obj00_vx_sigma_decode(r2_obj00_b.radar2_obj00_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_vx_sigma_is_in_range(r2_obj00_b.radar2_obj00_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj00_b_radar2_obj00_ax_sigma_decode(r2_obj00_b.radar2_obj00_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_ax_sigma_is_in_range(r2_obj00_b.radar2_obj00_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj00_b_radar2_obj00_dy_sigma_decode(r2_obj00_b.radar2_obj00_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_dy_sigma_is_in_range(r2_obj00_b.radar2_obj00_dy_sigma);
              w_class_decoded = xgu_radar2_obj00_b_radar2_obj00_w_class_decode(r2_obj00_b.radar2_obj00_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj00_b_radar2_obj00_w_class_is_in_range(r2_obj00_b.radar2_obj00_w_class);
              class_decoded = xgu_radar2_obj00_b_radar2_obj00_class_decode(r2_obj00_b.radar2_obj00_class);
              class_is_in_range = xgu_radar2_obj00_b_radar2_obj00_class_is_in_range(r2_obj00_b.radar2_obj00_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj00_b_radar2_obj00_dx_rear_end_loss_decode(r2_obj00_b.radar2_obj00_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj00_b_radar2_obj00_dx_rear_end_loss_is_in_range(
                  r2_obj00_b.radar2_obj00_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj00_b_radar2_obj00_mess_bconsist_bit_encode(r2_obj00_b.radar2_obj00_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj00_b_radar2_obj00_mess_bconsist_bit_is_in_range(
                  r2_obj00_b.radar2_obj00_mess_bconsist_bit);
              break;
            case 1298:
              xgu_radar2_obj01_b_t r2_obj01_b;

              vy_decoded = xgu_radar2_obj01_b_radar2_obj01_vy_decode(r2_obj01_b.radar2_obj01_vy);
              vy_is_in_range = xgu_radar2_obj01_b_radar2_obj01_vy_is_in_range(r2_obj01_b.radar2_obj01_vy);
              d_length_decoded = xgu_radar2_obj01_b_radar2_obj01_d_length_decode(r2_obj01_b.radar2_obj01_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_d_length_is_in_range(r2_obj01_b.radar2_obj01_d_length);
              dz_decoded = xgu_radar2_obj01_b_radar2_obj01_dz_decode(r2_obj01_b.radar2_obj01_dz);
              dz_is_in_range = xgu_radar2_obj01_b_radar2_obj01_dz_is_in_range(r2_obj01_b.radar2_obj01_dz);
              moving_state_decoded =
                  xgu_radar2_obj01_b_radar2_obj01_moving_state_decode(r2_obj01_b.radar2_obj01_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_moving_state_is_in_range(r2_obj01_b.radar2_obj01_moving_state);
              dx_sigma_decoded = xgu_radar2_obj01_b_radar2_obj01_dx_sigma_decode(r2_obj01_b.radar2_obj01_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_dx_sigma_is_in_range(r2_obj01_b.radar2_obj01_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj01_b_radar2_obj01_vx_sigma_decode(r2_obj01_b.radar2_obj01_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_vx_sigma_is_in_range(r2_obj01_b.radar2_obj01_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj01_b_radar2_obj01_ax_sigma_decode(r2_obj01_b.radar2_obj01_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_ax_sigma_is_in_range(r2_obj01_b.radar2_obj01_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj01_b_radar2_obj01_dy_sigma_decode(r2_obj01_b.radar2_obj01_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_dy_sigma_is_in_range(r2_obj01_b.radar2_obj01_dy_sigma);
              w_class_decoded = xgu_radar2_obj01_b_radar2_obj01_w_class_decode(r2_obj01_b.radar2_obj01_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj01_b_radar2_obj01_w_class_is_in_range(r2_obj01_b.radar2_obj01_w_class);
              class_decoded = xgu_radar2_obj01_b_radar2_obj01_class_decode(r2_obj01_b.radar2_obj01_class);
              class_is_in_range = xgu_radar2_obj01_b_radar2_obj01_class_is_in_range(r2_obj01_b.radar2_obj01_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj01_b_radar2_obj01_dx_rear_end_loss_decode(r2_obj01_b.radar2_obj01_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj01_b_radar2_obj01_dx_rear_end_loss_is_in_range(
                  r2_obj01_b.radar2_obj01_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj01_b_radar2_obj01_mess_bconsist_bit_encode(r2_obj01_b.radar2_obj01_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj01_b_radar2_obj01_mess_bconsist_bit_is_in_range(
                  r2_obj01_b.radar2_obj01_mess_bconsist_bit);
              break;
            case 1308:
              xgu_radar2_obj02_b_t r2_obj02_b;

              vy_decoded = xgu_radar2_obj02_b_radar2_obj02_vy_decode(r2_obj02_b.radar2_obj02_vy);
              vy_is_in_range = xgu_radar2_obj02_b_radar2_obj02_vy_is_in_range(r2_obj02_b.radar2_obj02_vy);
              d_length_decoded = xgu_radar2_obj02_b_radar2_obj02_d_length_decode(r2_obj02_b.radar2_obj02_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_d_length_is_in_range(r2_obj02_b.radar2_obj02_d_length);
              dz_decoded = xgu_radar2_obj02_b_radar2_obj02_dz_decode(r2_obj02_b.radar2_obj02_dz);
              dz_is_in_range = xgu_radar2_obj02_b_radar2_obj02_dz_is_in_range(r2_obj02_b.radar2_obj02_dz);
              moving_state_decoded =
                  xgu_radar2_obj02_b_radar2_obj02_moving_state_decode(r2_obj02_b.radar2_obj02_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_moving_state_is_in_range(r2_obj02_b.radar2_obj02_moving_state);
              dx_sigma_decoded = xgu_radar2_obj02_b_radar2_obj02_dx_sigma_decode(r2_obj02_b.radar2_obj02_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_dx_sigma_is_in_range(r2_obj02_b.radar2_obj02_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj02_b_radar2_obj02_vx_sigma_decode(r2_obj02_b.radar2_obj02_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_vx_sigma_is_in_range(r2_obj02_b.radar2_obj02_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj02_b_radar2_obj02_ax_sigma_decode(r2_obj02_b.radar2_obj02_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_ax_sigma_is_in_range(r2_obj02_b.radar2_obj02_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj02_b_radar2_obj02_dy_sigma_decode(r2_obj02_b.radar2_obj02_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_dy_sigma_is_in_range(r2_obj02_b.radar2_obj02_dy_sigma);
              w_class_decoded = xgu_radar2_obj02_b_radar2_obj02_w_class_decode(r2_obj02_b.radar2_obj02_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj02_b_radar2_obj02_w_class_is_in_range(r2_obj02_b.radar2_obj02_w_class);
              class_decoded = xgu_radar2_obj02_b_radar2_obj02_class_decode(r2_obj02_b.radar2_obj02_class);
              class_is_in_range = xgu_radar2_obj02_b_radar2_obj02_class_is_in_range(r2_obj02_b.radar2_obj02_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj02_b_radar2_obj02_dx_rear_end_loss_decode(r2_obj02_b.radar2_obj02_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj02_b_radar2_obj02_dx_rear_end_loss_is_in_range(
                  r2_obj02_b.radar2_obj02_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj02_b_radar2_obj02_mess_bconsist_bit_encode(r2_obj02_b.radar2_obj02_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj02_b_radar2_obj02_mess_bconsist_bit_is_in_range(
                  r2_obj02_b.radar2_obj02_mess_bconsist_bit);
              break;
            case 1318:
              xgu_radar2_obj03_b_t r2_obj03_b;

              vy_decoded = xgu_radar2_obj03_b_radar2_obj03_vy_decode(r2_obj03_b.radar2_obj03_vy);
              vy_is_in_range = xgu_radar2_obj03_b_radar2_obj03_vy_is_in_range(r2_obj03_b.radar2_obj03_vy);
              d_length_decoded = xgu_radar2_obj03_b_radar2_obj03_d_length_decode(r2_obj03_b.radar2_obj03_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_d_length_is_in_range(r2_obj03_b.radar2_obj03_d_length);
              dz_decoded = xgu_radar2_obj03_b_radar2_obj03_dz_decode(r2_obj03_b.radar2_obj03_dz);
              dz_is_in_range = xgu_radar2_obj03_b_radar2_obj03_dz_is_in_range(r2_obj03_b.radar2_obj03_dz);
              moving_state_decoded =
                  xgu_radar2_obj03_b_radar2_obj03_moving_state_decode(r2_obj03_b.radar2_obj03_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_moving_state_is_in_range(r2_obj03_b.radar2_obj03_moving_state);
              dx_sigma_decoded = xgu_radar2_obj03_b_radar2_obj03_dx_sigma_decode(r2_obj03_b.radar2_obj03_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_dx_sigma_is_in_range(r2_obj03_b.radar2_obj03_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj03_b_radar2_obj03_vx_sigma_decode(r2_obj03_b.radar2_obj03_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_vx_sigma_is_in_range(r2_obj03_b.radar2_obj03_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj03_b_radar2_obj03_ax_sigma_decode(r2_obj03_b.radar2_obj03_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_ax_sigma_is_in_range(r2_obj03_b.radar2_obj03_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj03_b_radar2_obj03_dy_sigma_decode(r2_obj03_b.radar2_obj03_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_dy_sigma_is_in_range(r2_obj03_b.radar2_obj03_dy_sigma);
              w_class_decoded = xgu_radar2_obj03_b_radar2_obj03_w_class_decode(r2_obj03_b.radar2_obj03_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj03_b_radar2_obj03_w_class_is_in_range(r2_obj03_b.radar2_obj03_w_class);
              class_decoded = xgu_radar2_obj03_b_radar2_obj03_class_decode(r2_obj03_b.radar2_obj03_class);
              class_is_in_range = xgu_radar2_obj03_b_radar2_obj03_class_is_in_range(r2_obj03_b.radar2_obj03_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj03_b_radar2_obj03_dx_rear_end_loss_decode(r2_obj03_b.radar2_obj03_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj03_b_radar2_obj03_dx_rear_end_loss_is_in_range(
                  r2_obj03_b.radar2_obj03_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj03_b_radar2_obj03_mess_bconsist_bit_encode(r2_obj03_b.radar2_obj03_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj03_b_radar2_obj03_mess_bconsist_bit_is_in_range(
                  r2_obj03_b.radar2_obj03_mess_bconsist_bit);
              break;
            case 1328:
              xgu_radar2_obj04_b_t r2_obj04_b;

              vy_decoded = xgu_radar2_obj04_b_radar2_obj04_vy_decode(r2_obj04_b.radar2_obj04_vy);
              vy_is_in_range = xgu_radar2_obj04_b_radar2_obj04_vy_is_in_range(r2_obj04_b.radar2_obj04_vy);
              d_length_decoded = xgu_radar2_obj04_b_radar2_obj04_d_length_decode(r2_obj04_b.radar2_obj04_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_d_length_is_in_range(r2_obj04_b.radar2_obj04_d_length);
              dz_decoded = xgu_radar2_obj04_b_radar2_obj04_dz_decode(r2_obj04_b.radar2_obj04_dz);
              dz_is_in_range = xgu_radar2_obj04_b_radar2_obj04_dz_is_in_range(r2_obj04_b.radar2_obj04_dz);
              moving_state_decoded =
                  xgu_radar2_obj04_b_radar2_obj04_moving_state_decode(r2_obj04_b.radar2_obj04_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_moving_state_is_in_range(r2_obj04_b.radar2_obj04_moving_state);
              dx_sigma_decoded = xgu_radar2_obj04_b_radar2_obj04_dx_sigma_decode(r2_obj04_b.radar2_obj04_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_dx_sigma_is_in_range(r2_obj04_b.radar2_obj04_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj04_b_radar2_obj04_vx_sigma_decode(r2_obj04_b.radar2_obj04_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_vx_sigma_is_in_range(r2_obj04_b.radar2_obj04_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj04_b_radar2_obj04_ax_sigma_decode(r2_obj04_b.radar2_obj04_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_ax_sigma_is_in_range(r2_obj04_b.radar2_obj04_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj04_b_radar2_obj04_dy_sigma_decode(r2_obj04_b.radar2_obj04_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_dy_sigma_is_in_range(r2_obj04_b.radar2_obj04_dy_sigma);
              w_class_decoded = xgu_radar2_obj04_b_radar2_obj04_w_class_decode(r2_obj04_b.radar2_obj04_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj04_b_radar2_obj04_w_class_is_in_range(r2_obj04_b.radar2_obj04_w_class);
              class_decoded = xgu_radar2_obj04_b_radar2_obj04_class_decode(r2_obj04_b.radar2_obj04_class);
              class_is_in_range = xgu_radar2_obj04_b_radar2_obj04_class_is_in_range(r2_obj04_b.radar2_obj04_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj04_b_radar2_obj04_dx_rear_end_loss_decode(r2_obj04_b.radar2_obj04_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj04_b_radar2_obj04_dx_rear_end_loss_is_in_range(
                  r2_obj04_b.radar2_obj04_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj04_b_radar2_obj04_mess_bconsist_bit_encode(r2_obj04_b.radar2_obj04_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj04_b_radar2_obj04_mess_bconsist_bit_is_in_range(
                  r2_obj04_b.radar2_obj04_mess_bconsist_bit);
              break;
            case 1338:
              xgu_radar2_obj05_b_t r2_obj05_b;

              vy_decoded = xgu_radar2_obj05_b_radar2_obj05_vy_decode(r2_obj05_b.radar2_obj05_vy);
              vy_is_in_range = xgu_radar2_obj05_b_radar2_obj05_vy_is_in_range(r2_obj05_b.radar2_obj05_vy);
              d_length_decoded = xgu_radar2_obj05_b_radar2_obj05_d_length_decode(r2_obj05_b.radar2_obj05_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_d_length_is_in_range(r2_obj05_b.radar2_obj05_d_length);
              dz_decoded = xgu_radar2_obj05_b_radar2_obj05_dz_decode(r2_obj05_b.radar2_obj05_dz);
              dz_is_in_range = xgu_radar2_obj05_b_radar2_obj05_dz_is_in_range(r2_obj05_b.radar2_obj05_dz);
              moving_state_decoded =
                  xgu_radar2_obj05_b_radar2_obj05_moving_state_decode(r2_obj05_b.radar2_obj05_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_moving_state_is_in_range(r2_obj05_b.radar2_obj05_moving_state);
              dx_sigma_decoded = xgu_radar2_obj05_b_radar2_obj05_dx_sigma_decode(r2_obj05_b.radar2_obj05_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_dx_sigma_is_in_range(r2_obj05_b.radar2_obj05_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj05_b_radar2_obj05_vx_sigma_decode(r2_obj05_b.radar2_obj05_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_vx_sigma_is_in_range(r2_obj05_b.radar2_obj05_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj05_b_radar2_obj05_ax_sigma_decode(r2_obj05_b.radar2_obj05_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_ax_sigma_is_in_range(r2_obj05_b.radar2_obj05_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj05_b_radar2_obj05_dy_sigma_decode(r2_obj05_b.radar2_obj05_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_dy_sigma_is_in_range(r2_obj05_b.radar2_obj05_dy_sigma);
              w_class_decoded = xgu_radar2_obj05_b_radar2_obj05_w_class_decode(r2_obj05_b.radar2_obj05_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj05_b_radar2_obj05_w_class_is_in_range(r2_obj05_b.radar2_obj05_w_class);
              class_decoded = xgu_radar2_obj05_b_radar2_obj05_class_decode(r2_obj05_b.radar2_obj05_class);
              class_is_in_range = xgu_radar2_obj05_b_radar2_obj05_class_is_in_range(r2_obj05_b.radar2_obj05_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj05_b_radar2_obj05_dx_rear_end_loss_decode(r2_obj05_b.radar2_obj05_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj05_b_radar2_obj05_dx_rear_end_loss_is_in_range(
                  r2_obj05_b.radar2_obj05_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj05_b_radar2_obj05_mess_bconsist_bit_encode(r2_obj05_b.radar2_obj05_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj05_b_radar2_obj05_mess_bconsist_bit_is_in_range(
                  r2_obj05_b.radar2_obj05_mess_bconsist_bit);
              break;
            case 1348:
              xgu_radar2_obj06_b_t r2_obj06_b;

              vy_decoded = xgu_radar2_obj06_b_radar2_obj06_vy_decode(r2_obj06_b.radar2_obj06_vy);
              vy_is_in_range = xgu_radar2_obj06_b_radar2_obj06_vy_is_in_range(r2_obj06_b.radar2_obj06_vy);
              d_length_decoded = xgu_radar2_obj06_b_radar2_obj06_d_length_decode(r2_obj06_b.radar2_obj06_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_d_length_is_in_range(r2_obj06_b.radar2_obj06_d_length);
              dz_decoded = xgu_radar2_obj06_b_radar2_obj06_dz_decode(r2_obj06_b.radar2_obj06_dz);
              dz_is_in_range = xgu_radar2_obj06_b_radar2_obj06_dz_is_in_range(r2_obj06_b.radar2_obj06_dz);
              moving_state_decoded =
                  xgu_radar2_obj06_b_radar2_obj06_moving_state_decode(r2_obj06_b.radar2_obj06_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_moving_state_is_in_range(r2_obj06_b.radar2_obj06_moving_state);
              dx_sigma_decoded = xgu_radar2_obj06_b_radar2_obj06_dx_sigma_decode(r2_obj06_b.radar2_obj06_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_dx_sigma_is_in_range(r2_obj06_b.radar2_obj06_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj06_b_radar2_obj06_vx_sigma_decode(r2_obj06_b.radar2_obj06_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_vx_sigma_is_in_range(r2_obj06_b.radar2_obj06_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj06_b_radar2_obj06_ax_sigma_decode(r2_obj06_b.radar2_obj06_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_ax_sigma_is_in_range(r2_obj06_b.radar2_obj06_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj06_b_radar2_obj06_dy_sigma_decode(r2_obj06_b.radar2_obj06_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_dy_sigma_is_in_range(r2_obj06_b.radar2_obj06_dy_sigma);
              w_class_decoded = xgu_radar2_obj06_b_radar2_obj06_w_class_decode(r2_obj06_b.radar2_obj06_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj06_b_radar2_obj06_w_class_is_in_range(r2_obj06_b.radar2_obj06_w_class);
              class_decoded = xgu_radar2_obj06_b_radar2_obj06_class_decode(r2_obj06_b.radar2_obj06_class);
              class_is_in_range = xgu_radar2_obj06_b_radar2_obj06_class_is_in_range(r2_obj06_b.radar2_obj06_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj06_b_radar2_obj06_dx_rear_end_loss_decode(r2_obj06_b.radar2_obj06_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj06_b_radar2_obj06_dx_rear_end_loss_is_in_range(
                  r2_obj06_b.radar2_obj06_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj06_b_radar2_obj06_mess_bconsist_bit_encode(r2_obj06_b.radar2_obj06_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj06_b_radar2_obj06_mess_bconsist_bit_is_in_range(
                  r2_obj06_b.radar2_obj06_mess_bconsist_bit);
              break;
            case 1358:
              xgu_radar2_obj07_b_t r2_obj07_b;

              vy_decoded = xgu_radar2_obj07_b_radar2_obj07_vy_decode(r2_obj07_b.radar2_obj07_vy);
              vy_is_in_range = xgu_radar2_obj07_b_radar2_obj07_vy_is_in_range(r2_obj07_b.radar2_obj07_vy);
              d_length_decoded = xgu_radar2_obj07_b_radar2_obj07_d_length_decode(r2_obj07_b.radar2_obj07_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_d_length_is_in_range(r2_obj07_b.radar2_obj07_d_length);
              dz_decoded = xgu_radar2_obj07_b_radar2_obj07_dz_decode(r2_obj07_b.radar2_obj07_dz);
              dz_is_in_range = xgu_radar2_obj07_b_radar2_obj07_dz_is_in_range(r2_obj07_b.radar2_obj07_dz);
              moving_state_decoded =
                  xgu_radar2_obj07_b_radar2_obj07_moving_state_decode(r2_obj07_b.radar2_obj07_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_moving_state_is_in_range(r2_obj07_b.radar2_obj07_moving_state);
              dx_sigma_decoded = xgu_radar2_obj07_b_radar2_obj07_dx_sigma_decode(r2_obj07_b.radar2_obj07_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_dx_sigma_is_in_range(r2_obj07_b.radar2_obj07_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj07_b_radar2_obj07_vx_sigma_decode(r2_obj07_b.radar2_obj07_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_vx_sigma_is_in_range(r2_obj07_b.radar2_obj07_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj07_b_radar2_obj07_ax_sigma_decode(r2_obj07_b.radar2_obj07_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_ax_sigma_is_in_range(r2_obj07_b.radar2_obj07_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj07_b_radar2_obj07_dy_sigma_decode(r2_obj07_b.radar2_obj07_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_dy_sigma_is_in_range(r2_obj07_b.radar2_obj07_dy_sigma);
              w_class_decoded = xgu_radar2_obj07_b_radar2_obj07_w_class_decode(r2_obj07_b.radar2_obj07_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj07_b_radar2_obj07_w_class_is_in_range(r2_obj07_b.radar2_obj07_w_class);
              class_decoded = xgu_radar2_obj07_b_radar2_obj07_class_decode(r2_obj07_b.radar2_obj07_class);
              class_is_in_range = xgu_radar2_obj07_b_radar2_obj07_class_is_in_range(r2_obj07_b.radar2_obj07_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj07_b_radar2_obj07_dx_rear_end_loss_decode(r2_obj07_b.radar2_obj07_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj07_b_radar2_obj07_dx_rear_end_loss_is_in_range(
                  r2_obj07_b.radar2_obj07_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj07_b_radar2_obj07_mess_bconsist_bit_encode(r2_obj07_b.radar2_obj07_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj07_b_radar2_obj07_mess_bconsist_bit_is_in_range(
                  r2_obj07_b.radar2_obj07_mess_bconsist_bit);
              break;
            case 1368:
              xgu_radar2_obj08_b_t r2_obj08_b;

              vy_decoded = xgu_radar2_obj08_b_radar2_obj08_vy_decode(r2_obj08_b.radar2_obj08_vy);
              vy_is_in_range = xgu_radar2_obj08_b_radar2_obj08_vy_is_in_range(r2_obj08_b.radar2_obj08_vy);
              d_length_decoded = xgu_radar2_obj08_b_radar2_obj08_d_length_decode(r2_obj08_b.radar2_obj08_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_d_length_is_in_range(r2_obj08_b.radar2_obj08_d_length);
              dz_decoded = xgu_radar2_obj08_b_radar2_obj08_dz_decode(r2_obj08_b.radar2_obj08_dz);
              dz_is_in_range = xgu_radar2_obj08_b_radar2_obj08_dz_is_in_range(r2_obj08_b.radar2_obj08_dz);
              moving_state_decoded =
                  xgu_radar2_obj08_b_radar2_obj08_moving_state_decode(r2_obj08_b.radar2_obj08_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_moving_state_is_in_range(r2_obj08_b.radar2_obj08_moving_state);
              dx_sigma_decoded = xgu_radar2_obj08_b_radar2_obj08_dx_sigma_decode(r2_obj08_b.radar2_obj08_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_dx_sigma_is_in_range(r2_obj08_b.radar2_obj08_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj08_b_radar2_obj08_vx_sigma_decode(r2_obj08_b.radar2_obj08_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_vx_sigma_is_in_range(r2_obj08_b.radar2_obj08_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj08_b_radar2_obj08_ax_sigma_decode(r2_obj08_b.radar2_obj08_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_ax_sigma_is_in_range(r2_obj08_b.radar2_obj08_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj08_b_radar2_obj08_dy_sigma_decode(r2_obj08_b.radar2_obj08_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_dy_sigma_is_in_range(r2_obj08_b.radar2_obj08_dy_sigma);
              w_class_decoded = xgu_radar2_obj08_b_radar2_obj08_w_class_decode(r2_obj08_b.radar2_obj08_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj08_b_radar2_obj08_w_class_is_in_range(r2_obj08_b.radar2_obj08_w_class);
              class_decoded = xgu_radar2_obj08_b_radar2_obj08_class_decode(r2_obj08_b.radar2_obj08_class);
              class_is_in_range = xgu_radar2_obj08_b_radar2_obj08_class_is_in_range(r2_obj08_b.radar2_obj08_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj08_b_radar2_obj08_dx_rear_end_loss_decode(r2_obj08_b.radar2_obj08_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj08_b_radar2_obj08_dx_rear_end_loss_is_in_range(
                  r2_obj08_b.radar2_obj08_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj08_b_radar2_obj08_mess_bconsist_bit_encode(r2_obj08_b.radar2_obj08_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj08_b_radar2_obj08_mess_bconsist_bit_is_in_range(
                  r2_obj08_b.radar2_obj08_mess_bconsist_bit);
              break;
            case 1378:
              xgu_radar2_obj09_b_t r2_obj09_b;

              vy_decoded = xgu_radar2_obj09_b_radar2_obj09_vy_decode(r2_obj09_b.radar2_obj09_vy);
              vy_is_in_range = xgu_radar2_obj09_b_radar2_obj09_vy_is_in_range(r2_obj09_b.radar2_obj09_vy);
              d_length_decoded = xgu_radar2_obj09_b_radar2_obj09_d_length_decode(r2_obj09_b.radar2_obj09_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_d_length_is_in_range(r2_obj09_b.radar2_obj09_d_length);
              dz_decoded = xgu_radar2_obj09_b_radar2_obj09_dz_decode(r2_obj09_b.radar2_obj09_dz);
              dz_is_in_range = xgu_radar2_obj09_b_radar2_obj09_dz_is_in_range(r2_obj09_b.radar2_obj09_dz);
              moving_state_decoded =
                  xgu_radar2_obj09_b_radar2_obj09_moving_state_decode(r2_obj09_b.radar2_obj09_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_moving_state_is_in_range(r2_obj09_b.radar2_obj09_moving_state);
              dx_sigma_decoded = xgu_radar2_obj09_b_radar2_obj09_dx_sigma_decode(r2_obj09_b.radar2_obj09_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_dx_sigma_is_in_range(r2_obj09_b.radar2_obj09_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj09_b_radar2_obj09_vx_sigma_decode(r2_obj09_b.radar2_obj09_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_vx_sigma_is_in_range(r2_obj09_b.radar2_obj09_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj09_b_radar2_obj09_ax_sigma_decode(r2_obj09_b.radar2_obj09_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_ax_sigma_is_in_range(r2_obj09_b.radar2_obj09_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj09_b_radar2_obj09_dy_sigma_decode(r2_obj09_b.radar2_obj09_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_dy_sigma_is_in_range(r2_obj09_b.radar2_obj09_dy_sigma);
              w_class_decoded = xgu_radar2_obj09_b_radar2_obj09_w_class_decode(r2_obj09_b.radar2_obj09_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj09_b_radar2_obj09_w_class_is_in_range(r2_obj09_b.radar2_obj09_w_class);
              class_decoded = xgu_radar2_obj09_b_radar2_obj09_class_decode(r2_obj09_b.radar2_obj09_class);
              class_is_in_range = xgu_radar2_obj09_b_radar2_obj09_class_is_in_range(r2_obj09_b.radar2_obj09_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj09_b_radar2_obj09_dx_rear_end_loss_decode(r2_obj09_b.radar2_obj09_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj09_b_radar2_obj09_dx_rear_end_loss_is_in_range(
                  r2_obj09_b.radar2_obj09_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj09_b_radar2_obj09_mess_bconsist_bit_encode(r2_obj09_b.radar2_obj09_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj09_b_radar2_obj09_mess_bconsist_bit_is_in_range(
                  r2_obj09_b.radar2_obj09_mess_bconsist_bit);
              break;
            case 1388:
              xgu_radar2_obj10_b_t r2_obj10_b;

              vy_decoded = xgu_radar2_obj10_b_radar2_obj10_vy_decode(r2_obj10_b.radar2_obj10_vy);
              vy_is_in_range = xgu_radar2_obj10_b_radar2_obj10_vy_is_in_range(r2_obj10_b.radar2_obj10_vy);
              d_length_decoded = xgu_radar2_obj10_b_radar2_obj10_d_length_decode(r2_obj10_b.radar2_obj10_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_d_length_is_in_range(r2_obj10_b.radar2_obj10_d_length);
              dz_decoded = xgu_radar2_obj10_b_radar2_obj10_dz_decode(r2_obj10_b.radar2_obj10_dz);
              dz_is_in_range = xgu_radar2_obj10_b_radar2_obj10_dz_is_in_range(r2_obj10_b.radar2_obj10_dz);
              moving_state_decoded =
                  xgu_radar2_obj10_b_radar2_obj10_moving_state_decode(r2_obj10_b.radar2_obj10_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_moving_state_is_in_range(r2_obj10_b.radar2_obj10_moving_state);
              dx_sigma_decoded = xgu_radar2_obj10_b_radar2_obj10_dx_sigma_decode(r2_obj10_b.radar2_obj10_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_dx_sigma_is_in_range(r2_obj10_b.radar2_obj10_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj10_b_radar2_obj10_vx_sigma_decode(r2_obj10_b.radar2_obj10_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_vx_sigma_is_in_range(r2_obj10_b.radar2_obj10_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj10_b_radar2_obj10_ax_sigma_decode(r2_obj10_b.radar2_obj10_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_ax_sigma_is_in_range(r2_obj10_b.radar2_obj10_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj10_b_radar2_obj10_dy_sigma_decode(r2_obj10_b.radar2_obj10_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_dy_sigma_is_in_range(r2_obj10_b.radar2_obj10_dy_sigma);
              w_class_decoded = xgu_radar2_obj10_b_radar2_obj10_w_class_decode(r2_obj10_b.radar2_obj10_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj10_b_radar2_obj10_w_class_is_in_range(r2_obj10_b.radar2_obj10_w_class);
              class_decoded = xgu_radar2_obj10_b_radar2_obj10_class_decode(r2_obj10_b.radar2_obj10_class);
              class_is_in_range = xgu_radar2_obj10_b_radar2_obj10_class_is_in_range(r2_obj10_b.radar2_obj10_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj10_b_radar2_obj10_dx_rear_end_loss_decode(r2_obj10_b.radar2_obj10_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj10_b_radar2_obj10_dx_rear_end_loss_is_in_range(
                  r2_obj10_b.radar2_obj10_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj10_b_radar2_obj10_mess_bconsist_bit_encode(r2_obj10_b.radar2_obj10_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj10_b_radar2_obj10_mess_bconsist_bit_is_in_range(
                  r2_obj10_b.radar2_obj10_mess_bconsist_bit);
              break;
            case 1398:
              xgu_radar2_obj11_b_t r2_obj11_b;

              vy_decoded = xgu_radar2_obj11_b_radar2_obj11_vy_decode(r2_obj11_b.radar2_obj11_vy);
              vy_is_in_range = xgu_radar2_obj11_b_radar2_obj11_vy_is_in_range(r2_obj11_b.radar2_obj11_vy);
              d_length_decoded = xgu_radar2_obj11_b_radar2_obj11_d_length_decode(r2_obj11_b.radar2_obj11_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_d_length_is_in_range(r2_obj11_b.radar2_obj11_d_length);
              dz_decoded = xgu_radar2_obj11_b_radar2_obj11_dz_decode(r2_obj11_b.radar2_obj11_dz);
              dz_is_in_range = xgu_radar2_obj11_b_radar2_obj11_dz_is_in_range(r2_obj11_b.radar2_obj11_dz);
              moving_state_decoded =
                  xgu_radar2_obj11_b_radar2_obj11_moving_state_decode(r2_obj11_b.radar2_obj11_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_moving_state_is_in_range(r2_obj11_b.radar2_obj11_moving_state);
              dx_sigma_decoded = xgu_radar2_obj11_b_radar2_obj11_dx_sigma_decode(r2_obj11_b.radar2_obj11_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_dx_sigma_is_in_range(r2_obj11_b.radar2_obj11_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj11_b_radar2_obj11_vx_sigma_decode(r2_obj11_b.radar2_obj11_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_vx_sigma_is_in_range(r2_obj11_b.radar2_obj11_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj11_b_radar2_obj11_ax_sigma_decode(r2_obj11_b.radar2_obj11_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_ax_sigma_is_in_range(r2_obj11_b.radar2_obj11_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj11_b_radar2_obj11_dy_sigma_decode(r2_obj11_b.radar2_obj11_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_dy_sigma_is_in_range(r2_obj11_b.radar2_obj11_dy_sigma);
              w_class_decoded = xgu_radar2_obj11_b_radar2_obj11_w_class_decode(r2_obj11_b.radar2_obj11_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj11_b_radar2_obj11_w_class_is_in_range(r2_obj11_b.radar2_obj11_w_class);
              class_decoded = xgu_radar2_obj11_b_radar2_obj11_class_decode(r2_obj11_b.radar2_obj11_class);
              class_is_in_range = xgu_radar2_obj11_b_radar2_obj11_class_is_in_range(r2_obj11_b.radar2_obj11_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj11_b_radar2_obj11_dx_rear_end_loss_decode(r2_obj11_b.radar2_obj11_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj11_b_radar2_obj11_dx_rear_end_loss_is_in_range(
                  r2_obj11_b.radar2_obj11_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj11_b_radar2_obj11_mess_bconsist_bit_encode(r2_obj11_b.radar2_obj11_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj11_b_radar2_obj11_mess_bconsist_bit_is_in_range(
                  r2_obj11_b.radar2_obj11_mess_bconsist_bit);
              break;
            case 1408:
              xgu_radar2_obj12_b_t r2_obj12_b;

              vy_decoded = xgu_radar2_obj12_b_radar2_obj12_vy_decode(r2_obj12_b.radar2_obj12_vy);
              vy_is_in_range = xgu_radar2_obj12_b_radar2_obj12_vy_is_in_range(r2_obj12_b.radar2_obj12_vy);
              d_length_decoded = xgu_radar2_obj12_b_radar2_obj12_d_length_decode(r2_obj12_b.radar2_obj12_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_d_length_is_in_range(r2_obj12_b.radar2_obj12_d_length);
              dz_decoded = xgu_radar2_obj12_b_radar2_obj12_dz_decode(r2_obj12_b.radar2_obj12_dz);
              dz_is_in_range = xgu_radar2_obj12_b_radar2_obj12_dz_is_in_range(r2_obj12_b.radar2_obj12_dz);
              moving_state_decoded =
                  xgu_radar2_obj12_b_radar2_obj12_moving_state_decode(r2_obj12_b.radar2_obj12_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_moving_state_is_in_range(r2_obj12_b.radar2_obj12_moving_state);
              dx_sigma_decoded = xgu_radar2_obj12_b_radar2_obj12_dx_sigma_decode(r2_obj12_b.radar2_obj12_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_dx_sigma_is_in_range(r2_obj12_b.radar2_obj12_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj12_b_radar2_obj12_vx_sigma_decode(r2_obj12_b.radar2_obj12_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_vx_sigma_is_in_range(r2_obj12_b.radar2_obj12_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj12_b_radar2_obj12_ax_sigma_decode(r2_obj12_b.radar2_obj12_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_ax_sigma_is_in_range(r2_obj12_b.radar2_obj12_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj12_b_radar2_obj12_dy_sigma_decode(r2_obj12_b.radar2_obj12_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_dy_sigma_is_in_range(r2_obj12_b.radar2_obj12_dy_sigma);
              w_class_decoded = xgu_radar2_obj12_b_radar2_obj12_w_class_decode(r2_obj12_b.radar2_obj12_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj12_b_radar2_obj12_w_class_is_in_range(r2_obj12_b.radar2_obj12_w_class);
              class_decoded = xgu_radar2_obj12_b_radar2_obj12_class_decode(r2_obj12_b.radar2_obj12_class);
              class_is_in_range = xgu_radar2_obj12_b_radar2_obj12_class_is_in_range(r2_obj12_b.radar2_obj12_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj12_b_radar2_obj12_dx_rear_end_loss_decode(r2_obj12_b.radar2_obj12_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj12_b_radar2_obj12_dx_rear_end_loss_is_in_range(
                  r2_obj12_b.radar2_obj12_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj12_b_radar2_obj12_mess_bconsist_bit_encode(r2_obj12_b.radar2_obj12_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj12_b_radar2_obj12_mess_bconsist_bit_is_in_range(
                  r2_obj12_b.radar2_obj12_mess_bconsist_bit);
              break;
            case 1418:
              xgu_radar2_obj13_b_t r2_obj13_b;

              vy_decoded = xgu_radar2_obj13_b_radar2_obj13_vy_decode(r2_obj13_b.radar2_obj13_vy);
              vy_is_in_range = xgu_radar2_obj13_b_radar2_obj13_vy_is_in_range(r2_obj13_b.radar2_obj13_vy);
              d_length_decoded = xgu_radar2_obj13_b_radar2_obj13_d_length_decode(r2_obj13_b.radar2_obj13_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_d_length_is_in_range(r2_obj13_b.radar2_obj13_d_length);
              dz_decoded = xgu_radar2_obj13_b_radar2_obj13_dz_decode(r2_obj13_b.radar2_obj13_dz);
              dz_is_in_range = xgu_radar2_obj13_b_radar2_obj13_dz_is_in_range(r2_obj13_b.radar2_obj13_dz);
              moving_state_decoded =
                  xgu_radar2_obj13_b_radar2_obj13_moving_state_decode(r2_obj13_b.radar2_obj13_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_moving_state_is_in_range(r2_obj13_b.radar2_obj13_moving_state);
              dx_sigma_decoded = xgu_radar2_obj13_b_radar2_obj13_dx_sigma_decode(r2_obj13_b.radar2_obj13_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_dx_sigma_is_in_range(r2_obj13_b.radar2_obj13_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj13_b_radar2_obj13_vx_sigma_decode(r2_obj13_b.radar2_obj13_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_vx_sigma_is_in_range(r2_obj13_b.radar2_obj13_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj13_b_radar2_obj13_ax_sigma_decode(r2_obj13_b.radar2_obj13_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_ax_sigma_is_in_range(r2_obj13_b.radar2_obj13_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj13_b_radar2_obj13_dy_sigma_decode(r2_obj13_b.radar2_obj13_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_dy_sigma_is_in_range(r2_obj13_b.radar2_obj13_dy_sigma);
              w_class_decoded = xgu_radar2_obj13_b_radar2_obj13_w_class_decode(r2_obj13_b.radar2_obj13_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj13_b_radar2_obj13_w_class_is_in_range(r2_obj13_b.radar2_obj13_w_class);
              class_decoded = xgu_radar2_obj13_b_radar2_obj13_class_decode(r2_obj13_b.radar2_obj13_class);
              class_is_in_range = xgu_radar2_obj13_b_radar2_obj13_class_is_in_range(r2_obj13_b.radar2_obj13_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj13_b_radar2_obj13_dx_rear_end_loss_decode(r2_obj13_b.radar2_obj13_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj13_b_radar2_obj13_dx_rear_end_loss_is_in_range(
                  r2_obj13_b.radar2_obj13_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj13_b_radar2_obj13_mess_bconsist_bit_encode(r2_obj13_b.radar2_obj13_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj13_b_radar2_obj13_mess_bconsist_bit_is_in_range(
                  r2_obj13_b.radar2_obj13_mess_bconsist_bit);
              break;
            case 1428:
              xgu_radar2_obj14_b_t r2_obj14_b;

              vy_decoded = xgu_radar2_obj14_b_radar2_obj14_vy_decode(r2_obj14_b.radar2_obj14_vy);
              vy_is_in_range = xgu_radar2_obj14_b_radar2_obj14_vy_is_in_range(r2_obj14_b.radar2_obj14_vy);
              d_length_decoded = xgu_radar2_obj14_b_radar2_obj14_d_length_decode(r2_obj14_b.radar2_obj14_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_d_length_is_in_range(r2_obj14_b.radar2_obj14_d_length);
              dz_decoded = xgu_radar2_obj14_b_radar2_obj14_dz_decode(r2_obj14_b.radar2_obj14_dz);
              dz_is_in_range = xgu_radar2_obj14_b_radar2_obj14_dz_is_in_range(r2_obj14_b.radar2_obj14_dz);
              moving_state_decoded =
                  xgu_radar2_obj14_b_radar2_obj14_moving_state_decode(r2_obj14_b.radar2_obj14_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_moving_state_is_in_range(r2_obj14_b.radar2_obj14_moving_state);
              dx_sigma_decoded = xgu_radar2_obj14_b_radar2_obj14_dx_sigma_decode(r2_obj14_b.radar2_obj14_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_dx_sigma_is_in_range(r2_obj14_b.radar2_obj14_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj14_b_radar2_obj14_vx_sigma_decode(r2_obj14_b.radar2_obj14_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_vx_sigma_is_in_range(r2_obj14_b.radar2_obj14_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj14_b_radar2_obj14_ax_sigma_decode(r2_obj14_b.radar2_obj14_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_ax_sigma_is_in_range(r2_obj14_b.radar2_obj14_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj14_b_radar2_obj14_dy_sigma_decode(r2_obj14_b.radar2_obj14_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_dy_sigma_is_in_range(r2_obj14_b.radar2_obj14_dy_sigma);
              w_class_decoded = xgu_radar2_obj14_b_radar2_obj14_w_class_decode(r2_obj14_b.radar2_obj14_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj14_b_radar2_obj14_w_class_is_in_range(r2_obj14_b.radar2_obj14_w_class);
              class_decoded = xgu_radar2_obj14_b_radar2_obj14_class_decode(r2_obj14_b.radar2_obj14_class);
              class_is_in_range = xgu_radar2_obj14_b_radar2_obj14_class_is_in_range(r2_obj14_b.radar2_obj14_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj14_b_radar2_obj14_dx_rear_end_loss_decode(r2_obj14_b.radar2_obj14_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj14_b_radar2_obj14_dx_rear_end_loss_is_in_range(
                  r2_obj14_b.radar2_obj14_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj14_b_radar2_obj14_mess_bconsist_bit_encode(r2_obj14_b.radar2_obj14_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj14_b_radar2_obj14_mess_bconsist_bit_is_in_range(
                  r2_obj14_b.radar2_obj14_mess_bconsist_bit);
              break;
            case 1438:
              xgu_radar2_obj15_b_t r2_obj15_b;

              vy_decoded = xgu_radar2_obj15_b_radar2_obj15_vy_decode(r2_obj15_b.radar2_obj15_vy);
              vy_is_in_range = xgu_radar2_obj15_b_radar2_obj15_vy_is_in_range(r2_obj15_b.radar2_obj15_vy);
              d_length_decoded = xgu_radar2_obj15_b_radar2_obj15_d_length_decode(r2_obj15_b.radar2_obj15_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_d_length_is_in_range(r2_obj15_b.radar2_obj15_d_length);
              dz_decoded = xgu_radar2_obj15_b_radar2_obj15_dz_decode(r2_obj15_b.radar2_obj15_dz);
              dz_is_in_range = xgu_radar2_obj15_b_radar2_obj15_dz_is_in_range(r2_obj15_b.radar2_obj15_dz);
              moving_state_decoded =
                  xgu_radar2_obj15_b_radar2_obj15_moving_state_decode(r2_obj15_b.radar2_obj15_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_moving_state_is_in_range(r2_obj15_b.radar2_obj15_moving_state);
              dx_sigma_decoded = xgu_radar2_obj15_b_radar2_obj15_dx_sigma_decode(r2_obj15_b.radar2_obj15_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_dx_sigma_is_in_range(r2_obj15_b.radar2_obj15_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj15_b_radar2_obj15_vx_sigma_decode(r2_obj15_b.radar2_obj15_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_vx_sigma_is_in_range(r2_obj15_b.radar2_obj15_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj15_b_radar2_obj15_ax_sigma_decode(r2_obj15_b.radar2_obj15_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_ax_sigma_is_in_range(r2_obj15_b.radar2_obj15_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj15_b_radar2_obj15_dy_sigma_decode(r2_obj15_b.radar2_obj15_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_dy_sigma_is_in_range(r2_obj15_b.radar2_obj15_dy_sigma);
              w_class_decoded = xgu_radar2_obj15_b_radar2_obj15_w_class_decode(r2_obj15_b.radar2_obj15_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj15_b_radar2_obj15_w_class_is_in_range(r2_obj15_b.radar2_obj15_w_class);
              class_decoded = xgu_radar2_obj15_b_radar2_obj15_class_decode(r2_obj15_b.radar2_obj15_class);
              class_is_in_range = xgu_radar2_obj15_b_radar2_obj15_class_is_in_range(r2_obj15_b.radar2_obj15_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj15_b_radar2_obj15_dx_rear_end_loss_decode(r2_obj15_b.radar2_obj15_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj15_b_radar2_obj15_dx_rear_end_loss_is_in_range(
                  r2_obj15_b.radar2_obj15_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj15_b_radar2_obj15_mess_bconsist_bit_encode(r2_obj15_b.radar2_obj15_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj15_b_radar2_obj15_mess_bconsist_bit_is_in_range(
                  r2_obj15_b.radar2_obj15_mess_bconsist_bit);
              break;
            case 1448:
              xgu_radar2_obj16_b_t r2_obj16_b;

              vy_decoded = xgu_radar2_obj16_b_radar2_obj16_vy_decode(r2_obj16_b.radar2_obj16_vy);
              vy_is_in_range = xgu_radar2_obj16_b_radar2_obj16_vy_is_in_range(r2_obj16_b.radar2_obj16_vy);
              d_length_decoded = xgu_radar2_obj16_b_radar2_obj16_d_length_decode(r2_obj16_b.radar2_obj16_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_d_length_is_in_range(r2_obj16_b.radar2_obj16_d_length);
              dz_decoded = xgu_radar2_obj16_b_radar2_obj16_dz_decode(r2_obj16_b.radar2_obj16_dz);
              dz_is_in_range = xgu_radar2_obj16_b_radar2_obj16_dz_is_in_range(r2_obj16_b.radar2_obj16_dz);
              moving_state_decoded =
                  xgu_radar2_obj16_b_radar2_obj16_moving_state_decode(r2_obj16_b.radar2_obj16_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_moving_state_is_in_range(r2_obj16_b.radar2_obj16_moving_state);
              dx_sigma_decoded = xgu_radar2_obj16_b_radar2_obj16_dx_sigma_decode(r2_obj16_b.radar2_obj16_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_dx_sigma_is_in_range(r2_obj16_b.radar2_obj16_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj16_b_radar2_obj16_vx_sigma_decode(r2_obj16_b.radar2_obj16_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_vx_sigma_is_in_range(r2_obj16_b.radar2_obj16_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj16_b_radar2_obj16_ax_sigma_decode(r2_obj16_b.radar2_obj16_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_ax_sigma_is_in_range(r2_obj16_b.radar2_obj16_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj16_b_radar2_obj16_dy_sigma_decode(r2_obj16_b.radar2_obj16_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_dy_sigma_is_in_range(r2_obj16_b.radar2_obj16_dy_sigma);
              w_class_decoded = xgu_radar2_obj16_b_radar2_obj16_w_class_decode(r2_obj16_b.radar2_obj16_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj16_b_radar2_obj16_w_class_is_in_range(r2_obj16_b.radar2_obj16_w_class);
              class_decoded = xgu_radar2_obj16_b_radar2_obj16_class_decode(r2_obj16_b.radar2_obj16_class);
              class_is_in_range = xgu_radar2_obj16_b_radar2_obj16_class_is_in_range(r2_obj16_b.radar2_obj16_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj16_b_radar2_obj16_dx_rear_end_loss_decode(r2_obj16_b.radar2_obj16_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj16_b_radar2_obj16_dx_rear_end_loss_is_in_range(
                  r2_obj16_b.radar2_obj16_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj16_b_radar2_obj16_mess_bconsist_bit_encode(r2_obj16_b.radar2_obj16_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj16_b_radar2_obj16_mess_bconsist_bit_is_in_range(
                  r2_obj16_b.radar2_obj16_mess_bconsist_bit);
              break;
            case 1458:
              xgu_radar2_obj17_b_t r2_obj17_b;

              vy_decoded = xgu_radar2_obj17_b_radar2_obj17_vy_decode(r2_obj17_b.radar2_obj17_vy);
              vy_is_in_range = xgu_radar2_obj17_b_radar2_obj17_vy_is_in_range(r2_obj17_b.radar2_obj17_vy);
              d_length_decoded = xgu_radar2_obj17_b_radar2_obj17_d_length_decode(r2_obj17_b.radar2_obj17_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_d_length_is_in_range(r2_obj17_b.radar2_obj17_d_length);
              dz_decoded = xgu_radar2_obj17_b_radar2_obj17_dz_decode(r2_obj17_b.radar2_obj17_dz);
              dz_is_in_range = xgu_radar2_obj17_b_radar2_obj17_dz_is_in_range(r2_obj17_b.radar2_obj17_dz);
              moving_state_decoded =
                  xgu_radar2_obj17_b_radar2_obj17_moving_state_decode(r2_obj17_b.radar2_obj17_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_moving_state_is_in_range(r2_obj17_b.radar2_obj17_moving_state);
              dx_sigma_decoded = xgu_radar2_obj17_b_radar2_obj17_dx_sigma_decode(r2_obj17_b.radar2_obj17_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_dx_sigma_is_in_range(r2_obj17_b.radar2_obj17_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj17_b_radar2_obj17_vx_sigma_decode(r2_obj17_b.radar2_obj17_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_vx_sigma_is_in_range(r2_obj17_b.radar2_obj17_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj17_b_radar2_obj17_ax_sigma_decode(r2_obj17_b.radar2_obj17_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_ax_sigma_is_in_range(r2_obj17_b.radar2_obj17_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj17_b_radar2_obj17_dy_sigma_decode(r2_obj17_b.radar2_obj17_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_dy_sigma_is_in_range(r2_obj17_b.radar2_obj17_dy_sigma);
              w_class_decoded = xgu_radar2_obj17_b_radar2_obj17_w_class_decode(r2_obj17_b.radar2_obj17_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj17_b_radar2_obj17_w_class_is_in_range(r2_obj17_b.radar2_obj17_w_class);
              class_decoded = xgu_radar2_obj17_b_radar2_obj17_class_decode(r2_obj17_b.radar2_obj17_class);
              class_is_in_range = xgu_radar2_obj17_b_radar2_obj17_class_is_in_range(r2_obj17_b.radar2_obj17_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj17_b_radar2_obj17_dx_rear_end_loss_decode(r2_obj17_b.radar2_obj17_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj17_b_radar2_obj17_dx_rear_end_loss_is_in_range(
                  r2_obj17_b.radar2_obj17_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj17_b_radar2_obj17_mess_bconsist_bit_encode(r2_obj17_b.radar2_obj17_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj17_b_radar2_obj17_mess_bconsist_bit_is_in_range(
                  r2_obj17_b.radar2_obj17_mess_bconsist_bit);
              break;
            case 1468:
              xgu_radar2_obj18_b_t r2_obj18_b;

              vy_decoded = xgu_radar2_obj18_b_radar2_obj18_vy_decode(r2_obj18_b.radar2_obj18_vy);
              vy_is_in_range = xgu_radar2_obj18_b_radar2_obj18_vy_is_in_range(r2_obj18_b.radar2_obj18_vy);
              d_length_decoded = xgu_radar2_obj18_b_radar2_obj18_d_length_decode(r2_obj18_b.radar2_obj18_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_d_length_is_in_range(r2_obj18_b.radar2_obj18_d_length);
              dz_decoded = xgu_radar2_obj18_b_radar2_obj18_dz_decode(r2_obj18_b.radar2_obj18_dz);
              dz_is_in_range = xgu_radar2_obj18_b_radar2_obj18_dz_is_in_range(r2_obj18_b.radar2_obj18_dz);
              moving_state_decoded =
                  xgu_radar2_obj18_b_radar2_obj18_moving_state_decode(r2_obj18_b.radar2_obj18_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_moving_state_is_in_range(r2_obj18_b.radar2_obj18_moving_state);
              dx_sigma_decoded = xgu_radar2_obj18_b_radar2_obj18_dx_sigma_decode(r2_obj18_b.radar2_obj18_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_dx_sigma_is_in_range(r2_obj18_b.radar2_obj18_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj18_b_radar2_obj18_vx_sigma_decode(r2_obj18_b.radar2_obj18_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_vx_sigma_is_in_range(r2_obj18_b.radar2_obj18_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj18_b_radar2_obj18_ax_sigma_decode(r2_obj18_b.radar2_obj18_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_ax_sigma_is_in_range(r2_obj18_b.radar2_obj18_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj18_b_radar2_obj18_dy_sigma_decode(r2_obj18_b.radar2_obj18_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_dy_sigma_is_in_range(r2_obj18_b.radar2_obj18_dy_sigma);
              w_class_decoded = xgu_radar2_obj18_b_radar2_obj18_w_class_decode(r2_obj18_b.radar2_obj18_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj18_b_radar2_obj18_w_class_is_in_range(r2_obj18_b.radar2_obj18_w_class);
              class_decoded = xgu_radar2_obj18_b_radar2_obj18_class_decode(r2_obj18_b.radar2_obj18_class);
              class_is_in_range = xgu_radar2_obj18_b_radar2_obj18_class_is_in_range(r2_obj18_b.radar2_obj18_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj18_b_radar2_obj18_dx_rear_end_loss_decode(r2_obj18_b.radar2_obj18_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj18_b_radar2_obj18_dx_rear_end_loss_is_in_range(
                  r2_obj18_b.radar2_obj18_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj18_b_radar2_obj18_mess_bconsist_bit_encode(r2_obj18_b.radar2_obj18_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj18_b_radar2_obj18_mess_bconsist_bit_is_in_range(
                  r2_obj18_b.radar2_obj18_mess_bconsist_bit);
              break;
            case 1478:
              xgu_radar2_obj19_b_t r2_obj19_b;

              vy_decoded = xgu_radar2_obj19_b_radar2_obj19_vy_decode(r2_obj19_b.radar2_obj19_vy);
              vy_is_in_range = xgu_radar2_obj19_b_radar2_obj19_vy_is_in_range(r2_obj19_b.radar2_obj19_vy);
              d_length_decoded = xgu_radar2_obj19_b_radar2_obj19_d_length_decode(r2_obj19_b.radar2_obj19_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_d_length_is_in_range(r2_obj19_b.radar2_obj19_d_length);
              dz_decoded = xgu_radar2_obj19_b_radar2_obj19_dz_decode(r2_obj19_b.radar2_obj19_dz);
              dz_is_in_range = xgu_radar2_obj19_b_radar2_obj19_dz_is_in_range(r2_obj19_b.radar2_obj19_dz);
              moving_state_decoded =
                  xgu_radar2_obj19_b_radar2_obj19_moving_state_decode(r2_obj19_b.radar2_obj19_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_moving_state_is_in_range(r2_obj19_b.radar2_obj19_moving_state);
              dx_sigma_decoded = xgu_radar2_obj19_b_radar2_obj19_dx_sigma_decode(r2_obj19_b.radar2_obj19_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_dx_sigma_is_in_range(r2_obj19_b.radar2_obj19_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj19_b_radar2_obj19_vx_sigma_decode(r2_obj19_b.radar2_obj19_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_vx_sigma_is_in_range(r2_obj19_b.radar2_obj19_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj19_b_radar2_obj19_ax_sigma_decode(r2_obj19_b.radar2_obj19_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_ax_sigma_is_in_range(r2_obj19_b.radar2_obj19_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj19_b_radar2_obj19_dy_sigma_decode(r2_obj19_b.radar2_obj19_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_dy_sigma_is_in_range(r2_obj19_b.radar2_obj19_dy_sigma);
              w_class_decoded = xgu_radar2_obj19_b_radar2_obj19_w_class_decode(r2_obj19_b.radar2_obj19_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj19_b_radar2_obj19_w_class_is_in_range(r2_obj19_b.radar2_obj19_w_class);
              class_decoded = xgu_radar2_obj19_b_radar2_obj19_class_decode(r2_obj19_b.radar2_obj19_class);
              class_is_in_range = xgu_radar2_obj19_b_radar2_obj19_class_is_in_range(r2_obj19_b.radar2_obj19_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj19_b_radar2_obj19_dx_rear_end_loss_decode(r2_obj19_b.radar2_obj19_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj19_b_radar2_obj19_dx_rear_end_loss_is_in_range(
                  r2_obj19_b.radar2_obj19_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj19_b_radar2_obj19_mess_bconsist_bit_encode(r2_obj19_b.radar2_obj19_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj19_b_radar2_obj19_mess_bconsist_bit_is_in_range(
                  r2_obj19_b.radar2_obj19_mess_bconsist_bit);
              break;
            case 1488:
              xgu_radar2_obj20_b_t r2_obj20_b;

              vy_decoded = xgu_radar2_obj20_b_radar2_obj20_vy_decode(r2_obj20_b.radar2_obj20_vy);
              vy_is_in_range = xgu_radar2_obj20_b_radar2_obj20_vy_is_in_range(r2_obj20_b.radar2_obj20_vy);
              d_length_decoded = xgu_radar2_obj20_b_radar2_obj20_d_length_decode(r2_obj20_b.radar2_obj20_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_d_length_is_in_range(r2_obj20_b.radar2_obj20_d_length);
              dz_decoded = xgu_radar2_obj20_b_radar2_obj20_dz_decode(r2_obj20_b.radar2_obj20_dz);
              dz_is_in_range = xgu_radar2_obj20_b_radar2_obj20_dz_is_in_range(r2_obj20_b.radar2_obj20_dz);
              moving_state_decoded =
                  xgu_radar2_obj20_b_radar2_obj20_moving_state_decode(r2_obj20_b.radar2_obj20_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_moving_state_is_in_range(r2_obj20_b.radar2_obj20_moving_state);
              dx_sigma_decoded = xgu_radar2_obj20_b_radar2_obj20_dx_sigma_decode(r2_obj20_b.radar2_obj20_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_dx_sigma_is_in_range(r2_obj20_b.radar2_obj20_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj20_b_radar2_obj20_vx_sigma_decode(r2_obj20_b.radar2_obj20_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_vx_sigma_is_in_range(r2_obj20_b.radar2_obj20_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj20_b_radar2_obj20_ax_sigma_decode(r2_obj20_b.radar2_obj20_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_ax_sigma_is_in_range(r2_obj20_b.radar2_obj20_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj20_b_radar2_obj20_dy_sigma_decode(r2_obj20_b.radar2_obj20_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_dy_sigma_is_in_range(r2_obj20_b.radar2_obj20_dy_sigma);
              w_class_decoded = xgu_radar2_obj20_b_radar2_obj20_w_class_decode(r2_obj20_b.radar2_obj20_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj20_b_radar2_obj20_w_class_is_in_range(r2_obj20_b.radar2_obj20_w_class);
              class_decoded = xgu_radar2_obj20_b_radar2_obj20_class_decode(r2_obj20_b.radar2_obj20_class);
              class_is_in_range = xgu_radar2_obj20_b_radar2_obj20_class_is_in_range(r2_obj20_b.radar2_obj20_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj20_b_radar2_obj20_dx_rear_end_loss_decode(r2_obj20_b.radar2_obj20_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj20_b_radar2_obj20_dx_rear_end_loss_is_in_range(
                  r2_obj20_b.radar2_obj20_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj20_b_radar2_obj20_mess_bconsist_bit_encode(r2_obj20_b.radar2_obj20_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj20_b_radar2_obj20_mess_bconsist_bit_is_in_range(
                  r2_obj20_b.radar2_obj20_mess_bconsist_bit);
              break;
            case 1498:
              xgu_radar2_obj21_b_t r2_obj21_b;

              vy_decoded = xgu_radar2_obj21_b_radar2_obj21_vy_decode(r2_obj21_b.radar2_obj21_vy);
              vy_is_in_range = xgu_radar2_obj21_b_radar2_obj21_vy_is_in_range(r2_obj21_b.radar2_obj21_vy);
              d_length_decoded = xgu_radar2_obj21_b_radar2_obj21_d_length_decode(r2_obj21_b.radar2_obj21_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_d_length_is_in_range(r2_obj21_b.radar2_obj21_d_length);
              dz_decoded = xgu_radar2_obj21_b_radar2_obj21_dz_decode(r2_obj21_b.radar2_obj21_dz);
              dz_is_in_range = xgu_radar2_obj21_b_radar2_obj21_dz_is_in_range(r2_obj21_b.radar2_obj21_dz);
              moving_state_decoded =
                  xgu_radar2_obj21_b_radar2_obj21_moving_state_decode(r2_obj21_b.radar2_obj21_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_moving_state_is_in_range(r2_obj21_b.radar2_obj21_moving_state);
              dx_sigma_decoded = xgu_radar2_obj21_b_radar2_obj21_dx_sigma_decode(r2_obj21_b.radar2_obj21_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_dx_sigma_is_in_range(r2_obj21_b.radar2_obj21_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj21_b_radar2_obj21_vx_sigma_decode(r2_obj21_b.radar2_obj21_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_vx_sigma_is_in_range(r2_obj21_b.radar2_obj21_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj21_b_radar2_obj21_ax_sigma_decode(r2_obj21_b.radar2_obj21_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_ax_sigma_is_in_range(r2_obj21_b.radar2_obj21_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj21_b_radar2_obj21_dy_sigma_decode(r2_obj21_b.radar2_obj21_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_dy_sigma_is_in_range(r2_obj21_b.radar2_obj21_dy_sigma);
              w_class_decoded = xgu_radar2_obj21_b_radar2_obj21_w_class_decode(r2_obj21_b.radar2_obj21_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj21_b_radar2_obj21_w_class_is_in_range(r2_obj21_b.radar2_obj21_w_class);
              class_decoded = xgu_radar2_obj21_b_radar2_obj21_class_decode(r2_obj21_b.radar2_obj21_class);
              class_is_in_range = xgu_radar2_obj21_b_radar2_obj21_class_is_in_range(r2_obj21_b.radar2_obj21_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj21_b_radar2_obj21_dx_rear_end_loss_decode(r2_obj21_b.radar2_obj21_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj21_b_radar2_obj21_dx_rear_end_loss_is_in_range(
                  r2_obj21_b.radar2_obj21_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj21_b_radar2_obj21_mess_bconsist_bit_encode(r2_obj21_b.radar2_obj21_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj21_b_radar2_obj21_mess_bconsist_bit_is_in_range(
                  r2_obj21_b.radar2_obj21_mess_bconsist_bit);
              break;
            case 1508:
              xgu_radar2_obj22_b_t r2_obj22_b;

              vy_decoded = xgu_radar2_obj22_b_radar2_obj22_vy_decode(r2_obj22_b.radar2_obj22_vy);
              vy_is_in_range = xgu_radar2_obj22_b_radar2_obj22_vy_is_in_range(r2_obj22_b.radar2_obj22_vy);
              d_length_decoded = xgu_radar2_obj22_b_radar2_obj22_d_length_decode(r2_obj22_b.radar2_obj22_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_d_length_is_in_range(r2_obj22_b.radar2_obj22_d_length);
              dz_decoded = xgu_radar2_obj22_b_radar2_obj22_dz_decode(r2_obj22_b.radar2_obj22_dz);
              dz_is_in_range = xgu_radar2_obj22_b_radar2_obj22_dz_is_in_range(r2_obj22_b.radar2_obj22_dz);
              moving_state_decoded =
                  xgu_radar2_obj22_b_radar2_obj22_moving_state_decode(r2_obj22_b.radar2_obj22_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_moving_state_is_in_range(r2_obj22_b.radar2_obj22_moving_state);
              dx_sigma_decoded = xgu_radar2_obj22_b_radar2_obj22_dx_sigma_decode(r2_obj22_b.radar2_obj22_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_dx_sigma_is_in_range(r2_obj22_b.radar2_obj22_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj22_b_radar2_obj22_vx_sigma_decode(r2_obj22_b.radar2_obj22_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_vx_sigma_is_in_range(r2_obj22_b.radar2_obj22_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj22_b_radar2_obj22_ax_sigma_decode(r2_obj22_b.radar2_obj22_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_ax_sigma_is_in_range(r2_obj22_b.radar2_obj22_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj22_b_radar2_obj22_dy_sigma_decode(r2_obj22_b.radar2_obj22_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_dy_sigma_is_in_range(r2_obj22_b.radar2_obj22_dy_sigma);
              w_class_decoded = xgu_radar2_obj22_b_radar2_obj22_w_class_decode(r2_obj22_b.radar2_obj22_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj22_b_radar2_obj22_w_class_is_in_range(r2_obj22_b.radar2_obj22_w_class);
              class_decoded = xgu_radar2_obj22_b_radar2_obj22_class_decode(r2_obj22_b.radar2_obj22_class);
              class_is_in_range = xgu_radar2_obj22_b_radar2_obj22_class_is_in_range(r2_obj22_b.radar2_obj22_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj22_b_radar2_obj22_dx_rear_end_loss_decode(r2_obj22_b.radar2_obj22_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj22_b_radar2_obj22_dx_rear_end_loss_is_in_range(
                  r2_obj22_b.radar2_obj22_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj22_b_radar2_obj22_mess_bconsist_bit_encode(r2_obj22_b.radar2_obj22_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj22_b_radar2_obj22_mess_bconsist_bit_is_in_range(
                  r2_obj22_b.radar2_obj22_mess_bconsist_bit);
              break;
            case 1518:
              xgu_radar2_obj23_b_t r2_obj23_b;

              vy_decoded = xgu_radar2_obj23_b_radar2_obj23_vy_decode(r2_obj23_b.radar2_obj23_vy);
              vy_is_in_range = xgu_radar2_obj23_b_radar2_obj23_vy_is_in_range(r2_obj23_b.radar2_obj23_vy);
              d_length_decoded = xgu_radar2_obj23_b_radar2_obj23_d_length_decode(r2_obj23_b.radar2_obj23_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_d_length_is_in_range(r2_obj23_b.radar2_obj23_d_length);
              dz_decoded = xgu_radar2_obj23_b_radar2_obj23_dz_decode(r2_obj23_b.radar2_obj23_dz);
              dz_is_in_range = xgu_radar2_obj23_b_radar2_obj23_dz_is_in_range(r2_obj23_b.radar2_obj23_dz);
              moving_state_decoded =
                  xgu_radar2_obj23_b_radar2_obj23_moving_state_decode(r2_obj23_b.radar2_obj23_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_moving_state_is_in_range(r2_obj23_b.radar2_obj23_moving_state);
              dx_sigma_decoded = xgu_radar2_obj23_b_radar2_obj23_dx_sigma_decode(r2_obj23_b.radar2_obj23_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_dx_sigma_is_in_range(r2_obj23_b.radar2_obj23_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj23_b_radar2_obj23_vx_sigma_decode(r2_obj23_b.radar2_obj23_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_vx_sigma_is_in_range(r2_obj23_b.radar2_obj23_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj23_b_radar2_obj23_ax_sigma_decode(r2_obj23_b.radar2_obj23_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_ax_sigma_is_in_range(r2_obj23_b.radar2_obj23_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj23_b_radar2_obj23_dy_sigma_decode(r2_obj23_b.radar2_obj23_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_dy_sigma_is_in_range(r2_obj23_b.radar2_obj23_dy_sigma);
              w_class_decoded = xgu_radar2_obj23_b_radar2_obj23_w_class_decode(r2_obj23_b.radar2_obj23_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj23_b_radar2_obj23_w_class_is_in_range(r2_obj23_b.radar2_obj23_w_class);
              class_decoded = xgu_radar2_obj23_b_radar2_obj23_class_decode(r2_obj23_b.radar2_obj23_class);
              class_is_in_range = xgu_radar2_obj23_b_radar2_obj23_class_is_in_range(r2_obj23_b.radar2_obj23_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj23_b_radar2_obj23_dx_rear_end_loss_decode(r2_obj23_b.radar2_obj23_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj23_b_radar2_obj23_dx_rear_end_loss_is_in_range(
                  r2_obj23_b.radar2_obj23_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj23_b_radar2_obj23_mess_bconsist_bit_encode(r2_obj23_b.radar2_obj23_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj23_b_radar2_obj23_mess_bconsist_bit_is_in_range(
                  r2_obj23_b.radar2_obj23_mess_bconsist_bit);
              break;
            case 1528:
              xgu_radar2_obj24_b_t r2_obj24_b;

              vy_decoded = xgu_radar2_obj24_b_radar2_obj24_vy_decode(r2_obj24_b.radar2_obj24_vy);
              vy_is_in_range = xgu_radar2_obj24_b_radar2_obj24_vy_is_in_range(r2_obj24_b.radar2_obj24_vy);
              d_length_decoded = xgu_radar2_obj24_b_radar2_obj24_d_length_decode(r2_obj24_b.radar2_obj24_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_d_length_is_in_range(r2_obj24_b.radar2_obj24_d_length);
              dz_decoded = xgu_radar2_obj24_b_radar2_obj24_dz_decode(r2_obj24_b.radar2_obj24_dz);
              dz_is_in_range = xgu_radar2_obj24_b_radar2_obj24_dz_is_in_range(r2_obj24_b.radar2_obj24_dz);
              moving_state_decoded =
                  xgu_radar2_obj24_b_radar2_obj24_moving_state_decode(r2_obj24_b.radar2_obj24_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_moving_state_is_in_range(r2_obj24_b.radar2_obj24_moving_state);
              dx_sigma_decoded = xgu_radar2_obj24_b_radar2_obj24_dx_sigma_decode(r2_obj24_b.radar2_obj24_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_dx_sigma_is_in_range(r2_obj24_b.radar2_obj24_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj24_b_radar2_obj24_vx_sigma_decode(r2_obj24_b.radar2_obj24_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_vx_sigma_is_in_range(r2_obj24_b.radar2_obj24_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj24_b_radar2_obj24_ax_sigma_decode(r2_obj24_b.radar2_obj24_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_ax_sigma_is_in_range(r2_obj24_b.radar2_obj24_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj24_b_radar2_obj24_dy_sigma_decode(r2_obj24_b.radar2_obj24_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_dy_sigma_is_in_range(r2_obj24_b.radar2_obj24_dy_sigma);
              w_class_decoded = xgu_radar2_obj24_b_radar2_obj24_w_class_decode(r2_obj24_b.radar2_obj24_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj24_b_radar2_obj24_w_class_is_in_range(r2_obj24_b.radar2_obj24_w_class);
              class_decoded = xgu_radar2_obj24_b_radar2_obj24_class_decode(r2_obj24_b.radar2_obj24_class);
              class_is_in_range = xgu_radar2_obj24_b_radar2_obj24_class_is_in_range(r2_obj24_b.radar2_obj24_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj24_b_radar2_obj24_dx_rear_end_loss_decode(r2_obj24_b.radar2_obj24_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj24_b_radar2_obj24_dx_rear_end_loss_is_in_range(
                  r2_obj24_b.radar2_obj24_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj24_b_radar2_obj24_mess_bconsist_bit_encode(r2_obj24_b.radar2_obj24_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj24_b_radar2_obj24_mess_bconsist_bit_is_in_range(
                  r2_obj24_b.radar2_obj24_mess_bconsist_bit);
              break;
            case 1538:
              xgu_radar2_obj25_b_t r2_obj25_b;

              vy_decoded = xgu_radar2_obj25_b_radar2_obj25_vy_decode(r2_obj25_b.radar2_obj25_vy);
              vy_is_in_range = xgu_radar2_obj25_b_radar2_obj25_vy_is_in_range(r2_obj25_b.radar2_obj25_vy);
              d_length_decoded = xgu_radar2_obj25_b_radar2_obj25_d_length_decode(r2_obj25_b.radar2_obj25_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_d_length_is_in_range(r2_obj25_b.radar2_obj25_d_length);
              dz_decoded = xgu_radar2_obj25_b_radar2_obj25_dz_decode(r2_obj25_b.radar2_obj25_dz);
              dz_is_in_range = xgu_radar2_obj25_b_radar2_obj25_dz_is_in_range(r2_obj25_b.radar2_obj25_dz);
              moving_state_decoded =
                  xgu_radar2_obj25_b_radar2_obj25_moving_state_decode(r2_obj25_b.radar2_obj25_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_moving_state_is_in_range(r2_obj25_b.radar2_obj25_moving_state);
              dx_sigma_decoded = xgu_radar2_obj25_b_radar2_obj25_dx_sigma_decode(r2_obj25_b.radar2_obj25_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_dx_sigma_is_in_range(r2_obj25_b.radar2_obj25_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj25_b_radar2_obj25_vx_sigma_decode(r2_obj25_b.radar2_obj25_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_vx_sigma_is_in_range(r2_obj25_b.radar2_obj25_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj25_b_radar2_obj25_ax_sigma_decode(r2_obj25_b.radar2_obj25_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_ax_sigma_is_in_range(r2_obj25_b.radar2_obj25_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj25_b_radar2_obj25_dy_sigma_decode(r2_obj25_b.radar2_obj25_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_dy_sigma_is_in_range(r2_obj25_b.radar2_obj25_dy_sigma);
              w_class_decoded = xgu_radar2_obj25_b_radar2_obj25_w_class_decode(r2_obj25_b.radar2_obj25_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj25_b_radar2_obj25_w_class_is_in_range(r2_obj25_b.radar2_obj25_w_class);
              class_decoded = xgu_radar2_obj25_b_radar2_obj25_class_decode(r2_obj25_b.radar2_obj25_class);
              class_is_in_range = xgu_radar2_obj25_b_radar2_obj25_class_is_in_range(r2_obj25_b.radar2_obj25_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj25_b_radar2_obj25_dx_rear_end_loss_decode(r2_obj25_b.radar2_obj25_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj25_b_radar2_obj25_dx_rear_end_loss_is_in_range(
                  r2_obj25_b.radar2_obj25_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj25_b_radar2_obj25_mess_bconsist_bit_encode(r2_obj25_b.radar2_obj25_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj25_b_radar2_obj25_mess_bconsist_bit_is_in_range(
                  r2_obj25_b.radar2_obj25_mess_bconsist_bit);
              break;
            case 1548:
              xgu_radar2_obj26_b_t r2_obj26_b;

              vy_decoded = xgu_radar2_obj26_b_radar2_obj26_vy_decode(r2_obj26_b.radar2_obj26_vy);
              vy_is_in_range = xgu_radar2_obj26_b_radar2_obj26_vy_is_in_range(r2_obj26_b.radar2_obj26_vy);
              d_length_decoded = xgu_radar2_obj26_b_radar2_obj26_d_length_decode(r2_obj26_b.radar2_obj26_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_d_length_is_in_range(r2_obj26_b.radar2_obj26_d_length);
              dz_decoded = xgu_radar2_obj26_b_radar2_obj26_dz_decode(r2_obj26_b.radar2_obj26_dz);
              dz_is_in_range = xgu_radar2_obj26_b_radar2_obj26_dz_is_in_range(r2_obj26_b.radar2_obj26_dz);
              moving_state_decoded =
                  xgu_radar2_obj26_b_radar2_obj26_moving_state_decode(r2_obj26_b.radar2_obj26_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_moving_state_is_in_range(r2_obj26_b.radar2_obj26_moving_state);
              dx_sigma_decoded = xgu_radar2_obj26_b_radar2_obj26_dx_sigma_decode(r2_obj26_b.radar2_obj26_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_dx_sigma_is_in_range(r2_obj26_b.radar2_obj26_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj26_b_radar2_obj26_vx_sigma_decode(r2_obj26_b.radar2_obj26_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_vx_sigma_is_in_range(r2_obj26_b.radar2_obj26_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj26_b_radar2_obj26_ax_sigma_decode(r2_obj26_b.radar2_obj26_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_ax_sigma_is_in_range(r2_obj26_b.radar2_obj26_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj26_b_radar2_obj26_dy_sigma_decode(r2_obj26_b.radar2_obj26_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_dy_sigma_is_in_range(r2_obj26_b.radar2_obj26_dy_sigma);
              w_class_decoded = xgu_radar2_obj26_b_radar2_obj26_w_class_decode(r2_obj26_b.radar2_obj26_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj26_b_radar2_obj26_w_class_is_in_range(r2_obj26_b.radar2_obj26_w_class);
              class_decoded = xgu_radar2_obj26_b_radar2_obj26_class_decode(r2_obj26_b.radar2_obj26_class);
              class_is_in_range = xgu_radar2_obj26_b_radar2_obj26_class_is_in_range(r2_obj26_b.radar2_obj26_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj26_b_radar2_obj26_dx_rear_end_loss_decode(r2_obj26_b.radar2_obj26_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj26_b_radar2_obj26_dx_rear_end_loss_is_in_range(
                  r2_obj26_b.radar2_obj26_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj26_b_radar2_obj26_mess_bconsist_bit_encode(r2_obj26_b.radar2_obj26_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj26_b_radar2_obj26_mess_bconsist_bit_is_in_range(
                  r2_obj26_b.radar2_obj26_mess_bconsist_bit);
              break;
            case 1558:
              xgu_radar2_obj27_b_t r2_obj27_b;

              vy_decoded = xgu_radar2_obj27_b_radar2_obj27_vy_decode(r2_obj27_b.radar2_obj27_vy);
              vy_is_in_range = xgu_radar2_obj27_b_radar2_obj27_vy_is_in_range(r2_obj27_b.radar2_obj27_vy);
              d_length_decoded = xgu_radar2_obj27_b_radar2_obj27_d_length_decode(r2_obj27_b.radar2_obj27_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_d_length_is_in_range(r2_obj27_b.radar2_obj27_d_length);
              dz_decoded = xgu_radar2_obj27_b_radar2_obj27_dz_decode(r2_obj27_b.radar2_obj27_dz);
              dz_is_in_range = xgu_radar2_obj27_b_radar2_obj27_dz_is_in_range(r2_obj27_b.radar2_obj27_dz);
              moving_state_decoded =
                  xgu_radar2_obj27_b_radar2_obj27_moving_state_decode(r2_obj27_b.radar2_obj27_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_moving_state_is_in_range(r2_obj27_b.radar2_obj27_moving_state);
              dx_sigma_decoded = xgu_radar2_obj27_b_radar2_obj27_dx_sigma_decode(r2_obj27_b.radar2_obj27_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_dx_sigma_is_in_range(r2_obj27_b.radar2_obj27_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj27_b_radar2_obj27_vx_sigma_decode(r2_obj27_b.radar2_obj27_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_vx_sigma_is_in_range(r2_obj27_b.radar2_obj27_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj27_b_radar2_obj27_ax_sigma_decode(r2_obj27_b.radar2_obj27_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_ax_sigma_is_in_range(r2_obj27_b.radar2_obj27_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj27_b_radar2_obj27_dy_sigma_decode(r2_obj27_b.radar2_obj27_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_dy_sigma_is_in_range(r2_obj27_b.radar2_obj27_dy_sigma);
              w_class_decoded = xgu_radar2_obj27_b_radar2_obj27_w_class_decode(r2_obj27_b.radar2_obj27_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj27_b_radar2_obj27_w_class_is_in_range(r2_obj27_b.radar2_obj27_w_class);
              class_decoded = xgu_radar2_obj27_b_radar2_obj27_class_decode(r2_obj27_b.radar2_obj27_class);
              class_is_in_range = xgu_radar2_obj27_b_radar2_obj27_class_is_in_range(r2_obj27_b.radar2_obj27_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj27_b_radar2_obj27_dx_rear_end_loss_decode(r2_obj27_b.radar2_obj27_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj27_b_radar2_obj27_dx_rear_end_loss_is_in_range(
                  r2_obj27_b.radar2_obj27_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj27_b_radar2_obj27_mess_bconsist_bit_encode(r2_obj27_b.radar2_obj27_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj27_b_radar2_obj27_mess_bconsist_bit_is_in_range(
                  r2_obj27_b.radar2_obj27_mess_bconsist_bit);
              break;
            case 1568:
              xgu_radar2_obj28_b_t r2_obj28_b;

              vy_decoded = xgu_radar2_obj28_b_radar2_obj28_vy_decode(r2_obj28_b.radar2_obj28_vy);
              vy_is_in_range = xgu_radar2_obj28_b_radar2_obj28_vy_is_in_range(r2_obj28_b.radar2_obj28_vy);
              d_length_decoded = xgu_radar2_obj28_b_radar2_obj28_d_length_decode(r2_obj28_b.radar2_obj28_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_d_length_is_in_range(r2_obj28_b.radar2_obj28_d_length);
              dz_decoded = xgu_radar2_obj28_b_radar2_obj28_dz_decode(r2_obj28_b.radar2_obj28_dz);
              dz_is_in_range = xgu_radar2_obj28_b_radar2_obj28_dz_is_in_range(r2_obj28_b.radar2_obj28_dz);
              moving_state_decoded =
                  xgu_radar2_obj28_b_radar2_obj28_moving_state_decode(r2_obj28_b.radar2_obj28_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_moving_state_is_in_range(r2_obj28_b.radar2_obj28_moving_state);
              dx_sigma_decoded = xgu_radar2_obj28_b_radar2_obj28_dx_sigma_decode(r2_obj28_b.radar2_obj28_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_dx_sigma_is_in_range(r2_obj28_b.radar2_obj28_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj28_b_radar2_obj28_vx_sigma_decode(r2_obj28_b.radar2_obj28_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_vx_sigma_is_in_range(r2_obj28_b.radar2_obj28_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj28_b_radar2_obj28_ax_sigma_decode(r2_obj28_b.radar2_obj28_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_ax_sigma_is_in_range(r2_obj28_b.radar2_obj28_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj28_b_radar2_obj28_dy_sigma_decode(r2_obj28_b.radar2_obj28_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_dy_sigma_is_in_range(r2_obj28_b.radar2_obj28_dy_sigma);
              w_class_decoded = xgu_radar2_obj28_b_radar2_obj28_w_class_decode(r2_obj28_b.radar2_obj28_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj28_b_radar2_obj28_w_class_is_in_range(r2_obj28_b.radar2_obj28_w_class);
              class_decoded = xgu_radar2_obj28_b_radar2_obj28_class_decode(r2_obj28_b.radar2_obj28_class);
              class_is_in_range = xgu_radar2_obj28_b_radar2_obj28_class_is_in_range(r2_obj28_b.radar2_obj28_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj28_b_radar2_obj28_dx_rear_end_loss_decode(r2_obj28_b.radar2_obj28_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj28_b_radar2_obj28_dx_rear_end_loss_is_in_range(
                  r2_obj28_b.radar2_obj28_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj28_b_radar2_obj28_mess_bconsist_bit_encode(r2_obj28_b.radar2_obj28_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj28_b_radar2_obj28_mess_bconsist_bit_is_in_range(
                  r2_obj28_b.radar2_obj28_mess_bconsist_bit);
              break;
            case 1578:
              xgu_radar2_obj29_b_t r2_obj29_b;

              vy_decoded = xgu_radar2_obj29_b_radar2_obj29_vy_decode(r2_obj29_b.radar2_obj29_vy);
              vy_is_in_range = xgu_radar2_obj29_b_radar2_obj29_vy_is_in_range(r2_obj29_b.radar2_obj29_vy);
              d_length_decoded = xgu_radar2_obj29_b_radar2_obj29_d_length_decode(r2_obj29_b.radar2_obj29_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_d_length_is_in_range(r2_obj29_b.radar2_obj29_d_length);
              dz_decoded = xgu_radar2_obj29_b_radar2_obj29_dz_decode(r2_obj29_b.radar2_obj29_dz);
              dz_is_in_range = xgu_radar2_obj29_b_radar2_obj29_dz_is_in_range(r2_obj29_b.radar2_obj29_dz);
              moving_state_decoded =
                  xgu_radar2_obj29_b_radar2_obj29_moving_state_decode(r2_obj29_b.radar2_obj29_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_moving_state_is_in_range(r2_obj29_b.radar2_obj29_moving_state);
              dx_sigma_decoded = xgu_radar2_obj29_b_radar2_obj29_dx_sigma_decode(r2_obj29_b.radar2_obj29_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_dx_sigma_is_in_range(r2_obj29_b.radar2_obj29_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj29_b_radar2_obj29_vx_sigma_decode(r2_obj29_b.radar2_obj29_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_vx_sigma_is_in_range(r2_obj29_b.radar2_obj29_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj29_b_radar2_obj29_ax_sigma_decode(r2_obj29_b.radar2_obj29_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_ax_sigma_is_in_range(r2_obj29_b.radar2_obj29_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj29_b_radar2_obj29_dy_sigma_decode(r2_obj29_b.radar2_obj29_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_dy_sigma_is_in_range(r2_obj29_b.radar2_obj29_dy_sigma);
              w_class_decoded = xgu_radar2_obj29_b_radar2_obj29_w_class_decode(r2_obj29_b.radar2_obj29_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj29_b_radar2_obj29_w_class_is_in_range(r2_obj29_b.radar2_obj29_w_class);
              class_decoded = xgu_radar2_obj29_b_radar2_obj29_class_decode(r2_obj29_b.radar2_obj29_class);
              class_is_in_range = xgu_radar2_obj29_b_radar2_obj29_class_is_in_range(r2_obj29_b.radar2_obj29_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj29_b_radar2_obj29_dx_rear_end_loss_decode(r2_obj29_b.radar2_obj29_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj29_b_radar2_obj29_dx_rear_end_loss_is_in_range(
                  r2_obj29_b.radar2_obj29_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj29_b_radar2_obj29_mess_bconsist_bit_encode(r2_obj29_b.radar2_obj29_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj29_b_radar2_obj29_mess_bconsist_bit_is_in_range(
                  r2_obj29_b.radar2_obj29_mess_bconsist_bit);
              break;
            case 1588:
              xgu_radar2_obj30_b_t r2_obj30_b;

              vy_decoded = xgu_radar2_obj30_b_radar2_obj30_vy_decode(r2_obj30_b.radar2_obj30_vy);
              vy_is_in_range = xgu_radar2_obj30_b_radar2_obj30_vy_is_in_range(r2_obj30_b.radar2_obj30_vy);
              d_length_decoded = xgu_radar2_obj30_b_radar2_obj30_d_length_decode(r2_obj30_b.radar2_obj30_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_d_length_is_in_range(r2_obj30_b.radar2_obj30_d_length);
              dz_decoded = xgu_radar2_obj30_b_radar2_obj30_dz_decode(r2_obj30_b.radar2_obj30_dz);
              dz_is_in_range = xgu_radar2_obj30_b_radar2_obj30_dz_is_in_range(r2_obj30_b.radar2_obj30_dz);
              moving_state_decoded =
                  xgu_radar2_obj30_b_radar2_obj30_moving_state_decode(r2_obj30_b.radar2_obj30_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_moving_state_is_in_range(r2_obj30_b.radar2_obj30_moving_state);
              dx_sigma_decoded = xgu_radar2_obj30_b_radar2_obj30_dx_sigma_decode(r2_obj30_b.radar2_obj30_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_dx_sigma_is_in_range(r2_obj30_b.radar2_obj30_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj30_b_radar2_obj30_vx_sigma_decode(r2_obj30_b.radar2_obj30_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_vx_sigma_is_in_range(r2_obj30_b.radar2_obj30_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj30_b_radar2_obj30_ax_sigma_decode(r2_obj30_b.radar2_obj30_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_ax_sigma_is_in_range(r2_obj30_b.radar2_obj30_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj30_b_radar2_obj30_dy_sigma_decode(r2_obj30_b.radar2_obj30_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_dy_sigma_is_in_range(r2_obj30_b.radar2_obj30_dy_sigma);
              w_class_decoded = xgu_radar2_obj30_b_radar2_obj30_w_class_decode(r2_obj30_b.radar2_obj30_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj30_b_radar2_obj30_w_class_is_in_range(r2_obj30_b.radar2_obj30_w_class);
              class_decoded = xgu_radar2_obj30_b_radar2_obj30_class_decode(r2_obj30_b.radar2_obj30_class);
              class_is_in_range = xgu_radar2_obj30_b_radar2_obj30_class_is_in_range(r2_obj30_b.radar2_obj30_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj30_b_radar2_obj30_dx_rear_end_loss_decode(r2_obj30_b.radar2_obj30_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj30_b_radar2_obj30_dx_rear_end_loss_is_in_range(
                  r2_obj30_b.radar2_obj30_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj30_b_radar2_obj30_mess_bconsist_bit_encode(r2_obj30_b.radar2_obj30_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj30_b_radar2_obj30_mess_bconsist_bit_is_in_range(
                  r2_obj30_b.radar2_obj30_mess_bconsist_bit);
              break;
            case 1598:
              xgu_radar2_obj31_b_t r2_obj31_b;

              vy_decoded = xgu_radar2_obj31_b_radar2_obj31_vy_decode(r2_obj31_b.radar2_obj31_vy);
              vy_is_in_range = xgu_radar2_obj31_b_radar2_obj31_vy_is_in_range(r2_obj31_b.radar2_obj31_vy);
              d_length_decoded = xgu_radar2_obj31_b_radar2_obj31_d_length_decode(r2_obj31_b.radar2_obj31_d_length);
              d_length_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_d_length_is_in_range(r2_obj31_b.radar2_obj31_d_length);
              dz_decoded = xgu_radar2_obj31_b_radar2_obj31_dz_decode(r2_obj31_b.radar2_obj31_dz);
              dz_is_in_range = xgu_radar2_obj31_b_radar2_obj31_dz_is_in_range(r2_obj31_b.radar2_obj31_dz);
              moving_state_decoded =
                  xgu_radar2_obj31_b_radar2_obj31_moving_state_decode(r2_obj31_b.radar2_obj31_moving_state);
              moving_state_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_moving_state_is_in_range(r2_obj31_b.radar2_obj31_moving_state);
              dx_sigma_decoded = xgu_radar2_obj31_b_radar2_obj31_dx_sigma_decode(r2_obj31_b.radar2_obj31_dx_sigma);
              dx_sigma_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_dx_sigma_is_in_range(r2_obj31_b.radar2_obj31_dx_sigma);
              vx_sigma_decoded = xgu_radar2_obj31_b_radar2_obj31_vx_sigma_decode(r2_obj31_b.radar2_obj31_vx_sigma);
              vx_sigma_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_vx_sigma_is_in_range(r2_obj31_b.radar2_obj31_vx_sigma);
              ax_sigma_decoded = xgu_radar2_obj31_b_radar2_obj31_ax_sigma_decode(r2_obj31_b.radar2_obj31_ax_sigma);
              ax_sigma_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_ax_sigma_is_in_range(r2_obj31_b.radar2_obj31_ax_sigma);
              dy_sigma_decoded = xgu_radar2_obj31_b_radar2_obj31_dy_sigma_decode(r2_obj31_b.radar2_obj31_dy_sigma);
              dy_sigma_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_dy_sigma_is_in_range(r2_obj31_b.radar2_obj31_dy_sigma);
              w_class_decoded = xgu_radar2_obj31_b_radar2_obj31_w_class_decode(r2_obj31_b.radar2_obj31_w_class);
              w_class_is_in_range =
                  xgu_radar2_obj31_b_radar2_obj31_w_class_is_in_range(r2_obj31_b.radar2_obj31_w_class);
              class_decoded = xgu_radar2_obj31_b_radar2_obj31_class_decode(r2_obj31_b.radar2_obj31_class);
              class_is_in_range = xgu_radar2_obj31_b_radar2_obj31_class_is_in_range(r2_obj31_b.radar2_obj31_class);
              dx_rear_end_loss_decoded =
                  xgu_radar2_obj31_b_radar2_obj31_dx_rear_end_loss_decode(r2_obj31_b.radar2_obj31_dx_rear_end_loss);
              dx_rear_end_loss_is_in_range = xgu_radar2_obj31_b_radar2_obj31_dx_rear_end_loss_is_in_range(
                  r2_obj31_b.radar2_obj31_dx_rear_end_loss);
              mess_bconsist_bit_decoded =
                  xgu_radar2_obj31_b_radar2_obj31_mess_bconsist_bit_encode(r2_obj31_b.radar2_obj31_mess_bconsist_bit);
              mess_bconsist_bit_is_in_range = xgu_radar2_obj31_b_radar2_obj31_mess_bconsist_bit_is_in_range(
                  r2_obj31_b.radar2_obj31_mess_bconsist_bit);
              break;
          }
          break;

      }
    }
  canBusOff(hnd);
  canClose(hnd);

  ros::spinOnce();
  }
  return 0;
}