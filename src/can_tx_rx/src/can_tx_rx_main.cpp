#include "can_tx_rx/can_tx_rx.h"
#include <canlib.h>
#include "ros/ros.h"

#include "bosch_xgu_corner_radar.h"
#include "xgu.h"

void get_nums(int id, uint8_t &case_n, uint8_t &radar_n, uint8_t &frame_n, uint8_t &obj_n, uint8_t &target_obj_n) {
  if (id == 1985 || id == 1958 || id == 1879 || id == 1957) {
    case_n = 1;
  } else if (id > 1604 && id < 1659) {
    case_n = 2;
  } else if (id == 1665 || id == 1667 || id == 1280 || id == 1282 || id == 1670 || id == 1672) {
    case_n = 3;
  } else if (id > 1284 && id < 1599) {
    case_n = 4;
  } else if (id == 512 || id == 513 || id == 514 || id == 1168 || id == 1170) {
    case_n = 5;
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
  ros::init(argc, argv, "can_tx_rx");
  ros::NodeHandle can_tx_rx_feeder_handle;
  Can_Tx_Rx_Class Can_Tx_Rx_Class(&can_tx_rx_feeder_handle);

  canHandle hnd_0;
  canHandle hnd_1;
  canHandle hnd_2;
  canHandle hnd_3;

  canInitializeLibrary();

  hnd_0 = canOpenChannel(0, canOPEN_EXCLUSIVE);
  hnd_1 = canOpenChannel(1, canOPEN_EXCLUSIVE);
  hnd_2 = canOpenChannel(2, canOPEN_EXCLUSIVE);
  hnd_3 = canOpenChannel(3, canOPEN_EXCLUSIVE);

  if (hnd_0 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_0, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  if (hnd_1 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_1, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  if (hnd_2 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_2, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  if (hnd_3 < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd_3, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);

  canSetBusParams(hnd_0, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusParams(hnd_1, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusParams(hnd_2, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusParams(hnd_3, canBITRATE_250K, 0, 0, 0, 0, 0);

  canSetBusOutputControl(hnd_0, canDRIVER_NORMAL);
  canSetBusOutputControl(hnd_1, canDRIVER_NORMAL);
  canSetBusOutputControl(hnd_2, canDRIVER_NORMAL);
  canSetBusOutputControl(hnd_3, canDRIVER_NORMAL);

  canBusOn(hnd_0);
  canBusOn(hnd_1);
  canBusOn(hnd_2);
  canBusOn(hnd_3);

  long int id;
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  uint8_t case_num = 0;
  uint8_t radar_num = 0;           // 1 or 2 == valid
  uint8_t frame_num = 0;           // 1 = Frame_A, 2 = Frame_B, 3 = general, other = error
  uint8_t obj_num = -1;            // 0 to 31 = valid
  uint8_t target_object_num = -1;  // 0 to 5 = valid
  uint8_t size_of_msg = 8;
  uint8_t can_data[8] = {0};

  int unpack_return = -1;  // 0 is successful, negative error code

  double xgu_radar_diag_decode_response = 0;
  bool xgu_radar_diag_is_in_range = 0;
  double xgu_radar_diag_request = 0;
  uint64_t xgu_radar_diag_request_encoded = 0;

  double xgu_radar_timestamp_decoded = 0;
  bool xgu_radar_timestamp_is_in_range = 0;
  double tc_counter_decoded = 0;
  bool tc_counter_is_in_range = 0;
  double obj_ender_consist_bit_decoded = 0;
  bool obj_ender_consist_bit_is_in_range = 0;
  double packet_checksum_encoded = 0;
  bool packet_checksum_is_in_range = 0;

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

  double target_dx_decoded = 0;
  bool target_dx_is_in_range = 0;
  double target_vx_decode = 0;
  bool target_vx_is_in_range = 0;
  double target_dy_decode = 0;
  bool target_dy_is_in_range = 0;
  double target_w_exist_decode = 0;
  bool target_w_exist_is_in_range = 0;
  double target_ax_decode = 0;
  bool target_ax_is_in_range = 0;
  double target_w_obstacle_decode = 0;
  bool target_w_obstacle_is_in_range = 0;
  double target_flag_valid_decode = 0;
  bool target_flag_valid_is_in_range = 0;
  double target_w_non_obstacle_decode = 0;
  bool target_w_non_obstacle_is_in_range = 0;
  double target_flag_meas_decode = 0;
  bool target_flag_meas_is_in_range = 0;
  double target_flag_hist_decode = 0;
  bool target_flag_hist_is_in_range = 0;
  double target_mess_aconsist_bit_decode = 0;
  bool target_mess_aconsist_bit_is_in_range = 0;

  double target_vy_decoded = 0;
  bool target_vy_is_in_range = 0;
  double target_d_length_decoded = 0;
  bool target_d_length_is_in_range = 0;
  double target_dz_decoded = 0;
  bool target_dz_is_in_range = 0;
  double target_moving_state_decoded = 0;
  bool target_moving_state_is_in_range = 0;
  double target_dx_sigma_decoded = 0;
  bool target_dx_sigma_is_in_range = 0;
  double target_vx_sigma_decoded = 0;
  bool target_vx_sigma_is_in_range = 0;
  double target_ax_sigma_decoded = 0;
  bool target_ax_sigma_is_in_range = 0;
  double target_dy_sigma_decoded = 0;
  bool target_dy_sigma_is_in_range = 0;
  double target_w_class_decoded = 0;
  bool target_w_class_is_in_range = 0;
  double target_class_decoded = 0;
  bool target_class_is_in_range = 0;
  double target_dx_rear_end_loss_decoded = 0;
  bool target_dx_rear_end_loss_is_in_range = 0;
  double target_mess_bconsist_bit_decoded = 0;
  bool target_mess_bconsist_bit_is_in_range = 0;

  while (ros::ok()) {
    /*  canStatus stat = canRead(hnd_0, &id, &can_data, &dlc, &flag, &time);

    if (canOK == stat) {
      // MABx and Jetson
    }
    */

    canStatus stat = canRead(hnd_1, &id, &can_data, &dlc, &flag, &time);

    if (canOK == stat) {
      // Front radar = radar_1 and rear radar = radar_2
      get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num);
      switch (case_num) {
        case 1:
          switch (id) {
            case 1985:
              xgu_radar1_diag_response_t r1_diag_response_obj;
              unpack_return = xgu_radar1_diag_response_unpack(&r1_diag_response_obj, can_data, size_of_msg);
              xgu_radar_diag_decode_response =
                  xgu_radar1_diag_response_r1_diag_response_decode(r1_diag_response_obj.r1_diag_response);
              xgu_radar_diag_is_in_range =
                  xgu_radar1_diag_response_r1_diag_response_is_in_range(r1_diag_response_obj.r1_diag_response);
              break;
            case 1958:
              xgu_radar2_diag_response_t r2_diag_response_obj;
              unpack_return = xgu_radar2_diag_response_unpack(&r2_diag_response_obj, can_data, size_of_msg);
              xgu_radar_diag_decode_response =
                  xgu_radar2_diag_response_r2_diag_response_decode(r2_diag_response_obj.r2_diag_response);
              xgu_radar_diag_is_in_range =
                  xgu_radar2_diag_response_r2_diag_response_is_in_range(r2_diag_response_obj.r2_diag_response);
              break;
          }
          break;
        case 2:
          switch (id) {
            case 1605:
              xgu_radar1_target00_a_t r1_target00_a_obj;
              unpack_return = xgu_radar1_target00_a_unpack(&r1_target00_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar1_target00_a_radar1_target00_dx_decode(r1_target00_a_obj.radar1_target00_dx);
              target_dx_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_dx_is_in_range(r1_target00_a_obj.radar1_target00_dx);
              target_vx_decode = xgu_radar1_target00_a_radar1_target00_vx_decode(r1_target00_a_obj.radar1_target00_vx);
              target_vx_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_vx_is_in_range(r1_target00_a_obj.radar1_target00_vx);
              target_dy_decode = xgu_radar1_target00_a_radar1_target00_dy_decode(r1_target00_a_obj.radar1_target00_dy);
              target_dy_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_dy_is_in_range(r1_target00_a_obj.radar1_target00_dy);
              target_w_exist_decode =
                  xgu_radar1_target00_a_radar1_target00_w_exist_decode(r1_target00_a_obj.radar1_target00_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_w_exist_is_in_range(r1_target00_a_obj.radar1_target00_w_exist);
              target_ax_decode = xgu_radar1_target00_a_radar1_target00_ax_decode(r1_target00_a_obj.radar1_target00_ax);
              target_ax_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_ax_is_in_range(r1_target00_a_obj.radar1_target00_ax);
              target_w_obstacle_decode =
                  xgu_radar1_target00_a_radar1_target00_w_obstacle_decode(r1_target00_a_obj.radar1_target00_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar1_target00_a_radar1_target00_w_obstacle_is_in_range(
                  r1_target00_a_obj.radar1_target00_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar1_target00_a_radar1_target00_flag_valid_decode(r1_target00_a_obj.radar1_target00_flag_valid);
              target_flag_valid_is_in_range = xgu_radar1_target00_a_radar1_target00_flag_valid_is_in_range(
                  r1_target00_a_obj.radar1_target00_flag_valid);
              target_w_non_obstacle_decode = xgu_radar1_target00_a_radar1_target00_w_non_obstacle_decode(
                  r1_target00_a_obj.radar1_target00_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar1_target00_a_radar1_target00_w_non_obstacle_is_in_range(
                  r1_target00_a_obj.radar1_target00_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar1_target00_a_radar1_target00_flag_meas_decode(r1_target00_a_obj.radar1_target00_flag_meas);
              target_flag_meas_is_in_range = xgu_radar1_target00_a_radar1_target00_flag_meas_is_in_range(
                  r1_target00_a_obj.radar1_target00_flag_meas);
              target_flag_hist_decode =
                  xgu_radar1_target00_a_radar1_target00_flag_hist_decode(r1_target00_a_obj.radar1_target00_flag_hist);
              target_flag_hist_is_in_range = xgu_radar1_target00_a_radar1_target00_flag_hist_is_in_range(
                  r1_target00_a_obj.radar1_target00_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar1_target00_a_radar1_target00_mess_aconsist_bit_decode(
                  r1_target00_a_obj.radar1_target00_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target00_a_radar1_target00_mess_aconsist_bit_is_in_range(
                      r1_target00_a_obj.radar1_target00_mess_aconsist_bit);
              break;
            case 1606:
              xgu_radar1_target00_b_t r1_target00_b_obj;
              unpack_return = xgu_radar1_target00_b_unpack(&r1_target00_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar1_target01_b_radar1_target01_vy_decode(r1_target00_b_obj.radar1_target00_vy);
              target_vy_is_in_range =
                  xgu_radar1_target01_b_radar1_target01_vy_is_in_range(r1_target00_b_obj.radar1_target00_vy);
              target_d_length_decoded =
                  xgu_radar1_target01_b_radar1_target01_d_length_decode(r1_target00_b_obj.radar1_target00_d_length);
              target_d_length_is_in_range = xgu_radar1_target01_b_radar1_target01_d_length_is_in_range(
                  r1_target00_b_obj.radar1_target00_d_length);
              target_dz_decoded = xgu_radar1_target01_b_radar1_target01_dz_decode(r1_target00_b_obj.radar1_target00_dz);
              target_dz_is_in_range =
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
              unpack_return = xgu_radar2_target00_a_unpack(&r2_target00_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar2_target00_a_radar2_target00_dx_decode(r2_target00_a_obj.radar2_target00_dx);
              target_dx_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_dx_is_in_range(r2_target00_a_obj.radar2_target00_dx);
              target_vx_decode = xgu_radar2_target00_a_radar2_target00_vx_decode(r2_target00_a_obj.radar2_target00_vx);
              target_vx_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_vx_is_in_range(r2_target00_a_obj.radar2_target00_vx);
              target_dy_decode = xgu_radar2_target00_a_radar2_target00_dy_decode(r2_target00_a_obj.radar2_target00_dy);
              target_dy_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_dy_is_in_range(r2_target00_a_obj.radar2_target00_dy);
              target_w_exist_decode =
                  xgu_radar2_target00_a_radar2_target00_w_exist_decode(r2_target00_a_obj.radar2_target00_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_w_exist_is_in_range(r2_target00_a_obj.radar2_target00_w_exist);
              target_ax_decode = xgu_radar2_target00_a_radar2_target00_ax_decode(r2_target00_a_obj.radar2_target00_ax);
              target_ax_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_ax_is_in_range(r2_target00_a_obj.radar2_target00_ax);
              target_w_obstacle_decode =
                  xgu_radar2_target00_a_radar2_target00_w_obstacle_decode(r2_target00_a_obj.radar2_target00_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar2_target00_a_radar2_target00_w_obstacle_is_in_range(
                  r2_target00_a_obj.radar2_target00_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar2_target00_a_radar2_target00_flag_valid_decode(r2_target00_a_obj.radar2_target00_flag_valid);
              target_flag_valid_is_in_range = xgu_radar2_target00_a_radar2_target00_flag_valid_is_in_range(
                  r2_target00_a_obj.radar2_target00_flag_valid);
              target_w_non_obstacle_decode = xgu_radar2_target00_a_radar2_target00_w_non_obstacle_decode(
                  r2_target00_a_obj.radar2_target00_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar2_target00_a_radar2_target00_w_non_obstacle_is_in_range(
                  r2_target00_a_obj.radar2_target00_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar2_target00_a_radar2_target00_flag_meas_decode(r2_target00_a_obj.radar2_target00_flag_meas);
              target_flag_meas_is_in_range = xgu_radar2_target00_a_radar2_target00_flag_meas_is_in_range(
                  r2_target00_a_obj.radar2_target00_flag_meas);
              target_flag_hist_decode =
                  xgu_radar2_target00_a_radar2_target00_flag_hist_decode(r2_target00_a_obj.radar2_target00_flag_hist);
              target_flag_hist_is_in_range = xgu_radar2_target00_a_radar2_target00_flag_hist_is_in_range(
                  r2_target00_a_obj.radar2_target00_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar2_target00_a_radar2_target00_mess_aconsist_bit_decode(
                  r2_target00_a_obj.radar2_target00_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar2_target00_a_radar2_target00_mess_aconsist_bit_is_in_range(
                      r2_target00_a_obj.radar2_target00_mess_aconsist_bit);
              break;
            case 1608:
              xgu_radar2_target00_b_t r2_target00_b_obj;
              unpack_return = xgu_radar2_target00_b_unpack(&r2_target00_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar2_target02_b_radar2_target02_vy_decode(r2_target00_b_obj.radar2_target00_vy);
              target_vy_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_vy_is_in_range(r2_target00_b_obj.radar2_target00_vy);
              target_d_length_decoded =
                  xgu_radar2_target02_b_radar2_target02_d_length_decode(r2_target00_b_obj.radar2_target00_d_length);
              target_d_length_is_in_range = xgu_radar2_target02_b_radar2_target02_d_length_is_in_range(
                  r2_target00_b_obj.radar2_target00_d_length);
              target_dz_decoded = xgu_radar2_target02_b_radar2_target02_dz_decode(r2_target00_b_obj.radar2_target00_dz);
              target_dz_is_in_range =
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
              unpack_return = xgu_radar1_target01_a_unpack(&r1_target01_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar1_target01_a_radar1_target01_dx_decode(r1_target01_a_obj.radar1_target01_dx);
              target_dx_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_dx_is_in_range(r1_target01_a_obj.radar1_target01_dx);
              target_vx_decode = xgu_radar1_target01_a_radar1_target01_vx_decode(r1_target01_a_obj.radar1_target01_vx);
              target_vx_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_vx_is_in_range(r1_target01_a_obj.radar1_target01_vx);
              target_dy_decode = xgu_radar1_target01_a_radar1_target01_dy_decode(r1_target01_a_obj.radar1_target01_dy);
              target_dy_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_dy_is_in_range(r1_target01_a_obj.radar1_target01_dy);
              target_w_exist_decode =
                  xgu_radar1_target01_a_radar1_target01_w_exist_decode(r1_target01_a_obj.radar1_target01_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_w_exist_is_in_range(r1_target01_a_obj.radar1_target01_w_exist);
              target_ax_decode = xgu_radar1_target01_a_radar1_target01_ax_decode(r1_target01_a_obj.radar1_target01_ax);
              target_ax_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_ax_is_in_range(r1_target01_a_obj.radar1_target01_ax);
              target_w_obstacle_decode =
                  xgu_radar1_target01_a_radar1_target01_w_obstacle_decode(r1_target01_a_obj.radar1_target01_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar1_target01_a_radar1_target01_w_obstacle_is_in_range(
                  r1_target01_a_obj.radar1_target01_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar1_target01_a_radar1_target01_flag_valid_decode(r1_target01_a_obj.radar1_target01_flag_valid);
              target_flag_valid_is_in_range = xgu_radar1_target01_a_radar1_target01_flag_valid_is_in_range(
                  r1_target01_a_obj.radar1_target01_flag_valid);
              target_w_non_obstacle_decode = xgu_radar1_target01_a_radar1_target01_w_non_obstacle_decode(
                  r1_target01_a_obj.radar1_target01_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar1_target01_a_radar1_target01_w_non_obstacle_is_in_range(
                  r1_target01_a_obj.radar1_target01_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar1_target01_a_radar1_target01_flag_meas_decode(r1_target01_a_obj.radar1_target01_flag_meas);
              target_flag_meas_is_in_range = xgu_radar1_target01_a_radar1_target01_flag_meas_is_in_range(
                  r1_target01_a_obj.radar1_target01_flag_meas);
              target_flag_hist_decode =
                  xgu_radar1_target01_a_radar1_target01_flag_hist_decode(r1_target01_a_obj.radar1_target01_flag_hist);
              target_flag_hist_is_in_range = xgu_radar1_target01_a_radar1_target01_flag_hist_is_in_range(
                  r1_target01_a_obj.radar1_target01_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar1_target01_a_radar1_target01_mess_aconsist_bit_decode(
                  r1_target01_a_obj.radar1_target01_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target01_a_radar1_target01_mess_aconsist_bit_is_in_range(
                      r1_target01_a_obj.radar1_target01_mess_aconsist_bit);
              break;
            case 1616:

              break;
            case 1617:
              break;
            case 1618:
              xgu_radar2_target01_b_t r2_target01_b_obj;
              unpack_return = xgu_radar2_target01_b_unpack(&r2_target01_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar2_target02_b_radar2_target02_vy_decode(r2_target01_b_obj.radar2_target01_vy);
              target_vy_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_vy_is_in_range(r2_target01_b_obj.radar2_target01_vy);
              target_d_length_decoded =
                  xgu_radar2_target02_b_radar2_target02_d_length_decode(r2_target01_b_obj.radar2_target01_d_length);
              target_d_length_is_in_range = xgu_radar2_target02_b_radar2_target02_d_length_is_in_range(
                  r2_target01_b_obj.radar2_target01_d_length);
              target_dz_decoded = xgu_radar2_target02_b_radar2_target02_dz_decode(r2_target01_b_obj.radar2_target01_dz);
              target_dz_is_in_range =
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
              unpack_return = xgu_radar1_target02_a_unpack(&r1_target02_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar1_target02_a_radar1_target02_dx_decode(r1_target02_a_obj.radar1_target02_dx);
              target_dx_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_dx_is_in_range(r1_target02_a_obj.radar1_target02_dx);
              target_vx_decode = xgu_radar1_target02_a_radar1_target02_vx_decode(r1_target02_a_obj.radar1_target02_vx);
              target_vx_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_vx_is_in_range(r1_target02_a_obj.radar1_target02_vx);
              target_dy_decode = xgu_radar1_target02_a_radar1_target02_dy_decode(r1_target02_a_obj.radar1_target02_dy);
              target_dy_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_dy_is_in_range(r1_target02_a_obj.radar1_target02_dy);
              target_w_exist_decode =
                  xgu_radar1_target02_a_radar1_target02_w_exist_decode(r1_target02_a_obj.radar1_target02_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_w_exist_is_in_range(r1_target02_a_obj.radar1_target02_w_exist);
              target_ax_decode = xgu_radar1_target02_a_radar1_target02_ax_decode(r1_target02_a_obj.radar1_target02_ax);
              target_ax_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_ax_is_in_range(r1_target02_a_obj.radar1_target02_ax);
              target_w_obstacle_decode =
                  xgu_radar1_target02_a_radar1_target02_w_obstacle_decode(r1_target02_a_obj.radar1_target02_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar1_target02_a_radar1_target02_w_obstacle_is_in_range(
                  r1_target02_a_obj.radar1_target02_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar1_target02_a_radar1_target02_flag_valid_decode(r1_target02_a_obj.radar1_target02_flag_valid);
              target_flag_valid_is_in_range = xgu_radar1_target02_a_radar1_target02_flag_valid_is_in_range(
                  r1_target02_a_obj.radar1_target02_flag_valid);
              target_w_non_obstacle_decode = xgu_radar1_target02_a_radar1_target02_w_non_obstacle_decode(
                  r1_target02_a_obj.radar1_target02_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar1_target02_a_radar1_target02_w_non_obstacle_is_in_range(
                  r1_target02_a_obj.radar1_target02_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar1_target02_a_radar1_target02_flag_meas_decode(r1_target02_a_obj.radar1_target02_flag_meas);
              target_flag_meas_is_in_range = xgu_radar1_target02_a_radar1_target02_flag_meas_is_in_range(
                  r1_target02_a_obj.radar1_target02_flag_meas);
              target_flag_hist_decode =
                  xgu_radar1_target02_a_radar1_target02_flag_hist_decode(r1_target02_a_obj.radar1_target02_flag_hist);
              target_flag_hist_is_in_range = xgu_radar1_target02_a_radar1_target02_flag_hist_is_in_range(
                  r1_target02_a_obj.radar1_target02_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar1_target02_a_radar1_target02_mess_aconsist_bit_decode(
                  r1_target02_a_obj.radar1_target02_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target02_a_radar1_target02_mess_aconsist_bit_is_in_range(
                      r1_target02_a_obj.radar1_target02_mess_aconsist_bit);
              break;
            case 1626:
              xgu_radar2_target02_b_t r2_target02_b_obj;
              unpack_return = xgu_radar2_target02_b_unpack(&r2_target02_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar2_target02_b_radar2_target02_vy_decode(r2_target02_b_obj.radar2_target02_vy);
              target_vy_is_in_range =
                  xgu_radar2_target02_b_radar2_target02_vy_is_in_range(r2_target02_b_obj.radar2_target02_vy);
              target_d_length_decoded =
                  xgu_radar2_target02_b_radar2_target02_d_length_decode(r2_target02_b_obj.radar2_target02_d_length);
              target_d_length_is_in_range = xgu_radar2_target02_b_radar2_target02_d_length_is_in_range(
                  r2_target02_b_obj.radar2_target02_d_length);
              target_dz_decoded = xgu_radar2_target02_b_radar2_target02_dz_decode(r2_target02_b_obj.radar2_target02_dz);
              target_dz_is_in_range =
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
            case 1627:
              break;
            case 1628:
              break;
            case 1635:
              xgu_radar1_target03_a_t r1_target03_a_obj;
              unpack_return = xgu_radar1_target03_a_unpack(&r1_target03_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar1_target03_a_radar1_target03_dx_decode(r1_target03_a_obj.radar1_target03_dx);
              target_dx_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_dx_is_in_range(r1_target03_a_obj.radar1_target03_dx);
              target_vx_decode = xgu_radar1_target03_a_radar1_target03_vx_decode(r1_target03_a_obj.radar1_target03_vx);
              target_vx_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_vx_is_in_range(r1_target03_a_obj.radar1_target03_vx);
              target_dy_decode = xgu_radar1_target03_a_radar1_target03_dy_decode(r1_target03_a_obj.radar1_target03_dy);
              target_dy_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_dy_is_in_range(r1_target03_a_obj.radar1_target03_dy);
              target_w_exist_decode =
                  xgu_radar1_target03_a_radar1_target03_w_exist_decode(r1_target03_a_obj.radar1_target03_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_w_exist_is_in_range(r1_target03_a_obj.radar1_target03_w_exist);
              target_ax_decode = xgu_radar1_target03_a_radar1_target03_ax_decode(r1_target03_a_obj.radar1_target03_ax);
              target_ax_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_ax_is_in_range(r1_target03_a_obj.radar1_target03_ax);
              target_w_obstacle_decode =
                  xgu_radar1_target03_a_radar1_target03_w_obstacle_decode(r1_target03_a_obj.radar1_target03_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar1_target03_a_radar1_target03_w_obstacle_is_in_range(
                  r1_target03_a_obj.radar1_target03_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar1_target03_a_radar1_target03_flag_valid_decode(r1_target03_a_obj.radar1_target03_flag_valid);
              target_flag_valid_is_in_range = xgu_radar1_target03_a_radar1_target03_flag_valid_is_in_range(
                  r1_target03_a_obj.radar1_target03_flag_valid);
              target_w_non_obstacle_decode = xgu_radar1_target03_a_radar1_target03_w_non_obstacle_decode(
                  r1_target03_a_obj.radar1_target03_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar1_target03_a_radar1_target03_w_non_obstacle_is_in_range(
                  r1_target03_a_obj.radar1_target03_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar1_target03_a_radar1_target03_flag_meas_decode(r1_target03_a_obj.radar1_target03_flag_meas);
              target_flag_meas_is_in_range = xgu_radar1_target03_a_radar1_target03_flag_meas_is_in_range(
                  r1_target03_a_obj.radar1_target03_flag_meas);
              target_flag_hist_decode =
                  xgu_radar1_target03_a_radar1_target03_flag_hist_decode(r1_target03_a_obj.radar1_target03_flag_hist);
              target_flag_hist_is_in_range = xgu_radar1_target03_a_radar1_target03_flag_hist_is_in_range(
                  r1_target03_a_obj.radar1_target03_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar1_target03_a_radar1_target03_mess_aconsist_bit_decode(
                  r1_target03_a_obj.radar1_target03_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target03_a_radar1_target03_mess_aconsist_bit_is_in_range(
                      r1_target03_a_obj.radar1_target03_mess_aconsist_bit);
              break;
            case 1636:
              xgu_radar2_target03_b_t r2_target03_b_obj;
              unpack_return = xgu_radar2_target03_b_unpack(&r2_target03_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar2_target03_b_radar2_target03_vy_decode(r2_target03_b_obj.radar2_target03_vy);
              target_vy_is_in_range =
                  xgu_radar2_target03_b_radar2_target03_vy_is_in_range(r2_target03_b_obj.radar2_target03_vy);
              target_d_length_decoded =
                  xgu_radar2_target03_b_radar2_target03_d_length_decode(r2_target03_b_obj.radar2_target03_d_length);
              target_d_length_is_in_range = xgu_radar2_target03_b_radar2_target03_d_length_is_in_range(
                  r2_target03_b_obj.radar2_target03_d_length);
              target_dz_decoded = xgu_radar2_target03_b_radar2_target03_dz_decode(r2_target03_b_obj.radar2_target03_dz);
              target_dz_is_in_range =
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
            case 1637:
              break;
            case 1638:
              break;
            case 1645:
              xgu_radar1_target04_a_t r1_target04_a_obj;
              unpack_return = xgu_radar1_target04_a_unpack(&r1_target04_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar1_target04_a_radar1_target04_dx_decode(r1_target04_a_obj.radar1_target04_dx);
              target_dx_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_dx_is_in_range(r1_target04_a_obj.radar1_target04_dx);
              target_vx_decode = xgu_radar1_target04_a_radar1_target04_vx_decode(r1_target04_a_obj.radar1_target04_vx);
              target_vx_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_vx_is_in_range(r1_target04_a_obj.radar1_target04_vx);
              target_dy_decode = xgu_radar1_target04_a_radar1_target04_dy_decode(r1_target04_a_obj.radar1_target04_dy);
              target_dy_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_dy_is_in_range(r1_target04_a_obj.radar1_target04_dy);
              target_w_exist_decode =
                  xgu_radar1_target04_a_radar1_target04_w_exist_decode(r1_target04_a_obj.radar1_target04_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_w_exist_is_in_range(r1_target04_a_obj.radar1_target04_w_exist);
              target_ax_decode = xgu_radar1_target04_a_radar1_target04_ax_decode(r1_target04_a_obj.radar1_target04_ax);
              target_ax_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_ax_is_in_range(r1_target04_a_obj.radar1_target04_ax);
              target_w_obstacle_decode =
                  xgu_radar1_target04_a_radar1_target04_w_obstacle_decode(r1_target04_a_obj.radar1_target04_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar1_target04_a_radar1_target04_w_obstacle_is_in_range(
                  r1_target04_a_obj.radar1_target04_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar1_target04_a_radar1_target04_flag_valid_decode(r1_target04_a_obj.radar1_target04_flag_valid);
              target_flag_valid_is_in_range = xgu_radar1_target04_a_radar1_target04_flag_valid_is_in_range(
                  r1_target04_a_obj.radar1_target04_flag_valid);
              target_w_non_obstacle_decode = xgu_radar1_target04_a_radar1_target04_w_non_obstacle_decode(
                  r1_target04_a_obj.radar1_target04_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar1_target04_a_radar1_target04_w_non_obstacle_is_in_range(
                  r1_target04_a_obj.radar1_target04_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar1_target04_a_radar1_target04_flag_meas_decode(r1_target04_a_obj.radar1_target04_flag_meas);
              target_flag_meas_is_in_range = xgu_radar1_target04_a_radar1_target04_flag_meas_is_in_range(
                  r1_target04_a_obj.radar1_target04_flag_meas);
              target_flag_hist_decode =
                  xgu_radar1_target04_a_radar1_target04_flag_hist_decode(r1_target04_a_obj.radar1_target04_flag_hist);
              target_flag_hist_is_in_range = xgu_radar1_target04_a_radar1_target04_flag_hist_is_in_range(
                  r1_target04_a_obj.radar1_target04_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar1_target04_a_radar1_target04_mess_aconsist_bit_decode(
                  r1_target04_a_obj.radar1_target04_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target04_a_radar1_target04_mess_aconsist_bit_is_in_range(
                      r1_target04_a_obj.radar1_target04_mess_aconsist_bit);
              break;
            case 1646:
              xgu_radar2_target04_b_t r2_target04_b_obj;
              unpack_return = xgu_radar2_target04_b_unpack(&r2_target04_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar2_target04_b_radar2_target04_vy_decode(r2_target04_b_obj.radar2_target04_vy);
              target_vy_is_in_range =
                  xgu_radar2_target04_b_radar2_target04_vy_is_in_range(r2_target04_b_obj.radar2_target04_vy);
              target_d_length_decoded =
                  xgu_radar2_target04_b_radar2_target04_d_length_decode(r2_target04_b_obj.radar2_target04_d_length);
              target_d_length_is_in_range = xgu_radar2_target04_b_radar2_target04_d_length_is_in_range(
                  r2_target04_b_obj.radar2_target04_d_length);
              target_dz_decoded = xgu_radar2_target04_b_radar2_target04_dz_decode(r2_target04_b_obj.radar2_target04_dz);
              target_dz_is_in_range =
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
            case 1647:
              break;
            case 1648:
              break;
            case 1655:
              xgu_radar1_target05_a_t r1_target05_a_obj;
              unpack_return = xgu_radar1_target05_a_unpack(&r1_target05_a_obj, can_data, size_of_msg);
              target_dx_decoded = xgu_radar1_target05_a_radar1_target05_dx_decode(r1_target05_a_obj.radar1_target05_dx);
              target_dx_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_dx_is_in_range(r1_target05_a_obj.radar1_target05_dx);
              target_vx_decode = xgu_radar1_target05_a_radar1_target05_vx_decode(r1_target05_a_obj.radar1_target05_vx);
              target_vx_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_vx_is_in_range(r1_target05_a_obj.radar1_target05_vx);
              target_dy_decode = xgu_radar1_target05_a_radar1_target05_dy_decode(r1_target05_a_obj.radar1_target05_dy);
              target_dy_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_dy_is_in_range(r1_target05_a_obj.radar1_target05_dy);
              target_w_exist_decode =
                  xgu_radar1_target05_a_radar1_target05_w_exist_decode(r1_target05_a_obj.radar1_target05_w_exist);
              target_w_exist_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_w_exist_is_in_range(r1_target05_a_obj.radar1_target05_w_exist);
              target_ax_decode = xgu_radar1_target05_a_radar1_target05_ax_decode(r1_target05_a_obj.radar1_target05_ax);
              target_ax_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_ax_is_in_range(r1_target05_a_obj.radar1_target05_ax);
              target_w_obstacle_decode =
                  xgu_radar1_target05_a_radar1_target05_w_obstacle_decode(r1_target05_a_obj.radar1_target05_w_obstacle);
              target_w_obstacle_is_in_range = xgu_radar1_target05_a_radar1_target05_w_obstacle_is_in_range(
                  r1_target05_a_obj.radar1_target05_w_obstacle);
              target_flag_valid_decode =
                  xgu_radar1_target05_a_radar1_target05_flag_valid_decode(r1_target05_a_obj.radar1_target05_flag_valid);
              target_flag_valid_is_in_range = xgu_radar1_target05_a_radar1_target05_flag_valid_is_in_range(
                  r1_target05_a_obj.radar1_target05_flag_valid);
              target_w_non_obstacle_decode = xgu_radar1_target05_a_radar1_target05_w_non_obstacle_decode(
                  r1_target05_a_obj.radar1_target05_w_non_obstacle);
              target_w_non_obstacle_is_in_range = xgu_radar1_target05_a_radar1_target05_w_non_obstacle_is_in_range(
                  r1_target05_a_obj.radar1_target05_w_non_obstacle);
              target_flag_meas_decode =
                  xgu_radar1_target05_a_radar1_target05_flag_meas_decode(r1_target05_a_obj.radar1_target05_flag_meas);
              target_flag_meas_is_in_range = xgu_radar1_target05_a_radar1_target05_flag_meas_is_in_range(
                  r1_target05_a_obj.radar1_target05_flag_meas);
              target_flag_hist_decode =
                  xgu_radar1_target05_a_radar1_target05_flag_hist_decode(r1_target05_a_obj.radar1_target05_flag_hist);
              target_flag_hist_is_in_range = xgu_radar1_target05_a_radar1_target05_flag_hist_is_in_range(
                  r1_target05_a_obj.radar1_target05_flag_hist);
              target_mess_aconsist_bit_decode = xgu_radar1_target05_a_radar1_target05_mess_aconsist_bit_decode(
                  r1_target05_a_obj.radar1_target05_mess_aconsist_bit);
              target_mess_aconsist_bit_is_in_range =
                  xgu_radar1_target05_a_radar1_target05_mess_aconsist_bit_is_in_range(
                      r1_target05_a_obj.radar1_target05_mess_aconsist_bit);
              break;
            case 1656:
              xgu_radar1_target05_b_t r1_target05_b_obj;
              unpack_return = xgu_radar1_target05_b_unpack(&r1_target05_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar1_target05_b_radar1_target05_vy_decode(r1_target05_b_obj.radar1_target05_vy);
              target_vy_is_in_range =
                  xgu_radar1_target05_b_radar1_target05_vy_is_in_range(r1_target05_b_obj.radar1_target05_vy);
              target_d_length_decoded =
                  xgu_radar1_target05_b_radar1_target05_d_length_decode(r1_target05_b_obj.radar1_target05_d_length);
              target_d_length_is_in_range = xgu_radar1_target05_b_radar1_target05_d_length_is_in_range(
                  r1_target05_b_obj.radar1_target05_d_length);
              target_dz_decoded = xgu_radar1_target05_b_radar1_target05_dz_decode(r1_target05_b_obj.radar1_target05_dz);
              target_dz_is_in_range =
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
              break;
            case 1658:
              xgu_radar2_target05_b_t r2_target05_b_obj;
              unpack_return = xgu_radar2_target05_b_unpack(&r2_target05_b_obj, can_data, size_of_msg);
              target_vy_decoded = xgu_radar2_target05_b_radar2_target05_vy_decode(r2_target05_b_obj.radar2_target05_vy);
              target_vy_is_in_range =
                  xgu_radar2_target05_b_radar2_target05_vy_is_in_range(r2_target05_b_obj.radar2_target05_vy);
              target_d_length_decoded =
                  xgu_radar2_target05_b_radar2_target05_d_length_decode(r2_target05_b_obj.radar2_target05_d_length);
              target_d_length_is_in_range = xgu_radar2_target05_b_radar2_target05_d_length_is_in_range(
                  r2_target05_b_obj.radar2_target05_d_length);
              target_dz_decoded = xgu_radar2_target05_b_radar2_target05_dz_decode(r2_target05_b_obj.radar2_target05_dz);
              target_dz_is_in_range =
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
          if (id == 1665) {
            xgu_radar1_object_ender_t r1_obj_ender_obj;
            unpack_return = xgu_radar1_object_ender_unpack(&r1_obj_ender_obj, can_data, size_of_msg);
            xgu_radar_timestamp_decoded =
                xgu_radar1_object_ender_radar1_timestamp_decode(r1_obj_ender_obj.radar1_timestamp);
            xgu_radar_timestamp_is_in_range =
                xgu_radar1_object_ender_radar1_timestamp_is_in_range(r1_obj_ender_obj.radar1_timestamp);
            tc_counter_decoded = xgu_radar1_object_ender_radar1_tc_counter_decode(r1_obj_ender_obj.radar1_tc_counter);
            tc_counter_is_in_range =
                xgu_radar1_object_ender_radar1_tc_counter_is_in_range(r1_obj_ender_obj.radar1_tc_counter);
            obj_ender_consist_bit_decoded = xgu_radar1_object_ender_radar1_mess_ender_consist_bit_decode(
                r1_obj_ender_obj.radar1_mess_ender_consist_bit);
            obj_ender_consist_bit_is_in_range = xgu_radar1_object_ender_radar1_mess_ender_consist_bit_is_in_range(
                r1_obj_ender_obj.radar1_mess_ender_consist_bit);
            packet_checksum_encoded =
                xgu_radar1_object_ender_radar1_packet_checksum_decode(r1_obj_ender_obj.radar1_packet_checksum);
            packet_checksum_is_in_range =
                xgu_radar1_object_ender_radar1_packet_checksum_is_in_range(r1_obj_ender_obj.radar1_packet_checksum);
          } else if (id == 1667) {
            xgu_radar2_object_ender_t r2_obj_ender_obj;
            unpack_return = xgu_radar2_object_ender_unpack(&r2_obj_ender_obj, can_data, size_of_msg);
            xgu_radar_timestamp_decoded =
                xgu_radar2_object_ender_radar2_object_timestamp_decode(r2_obj_ender_obj.radar2_object_timestamp);
            xgu_radar_timestamp_is_in_range =
                xgu_radar2_object_ender_radar2_object_timestamp_is_in_range(r2_obj_ender_obj.radar2_object_timestamp);
            tc_counter_decoded = xgu_radar2_object_ender_radar2_tc_counter_decode(r2_obj_ender_obj.radar2_tc_counter);
            tc_counter_is_in_range =
                xgu_radar2_object_ender_radar2_tc_counter_is_in_range(r2_obj_ender_obj.radar2_tc_counter);
            obj_ender_consist_bit_decoded = xgu_radar2_object_ender_radar2_mess_ender_consist_bit_decode(
                r2_obj_ender_obj.radar2_mess_ender_consist_bit);
            obj_ender_consist_bit_is_in_range = xgu_radar2_object_ender_radar2_mess_ender_consist_bit_is_in_range(
                r2_obj_ender_obj.radar2_mess_ender_consist_bit);
            packet_checksum_encoded =
                xgu_radar2_object_ender_radar2_packet_checksum_decode(r2_obj_ender_obj.radar2_packet_checksum);
            packet_checksum_is_in_range =
                xgu_radar2_object_ender_radar2_packet_checksum_is_in_range(r2_obj_ender_obj.radar2_packet_checksum);
          } else if (id == 1280) {
            xgu_radar1_object_starter_t r1_obj_starter_obj;
            unpack_return = xgu_radar1_object_starter_unpack(&r1_obj_starter_obj, can_data, size_of_msg);
            veh_psi_dt_decoded =
                xgu_radar1_object_starter_radar1_veh_psi_dt_decode(r1_obj_starter_obj.radar1_veh_psi_dt);
            veh_psi_dt_is_in_range =
                xgu_radar1_object_starter_radar1_veh_psi_dt_is_in_range(r1_obj_starter_obj.radar1_veh_psi_dt);
            veh_v_ego_decoded = xgu_radar1_object_starter_radar1_veh_v_ego_decode(r1_obj_starter_obj.radar1_veh_v_ego);
            veh_v_ego_is_in_range =
                xgu_radar1_object_starter_radar1_veh_v_ego_is_in_range(r1_obj_starter_obj.radar1_veh_v_ego);
            veh_a_ego_decoded = xgu_radar1_object_starter_radar1_veh_a_ego_decode(r1_obj_starter_obj.radar1_veh_a_ego);
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
          } else if (id == 1282) {
            xgu_radar2_object_starter_t r2_obj_starter_obj;
            unpack_return = xgu_radar2_object_starter_unpack(&r2_obj_starter_obj, can_data, size_of_msg);
            veh_psi_dt_decoded =
                xgu_radar2_object_starter_radar2_veh_psi_dt_decode(r2_obj_starter_obj.radar2_veh_psi_dt);
            veh_psi_dt_is_in_range =
                xgu_radar2_object_starter_radar2_veh_psi_dt_is_in_range(r2_obj_starter_obj.radar2_veh_psi_dt);
            veh_v_ego_decoded = xgu_radar2_object_starter_radar2_veh_v_ego_decode(r2_obj_starter_obj.radar2_veh_v_ego);
            veh_v_ego_is_in_range =
                xgu_radar2_object_starter_radar2_veh_v_ego_is_in_range(r2_obj_starter_obj.radar2_veh_v_ego);
            veh_a_ego_decoded = xgu_radar2_object_starter_radar2_veh_a_ego_decode(r2_obj_starter_obj.radar2_veh_a_ego);
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
          } else if (id == 1670) {
            xgu_radar1_status_t r1_status;
            unpack_return = xgu_radar1_status_unpack(&r1_status, can_data, size_of_msg);
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
          } else if (id == 1672) {
            xgu_radar2_status_t r2_status;
            unpack_return = xgu_radar2_status_unpack(&r2_status, can_data, size_of_msg);
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
          }
          break;

        case 4:

          break;

        case 5:
          break;
          // case 0:
          break;
      }
    }
  }
  /*
        stat = canRead(hnd_2, &id, &can_data, &dlc, &flag, &time);

        if (canOK == stat) {
          // left corner radar = radar_1 and right corner radar = radar_2
        }

        stat = canRead(hnd_3, &id, &can_data, &dlc, &flag, &time);

        if (canOK == stat) {
          // mobileye
        }
      */
  // sensor_diagnostic_pub.publish(diag_msg);
  // raw_sensor_data_pub.publish(raw_object_msg);
  // drive_control_input_pub.publish(drive_ctrl_msg);
  // ROS_INFO_STREAM("\n" << data << "\n");

  canBusOff(hnd_0);
  canBusOff(hnd_1);
  canBusOff(hnd_2);
  canBusOff(hnd_3);

  canClose(hnd_0);
  canClose(hnd_1);
  canClose(hnd_2);
  canClose(hnd_3);

  Can_Tx_Rx_Class.publish_drive_ctrl_input();
  Can_Tx_Rx_Class.publish_raw_sensor_object();
  Can_Tx_Rx_Class.publish_sensor_diag();
  ros::spinOnce();
  }
  return 0;
}