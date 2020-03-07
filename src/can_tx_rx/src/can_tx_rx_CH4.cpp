#include <canlib.h>

#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/bosch_xgu_corner_radar.c"
#include "can_tx_rx/bosch_xgu_corner_radar.h"

#include "can_tx_rx/raw_sensor_object_data_msg.h"
#include "can_tx_rx/sensor_diagnostic_data_msg.h"

#include "can_tx_rx/output_structs.h"

static const uint16_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  ros::Publisher diag_data_pub = can_tx_rx_CH4_handle.advertise<can_tx_rx::sensor_diagnostic_data_msg>(
      "sensor_diagnostic_data", TX_RX_MESSAGE_BUFFER_SIZE);

  ros::Publisher raw_obj_data_pub = can_tx_rx_CH4_handle.advertise<can_tx_rx::raw_sensor_object_data_msg>(
      "raw_sensor_object_data", TX_RX_MESSAGE_BUFFER_SIZE);

  can_tx_rx::sensor_diagnostic_data_msg diag_data_msg;
  can_tx_rx::raw_sensor_object_data_msg raw_obj_data_msg;

  canHandle hnd;

  canInitializeLibrary();

  hnd = canOpenChannel(2, canOPEN_EXCLUSIVE);

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

  int unpack_return = -1;  // 0 is successful, negative error code

  uint8_t serialized_radar_diag_response[sizeof(diag_response)];
  uint8_t serialized_radar_info[sizeof(radar_info)];
  uint8_t serialized_target_info[sizeof(target_info)];
  uint8_t serialized_all_object_info[sizeof(all_object_info)];

  diag_response.channel_number = 3;
  radar_info.channel_number = 3;
  target_info.channel_number = 3;
  all_object_info.channel_number = 3;

  while (ros::ok()) {
    canStatus stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);

    if (canOK == stat) {
      // Left corner radar = radar_1 and right corner radar = radar_2
      get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num);
      switch (case_num) {
        case 1:
          switch (id) {
            case 1985:
              bosch_xgu_corner_radar_radar1_diag_response_t r1_diag_response_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_diag_response_unpack(&r1_diag_response_obj, can_data, size_of_msg);
              diag_response.diagnostic_decoded = bosch_xgu_corner_radar_radar1_diag_response_r1_diag_response_decode(
                  r1_diag_response_obj.r1_diag_response);
              diag_response.diagnostic_is_in_range =
                  bosch_xgu_corner_radar_radar1_diag_response_r1_diag_response_is_in_range(
                      r1_diag_response_obj.r1_diag_response);
              break;
          }
          diag_response.timestamp = time;
          diag_response.radar_number = radar_num;
          memcpy(serialized_radar_diag_response, &diag_response, sizeof(diag_response));
          (diag_data_msg.radar_diag_input)
              .insert((diag_data_msg.radar_diag_input).begin(), std::begin(serialized_radar_diag_response),
                      std::end(serialized_radar_diag_response));
          diag_data_pub.publish(diag_data_msg);
          break;
        case 2:
          switch (id) {
            case 1605:
              bosch_xgu_corner_radar_radar1_target00_a_t r1_target00_a_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target00_a_unpack(&r1_target00_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_dx_decode(
                  r1_target00_a_obj.radar1_target00_dx);
              target_info.target_dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_dx_is_in_range(
                      r1_target00_a_obj.radar1_target00_dx);
              target_info.target_vx_decode = bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_vx_decode(
                  r1_target00_a_obj.radar1_target00_vx);
              target_info.target_vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_vx_is_in_range(
                      r1_target00_a_obj.radar1_target00_vx);
              target_info.target_dy_decode = bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_dy_decode(
                  r1_target00_a_obj.radar1_target00_dy);
              target_info.target_dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_dy_is_in_range(
                      r1_target00_a_obj.radar1_target00_dy);
              target_info.target_flag_hist_decode =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_flag_hist_decode(
                      r1_target00_a_obj.radar1_target00_flag_hist);
              target_info.target_flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_flag_hist_is_in_range(
                      r1_target00_a_obj.radar1_target00_flag_hist);
              target_info.target_mess_aconsist_bit_decode =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_mess_aconsist_bit_decode(
                      r1_target00_a_obj.radar1_target00_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target00_a_radar1_target00_mess_aconsist_bit_is_in_range(
                      r1_target00_a_obj.radar1_target00_mess_aconsist_bit);
              break;
            case 1606:
              bosch_xgu_corner_radar_radar1_target00_b_t r1_target00_b_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target00_b_unpack(&r1_target00_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vy_decode(
                  r1_target00_b_obj.radar1_target00_vy);
              target_info.target_vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vy_is_in_range(
                      r1_target00_b_obj.radar1_target00_vy);
              target_info.target_d_length_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_d_length_decode(
                      r1_target00_b_obj.radar1_target00_d_length);
              target_info.target_d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_d_length_is_in_range(
                      r1_target00_b_obj.radar1_target00_d_length);
              target_info.target_dz_decoded = bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dz_decode(
                  r1_target00_b_obj.radar1_target00_dz);
              target_info.target_dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dz_is_in_range(
                      r1_target00_b_obj.radar1_target00_dz);
              target_info.target_moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_moving_state_decode(
                      r1_target00_b_obj.radar1_target00_moving_state);
              target_info.target_moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_moving_state_is_in_range(
                      r1_target00_b_obj.radar1_target00_moving_state);
              target_info.target_dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dx_sigma_decode(
                      r1_target00_b_obj.radar1_target00_dx_sigma);
              target_info.target_dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dx_sigma_is_in_range(
                      r1_target00_b_obj.radar1_target00_dx_sigma);
              target_info.target_vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vx_sigma_decode(
                      r1_target00_b_obj.radar1_target00_vx_sigma);
              target_info.target_vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vx_sigma_is_in_range(
                      r1_target00_b_obj.radar1_target00_vx_sigma);
              target_info.target_dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dy_sigma_decode(
                      r1_target00_b_obj.radar1_target00_dy_sigma);
              target_info.target_dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dy_sigma_is_in_range(
                      r1_target00_b_obj.radar1_target00_dy_sigma);
              target_info.target_w_class_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_w_class_decode(
                      r1_target00_b_obj.radar1_target00_w_class);
              target_info.target_w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_w_class_is_in_range(
                      r1_target00_b_obj.radar1_target00_w_class);
              target_info.target_class_decoded = bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_class_decode(
                  r1_target00_b_obj.radar1_target00_class);
              target_info.target_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_class_is_in_range(
                      r1_target00_b_obj.radar1_target00_class);
              target_info.target_mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_mess_bconsist_bit_decode(
                      r1_target00_b_obj.radar1_target00_mess_bconsist_bit);
              target_info.target_mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_mess_bconsist_bit_is_in_range(
                      r1_target00_b_obj.radar1_target00_mess_bconsist_bit);
              break;
            
            case 1615:
              bosch_xgu_corner_radar_radar1_target01_a_t r1_target01_a_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target01_a_unpack(&r1_target01_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_dx_decode(
                  r1_target01_a_obj.radar1_target01_dx);
              target_info.target_dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_dx_is_in_range(
                      r1_target01_a_obj.radar1_target01_dx);
              target_info.target_vx_decode = bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_vx_decode(
                  r1_target01_a_obj.radar1_target01_vx);
              target_info.target_vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_vx_is_in_range(
                      r1_target01_a_obj.radar1_target01_vx);
              target_info.target_dy_decode = bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_dy_decode(
                  r1_target01_a_obj.radar1_target01_dy);
              target_info.target_dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_dy_is_in_range(
                      r1_target01_a_obj.radar1_target01_dy);
              target_info.target_flag_hist_decode =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_flag_hist_decode(
                      r1_target01_a_obj.radar1_target01_flag_hist);
              target_info.target_flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_flag_hist_is_in_range(
                      r1_target01_a_obj.radar1_target01_flag_hist);
              target_info.target_mess_aconsist_bit_decode =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_mess_aconsist_bit_decode(
                      r1_target01_a_obj.radar1_target01_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_a_radar1_target01_mess_aconsist_bit_is_in_range(
                      r1_target01_a_obj.radar1_target01_mess_aconsist_bit);
              break;
            case 1616:
              bosch_xgu_corner_radar_radar1_target01_b_t r1_target01_b_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target01_b_unpack(&r1_target01_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vy_decode(
                  r1_target01_b_obj.radar1_target01_vy);
              target_info.target_vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vy_is_in_range(
                      r1_target01_b_obj.radar1_target01_vy);
              target_info.target_d_length_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_d_length_decode(
                      r1_target01_b_obj.radar1_target01_d_length);
              target_info.target_d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_d_length_is_in_range(
                      r1_target01_b_obj.radar1_target01_d_length);
              target_info.target_dz_decoded = bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dz_decode(
                  r1_target01_b_obj.radar1_target01_dz);
              target_info.target_dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dz_is_in_range(
                      r1_target01_b_obj.radar1_target01_dz);
              target_info.target_moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_moving_state_decode(
                      r1_target01_b_obj.radar1_target01_moving_state);
              target_info.target_moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_moving_state_is_in_range(
                      r1_target01_b_obj.radar1_target01_moving_state);
              target_info.target_dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dx_sigma_decode(
                      r1_target01_b_obj.radar1_target01_dx_sigma);
              target_info.target_dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dx_sigma_is_in_range(
                      r1_target01_b_obj.radar1_target01_dx_sigma);
              target_info.target_vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vx_sigma_decode(
                      r1_target01_b_obj.radar1_target01_vx_sigma);
              target_info.target_vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_vx_sigma_is_in_range(
                      r1_target01_b_obj.radar1_target01_vx_sigma);
              target_info.target_dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dy_sigma_decode(
                      r1_target01_b_obj.radar1_target01_dy_sigma);
              target_info.target_dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_dy_sigma_is_in_range(
                      r1_target01_b_obj.radar1_target01_dy_sigma);
              target_info.target_w_class_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_w_class_decode(
                      r1_target01_b_obj.radar1_target01_w_class);
              target_info.target_w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_w_class_is_in_range(
                      r1_target01_b_obj.radar1_target01_w_class);
              target_info.target_class_decoded = bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_class_decode(
                  r1_target01_b_obj.radar1_target01_class);
              target_info.target_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_class_is_in_range(
                      r1_target01_b_obj.radar1_target01_class);
              target_info.target_mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_mess_bconsist_bit_decode(
                      r1_target01_b_obj.radar1_target01_mess_bconsist_bit);
              target_info.target_mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target01_b_radar1_target01_mess_bconsist_bit_is_in_range(
                      r1_target01_b_obj.radar1_target01_mess_bconsist_bit);
              break;
            
            case 1625:
              bosch_xgu_corner_radar_radar1_target02_a_t r1_target02_a_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target02_a_unpack(&r1_target02_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_dx_decode(
                  r1_target02_a_obj.radar1_target02_dx);
              target_info.target_dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_dx_is_in_range(
                      r1_target02_a_obj.radar1_target02_dx);
              target_info.target_vx_decode = bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_vx_decode(
                  r1_target02_a_obj.radar1_target02_vx);
              target_info.target_vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_vx_is_in_range(
                      r1_target02_a_obj.radar1_target02_vx);
              target_info.target_dy_decode = bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_dy_decode(
                  r1_target02_a_obj.radar1_target02_dy);
              target_info.target_dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_dy_is_in_range(
                      r1_target02_a_obj.radar1_target02_dy);
              target_info.target_flag_hist_decode =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_flag_hist_decode(
                      r1_target02_a_obj.radar1_target02_flag_hist);
              target_info.target_flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_flag_hist_is_in_range(
                      r1_target02_a_obj.radar1_target02_flag_hist);
              target_info.target_mess_aconsist_bit_decode =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_mess_aconsist_bit_decode(
                      r1_target02_a_obj.radar1_target02_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_a_radar1_target02_mess_aconsist_bit_is_in_range(
                      r1_target02_a_obj.radar1_target02_mess_aconsist_bit);
              break;
            case 1626:
              bosch_xgu_corner_radar_radar1_target02_b_t r1_target02_b_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target02_b_unpack(&r1_target02_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_vy_decode(
                  r1_target02_b_obj.radar1_target02_vy);
              target_info.target_vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_vy_is_in_range(
                      r1_target02_b_obj.radar1_target02_vy);
              target_info.target_d_length_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_d_length_decode(
                      r1_target02_b_obj.radar1_target02_d_length);
              target_info.target_d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_d_length_is_in_range(
                      r1_target02_b_obj.radar1_target02_d_length);
              target_info.target_dz_decoded = bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_dz_decode(
                  r1_target02_b_obj.radar1_target02_dz);
              target_info.target_dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_dz_is_in_range(
                      r1_target02_b_obj.radar1_target02_dz);
              target_info.target_moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_moving_state_decode(
                      r1_target02_b_obj.radar1_target02_moving_state);
              target_info.target_moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_moving_state_is_in_range(
                      r1_target02_b_obj.radar1_target02_moving_state);
              target_info.target_dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_dx_sigma_decode(
                      r1_target02_b_obj.radar1_target02_dx_sigma);
              target_info.target_dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_dx_sigma_is_in_range(
                      r1_target02_b_obj.radar1_target02_dx_sigma);
              target_info.target_vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_vx_sigma_decode(
                      r1_target02_b_obj.radar1_target02_vx_sigma);
              target_info.target_vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_vx_sigma_is_in_range(
                      r1_target02_b_obj.radar1_target02_vx_sigma);
              target_info.target_dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_dy_sigma_decode(
                      r1_target02_b_obj.radar1_target02_dy_sigma);
              target_info.target_dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_dy_sigma_is_in_range(
                      r1_target02_b_obj.radar1_target02_dy_sigma);
              target_info.target_w_class_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_w_class_decode(
                      r1_target02_b_obj.radar1_target02_w_class);
              target_info.target_w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_w_class_is_in_range(
                      r1_target02_b_obj.radar1_target02_w_class);
              target_info.target_class_decoded = bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_class_decode(
                  r1_target02_b_obj.radar1_target02_class);
              target_info.target_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_class_is_in_range(
                      r1_target02_b_obj.radar1_target02_class);
              target_info.target_mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_mess_bconsist_bit_decode(
                      r1_target02_b_obj.radar1_target02_mess_bconsist_bit);
              target_info.target_mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target02_b_radar1_target02_mess_bconsist_bit_is_in_range(
                      r1_target02_b_obj.radar1_target02_mess_bconsist_bit);
              break;
           
            case 1635:
              bosch_xgu_corner_radar_radar1_target03_a_t r1_target03_a_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target03_a_unpack(&r1_target03_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_dx_decode(
                  r1_target03_a_obj.radar1_target03_dx);
              target_info.target_dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_dx_is_in_range(
                      r1_target03_a_obj.radar1_target03_dx);
              target_info.target_vx_decode = bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_vx_decode(
                  r1_target03_a_obj.radar1_target03_vx);
              target_info.target_vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_vx_is_in_range(
                      r1_target03_a_obj.radar1_target03_vx);
              target_info.target_dy_decode = bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_dy_decode(
                  r1_target03_a_obj.radar1_target03_dy);
              target_info.target_dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_dy_is_in_range(
                      r1_target03_a_obj.radar1_target03_dy);
              target_info.target_flag_hist_decode =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_flag_hist_decode(
                      r1_target03_a_obj.radar1_target03_flag_hist);
              target_info.target_flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_flag_hist_is_in_range(
                      r1_target03_a_obj.radar1_target03_flag_hist);
              target_info.target_mess_aconsist_bit_decode =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_mess_aconsist_bit_decode(
                      r1_target03_a_obj.radar1_target03_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_a_radar1_target03_mess_aconsist_bit_is_in_range(
                      r1_target03_a_obj.radar1_target03_mess_aconsist_bit);
              break;
            case 1636:
              bosch_xgu_corner_radar_radar1_target03_b_t r1_target03_b_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target03_b_unpack(&r1_target03_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_vy_decode(
                  r1_target03_b_obj.radar1_target03_vy);
              target_info.target_vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_vy_is_in_range(
                      r1_target03_b_obj.radar1_target03_vy);
              target_info.target_d_length_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_d_length_decode(
                      r1_target03_b_obj.radar1_target03_d_length);
              target_info.target_d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_d_length_is_in_range(
                      r1_target03_b_obj.radar1_target03_d_length);
              target_info.target_dz_decoded = bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_dz_decode(
                  r1_target03_b_obj.radar1_target03_dz);
              target_info.target_dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_dz_is_in_range(
                      r1_target03_b_obj.radar1_target03_dz);
              target_info.target_moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_moving_state_decode(
                      r1_target03_b_obj.radar1_target03_moving_state);
              target_info.target_moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_moving_state_is_in_range(
                      r1_target03_b_obj.radar1_target03_moving_state);
              target_info.target_dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_dx_sigma_decode(
                      r1_target03_b_obj.radar1_target03_dx_sigma);
              target_info.target_dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_dx_sigma_is_in_range(
                      r1_target03_b_obj.radar1_target03_dx_sigma);
              target_info.target_vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_vx_sigma_decode(
                      r1_target03_b_obj.radar1_target03_vx_sigma);
              target_info.target_vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_vx_sigma_is_in_range(
                      r1_target03_b_obj.radar1_target03_vx_sigma);
              target_info.target_dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_dy_sigma_decode(
                      r1_target03_b_obj.radar1_target03_dy_sigma);
              target_info.target_dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_dy_sigma_is_in_range(
                      r1_target03_b_obj.radar1_target03_dy_sigma);
              target_info.target_w_class_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_w_class_decode(
                      r1_target03_b_obj.radar1_target03_w_class);
              target_info.target_w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_w_class_is_in_range(
                      r1_target03_b_obj.radar1_target03_w_class);
              target_info.target_class_decoded = bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_class_decode(
                  r1_target03_b_obj.radar1_target03_class);
              target_info.target_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_class_is_in_range(
                      r1_target03_b_obj.radar1_target03_class);
              target_info.target_mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_mess_bconsist_bit_decode(
                      r1_target03_b_obj.radar1_target03_mess_bconsist_bit);
              target_info.target_mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target03_b_radar1_target03_mess_bconsist_bit_is_in_range(
                      r1_target03_b_obj.radar1_target03_mess_bconsist_bit);
              break;
           
            case 1645:
              bosch_xgu_corner_radar_radar1_target04_a_t r1_target04_a_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target04_a_unpack(&r1_target04_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_dx_decode(
                  r1_target04_a_obj.radar1_target04_dx);
              target_info.target_dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_dx_is_in_range(
                      r1_target04_a_obj.radar1_target04_dx);
              target_info.target_vx_decode = bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_vx_decode(
                  r1_target04_a_obj.radar1_target04_vx);
              target_info.target_vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_vx_is_in_range(
                      r1_target04_a_obj.radar1_target04_vx);
              target_info.target_dy_decode = bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_dy_decode(
                  r1_target04_a_obj.radar1_target04_dy);
              target_info.target_dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_dy_is_in_range(
                      r1_target04_a_obj.radar1_target04_dy);
              target_info.target_flag_hist_decode =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_flag_hist_decode(
                      r1_target04_a_obj.radar1_target04_flag_hist);
              target_info.target_flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_flag_hist_is_in_range(
                      r1_target04_a_obj.radar1_target04_flag_hist);
              target_info.target_mess_aconsist_bit_decode =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_mess_aconsist_bit_decode(
                      r1_target04_a_obj.radar1_target04_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_a_radar1_target04_mess_aconsist_bit_is_in_range(
                      r1_target04_a_obj.radar1_target04_mess_aconsist_bit);
              break;
            case 1646:
              bosch_xgu_corner_radar_radar1_target04_b_t r1_target04_b_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target04_b_unpack(&r1_target04_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_vy_decode(
                  r1_target04_b_obj.radar1_target04_vy);
              target_info.target_vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_vy_is_in_range(
                      r1_target04_b_obj.radar1_target04_vy);
              target_info.target_d_length_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_d_length_decode(
                      r1_target04_b_obj.radar1_target04_d_length);
              target_info.target_d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_d_length_is_in_range(
                      r1_target04_b_obj.radar1_target04_d_length);
              target_info.target_dz_decoded = bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_dz_decode(
                  r1_target04_b_obj.radar1_target04_dz);
              target_info.target_dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_dz_is_in_range(
                      r1_target04_b_obj.radar1_target04_dz);
              target_info.target_moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_moving_state_decode(
                      r1_target04_b_obj.radar1_target04_moving_state);
              target_info.target_moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_moving_state_is_in_range(
                      r1_target04_b_obj.radar1_target04_moving_state);
              target_info.target_dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_dx_sigma_decode(
                      r1_target04_b_obj.radar1_target04_dx_sigma);
              target_info.target_dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_dx_sigma_is_in_range(
                      r1_target04_b_obj.radar1_target04_dx_sigma);
              target_info.target_vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_vx_sigma_decode(
                      r1_target04_b_obj.radar1_target04_vx_sigma);
              target_info.target_vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_vx_sigma_is_in_range(
                      r1_target04_b_obj.radar1_target04_vx_sigma);
              target_info.target_dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_dy_sigma_decode(
                      r1_target04_b_obj.radar1_target04_dy_sigma);
              target_info.target_dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_dy_sigma_is_in_range(
                      r1_target04_b_obj.radar1_target04_dy_sigma);
              target_info.target_w_class_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_w_class_decode(
                      r1_target04_b_obj.radar1_target04_w_class);
              target_info.target_w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_w_class_is_in_range(
                      r1_target04_b_obj.radar1_target04_w_class);
              target_info.target_class_decoded = bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_class_decode(
                  r1_target04_b_obj.radar1_target04_class);
              target_info.target_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_class_is_in_range(
                      r1_target04_b_obj.radar1_target04_class);
              target_info.target_mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_mess_bconsist_bit_decode(
                      r1_target04_b_obj.radar1_target04_mess_bconsist_bit);
              target_info.target_mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target04_b_radar1_target04_mess_bconsist_bit_is_in_range(
                      r1_target04_b_obj.radar1_target04_mess_bconsist_bit);
              break;
            
            case 1655:
              bosch_xgu_corner_radar_radar1_target05_a_t r1_target05_a_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target05_a_unpack(&r1_target05_a_obj, can_data, size_of_msg);
              target_info.target_dx_decoded = bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_dx_decode(
                  r1_target05_a_obj.radar1_target05_dx);
              target_info.target_dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_dx_is_in_range(
                      r1_target05_a_obj.radar1_target05_dx);
              target_info.target_vx_decode = bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_vx_decode(
                  r1_target05_a_obj.radar1_target05_vx);
              target_info.target_vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_vx_is_in_range(
                      r1_target05_a_obj.radar1_target05_vx);
              target_info.target_dy_decode = bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_dy_decode(
                  r1_target05_a_obj.radar1_target05_dy);
              target_info.target_dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_dy_is_in_range(
                      r1_target05_a_obj.radar1_target05_dy);
              target_info.target_flag_hist_decode =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_flag_hist_decode(
                      r1_target05_a_obj.radar1_target05_flag_hist);
              target_info.target_flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_flag_hist_is_in_range(
                      r1_target05_a_obj.radar1_target05_flag_hist);
              target_info.target_mess_aconsist_bit_decode =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_mess_aconsist_bit_decode(
                      r1_target05_a_obj.radar1_target05_mess_aconsist_bit);
              target_info.target_mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_a_radar1_target05_mess_aconsist_bit_is_in_range(
                      r1_target05_a_obj.radar1_target05_mess_aconsist_bit);
              break;
            case 1656:
              bosch_xgu_corner_radar_radar1_target05_b_t r1_target05_b_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_target05_b_unpack(&r1_target05_b_obj, can_data, size_of_msg);
              target_info.target_vy_decoded = bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_vy_decode(
                  r1_target05_b_obj.radar1_target05_vy);
              target_info.target_vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_vy_is_in_range(
                      r1_target05_b_obj.radar1_target05_vy);
              target_info.target_d_length_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_d_length_decode(
                      r1_target05_b_obj.radar1_target05_d_length);
              target_info.target_d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_d_length_is_in_range(
                      r1_target05_b_obj.radar1_target05_d_length);
              target_info.target_dz_decoded = bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_dz_decode(
                  r1_target05_b_obj.radar1_target05_dz);
              target_info.target_dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_dz_is_in_range(
                      r1_target05_b_obj.radar1_target05_dz);
              target_info.target_moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_moving_state_decode(
                      r1_target05_b_obj.radar1_target05_moving_state);
              target_info.target_moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_moving_state_is_in_range(
                      r1_target05_b_obj.radar1_target05_moving_state);
              target_info.target_dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_dx_sigma_decode(
                      r1_target05_b_obj.radar1_target05_dx_sigma);
              target_info.target_dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_dx_sigma_is_in_range(
                      r1_target05_b_obj.radar1_target05_dx_sigma);
              target_info.target_vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_vx_sigma_decode(
                      r1_target05_b_obj.radar1_target05_vx_sigma);
              target_info.target_vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_vx_sigma_is_in_range(
                      r1_target05_b_obj.radar1_target05_vx_sigma);
              target_info.target_dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_dy_sigma_decode(
                      r1_target05_b_obj.radar1_target05_dy_sigma);
              target_info.target_dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_dy_sigma_is_in_range(
                      r1_target05_b_obj.radar1_target05_dy_sigma);
              target_info.target_w_class_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_w_class_decode(
                      r1_target05_b_obj.radar1_target05_w_class);
              target_info.target_w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_w_class_is_in_range(
                      r1_target05_b_obj.radar1_target05_w_class);
              target_info.target_class_decoded = bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_class_decode(
                  r1_target05_b_obj.radar1_target05_class);
              target_info.target_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_class_is_in_range(
                      r1_target05_b_obj.radar1_target05_class);
              target_info.target_mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_mess_bconsist_bit_decode(
                      r1_target05_b_obj.radar1_target05_mess_bconsist_bit);
              target_info.target_mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_target05_b_radar1_target05_mess_bconsist_bit_is_in_range(
                      r1_target05_b_obj.radar1_target05_mess_bconsist_bit);

              break; 
          }
          target_info.timestamp = time;
          target_info.radar_number = radar_num;
          target_info.target_object_number = target_object_num;
          memcpy(serialized_target_info, &diag_response, sizeof(target_info));
          (raw_obj_data_msg.target_info)
              .insert((raw_obj_data_msg.target_info).begin(), std::begin(serialized_target_info),
                      std::end(serialized_target_info));
          raw_obj_data_pub.publish(raw_obj_data_msg);
          break;
        case 3:
          switch (id) {
            case 1665:
              bosch_xgu_corner_radar_radar1_object_ender_t r1_obj_ender_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_object_ender_unpack(&r1_obj_ender_obj, can_data, size_of_msg);
              radar_info.radar_timestamp_decoded =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_timestamp_decode(r1_obj_ender_obj.radar1_timestamp);
              radar_info.radar_timestamp_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_timestamp_is_in_range(
                      r1_obj_ender_obj.radar1_timestamp);
              radar_info.tc_counter_decoded = bosch_xgu_corner_radar_radar1_object_ender_radar1_tc_counter_decode(
                  r1_obj_ender_obj.radar1_tc_counter);
              radar_info.tc_counter_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_tc_counter_is_in_range(
                      r1_obj_ender_obj.radar1_tc_counter);
              radar_info.obj_ender_consist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_mess_ender_consist_bit_decode(
                      r1_obj_ender_obj.radar1_mess_ender_consist_bit);
              radar_info.obj_ender_consist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_mess_ender_consist_bit_is_in_range(
                      r1_obj_ender_obj.radar1_mess_ender_consist_bit);
              radar_info.packet_checksum_encoded =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_packet_checksum_decode(
                      r1_obj_ender_obj.radar1_packet_checksum);
              radar_info.packet_checksum_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_ender_radar1_packet_checksum_is_in_range(
                      r1_obj_ender_obj.radar1_packet_checksum);
              break;
            
            case 1280:
              bosch_xgu_corner_radar_radar1_object_starter_t r1_obj_starter_obj;
              unpack_return =
                  bosch_xgu_corner_radar_radar1_object_starter_unpack(&r1_obj_starter_obj, can_data, size_of_msg);
              radar_info.veh_psi_dt_decoded = bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_psi_dt_decode(
                  r1_obj_starter_obj.radar1_veh_psi_dt);
              radar_info.veh_psi_dt_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_psi_dt_is_in_range(
                      r1_obj_starter_obj.radar1_veh_psi_dt);
              radar_info.veh_v_ego_decoded = bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_v_ego_decode(
                  r1_obj_starter_obj.radar1_veh_v_ego);
              radar_info.veh_v_ego_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_v_ego_is_in_range(
                      r1_obj_starter_obj.radar1_veh_v_ego);
              radar_info.veh_a_ego_decoded = bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_a_ego_decode(
                  r1_obj_starter_obj.radar1_veh_a_ego);
              radar_info.veh_a_ego_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_a_ego_is_in_range(
                      r1_obj_starter_obj.radar1_veh_a_ego);
              radar_info.veh_slip_angle_decoded =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_slip_angle_decode(
                      r1_obj_starter_obj.radar1_veh_slip_angle);
              radar_info.veh_slip_angle_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_veh_slip_angle_is_in_range(
                      r1_obj_starter_obj.radar1_veh_slip_angle);
              radar_info.mess_starter_consist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_mess_starter_consist_bit_decode(
                      r1_obj_starter_obj.radar1_mess_starter_consist_bit);
              radar_info.mess_starter_consist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_object_starter_radar1_mess_starter_consist_bit_is_in_range(
                      r1_obj_starter_obj.radar1_mess_starter_consist_bit);
              break;
            
            case 1670:
              bosch_xgu_corner_radar_radar1_status_t r1_status;
              unpack_return = bosch_xgu_corner_radar_radar1_status_unpack(&r1_status, can_data, size_of_msg);
              radar_info.itc_info_decoded =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_itc_info_decode(r1_status.r1_stat_itc_info);
              radar_info.itc_info_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_itc_info_is_in_range(r1_status.r1_stat_itc_info);
              radar_info.sgu_fail_decoded =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_sgu_fail_decode(r1_status.r1_stat_sgu_fail);
              radar_info.sgu_fail_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_sgu_fail_is_in_range(r1_status.r1_stat_sgu_fail);
              radar_info.hw_fail_decoded =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_hw_fail_decode(r1_status.r1_stat_hw_fail);
              radar_info.hw_fail_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_hw_fail_is_in_range(r1_status.r1_stat_hw_fail);
              radar_info.horizontal_misalignment_decoded =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_horizontal_misalignment_decode(
                      r1_status.r1_stat_horizontal_misalignment);
              radar_info.horizontal_misalignment_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_horizontal_misalignment_is_in_range(
                      r1_status.r1_stat_horizontal_misalignment);
              radar_info.absorption_blindness_decoded =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_absorption_blindness_decode(
                      r1_status.r1_stat_absorption_blindness);
              radar_info.absorption_blindness_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_absorption_blindness_is_in_range(
                      r1_status.r1_stat_absorption_blindness);
              radar_info.distortion_blindness_decoded =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_distortion_blindness_decode(
                      r1_status.r1_stat_distortion_blindness);
              radar_info.distortion_blindness_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_distortion_blindness_is_in_range(
                      r1_status.r1_stat_distortion_blindness);
              radar_info.mc_decoded = bosch_xgu_corner_radar_radar1_status_r1_stat_mc_decode(r1_status.r1_stat_mc);
              radar_info.mc_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_mc_is_in_range(r1_status.r1_stat_mc);
              radar_info.crc_decoded = bosch_xgu_corner_radar_radar1_status_r1_stat_crc_decode(r1_status.r1_stat_crc);
              radar_info.crc_is_in_range =
                  bosch_xgu_corner_radar_radar1_status_r1_stat_crc_is_in_range(r1_status.r1_stat_crc);
              break;
          }
          radar_info.timestamp = time;
          radar_info.radar_number = radar_num;
          memcpy(serialized_radar_info, &radar_info, sizeof(radar_info));
          (diag_data_msg.radar_info)
              .insert((diag_data_msg.radar_info).begin(), std::begin(serialized_radar_info),
                      std::end(serialized_radar_info));
          diag_data_pub.publish(diag_data_msg);
          break;
        case 4:
          switch (id) {
            case 1285:
              bosch_xgu_corner_radar_radar1_obj00_a_t r1_obj00_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj00_a_unpack(&r1_obj00_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_dx_decode(r1_obj00_a.radar1_obj00_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_dx_is_in_range(r1_obj00_a.radar1_obj00_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_vx_decode(r1_obj00_a.radar1_obj00_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_vx_is_in_range(r1_obj00_a.radar1_obj00_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_dy_decode(r1_obj00_a.radar1_obj00_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_dy_is_in_range(r1_obj00_a.radar1_obj00_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_flag_hist_decode(
                  r1_obj00_a.radar1_obj00_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_flag_hist_is_in_range(
                      r1_obj00_a.radar1_obj00_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_mess_aconsist_bit_decode(
                      r1_obj00_a.radar1_obj00_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_a_radar1_obj00_mess_aconsist_bit_is_in_range(
                      r1_obj00_a.radar1_obj00_mess_aconsist_bit);
              break;
            case 1295:
              bosch_xgu_corner_radar_radar1_obj01_a_t r1_obj01_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj01_a_unpack(&r1_obj01_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_dx_decode(r1_obj01_a.radar1_obj01_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_dx_is_in_range(r1_obj01_a.radar1_obj01_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_vx_decode(r1_obj01_a.radar1_obj01_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_vx_is_in_range(r1_obj01_a.radar1_obj01_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_dy_decode(r1_obj01_a.radar1_obj01_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_dy_is_in_range(r1_obj01_a.radar1_obj01_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_flag_hist_decode(
                  r1_obj01_a.radar1_obj01_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_flag_hist_is_in_range(
                      r1_obj01_a.radar1_obj01_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_mess_aconsist_bit_decode(
                      r1_obj01_a.radar1_obj01_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_a_radar1_obj01_mess_aconsist_bit_is_in_range(
                      r1_obj01_a.radar1_obj01_mess_aconsist_bit);
              break;
            case 1305:
              bosch_xgu_corner_radar_radar1_obj02_a_t r1_obj02_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj02_a_unpack(&r1_obj02_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_dx_decode(r1_obj02_a.radar1_obj02_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_dx_is_in_range(r1_obj02_a.radar1_obj02_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_vx_decode(r1_obj02_a.radar1_obj02_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_vx_is_in_range(r1_obj02_a.radar1_obj02_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_dy_decode(r1_obj02_a.radar1_obj02_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_dy_is_in_range(r1_obj02_a.radar1_obj02_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_flag_hist_decode(
                  r1_obj02_a.radar1_obj02_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_flag_hist_is_in_range(
                      r1_obj02_a.radar1_obj02_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_mess_aconsist_bit_decode(
                      r1_obj02_a.radar1_obj02_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_a_radar1_obj02_mess_aconsist_bit_is_in_range(
                      r1_obj02_a.radar1_obj02_mess_aconsist_bit);
              break;
            case 1315:
              bosch_xgu_corner_radar_radar1_obj03_a_t r1_obj03_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj03_a_unpack(&r1_obj03_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_dx_decode(r1_obj03_a.radar1_obj03_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_dx_is_in_range(r1_obj03_a.radar1_obj03_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_vx_decode(r1_obj03_a.radar1_obj03_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_vx_is_in_range(r1_obj03_a.radar1_obj03_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_dy_decode(r1_obj03_a.radar1_obj03_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_dy_is_in_range(r1_obj03_a.radar1_obj03_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_flag_hist_decode(
                  r1_obj03_a.radar1_obj03_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_flag_hist_is_in_range(
                      r1_obj03_a.radar1_obj03_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_mess_aconsist_bit_decode(
                      r1_obj03_a.radar1_obj03_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_a_radar1_obj03_mess_aconsist_bit_is_in_range(
                      r1_obj03_a.radar1_obj03_mess_aconsist_bit);
              break;
            case 1325:
              bosch_xgu_corner_radar_radar1_obj04_a_t r1_obj04_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj04_a_unpack(&r1_obj04_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_dx_decode(r1_obj04_a.radar1_obj04_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_dx_is_in_range(r1_obj04_a.radar1_obj04_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_vx_decode(r1_obj04_a.radar1_obj04_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_vx_is_in_range(r1_obj04_a.radar1_obj04_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_dy_decode(r1_obj04_a.radar1_obj04_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_dy_is_in_range(r1_obj04_a.radar1_obj04_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_flag_hist_decode(
                  r1_obj04_a.radar1_obj04_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_flag_hist_is_in_range(
                      r1_obj04_a.radar1_obj04_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_mess_aconsist_bit_decode(
                      r1_obj04_a.radar1_obj04_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_a_radar1_obj04_mess_aconsist_bit_is_in_range(
                      r1_obj04_a.radar1_obj04_mess_aconsist_bit);
              break;
            case 1335:
              bosch_xgu_corner_radar_radar1_obj05_a_t r1_obj05_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj05_a_unpack(&r1_obj05_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_dx_decode(r1_obj05_a.radar1_obj05_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_dx_is_in_range(r1_obj05_a.radar1_obj05_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_vx_decode(r1_obj05_a.radar1_obj05_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_vx_is_in_range(r1_obj05_a.radar1_obj05_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_dy_decode(r1_obj05_a.radar1_obj05_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_dy_is_in_range(r1_obj05_a.radar1_obj05_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_flag_hist_decode(
                  r1_obj05_a.radar1_obj05_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_flag_hist_is_in_range(
                      r1_obj05_a.radar1_obj05_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_mess_aconsist_bit_decode(
                      r1_obj05_a.radar1_obj05_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_a_radar1_obj05_mess_aconsist_bit_is_in_range(
                      r1_obj05_a.radar1_obj05_mess_aconsist_bit);
              break;
            case 1345:
              bosch_xgu_corner_radar_radar1_obj06_a_t r1_obj06_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj06_a_unpack(&r1_obj06_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_dx_decode(r1_obj06_a.radar1_obj06_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_dx_is_in_range(r1_obj06_a.radar1_obj06_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_vx_decode(r1_obj06_a.radar1_obj06_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_vx_is_in_range(r1_obj06_a.radar1_obj06_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_dy_decode(r1_obj06_a.radar1_obj06_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_dy_is_in_range(r1_obj06_a.radar1_obj06_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_flag_hist_decode(
                  r1_obj06_a.radar1_obj06_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_flag_hist_is_in_range(
                      r1_obj06_a.radar1_obj06_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_mess_aconsist_bit_decode(
                      r1_obj06_a.radar1_obj06_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_a_radar1_obj06_mess_aconsist_bit_is_in_range(
                      r1_obj06_a.radar1_obj06_mess_aconsist_bit);
              break;
            case 1355:
              bosch_xgu_corner_radar_radar1_obj07_a_t r1_obj07_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj07_a_unpack(&r1_obj07_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_dx_decode(r1_obj07_a.radar1_obj07_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_dx_is_in_range(r1_obj07_a.radar1_obj07_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_vx_decode(r1_obj07_a.radar1_obj07_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_vx_is_in_range(r1_obj07_a.radar1_obj07_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_dy_decode(r1_obj07_a.radar1_obj07_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_dy_is_in_range(r1_obj07_a.radar1_obj07_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_flag_hist_decode(
                  r1_obj07_a.radar1_obj07_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_flag_hist_is_in_range(
                      r1_obj07_a.radar1_obj07_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_mess_aconsist_bit_decode(
                      r1_obj07_a.radar1_obj07_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_a_radar1_obj07_mess_aconsist_bit_is_in_range(
                      r1_obj07_a.radar1_obj07_mess_aconsist_bit);
              break;
            case 1365:
              bosch_xgu_corner_radar_radar1_obj08_a_t r1_obj08_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj08_a_unpack(&r1_obj08_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_dx_decode(r1_obj08_a.radar1_obj08_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_dx_is_in_range(r1_obj08_a.radar1_obj08_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_vx_decode(r1_obj08_a.radar1_obj08_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_vx_is_in_range(r1_obj08_a.radar1_obj08_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_dy_decode(r1_obj08_a.radar1_obj08_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_dy_is_in_range(r1_obj08_a.radar1_obj08_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_flag_hist_decode(
                  r1_obj08_a.radar1_obj08_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_flag_hist_is_in_range(
                      r1_obj08_a.radar1_obj08_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_mess_aconsist_bit_decode(
                      r1_obj08_a.radar1_obj08_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_a_radar1_obj08_mess_aconsist_bit_is_in_range(
                      r1_obj08_a.radar1_obj08_mess_aconsist_bit);
              break;
            case 1375:
              bosch_xgu_corner_radar_radar1_obj09_a_t r1_obj09_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj09_a_unpack(&r1_obj09_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_dx_decode(r1_obj09_a.radar1_obj09_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_dx_is_in_range(r1_obj09_a.radar1_obj09_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_vx_decode(r1_obj09_a.radar1_obj09_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_vx_is_in_range(r1_obj09_a.radar1_obj09_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_dy_decode(r1_obj09_a.radar1_obj09_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_dy_is_in_range(r1_obj09_a.radar1_obj09_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_flag_hist_decode(
                  r1_obj09_a.radar1_obj09_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_flag_hist_is_in_range(
                      r1_obj09_a.radar1_obj09_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_mess_aconsist_bit_decode(
                      r1_obj09_a.radar1_obj09_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_a_radar1_obj09_mess_aconsist_bit_is_in_range(
                      r1_obj09_a.radar1_obj09_mess_aconsist_bit);
              break;
            case 1385:
              bosch_xgu_corner_radar_radar1_obj10_a_t r1_obj10_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj10_a_unpack(&r1_obj10_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_dx_decode(r1_obj10_a.radar1_obj10_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_dx_is_in_range(r1_obj10_a.radar1_obj10_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_vx_decode(r1_obj10_a.radar1_obj10_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_vx_is_in_range(r1_obj10_a.radar1_obj10_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_dy_decode(r1_obj10_a.radar1_obj10_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_dy_is_in_range(r1_obj10_a.radar1_obj10_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_flag_hist_decode(
                  r1_obj10_a.radar1_obj10_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_flag_hist_is_in_range(
                      r1_obj10_a.radar1_obj10_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_mess_aconsist_bit_decode(
                      r1_obj10_a.radar1_obj10_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_a_radar1_obj10_mess_aconsist_bit_is_in_range(
                      r1_obj10_a.radar1_obj10_mess_aconsist_bit);
              break;
            case 1395:
              bosch_xgu_corner_radar_radar1_obj11_a_t r1_obj11_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj11_a_unpack(&r1_obj11_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_dx_decode(r1_obj11_a.radar1_obj11_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_dx_is_in_range(r1_obj11_a.radar1_obj11_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_vx_decode(r1_obj11_a.radar1_obj11_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_vx_is_in_range(r1_obj11_a.radar1_obj11_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_dy_decode(r1_obj11_a.radar1_obj11_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_dy_is_in_range(r1_obj11_a.radar1_obj11_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_flag_hist_decode(
                  r1_obj11_a.radar1_obj11_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_flag_hist_is_in_range(
                      r1_obj11_a.radar1_obj11_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_mess_aconsist_bit_decode(
                      r1_obj11_a.radar1_obj11_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_a_radar1_obj11_mess_aconsist_bit_is_in_range(
                      r1_obj11_a.radar1_obj11_mess_aconsist_bit);
              break;
            case 1405:
              bosch_xgu_corner_radar_radar1_obj12_a_t r1_obj12_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj12_a_unpack(&r1_obj12_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_dx_decode(r1_obj12_a.radar1_obj12_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_dx_is_in_range(r1_obj12_a.radar1_obj12_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_vx_decode(r1_obj12_a.radar1_obj12_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_vx_is_in_range(r1_obj12_a.radar1_obj12_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_dy_decode(r1_obj12_a.radar1_obj12_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_dy_is_in_range(r1_obj12_a.radar1_obj12_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_flag_hist_decode(
                  r1_obj12_a.radar1_obj12_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_flag_hist_is_in_range(
                      r1_obj12_a.radar1_obj12_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_mess_aconsist_bit_decode(
                      r1_obj12_a.radar1_obj12_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_a_radar1_obj12_mess_aconsist_bit_is_in_range(
                      r1_obj12_a.radar1_obj12_mess_aconsist_bit);
              break;
            case 1415:
              bosch_xgu_corner_radar_radar1_obj13_a_t r1_obj13_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj13_a_unpack(&r1_obj13_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_dx_decode(r1_obj13_a.radar1_obj13_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_dx_is_in_range(r1_obj13_a.radar1_obj13_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_vx_decode(r1_obj13_a.radar1_obj13_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_vx_is_in_range(r1_obj13_a.radar1_obj13_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_dy_decode(r1_obj13_a.radar1_obj13_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_dy_is_in_range(r1_obj13_a.radar1_obj13_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_flag_hist_decode(
                  r1_obj13_a.radar1_obj13_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_flag_hist_is_in_range(
                      r1_obj13_a.radar1_obj13_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_mess_aconsist_bit_decode(
                      r1_obj13_a.radar1_obj13_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_a_radar1_obj13_mess_aconsist_bit_is_in_range(
                      r1_obj13_a.radar1_obj13_mess_aconsist_bit);
              break;
            case 1425:
              bosch_xgu_corner_radar_radar1_obj14_a_t r1_obj14_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj14_a_unpack(&r1_obj14_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_dx_decode(r1_obj14_a.radar1_obj14_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_dx_is_in_range(r1_obj14_a.radar1_obj14_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_vx_decode(r1_obj14_a.radar1_obj14_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_vx_is_in_range(r1_obj14_a.radar1_obj14_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_dy_decode(r1_obj14_a.radar1_obj14_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_dy_is_in_range(r1_obj14_a.radar1_obj14_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_flag_hist_decode(
                  r1_obj14_a.radar1_obj14_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_flag_hist_is_in_range(
                      r1_obj14_a.radar1_obj14_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_mess_aconsist_bit_decode(
                      r1_obj14_a.radar1_obj14_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_a_radar1_obj14_mess_aconsist_bit_is_in_range(
                      r1_obj14_a.radar1_obj14_mess_aconsist_bit);
              break;
            case 1435:
              bosch_xgu_corner_radar_radar1_obj15_a_t r1_obj15_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj15_a_unpack(&r1_obj15_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_dx_decode(r1_obj15_a.radar1_obj15_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_dx_is_in_range(r1_obj15_a.radar1_obj15_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_vx_decode(r1_obj15_a.radar1_obj15_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_vx_is_in_range(r1_obj15_a.radar1_obj15_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_dy_decode(r1_obj15_a.radar1_obj15_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_dy_is_in_range(r1_obj15_a.radar1_obj15_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_flag_hist_decode(
                  r1_obj15_a.radar1_obj15_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_flag_hist_is_in_range(
                      r1_obj15_a.radar1_obj15_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_mess_aconsist_bit_decode(
                      r1_obj15_a.radar1_obj15_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_a_radar1_obj15_mess_aconsist_bit_is_in_range(
                      r1_obj15_a.radar1_obj15_mess_aconsist_bit);
              break;
            case 1445:
              bosch_xgu_corner_radar_radar1_obj16_a_t r1_obj16_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj16_a_unpack(&r1_obj16_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_dx_decode(r1_obj16_a.radar1_obj16_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_dx_is_in_range(r1_obj16_a.radar1_obj16_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_vx_decode(r1_obj16_a.radar1_obj16_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_vx_is_in_range(r1_obj16_a.radar1_obj16_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_dy_decode(r1_obj16_a.radar1_obj16_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_dy_is_in_range(r1_obj16_a.radar1_obj16_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_flag_hist_decode(
                  r1_obj16_a.radar1_obj16_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_flag_hist_is_in_range(
                      r1_obj16_a.radar1_obj16_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_mess_aconsist_bit_decode(
                      r1_obj16_a.radar1_obj16_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_a_radar1_obj16_mess_aconsist_bit_is_in_range(
                      r1_obj16_a.radar1_obj16_mess_aconsist_bit);
              break;
            case 1455:
              bosch_xgu_corner_radar_radar1_obj17_a_t r1_obj17_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj17_a_unpack(&r1_obj17_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_dx_decode(r1_obj17_a.radar1_obj17_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_dx_is_in_range(r1_obj17_a.radar1_obj17_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_vx_decode(r1_obj17_a.radar1_obj17_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_vx_is_in_range(r1_obj17_a.radar1_obj17_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_dy_decode(r1_obj17_a.radar1_obj17_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_dy_is_in_range(r1_obj17_a.radar1_obj17_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_flag_hist_decode(
                  r1_obj17_a.radar1_obj17_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_flag_hist_is_in_range(
                      r1_obj17_a.radar1_obj17_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_mess_aconsist_bit_decode(
                      r1_obj17_a.radar1_obj17_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_a_radar1_obj17_mess_aconsist_bit_is_in_range(
                      r1_obj17_a.radar1_obj17_mess_aconsist_bit);
              break;
            case 1465:
              bosch_xgu_corner_radar_radar1_obj18_a_t r1_obj18_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj18_a_unpack(&r1_obj18_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_dx_decode(r1_obj18_a.radar1_obj18_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_dx_is_in_range(r1_obj18_a.radar1_obj18_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_vx_decode(r1_obj18_a.radar1_obj18_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_vx_is_in_range(r1_obj18_a.radar1_obj18_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_dy_decode(r1_obj18_a.radar1_obj18_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_dy_is_in_range(r1_obj18_a.radar1_obj18_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_flag_hist_decode(
                  r1_obj18_a.radar1_obj18_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_flag_hist_is_in_range(
                      r1_obj18_a.radar1_obj18_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_mess_aconsist_bit_decode(
                      r1_obj18_a.radar1_obj18_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_a_radar1_obj18_mess_aconsist_bit_is_in_range(
                      r1_obj18_a.radar1_obj18_mess_aconsist_bit);
              break;
            case 1475:
              bosch_xgu_corner_radar_radar1_obj19_a_t r1_obj19_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj19_a_unpack(&r1_obj19_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_dx_decode(r1_obj19_a.radar1_obj19_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_dx_is_in_range(r1_obj19_a.radar1_obj19_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_vx_decode(r1_obj19_a.radar1_obj19_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_vx_is_in_range(r1_obj19_a.radar1_obj19_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_dy_decode(r1_obj19_a.radar1_obj19_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_dy_is_in_range(r1_obj19_a.radar1_obj19_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_flag_hist_decode(
                  r1_obj19_a.radar1_obj19_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_flag_hist_is_in_range(
                      r1_obj19_a.radar1_obj19_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_mess_aconsist_bit_decode(
                      r1_obj19_a.radar1_obj19_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_a_radar1_obj19_mess_aconsist_bit_is_in_range(
                      r1_obj19_a.radar1_obj19_mess_aconsist_bit);
              break;
            case 1485:
              bosch_xgu_corner_radar_radar1_obj20_a_t r1_obj20_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj20_a_unpack(&r1_obj20_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_dx_decode(r1_obj20_a.radar1_obj20_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_dx_is_in_range(r1_obj20_a.radar1_obj20_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_vx_decode(r1_obj20_a.radar1_obj20_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_vx_is_in_range(r1_obj20_a.radar1_obj20_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_dy_decode(r1_obj20_a.radar1_obj20_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_dy_is_in_range(r1_obj20_a.radar1_obj20_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_flag_hist_decode(
                  r1_obj20_a.radar1_obj20_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_flag_hist_is_in_range(
                      r1_obj20_a.radar1_obj20_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_mess_aconsist_bit_decode(
                      r1_obj20_a.radar1_obj20_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_a_radar1_obj20_mess_aconsist_bit_is_in_range(
                      r1_obj20_a.radar1_obj20_mess_aconsist_bit);
              break;
            case 1495:
              bosch_xgu_corner_radar_radar1_obj21_a_t r1_obj21_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj21_a_unpack(&r1_obj21_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_dx_decode(r1_obj21_a.radar1_obj21_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_dx_is_in_range(r1_obj21_a.radar1_obj21_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_vx_decode(r1_obj21_a.radar1_obj21_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_vx_is_in_range(r1_obj21_a.radar1_obj21_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_dy_decode(r1_obj21_a.radar1_obj21_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_dy_is_in_range(r1_obj21_a.radar1_obj21_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_flag_hist_decode(
                  r1_obj21_a.radar1_obj21_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_flag_hist_is_in_range(
                      r1_obj21_a.radar1_obj21_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_mess_aconsist_bit_decode(
                      r1_obj21_a.radar1_obj21_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_a_radar1_obj21_mess_aconsist_bit_is_in_range(
                      r1_obj21_a.radar1_obj21_mess_aconsist_bit);
              break;
            case 1505:
              bosch_xgu_corner_radar_radar1_obj22_a_t r1_obj22_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj22_a_unpack(&r1_obj22_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_dx_decode(r1_obj22_a.radar1_obj22_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_dx_is_in_range(r1_obj22_a.radar1_obj22_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_vx_decode(r1_obj22_a.radar1_obj22_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_vx_is_in_range(r1_obj22_a.radar1_obj22_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_dy_decode(r1_obj22_a.radar1_obj22_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_dy_is_in_range(r1_obj22_a.radar1_obj22_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_flag_hist_decode(
                  r1_obj22_a.radar1_obj22_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_flag_hist_is_in_range(
                      r1_obj22_a.radar1_obj22_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_mess_aconsist_bit_decode(
                      r1_obj22_a.radar1_obj22_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_a_radar1_obj22_mess_aconsist_bit_is_in_range(
                      r1_obj22_a.radar1_obj22_mess_aconsist_bit);
              break;
            case 1515:
              bosch_xgu_corner_radar_radar1_obj23_a_t r1_obj23_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj23_a_unpack(&r1_obj23_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_dx_decode(r1_obj23_a.radar1_obj23_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_dx_is_in_range(r1_obj23_a.radar1_obj23_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_vx_decode(r1_obj23_a.radar1_obj23_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_vx_is_in_range(r1_obj23_a.radar1_obj23_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_dy_decode(r1_obj23_a.radar1_obj23_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_dy_is_in_range(r1_obj23_a.radar1_obj23_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_flag_hist_decode(
                  r1_obj23_a.radar1_obj23_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_flag_hist_is_in_range(
                      r1_obj23_a.radar1_obj23_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_mess_aconsist_bit_decode(
                      r1_obj23_a.radar1_obj23_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_a_radar1_obj23_mess_aconsist_bit_is_in_range(
                      r1_obj23_a.radar1_obj23_mess_aconsist_bit);
              break;
            case 1525:
              bosch_xgu_corner_radar_radar1_obj24_a_t r1_obj24_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj24_a_unpack(&r1_obj24_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_dx_decode(r1_obj24_a.radar1_obj24_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_dx_is_in_range(r1_obj24_a.radar1_obj24_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_vx_decode(r1_obj24_a.radar1_obj24_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_vx_is_in_range(r1_obj24_a.radar1_obj24_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_dy_decode(r1_obj24_a.radar1_obj24_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_dy_is_in_range(r1_obj24_a.radar1_obj24_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_flag_hist_decode(
                  r1_obj24_a.radar1_obj24_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_flag_hist_is_in_range(
                      r1_obj24_a.radar1_obj24_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_mess_aconsist_bit_decode(
                      r1_obj24_a.radar1_obj24_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_a_radar1_obj24_mess_aconsist_bit_is_in_range(
                      r1_obj24_a.radar1_obj24_mess_aconsist_bit);
              break;
            case 1535:
              bosch_xgu_corner_radar_radar1_obj25_a_t r1_obj25_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj25_a_unpack(&r1_obj25_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_dx_decode(r1_obj25_a.radar1_obj25_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_dx_is_in_range(r1_obj25_a.radar1_obj25_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_vx_decode(r1_obj25_a.radar1_obj25_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_vx_is_in_range(r1_obj25_a.radar1_obj25_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_dy_decode(r1_obj25_a.radar1_obj25_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_dy_is_in_range(r1_obj25_a.radar1_obj25_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_flag_hist_decode(
                  r1_obj25_a.radar1_obj25_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_flag_hist_is_in_range(
                      r1_obj25_a.radar1_obj25_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_mess_aconsist_bit_decode(
                      r1_obj25_a.radar1_obj25_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_a_radar1_obj25_mess_aconsist_bit_is_in_range(
                      r1_obj25_a.radar1_obj25_mess_aconsist_bit);
              break;
            case 1545:
              bosch_xgu_corner_radar_radar1_obj26_a_t r1_obj26_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj26_a_unpack(&r1_obj26_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_dx_decode(r1_obj26_a.radar1_obj26_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_dx_is_in_range(r1_obj26_a.radar1_obj26_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_vx_decode(r1_obj26_a.radar1_obj26_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_vx_is_in_range(r1_obj26_a.radar1_obj26_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_dy_decode(r1_obj26_a.radar1_obj26_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_dy_is_in_range(r1_obj26_a.radar1_obj26_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_flag_hist_decode(
                  r1_obj26_a.radar1_obj26_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_flag_hist_is_in_range(
                      r1_obj26_a.radar1_obj26_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_mess_aconsist_bit_decode(
                      r1_obj26_a.radar1_obj26_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_a_radar1_obj26_mess_aconsist_bit_is_in_range(
                      r1_obj26_a.radar1_obj26_mess_aconsist_bit);
              break;
            case 1555:
              bosch_xgu_corner_radar_radar1_obj27_a_t r1_obj27_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj27_a_unpack(&r1_obj27_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_dx_decode(r1_obj27_a.radar1_obj27_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_dx_is_in_range(r1_obj27_a.radar1_obj27_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_vx_decode(r1_obj27_a.radar1_obj27_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_vx_is_in_range(r1_obj27_a.radar1_obj27_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_dy_decode(r1_obj27_a.radar1_obj27_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_dy_is_in_range(r1_obj27_a.radar1_obj27_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_flag_hist_decode(
                  r1_obj27_a.radar1_obj27_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_flag_hist_is_in_range(
                      r1_obj27_a.radar1_obj27_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_mess_aconsist_bit_decode(
                      r1_obj27_a.radar1_obj27_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_a_radar1_obj27_mess_aconsist_bit_is_in_range(
                      r1_obj27_a.radar1_obj27_mess_aconsist_bit);
              break;
            case 1565:
              bosch_xgu_corner_radar_radar1_obj28_a_t r1_obj28_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj28_a_unpack(&r1_obj28_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_dx_decode(r1_obj28_a.radar1_obj28_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_dx_is_in_range(r1_obj28_a.radar1_obj28_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_vx_decode(r1_obj28_a.radar1_obj28_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_vx_is_in_range(r1_obj28_a.radar1_obj28_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_dy_decode(r1_obj28_a.radar1_obj28_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_dy_is_in_range(r1_obj28_a.radar1_obj28_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_flag_hist_decode(
                  r1_obj28_a.radar1_obj28_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_flag_hist_is_in_range(
                      r1_obj28_a.radar1_obj28_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_mess_aconsist_bit_decode(
                      r1_obj28_a.radar1_obj28_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_a_radar1_obj28_mess_aconsist_bit_is_in_range(
                      r1_obj28_a.radar1_obj28_mess_aconsist_bit);
              break;
            case 1575:
              bosch_xgu_corner_radar_radar1_obj29_a_t r1_obj29_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj29_a_unpack(&r1_obj29_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_dx_decode(r1_obj29_a.radar1_obj29_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_dx_is_in_range(r1_obj29_a.radar1_obj29_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_vx_decode(r1_obj29_a.radar1_obj29_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_vx_is_in_range(r1_obj29_a.radar1_obj29_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_dy_decode(r1_obj29_a.radar1_obj29_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_dy_is_in_range(r1_obj29_a.radar1_obj29_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_flag_hist_decode(
                  r1_obj29_a.radar1_obj29_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_flag_hist_is_in_range(
                      r1_obj29_a.radar1_obj29_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_mess_aconsist_bit_decode(
                      r1_obj29_a.radar1_obj29_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_a_radar1_obj29_mess_aconsist_bit_is_in_range(
                      r1_obj29_a.radar1_obj29_mess_aconsist_bit);
              break;
            case 1585:
              bosch_xgu_corner_radar_radar1_obj30_a_t r1_obj30_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj30_a_unpack(&r1_obj30_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_dx_decode(r1_obj30_a.radar1_obj30_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_dx_is_in_range(r1_obj30_a.radar1_obj30_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_vx_decode(r1_obj30_a.radar1_obj30_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_vx_is_in_range(r1_obj30_a.radar1_obj30_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_dy_decode(r1_obj30_a.radar1_obj30_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_dy_is_in_range(r1_obj30_a.radar1_obj30_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_flag_hist_decode(
                  r1_obj30_a.radar1_obj30_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_flag_hist_is_in_range(
                      r1_obj30_a.radar1_obj30_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_mess_aconsist_bit_decode(
                      r1_obj30_a.radar1_obj30_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_a_radar1_obj30_mess_aconsist_bit_is_in_range(
                      r1_obj30_a.radar1_obj30_mess_aconsist_bit);
              break;
            case 1595:
              bosch_xgu_corner_radar_radar1_obj31_a_t r1_obj31_a;
              unpack_return = bosch_xgu_corner_radar_radar1_obj31_a_unpack(&r1_obj31_a, can_data, size_of_msg);
              all_object_info.dx_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_dx_decode(r1_obj31_a.radar1_obj31_dx);
              all_object_info.dx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_dx_is_in_range(r1_obj31_a.radar1_obj31_dx);
              all_object_info.vx_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_vx_decode(r1_obj31_a.radar1_obj31_vx);
              all_object_info.vx_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_vx_is_in_range(r1_obj31_a.radar1_obj31_vx);
              all_object_info.dy_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_dy_decode(r1_obj31_a.radar1_obj31_dy);
              all_object_info.dy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_dy_is_in_range(r1_obj31_a.radar1_obj31_dy);
              all_object_info.flag_hist_decoded = bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_flag_hist_decode(
                  r1_obj31_a.radar1_obj31_flag_hist);
              all_object_info.flag_hist_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_flag_hist_is_in_range(
                      r1_obj31_a.radar1_obj31_flag_hist);
              all_object_info.mess_aconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_mess_aconsist_bit_decode(
                      r1_obj31_a.radar1_obj31_mess_aconsist_bit);
              all_object_info.mess_aconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_a_radar1_obj31_mess_aconsist_bit_is_in_range(
                      r1_obj31_a.radar1_obj31_mess_aconsist_bit);
              break;
            case 1286:
              bosch_xgu_corner_radar_radar1_obj00_b_t r1_obj00_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj00_b_unpack(&r1_obj00_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_vy_decode(r1_obj00_b.radar1_obj00_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_vy_is_in_range(r1_obj00_b.radar1_obj00_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_d_length_decode(r1_obj00_b.radar1_obj00_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_d_length_is_in_range(
                      r1_obj00_b.radar1_obj00_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_dz_decode(r1_obj00_b.radar1_obj00_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_dz_is_in_range(r1_obj00_b.radar1_obj00_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_moving_state_decode(
                      r1_obj00_b.radar1_obj00_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_moving_state_is_in_range(
                      r1_obj00_b.radar1_obj00_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_dx_sigma_decode(r1_obj00_b.radar1_obj00_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_dx_sigma_is_in_range(
                      r1_obj00_b.radar1_obj00_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_vx_sigma_decode(r1_obj00_b.radar1_obj00_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_vx_sigma_is_in_range(
                      r1_obj00_b.radar1_obj00_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_dy_sigma_decode(r1_obj00_b.radar1_obj00_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_dy_sigma_is_in_range(
                      r1_obj00_b.radar1_obj00_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_w_class_decode(r1_obj00_b.radar1_obj00_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_w_class_is_in_range(
                      r1_obj00_b.radar1_obj00_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_class_decode(r1_obj00_b.radar1_obj00_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_class_is_in_range(r1_obj00_b.radar1_obj00_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_mess_bconsist_bit_encode(
                      r1_obj00_b.radar1_obj00_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj00_b_radar1_obj00_mess_bconsist_bit_is_in_range(
                      r1_obj00_b.radar1_obj00_mess_bconsist_bit);
              break;
            case 1296:
              bosch_xgu_corner_radar_radar1_obj01_b_t r1_obj01_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj01_b_unpack(&r1_obj01_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_vy_decode(r1_obj01_b.radar1_obj01_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_vy_is_in_range(r1_obj01_b.radar1_obj01_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_d_length_decode(r1_obj01_b.radar1_obj01_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_d_length_is_in_range(
                      r1_obj01_b.radar1_obj01_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_dz_decode(r1_obj01_b.radar1_obj01_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_dz_is_in_range(r1_obj01_b.radar1_obj01_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_moving_state_decode(
                      r1_obj01_b.radar1_obj01_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_moving_state_is_in_range(
                      r1_obj01_b.radar1_obj01_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_dx_sigma_decode(r1_obj01_b.radar1_obj01_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_dx_sigma_is_in_range(
                      r1_obj01_b.radar1_obj01_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_vx_sigma_decode(r1_obj01_b.radar1_obj01_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_vx_sigma_is_in_range(
                      r1_obj01_b.radar1_obj01_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_dy_sigma_decode(r1_obj01_b.radar1_obj01_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_dy_sigma_is_in_range(
                      r1_obj01_b.radar1_obj01_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_w_class_decode(r1_obj01_b.radar1_obj01_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_w_class_is_in_range(
                      r1_obj01_b.radar1_obj01_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_class_decode(r1_obj01_b.radar1_obj01_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_class_is_in_range(r1_obj01_b.radar1_obj01_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_mess_bconsist_bit_encode(
                      r1_obj01_b.radar1_obj01_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj01_b_radar1_obj01_mess_bconsist_bit_is_in_range(
                      r1_obj01_b.radar1_obj01_mess_bconsist_bit);
              break;
            case 1306:
              bosch_xgu_corner_radar_radar1_obj02_b_t r1_obj02_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj02_b_unpack(&r1_obj02_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_vy_decode(r1_obj02_b.radar1_obj02_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_vy_is_in_range(r1_obj02_b.radar1_obj02_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_d_length_decode(r1_obj02_b.radar1_obj02_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_d_length_is_in_range(
                      r1_obj02_b.radar1_obj02_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_dz_decode(r1_obj02_b.radar1_obj02_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_dz_is_in_range(r1_obj02_b.radar1_obj02_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_moving_state_decode(
                      r1_obj02_b.radar1_obj02_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_moving_state_is_in_range(
                      r1_obj02_b.radar1_obj02_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_dx_sigma_decode(r1_obj02_b.radar1_obj02_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_dx_sigma_is_in_range(
                      r1_obj02_b.radar1_obj02_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_vx_sigma_decode(r1_obj02_b.radar1_obj02_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_vx_sigma_is_in_range(
                      r1_obj02_b.radar1_obj02_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_dy_sigma_decode(r1_obj02_b.radar1_obj02_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_dy_sigma_is_in_range(
                      r1_obj02_b.radar1_obj02_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_w_class_decode(r1_obj02_b.radar1_obj02_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_w_class_is_in_range(
                      r1_obj02_b.radar1_obj02_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_class_decode(r1_obj02_b.radar1_obj02_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_class_is_in_range(r1_obj02_b.radar1_obj02_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_mess_bconsist_bit_encode(
                      r1_obj02_b.radar1_obj02_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj02_b_radar1_obj02_mess_bconsist_bit_is_in_range(
                      r1_obj02_b.radar1_obj02_mess_bconsist_bit);
              break;
            case 1316:
              bosch_xgu_corner_radar_radar1_obj03_b_t r1_obj03_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj03_b_unpack(&r1_obj03_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_vy_decode(r1_obj03_b.radar1_obj03_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_vy_is_in_range(r1_obj03_b.radar1_obj03_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_d_length_decode(r1_obj03_b.radar1_obj03_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_d_length_is_in_range(
                      r1_obj03_b.radar1_obj03_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_dz_decode(r1_obj03_b.radar1_obj03_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_dz_is_in_range(r1_obj03_b.radar1_obj03_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_moving_state_decode(
                      r1_obj03_b.radar1_obj03_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_moving_state_is_in_range(
                      r1_obj03_b.radar1_obj03_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_dx_sigma_decode(r1_obj03_b.radar1_obj03_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_dx_sigma_is_in_range(
                      r1_obj03_b.radar1_obj03_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_vx_sigma_decode(r1_obj03_b.radar1_obj03_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_vx_sigma_is_in_range(
                      r1_obj03_b.radar1_obj03_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_dy_sigma_decode(r1_obj03_b.radar1_obj03_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_dy_sigma_is_in_range(
                      r1_obj03_b.radar1_obj03_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_w_class_decode(r1_obj03_b.radar1_obj03_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_w_class_is_in_range(
                      r1_obj03_b.radar1_obj03_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_class_decode(r1_obj03_b.radar1_obj03_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_class_is_in_range(r1_obj03_b.radar1_obj03_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_mess_bconsist_bit_encode(
                      r1_obj03_b.radar1_obj03_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj03_b_radar1_obj03_mess_bconsist_bit_is_in_range(
                      r1_obj03_b.radar1_obj03_mess_bconsist_bit);
              break;
            case 1326:
              bosch_xgu_corner_radar_radar1_obj04_b_t r1_obj04_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj04_b_unpack(&r1_obj04_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_vy_decode(r1_obj04_b.radar1_obj04_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_vy_is_in_range(r1_obj04_b.radar1_obj04_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_d_length_decode(r1_obj04_b.radar1_obj04_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_d_length_is_in_range(
                      r1_obj04_b.radar1_obj04_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_dz_decode(r1_obj04_b.radar1_obj04_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_dz_is_in_range(r1_obj04_b.radar1_obj04_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_moving_state_decode(
                      r1_obj04_b.radar1_obj04_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_moving_state_is_in_range(
                      r1_obj04_b.radar1_obj04_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_dx_sigma_decode(r1_obj04_b.radar1_obj04_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_dx_sigma_is_in_range(
                      r1_obj04_b.radar1_obj04_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_vx_sigma_decode(r1_obj04_b.radar1_obj04_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_vx_sigma_is_in_range(
                      r1_obj04_b.radar1_obj04_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_dy_sigma_decode(r1_obj04_b.radar1_obj04_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_dy_sigma_is_in_range(
                      r1_obj04_b.radar1_obj04_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_w_class_decode(r1_obj04_b.radar1_obj04_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_w_class_is_in_range(
                      r1_obj04_b.radar1_obj04_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_class_decode(r1_obj04_b.radar1_obj04_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_class_is_in_range(r1_obj04_b.radar1_obj04_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_mess_bconsist_bit_encode(
                      r1_obj04_b.radar1_obj04_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj04_b_radar1_obj04_mess_bconsist_bit_is_in_range(
                      r1_obj04_b.radar1_obj04_mess_bconsist_bit);
              break;
            case 1336:
              bosch_xgu_corner_radar_radar1_obj05_b_t r1_obj05_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj05_b_unpack(&r1_obj05_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_vy_decode(r1_obj05_b.radar1_obj05_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_vy_is_in_range(r1_obj05_b.radar1_obj05_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_d_length_decode(r1_obj05_b.radar1_obj05_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_d_length_is_in_range(
                      r1_obj05_b.radar1_obj05_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_dz_decode(r1_obj05_b.radar1_obj05_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_dz_is_in_range(r1_obj05_b.radar1_obj05_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_moving_state_decode(
                      r1_obj05_b.radar1_obj05_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_moving_state_is_in_range(
                      r1_obj05_b.radar1_obj05_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_dx_sigma_decode(r1_obj05_b.radar1_obj05_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_dx_sigma_is_in_range(
                      r1_obj05_b.radar1_obj05_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_vx_sigma_decode(r1_obj05_b.radar1_obj05_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_vx_sigma_is_in_range(
                      r1_obj05_b.radar1_obj05_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_dy_sigma_decode(r1_obj05_b.radar1_obj05_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_dy_sigma_is_in_range(
                      r1_obj05_b.radar1_obj05_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_w_class_decode(r1_obj05_b.radar1_obj05_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_w_class_is_in_range(
                      r1_obj05_b.radar1_obj05_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_class_decode(r1_obj05_b.radar1_obj05_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_class_is_in_range(r1_obj05_b.radar1_obj05_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_mess_bconsist_bit_encode(
                      r1_obj05_b.radar1_obj05_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj05_b_radar1_obj05_mess_bconsist_bit_is_in_range(
                      r1_obj05_b.radar1_obj05_mess_bconsist_bit);
              break;
            case 1346:
              bosch_xgu_corner_radar_radar1_obj06_b_t r1_obj06_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj06_b_unpack(&r1_obj06_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_vy_decode(r1_obj06_b.radar1_obj06_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_vy_is_in_range(r1_obj06_b.radar1_obj06_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_d_length_decode(r1_obj06_b.radar1_obj06_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_d_length_is_in_range(
                      r1_obj06_b.radar1_obj06_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_dz_decode(r1_obj06_b.radar1_obj06_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_dz_is_in_range(r1_obj06_b.radar1_obj06_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_moving_state_decode(
                      r1_obj06_b.radar1_obj06_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_moving_state_is_in_range(
                      r1_obj06_b.radar1_obj06_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_dx_sigma_decode(r1_obj06_b.radar1_obj06_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_dx_sigma_is_in_range(
                      r1_obj06_b.radar1_obj06_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_vx_sigma_decode(r1_obj06_b.radar1_obj06_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_vx_sigma_is_in_range(
                      r1_obj06_b.radar1_obj06_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_dy_sigma_decode(r1_obj06_b.radar1_obj06_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_dy_sigma_is_in_range(
                      r1_obj06_b.radar1_obj06_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_w_class_decode(r1_obj06_b.radar1_obj06_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_w_class_is_in_range(
                      r1_obj06_b.radar1_obj06_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_class_decode(r1_obj06_b.radar1_obj06_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_class_is_in_range(r1_obj06_b.radar1_obj06_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_mess_bconsist_bit_encode(
                      r1_obj06_b.radar1_obj06_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj06_b_radar1_obj06_mess_bconsist_bit_is_in_range(
                      r1_obj06_b.radar1_obj06_mess_bconsist_bit);
              break;
            case 1356:
              bosch_xgu_corner_radar_radar1_obj07_b_t r1_obj07_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj07_b_unpack(&r1_obj07_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_vy_decode(r1_obj07_b.radar1_obj07_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_vy_is_in_range(r1_obj07_b.radar1_obj07_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_d_length_decode(r1_obj07_b.radar1_obj07_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_d_length_is_in_range(
                      r1_obj07_b.radar1_obj07_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_dz_decode(r1_obj07_b.radar1_obj07_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_dz_is_in_range(r1_obj07_b.radar1_obj07_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_moving_state_decode(
                      r1_obj07_b.radar1_obj07_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_moving_state_is_in_range(
                      r1_obj07_b.radar1_obj07_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_dx_sigma_decode(r1_obj07_b.radar1_obj07_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_dx_sigma_is_in_range(
                      r1_obj07_b.radar1_obj07_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_vx_sigma_decode(r1_obj07_b.radar1_obj07_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_vx_sigma_is_in_range(
                      r1_obj07_b.radar1_obj07_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_dy_sigma_decode(r1_obj07_b.radar1_obj07_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_dy_sigma_is_in_range(
                      r1_obj07_b.radar1_obj07_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_w_class_decode(r1_obj07_b.radar1_obj07_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_w_class_is_in_range(
                      r1_obj07_b.radar1_obj07_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_class_decode(r1_obj07_b.radar1_obj07_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_class_is_in_range(r1_obj07_b.radar1_obj07_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_mess_bconsist_bit_encode(
                      r1_obj07_b.radar1_obj07_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj07_b_radar1_obj07_mess_bconsist_bit_is_in_range(
                      r1_obj07_b.radar1_obj07_mess_bconsist_bit);
              break;
            case 1366:
              bosch_xgu_corner_radar_radar1_obj08_b_t r1_obj08_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj08_b_unpack(&r1_obj08_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_vy_decode(r1_obj08_b.radar1_obj08_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_vy_is_in_range(r1_obj08_b.radar1_obj08_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_d_length_decode(r1_obj08_b.radar1_obj08_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_d_length_is_in_range(
                      r1_obj08_b.radar1_obj08_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_dz_decode(r1_obj08_b.radar1_obj08_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_dz_is_in_range(r1_obj08_b.radar1_obj08_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_moving_state_decode(
                      r1_obj08_b.radar1_obj08_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_moving_state_is_in_range(
                      r1_obj08_b.radar1_obj08_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_dx_sigma_decode(r1_obj08_b.radar1_obj08_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_dx_sigma_is_in_range(
                      r1_obj08_b.radar1_obj08_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_vx_sigma_decode(r1_obj08_b.radar1_obj08_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_vx_sigma_is_in_range(
                      r1_obj08_b.radar1_obj08_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_dy_sigma_decode(r1_obj08_b.radar1_obj08_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_dy_sigma_is_in_range(
                      r1_obj08_b.radar1_obj08_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_w_class_decode(r1_obj08_b.radar1_obj08_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_w_class_is_in_range(
                      r1_obj08_b.radar1_obj08_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_class_decode(r1_obj08_b.radar1_obj08_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_class_is_in_range(r1_obj08_b.radar1_obj08_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_mess_bconsist_bit_encode(
                      r1_obj08_b.radar1_obj08_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj08_b_radar1_obj08_mess_bconsist_bit_is_in_range(
                      r1_obj08_b.radar1_obj08_mess_bconsist_bit);
              break;
            case 1376:
              bosch_xgu_corner_radar_radar1_obj09_b_t r1_obj09_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj09_b_unpack(&r1_obj09_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_vy_decode(r1_obj09_b.radar1_obj09_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_vy_is_in_range(r1_obj09_b.radar1_obj09_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_d_length_decode(r1_obj09_b.radar1_obj09_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_d_length_is_in_range(
                      r1_obj09_b.radar1_obj09_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_dz_decode(r1_obj09_b.radar1_obj09_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_dz_is_in_range(r1_obj09_b.radar1_obj09_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_moving_state_decode(
                      r1_obj09_b.radar1_obj09_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_moving_state_is_in_range(
                      r1_obj09_b.radar1_obj09_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_dx_sigma_decode(r1_obj09_b.radar1_obj09_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_dx_sigma_is_in_range(
                      r1_obj09_b.radar1_obj09_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_vx_sigma_decode(r1_obj09_b.radar1_obj09_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_vx_sigma_is_in_range(
                      r1_obj09_b.radar1_obj09_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_dy_sigma_decode(r1_obj09_b.radar1_obj09_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_dy_sigma_is_in_range(
                      r1_obj09_b.radar1_obj09_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_w_class_decode(r1_obj09_b.radar1_obj09_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_w_class_is_in_range(
                      r1_obj09_b.radar1_obj09_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_class_decode(r1_obj09_b.radar1_obj09_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_class_is_in_range(r1_obj09_b.radar1_obj09_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_mess_bconsist_bit_encode(
                      r1_obj09_b.radar1_obj09_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj09_b_radar1_obj09_mess_bconsist_bit_is_in_range(
                      r1_obj09_b.radar1_obj09_mess_bconsist_bit);
              break;
            case 1386:
              bosch_xgu_corner_radar_radar1_obj10_b_t r1_obj10_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj10_b_unpack(&r1_obj10_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_vy_decode(r1_obj10_b.radar1_obj10_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_vy_is_in_range(r1_obj10_b.radar1_obj10_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_d_length_decode(r1_obj10_b.radar1_obj10_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_d_length_is_in_range(
                      r1_obj10_b.radar1_obj10_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_dz_decode(r1_obj10_b.radar1_obj10_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_dz_is_in_range(r1_obj10_b.radar1_obj10_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_moving_state_decode(
                      r1_obj10_b.radar1_obj10_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_moving_state_is_in_range(
                      r1_obj10_b.radar1_obj10_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_dx_sigma_decode(r1_obj10_b.radar1_obj10_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_dx_sigma_is_in_range(
                      r1_obj10_b.radar1_obj10_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_vx_sigma_decode(r1_obj10_b.radar1_obj10_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_vx_sigma_is_in_range(
                      r1_obj10_b.radar1_obj10_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_dy_sigma_decode(r1_obj10_b.radar1_obj10_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_dy_sigma_is_in_range(
                      r1_obj10_b.radar1_obj10_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_w_class_decode(r1_obj10_b.radar1_obj10_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_w_class_is_in_range(
                      r1_obj10_b.radar1_obj10_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_class_decode(r1_obj10_b.radar1_obj10_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_class_is_in_range(r1_obj10_b.radar1_obj10_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_mess_bconsist_bit_encode(
                      r1_obj10_b.radar1_obj10_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj10_b_radar1_obj10_mess_bconsist_bit_is_in_range(
                      r1_obj10_b.radar1_obj10_mess_bconsist_bit);
              break;
            case 1396:
              bosch_xgu_corner_radar_radar1_obj11_b_t r1_obj11_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj11_b_unpack(&r1_obj11_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_vy_decode(r1_obj11_b.radar1_obj11_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_vy_is_in_range(r1_obj11_b.radar1_obj11_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_d_length_decode(r1_obj11_b.radar1_obj11_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_d_length_is_in_range(
                      r1_obj11_b.radar1_obj11_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_dz_decode(r1_obj11_b.radar1_obj11_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_dz_is_in_range(r1_obj11_b.radar1_obj11_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_moving_state_decode(
                      r1_obj11_b.radar1_obj11_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_moving_state_is_in_range(
                      r1_obj11_b.radar1_obj11_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_dx_sigma_decode(r1_obj11_b.radar1_obj11_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_dx_sigma_is_in_range(
                      r1_obj11_b.radar1_obj11_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_vx_sigma_decode(r1_obj11_b.radar1_obj11_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_vx_sigma_is_in_range(
                      r1_obj11_b.radar1_obj11_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_dy_sigma_decode(r1_obj11_b.radar1_obj11_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_dy_sigma_is_in_range(
                      r1_obj11_b.radar1_obj11_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_w_class_decode(r1_obj11_b.radar1_obj11_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_w_class_is_in_range(
                      r1_obj11_b.radar1_obj11_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_class_decode(r1_obj11_b.radar1_obj11_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_class_is_in_range(r1_obj11_b.radar1_obj11_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_mess_bconsist_bit_encode(
                      r1_obj11_b.radar1_obj11_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj11_b_radar1_obj11_mess_bconsist_bit_is_in_range(
                      r1_obj11_b.radar1_obj11_mess_bconsist_bit);
              break;
            case 1406:
              bosch_xgu_corner_radar_radar1_obj12_b_t r1_obj12_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj12_b_unpack(&r1_obj12_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_vy_decode(r1_obj12_b.radar1_obj12_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_vy_is_in_range(r1_obj12_b.radar1_obj12_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_d_length_decode(r1_obj12_b.radar1_obj12_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_d_length_is_in_range(
                      r1_obj12_b.radar1_obj12_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_dz_decode(r1_obj12_b.radar1_obj12_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_dz_is_in_range(r1_obj12_b.radar1_obj12_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_moving_state_decode(
                      r1_obj12_b.radar1_obj12_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_moving_state_is_in_range(
                      r1_obj12_b.radar1_obj12_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_dx_sigma_decode(r1_obj12_b.radar1_obj12_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_dx_sigma_is_in_range(
                      r1_obj12_b.radar1_obj12_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_vx_sigma_decode(r1_obj12_b.radar1_obj12_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_vx_sigma_is_in_range(
                      r1_obj12_b.radar1_obj12_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_dy_sigma_decode(r1_obj12_b.radar1_obj12_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_dy_sigma_is_in_range(
                      r1_obj12_b.radar1_obj12_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_w_class_decode(r1_obj12_b.radar1_obj12_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_w_class_is_in_range(
                      r1_obj12_b.radar1_obj12_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_class_decode(r1_obj12_b.radar1_obj12_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_class_is_in_range(r1_obj12_b.radar1_obj12_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_mess_bconsist_bit_encode(
                      r1_obj12_b.radar1_obj12_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj12_b_radar1_obj12_mess_bconsist_bit_is_in_range(
                      r1_obj12_b.radar1_obj12_mess_bconsist_bit);
              break;
            case 1416:
              bosch_xgu_corner_radar_radar1_obj13_b_t r1_obj13_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj13_b_unpack(&r1_obj13_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_vy_decode(r1_obj13_b.radar1_obj13_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_vy_is_in_range(r1_obj13_b.radar1_obj13_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_d_length_decode(r1_obj13_b.radar1_obj13_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_d_length_is_in_range(
                      r1_obj13_b.radar1_obj13_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_dz_decode(r1_obj13_b.radar1_obj13_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_dz_is_in_range(r1_obj13_b.radar1_obj13_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_moving_state_decode(
                      r1_obj13_b.radar1_obj13_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_moving_state_is_in_range(
                      r1_obj13_b.radar1_obj13_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_dx_sigma_decode(r1_obj13_b.radar1_obj13_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_dx_sigma_is_in_range(
                      r1_obj13_b.radar1_obj13_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_vx_sigma_decode(r1_obj13_b.radar1_obj13_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_vx_sigma_is_in_range(
                      r1_obj13_b.radar1_obj13_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_dy_sigma_decode(r1_obj13_b.radar1_obj13_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_dy_sigma_is_in_range(
                      r1_obj13_b.radar1_obj13_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_w_class_decode(r1_obj13_b.radar1_obj13_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_w_class_is_in_range(
                      r1_obj13_b.radar1_obj13_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_class_decode(r1_obj13_b.radar1_obj13_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_class_is_in_range(r1_obj13_b.radar1_obj13_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_mess_bconsist_bit_encode(
                      r1_obj13_b.radar1_obj13_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj13_b_radar1_obj13_mess_bconsist_bit_is_in_range(
                      r1_obj13_b.radar1_obj13_mess_bconsist_bit);
              break;
            case 1426:
              bosch_xgu_corner_radar_radar1_obj14_b_t r1_obj14_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj14_b_unpack(&r1_obj14_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_vy_decode(r1_obj14_b.radar1_obj14_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_vy_is_in_range(r1_obj14_b.radar1_obj14_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_d_length_decode(r1_obj14_b.radar1_obj14_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_d_length_is_in_range(
                      r1_obj14_b.radar1_obj14_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_dz_decode(r1_obj14_b.radar1_obj14_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_dz_is_in_range(r1_obj14_b.radar1_obj14_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_moving_state_decode(
                      r1_obj14_b.radar1_obj14_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_moving_state_is_in_range(
                      r1_obj14_b.radar1_obj14_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_dx_sigma_decode(r1_obj14_b.radar1_obj14_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_dx_sigma_is_in_range(
                      r1_obj14_b.radar1_obj14_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_vx_sigma_decode(r1_obj14_b.radar1_obj14_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_vx_sigma_is_in_range(
                      r1_obj14_b.radar1_obj14_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_dy_sigma_decode(r1_obj14_b.radar1_obj14_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_dy_sigma_is_in_range(
                      r1_obj14_b.radar1_obj14_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_w_class_decode(r1_obj14_b.radar1_obj14_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_w_class_is_in_range(
                      r1_obj14_b.radar1_obj14_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_class_decode(r1_obj14_b.radar1_obj14_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_class_is_in_range(r1_obj14_b.radar1_obj14_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_mess_bconsist_bit_encode(
                      r1_obj14_b.radar1_obj14_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj14_b_radar1_obj14_mess_bconsist_bit_is_in_range(
                      r1_obj14_b.radar1_obj14_mess_bconsist_bit);
              break;
            case 1436:
              bosch_xgu_corner_radar_radar1_obj15_b_t r1_obj15_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj15_b_unpack(&r1_obj15_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_vy_decode(r1_obj15_b.radar1_obj15_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_vy_is_in_range(r1_obj15_b.radar1_obj15_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_d_length_decode(r1_obj15_b.radar1_obj15_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_d_length_is_in_range(
                      r1_obj15_b.radar1_obj15_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_dz_decode(r1_obj15_b.radar1_obj15_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_dz_is_in_range(r1_obj15_b.radar1_obj15_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_moving_state_decode(
                      r1_obj15_b.radar1_obj15_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_moving_state_is_in_range(
                      r1_obj15_b.radar1_obj15_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_dx_sigma_decode(r1_obj15_b.radar1_obj15_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_dx_sigma_is_in_range(
                      r1_obj15_b.radar1_obj15_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_vx_sigma_decode(r1_obj15_b.radar1_obj15_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_vx_sigma_is_in_range(
                      r1_obj15_b.radar1_obj15_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_dy_sigma_decode(r1_obj15_b.radar1_obj15_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_dy_sigma_is_in_range(
                      r1_obj15_b.radar1_obj15_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_w_class_decode(r1_obj15_b.radar1_obj15_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_w_class_is_in_range(
                      r1_obj15_b.radar1_obj15_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_class_decode(r1_obj15_b.radar1_obj15_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_class_is_in_range(r1_obj15_b.radar1_obj15_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_mess_bconsist_bit_encode(
                      r1_obj15_b.radar1_obj15_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj15_b_radar1_obj15_mess_bconsist_bit_is_in_range(
                      r1_obj15_b.radar1_obj15_mess_bconsist_bit);
              break;
            case 1446:
              bosch_xgu_corner_radar_radar1_obj16_b_t r1_obj16_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj16_b_unpack(&r1_obj16_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_vy_decode(r1_obj16_b.radar1_obj16_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_vy_is_in_range(r1_obj16_b.radar1_obj16_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_d_length_decode(r1_obj16_b.radar1_obj16_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_d_length_is_in_range(
                      r1_obj16_b.radar1_obj16_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_dz_decode(r1_obj16_b.radar1_obj16_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_dz_is_in_range(r1_obj16_b.radar1_obj16_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_moving_state_decode(
                      r1_obj16_b.radar1_obj16_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_moving_state_is_in_range(
                      r1_obj16_b.radar1_obj16_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_dx_sigma_decode(r1_obj16_b.radar1_obj16_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_dx_sigma_is_in_range(
                      r1_obj16_b.radar1_obj16_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_vx_sigma_decode(r1_obj16_b.radar1_obj16_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_vx_sigma_is_in_range(
                      r1_obj16_b.radar1_obj16_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_dy_sigma_decode(r1_obj16_b.radar1_obj16_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_dy_sigma_is_in_range(
                      r1_obj16_b.radar1_obj16_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_w_class_decode(r1_obj16_b.radar1_obj16_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_w_class_is_in_range(
                      r1_obj16_b.radar1_obj16_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_class_decode(r1_obj16_b.radar1_obj16_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_class_is_in_range(r1_obj16_b.radar1_obj16_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_mess_bconsist_bit_encode(
                      r1_obj16_b.radar1_obj16_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj16_b_radar1_obj16_mess_bconsist_bit_is_in_range(
                      r1_obj16_b.radar1_obj16_mess_bconsist_bit);
              break;
            case 1456:
              bosch_xgu_corner_radar_radar1_obj17_b_t r1_obj17_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj17_b_unpack(&r1_obj17_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_vy_decode(r1_obj17_b.radar1_obj17_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_vy_is_in_range(r1_obj17_b.radar1_obj17_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_d_length_decode(r1_obj17_b.radar1_obj17_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_d_length_is_in_range(
                      r1_obj17_b.radar1_obj17_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_dz_decode(r1_obj17_b.radar1_obj17_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_dz_is_in_range(r1_obj17_b.radar1_obj17_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_moving_state_decode(
                      r1_obj17_b.radar1_obj17_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_moving_state_is_in_range(
                      r1_obj17_b.radar1_obj17_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_dx_sigma_decode(r1_obj17_b.radar1_obj17_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_dx_sigma_is_in_range(
                      r1_obj17_b.radar1_obj17_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_vx_sigma_decode(r1_obj17_b.radar1_obj17_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_vx_sigma_is_in_range(
                      r1_obj17_b.radar1_obj17_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_dy_sigma_decode(r1_obj17_b.radar1_obj17_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_dy_sigma_is_in_range(
                      r1_obj17_b.radar1_obj17_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_w_class_decode(r1_obj17_b.radar1_obj17_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_w_class_is_in_range(
                      r1_obj17_b.radar1_obj17_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_class_decode(r1_obj17_b.radar1_obj17_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_class_is_in_range(r1_obj17_b.radar1_obj17_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_mess_bconsist_bit_encode(
                      r1_obj17_b.radar1_obj17_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj17_b_radar1_obj17_mess_bconsist_bit_is_in_range(
                      r1_obj17_b.radar1_obj17_mess_bconsist_bit);
              break;
            case 1466:
              bosch_xgu_corner_radar_radar1_obj18_b_t r1_obj18_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj18_b_unpack(&r1_obj18_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_vy_decode(r1_obj18_b.radar1_obj18_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_vy_is_in_range(r1_obj18_b.radar1_obj18_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_d_length_decode(r1_obj18_b.radar1_obj18_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_d_length_is_in_range(
                      r1_obj18_b.radar1_obj18_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_dz_decode(r1_obj18_b.radar1_obj18_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_dz_is_in_range(r1_obj18_b.radar1_obj18_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_moving_state_decode(
                      r1_obj18_b.radar1_obj18_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_moving_state_is_in_range(
                      r1_obj18_b.radar1_obj18_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_dx_sigma_decode(r1_obj18_b.radar1_obj18_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_dx_sigma_is_in_range(
                      r1_obj18_b.radar1_obj18_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_vx_sigma_decode(r1_obj18_b.radar1_obj18_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_vx_sigma_is_in_range(
                      r1_obj18_b.radar1_obj18_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_dy_sigma_decode(r1_obj18_b.radar1_obj18_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_dy_sigma_is_in_range(
                      r1_obj18_b.radar1_obj18_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_w_class_decode(r1_obj18_b.radar1_obj18_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_w_class_is_in_range(
                      r1_obj18_b.radar1_obj18_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_class_decode(r1_obj18_b.radar1_obj18_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_class_is_in_range(r1_obj18_b.radar1_obj18_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_mess_bconsist_bit_encode(
                      r1_obj18_b.radar1_obj18_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj18_b_radar1_obj18_mess_bconsist_bit_is_in_range(
                      r1_obj18_b.radar1_obj18_mess_bconsist_bit);
              break;
            case 1476:
              bosch_xgu_corner_radar_radar1_obj19_b_t r1_obj19_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj19_b_unpack(&r1_obj19_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_vy_decode(r1_obj19_b.radar1_obj19_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_vy_is_in_range(r1_obj19_b.radar1_obj19_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_d_length_decode(r1_obj19_b.radar1_obj19_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_d_length_is_in_range(
                      r1_obj19_b.radar1_obj19_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_dz_decode(r1_obj19_b.radar1_obj19_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_dz_is_in_range(r1_obj19_b.radar1_obj19_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_moving_state_decode(
                      r1_obj19_b.radar1_obj19_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_moving_state_is_in_range(
                      r1_obj19_b.radar1_obj19_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_dx_sigma_decode(r1_obj19_b.radar1_obj19_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_dx_sigma_is_in_range(
                      r1_obj19_b.radar1_obj19_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_vx_sigma_decode(r1_obj19_b.radar1_obj19_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_vx_sigma_is_in_range(
                      r1_obj19_b.radar1_obj19_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_dy_sigma_decode(r1_obj19_b.radar1_obj19_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_dy_sigma_is_in_range(
                      r1_obj19_b.radar1_obj19_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_w_class_decode(r1_obj19_b.radar1_obj19_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_w_class_is_in_range(
                      r1_obj19_b.radar1_obj19_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_class_decode(r1_obj19_b.radar1_obj19_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_class_is_in_range(r1_obj19_b.radar1_obj19_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_mess_bconsist_bit_encode(
                      r1_obj19_b.radar1_obj19_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj19_b_radar1_obj19_mess_bconsist_bit_is_in_range(
                      r1_obj19_b.radar1_obj19_mess_bconsist_bit);
              break;
            case 1486:
              bosch_xgu_corner_radar_radar1_obj20_b_t r1_obj20_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj20_b_unpack(&r1_obj20_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_vy_decode(r1_obj20_b.radar1_obj20_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_vy_is_in_range(r1_obj20_b.radar1_obj20_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_d_length_decode(r1_obj20_b.radar1_obj20_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_d_length_is_in_range(
                      r1_obj20_b.radar1_obj20_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_dz_decode(r1_obj20_b.radar1_obj20_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_dz_is_in_range(r1_obj20_b.radar1_obj20_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_moving_state_decode(
                      r1_obj20_b.radar1_obj20_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_moving_state_is_in_range(
                      r1_obj20_b.radar1_obj20_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_dx_sigma_decode(r1_obj20_b.radar1_obj20_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_dx_sigma_is_in_range(
                      r1_obj20_b.radar1_obj20_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_vx_sigma_decode(r1_obj20_b.radar1_obj20_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_vx_sigma_is_in_range(
                      r1_obj20_b.radar1_obj20_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_dy_sigma_decode(r1_obj20_b.radar1_obj20_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_dy_sigma_is_in_range(
                      r1_obj20_b.radar1_obj20_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_w_class_decode(r1_obj20_b.radar1_obj20_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_w_class_is_in_range(
                      r1_obj20_b.radar1_obj20_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_class_decode(r1_obj20_b.radar1_obj20_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_class_is_in_range(r1_obj20_b.radar1_obj20_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_mess_bconsist_bit_encode(
                      r1_obj20_b.radar1_obj20_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj20_b_radar1_obj20_mess_bconsist_bit_is_in_range(
                      r1_obj20_b.radar1_obj20_mess_bconsist_bit);
              break;
            case 1496:
              bosch_xgu_corner_radar_radar1_obj21_b_t r1_obj21_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj21_b_unpack(&r1_obj21_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_vy_decode(r1_obj21_b.radar1_obj21_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_vy_is_in_range(r1_obj21_b.radar1_obj21_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_d_length_decode(r1_obj21_b.radar1_obj21_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_d_length_is_in_range(
                      r1_obj21_b.radar1_obj21_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_dz_decode(r1_obj21_b.radar1_obj21_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_dz_is_in_range(r1_obj21_b.radar1_obj21_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_moving_state_decode(
                      r1_obj21_b.radar1_obj21_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_moving_state_is_in_range(
                      r1_obj21_b.radar1_obj21_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_dx_sigma_decode(r1_obj21_b.radar1_obj21_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_dx_sigma_is_in_range(
                      r1_obj21_b.radar1_obj21_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_vx_sigma_decode(r1_obj21_b.radar1_obj21_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_vx_sigma_is_in_range(
                      r1_obj21_b.radar1_obj21_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_dy_sigma_decode(r1_obj21_b.radar1_obj21_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_dy_sigma_is_in_range(
                      r1_obj21_b.radar1_obj21_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_w_class_decode(r1_obj21_b.radar1_obj21_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_w_class_is_in_range(
                      r1_obj21_b.radar1_obj21_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_class_decode(r1_obj21_b.radar1_obj21_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_class_is_in_range(r1_obj21_b.radar1_obj21_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_mess_bconsist_bit_encode(
                      r1_obj21_b.radar1_obj21_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj21_b_radar1_obj21_mess_bconsist_bit_is_in_range(
                      r1_obj21_b.radar1_obj21_mess_bconsist_bit);
              break;
            case 1506:
              bosch_xgu_corner_radar_radar1_obj22_b_t r1_obj22_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj22_b_unpack(&r1_obj22_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_vy_decode(r1_obj22_b.radar1_obj22_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_vy_is_in_range(r1_obj22_b.radar1_obj22_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_d_length_decode(r1_obj22_b.radar1_obj22_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_d_length_is_in_range(
                      r1_obj22_b.radar1_obj22_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_dz_decode(r1_obj22_b.radar1_obj22_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_dz_is_in_range(r1_obj22_b.radar1_obj22_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_moving_state_decode(
                      r1_obj22_b.radar1_obj22_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_moving_state_is_in_range(
                      r1_obj22_b.radar1_obj22_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_dx_sigma_decode(r1_obj22_b.radar1_obj22_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_dx_sigma_is_in_range(
                      r1_obj22_b.radar1_obj22_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_vx_sigma_decode(r1_obj22_b.radar1_obj22_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_vx_sigma_is_in_range(
                      r1_obj22_b.radar1_obj22_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_dy_sigma_decode(r1_obj22_b.radar1_obj22_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_dy_sigma_is_in_range(
                      r1_obj22_b.radar1_obj22_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_w_class_decode(r1_obj22_b.radar1_obj22_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_w_class_is_in_range(
                      r1_obj22_b.radar1_obj22_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_class_decode(r1_obj22_b.radar1_obj22_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_class_is_in_range(r1_obj22_b.radar1_obj22_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_mess_bconsist_bit_encode(
                      r1_obj22_b.radar1_obj22_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj22_b_radar1_obj22_mess_bconsist_bit_is_in_range(
                      r1_obj22_b.radar1_obj22_mess_bconsist_bit);
              break;
            case 1516:
              bosch_xgu_corner_radar_radar1_obj23_b_t r1_obj23_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj23_b_unpack(&r1_obj23_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_vy_decode(r1_obj23_b.radar1_obj23_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_vy_is_in_range(r1_obj23_b.radar1_obj23_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_d_length_decode(r1_obj23_b.radar1_obj23_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_d_length_is_in_range(
                      r1_obj23_b.radar1_obj23_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_dz_decode(r1_obj23_b.radar1_obj23_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_dz_is_in_range(r1_obj23_b.radar1_obj23_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_moving_state_decode(
                      r1_obj23_b.radar1_obj23_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_moving_state_is_in_range(
                      r1_obj23_b.radar1_obj23_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_dx_sigma_decode(r1_obj23_b.radar1_obj23_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_dx_sigma_is_in_range(
                      r1_obj23_b.radar1_obj23_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_vx_sigma_decode(r1_obj23_b.radar1_obj23_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_vx_sigma_is_in_range(
                      r1_obj23_b.radar1_obj23_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_dy_sigma_decode(r1_obj23_b.radar1_obj23_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_dy_sigma_is_in_range(
                      r1_obj23_b.radar1_obj23_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_w_class_decode(r1_obj23_b.radar1_obj23_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_w_class_is_in_range(
                      r1_obj23_b.radar1_obj23_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_class_decode(r1_obj23_b.radar1_obj23_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_class_is_in_range(r1_obj23_b.radar1_obj23_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_mess_bconsist_bit_encode(
                      r1_obj23_b.radar1_obj23_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj23_b_radar1_obj23_mess_bconsist_bit_is_in_range(
                      r1_obj23_b.radar1_obj23_mess_bconsist_bit);
              break;
            case 1526:
              bosch_xgu_corner_radar_radar1_obj24_b_t r1_obj24_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj24_b_unpack(&r1_obj24_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_vy_decode(r1_obj24_b.radar1_obj24_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_vy_is_in_range(r1_obj24_b.radar1_obj24_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_d_length_decode(r1_obj24_b.radar1_obj24_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_d_length_is_in_range(
                      r1_obj24_b.radar1_obj24_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_dz_decode(r1_obj24_b.radar1_obj24_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_dz_is_in_range(r1_obj24_b.radar1_obj24_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_moving_state_decode(
                      r1_obj24_b.radar1_obj24_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_moving_state_is_in_range(
                      r1_obj24_b.radar1_obj24_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_dx_sigma_decode(r1_obj24_b.radar1_obj24_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_dx_sigma_is_in_range(
                      r1_obj24_b.radar1_obj24_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_vx_sigma_decode(r1_obj24_b.radar1_obj24_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_vx_sigma_is_in_range(
                      r1_obj24_b.radar1_obj24_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_dy_sigma_decode(r1_obj24_b.radar1_obj24_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_dy_sigma_is_in_range(
                      r1_obj24_b.radar1_obj24_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_w_class_decode(r1_obj24_b.radar1_obj24_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_w_class_is_in_range(
                      r1_obj24_b.radar1_obj24_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_class_decode(r1_obj24_b.radar1_obj24_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_class_is_in_range(r1_obj24_b.radar1_obj24_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_mess_bconsist_bit_encode(
                      r1_obj24_b.radar1_obj24_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj24_b_radar1_obj24_mess_bconsist_bit_is_in_range(
                      r1_obj24_b.radar1_obj24_mess_bconsist_bit);
              break;
            case 1536:
              bosch_xgu_corner_radar_radar1_obj25_b_t r1_obj25_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj25_b_unpack(&r1_obj25_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_vy_decode(r1_obj25_b.radar1_obj25_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_vy_is_in_range(r1_obj25_b.radar1_obj25_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_d_length_decode(r1_obj25_b.radar1_obj25_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_d_length_is_in_range(
                      r1_obj25_b.radar1_obj25_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_dz_decode(r1_obj25_b.radar1_obj25_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_dz_is_in_range(r1_obj25_b.radar1_obj25_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_moving_state_decode(
                      r1_obj25_b.radar1_obj25_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_moving_state_is_in_range(
                      r1_obj25_b.radar1_obj25_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_dx_sigma_decode(r1_obj25_b.radar1_obj25_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_dx_sigma_is_in_range(
                      r1_obj25_b.radar1_obj25_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_vx_sigma_decode(r1_obj25_b.radar1_obj25_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_vx_sigma_is_in_range(
                      r1_obj25_b.radar1_obj25_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_dy_sigma_decode(r1_obj25_b.radar1_obj25_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_dy_sigma_is_in_range(
                      r1_obj25_b.radar1_obj25_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_w_class_decode(r1_obj25_b.radar1_obj25_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_w_class_is_in_range(
                      r1_obj25_b.radar1_obj25_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_class_decode(r1_obj25_b.radar1_obj25_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_class_is_in_range(r1_obj25_b.radar1_obj25_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_mess_bconsist_bit_encode(
                      r1_obj25_b.radar1_obj25_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj25_b_radar1_obj25_mess_bconsist_bit_is_in_range(
                      r1_obj25_b.radar1_obj25_mess_bconsist_bit);
              break;
            case 1546:
              bosch_xgu_corner_radar_radar1_obj26_b_t r1_obj26_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj26_b_unpack(&r1_obj26_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_vy_decode(r1_obj26_b.radar1_obj26_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_vy_is_in_range(r1_obj26_b.radar1_obj26_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_d_length_decode(r1_obj26_b.radar1_obj26_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_d_length_is_in_range(
                      r1_obj26_b.radar1_obj26_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_dz_decode(r1_obj26_b.radar1_obj26_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_dz_is_in_range(r1_obj26_b.radar1_obj26_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_moving_state_decode(
                      r1_obj26_b.radar1_obj26_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_moving_state_is_in_range(
                      r1_obj26_b.radar1_obj26_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_dx_sigma_decode(r1_obj26_b.radar1_obj26_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_dx_sigma_is_in_range(
                      r1_obj26_b.radar1_obj26_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_vx_sigma_decode(r1_obj26_b.radar1_obj26_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_vx_sigma_is_in_range(
                      r1_obj26_b.radar1_obj26_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_dy_sigma_decode(r1_obj26_b.radar1_obj26_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_dy_sigma_is_in_range(
                      r1_obj26_b.radar1_obj26_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_w_class_decode(r1_obj26_b.radar1_obj26_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_w_class_is_in_range(
                      r1_obj26_b.radar1_obj26_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_class_decode(r1_obj26_b.radar1_obj26_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_class_is_in_range(r1_obj26_b.radar1_obj26_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_mess_bconsist_bit_encode(
                      r1_obj26_b.radar1_obj26_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj26_b_radar1_obj26_mess_bconsist_bit_is_in_range(
                      r1_obj26_b.radar1_obj26_mess_bconsist_bit);
              break;
            case 1556:
              bosch_xgu_corner_radar_radar1_obj27_b_t r1_obj27_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj27_b_unpack(&r1_obj27_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_vy_decode(r1_obj27_b.radar1_obj27_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_vy_is_in_range(r1_obj27_b.radar1_obj27_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_d_length_decode(r1_obj27_b.radar1_obj27_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_d_length_is_in_range(
                      r1_obj27_b.radar1_obj27_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_dz_decode(r1_obj27_b.radar1_obj27_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_dz_is_in_range(r1_obj27_b.radar1_obj27_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_moving_state_decode(
                      r1_obj27_b.radar1_obj27_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_moving_state_is_in_range(
                      r1_obj27_b.radar1_obj27_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_dx_sigma_decode(r1_obj27_b.radar1_obj27_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_dx_sigma_is_in_range(
                      r1_obj27_b.radar1_obj27_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_vx_sigma_decode(r1_obj27_b.radar1_obj27_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_vx_sigma_is_in_range(
                      r1_obj27_b.radar1_obj27_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_dy_sigma_decode(r1_obj27_b.radar1_obj27_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_dy_sigma_is_in_range(
                      r1_obj27_b.radar1_obj27_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_w_class_decode(r1_obj27_b.radar1_obj27_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_w_class_is_in_range(
                      r1_obj27_b.radar1_obj27_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_class_decode(r1_obj27_b.radar1_obj27_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_class_is_in_range(r1_obj27_b.radar1_obj27_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_mess_bconsist_bit_encode(
                      r1_obj27_b.radar1_obj27_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj27_b_radar1_obj27_mess_bconsist_bit_is_in_range(
                      r1_obj27_b.radar1_obj27_mess_bconsist_bit);
              break;
            case 1566:
              bosch_xgu_corner_radar_radar1_obj28_b_t r1_obj28_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj28_b_unpack(&r1_obj28_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_vy_decode(r1_obj28_b.radar1_obj28_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_vy_is_in_range(r1_obj28_b.radar1_obj28_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_d_length_decode(r1_obj28_b.radar1_obj28_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_d_length_is_in_range(
                      r1_obj28_b.radar1_obj28_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_dz_decode(r1_obj28_b.radar1_obj28_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_dz_is_in_range(r1_obj28_b.radar1_obj28_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_moving_state_decode(
                      r1_obj28_b.radar1_obj28_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_moving_state_is_in_range(
                      r1_obj28_b.radar1_obj28_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_dx_sigma_decode(r1_obj28_b.radar1_obj28_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_dx_sigma_is_in_range(
                      r1_obj28_b.radar1_obj28_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_vx_sigma_decode(r1_obj28_b.radar1_obj28_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_vx_sigma_is_in_range(
                      r1_obj28_b.radar1_obj28_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_dy_sigma_decode(r1_obj28_b.radar1_obj28_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_dy_sigma_is_in_range(
                      r1_obj28_b.radar1_obj28_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_w_class_decode(r1_obj28_b.radar1_obj28_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_w_class_is_in_range(
                      r1_obj28_b.radar1_obj28_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_class_decode(r1_obj28_b.radar1_obj28_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_class_is_in_range(r1_obj28_b.radar1_obj28_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_mess_bconsist_bit_encode(
                      r1_obj28_b.radar1_obj28_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj28_b_radar1_obj28_mess_bconsist_bit_is_in_range(
                      r1_obj28_b.radar1_obj28_mess_bconsist_bit);
              break;
            case 1576:
              bosch_xgu_corner_radar_radar1_obj29_b_t r1_obj29_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj29_b_unpack(&r1_obj29_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_vy_decode(r1_obj29_b.radar1_obj29_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_vy_is_in_range(r1_obj29_b.radar1_obj29_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_d_length_decode(r1_obj29_b.radar1_obj29_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_d_length_is_in_range(
                      r1_obj29_b.radar1_obj29_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_dz_decode(r1_obj29_b.radar1_obj29_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_dz_is_in_range(r1_obj29_b.radar1_obj29_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_moving_state_decode(
                      r1_obj29_b.radar1_obj29_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_moving_state_is_in_range(
                      r1_obj29_b.radar1_obj29_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_dx_sigma_decode(r1_obj29_b.radar1_obj29_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_dx_sigma_is_in_range(
                      r1_obj29_b.radar1_obj29_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_vx_sigma_decode(r1_obj29_b.radar1_obj29_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_vx_sigma_is_in_range(
                      r1_obj29_b.radar1_obj29_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_dy_sigma_decode(r1_obj29_b.radar1_obj29_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_dy_sigma_is_in_range(
                      r1_obj29_b.radar1_obj29_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_w_class_decode(r1_obj29_b.radar1_obj29_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_w_class_is_in_range(
                      r1_obj29_b.radar1_obj29_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_class_decode(r1_obj29_b.radar1_obj29_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_class_is_in_range(r1_obj29_b.radar1_obj29_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_mess_bconsist_bit_encode(
                      r1_obj29_b.radar1_obj29_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj29_b_radar1_obj29_mess_bconsist_bit_is_in_range(
                      r1_obj29_b.radar1_obj29_mess_bconsist_bit);
              break;
            case 1586:
              bosch_xgu_corner_radar_radar1_obj30_b_t r1_obj30_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj30_b_unpack(&r1_obj30_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_vy_decode(r1_obj30_b.radar1_obj30_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_vy_is_in_range(r1_obj30_b.radar1_obj30_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_d_length_decode(r1_obj30_b.radar1_obj30_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_d_length_is_in_range(
                      r1_obj30_b.radar1_obj30_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_dz_decode(r1_obj30_b.radar1_obj30_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_dz_is_in_range(r1_obj30_b.radar1_obj30_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_moving_state_decode(
                      r1_obj30_b.radar1_obj30_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_moving_state_is_in_range(
                      r1_obj30_b.radar1_obj30_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_dx_sigma_decode(r1_obj30_b.radar1_obj30_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_dx_sigma_is_in_range(
                      r1_obj30_b.radar1_obj30_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_vx_sigma_decode(r1_obj30_b.radar1_obj30_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_vx_sigma_is_in_range(
                      r1_obj30_b.radar1_obj30_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_dy_sigma_decode(r1_obj30_b.radar1_obj30_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_dy_sigma_is_in_range(
                      r1_obj30_b.radar1_obj30_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_w_class_decode(r1_obj30_b.radar1_obj30_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_w_class_is_in_range(
                      r1_obj30_b.radar1_obj30_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_class_decode(r1_obj30_b.radar1_obj30_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_class_is_in_range(r1_obj30_b.radar1_obj30_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_mess_bconsist_bit_encode(
                      r1_obj30_b.radar1_obj30_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj30_b_radar1_obj30_mess_bconsist_bit_is_in_range(
                      r1_obj30_b.radar1_obj30_mess_bconsist_bit);
              break;
            case 1596:
              bosch_xgu_corner_radar_radar1_obj31_b_t r1_obj31_b;
              unpack_return = bosch_xgu_corner_radar_radar1_obj31_b_unpack(&r1_obj31_b, can_data, size_of_msg);
              all_object_info.vy_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_vy_decode(r1_obj31_b.radar1_obj31_vy);
              all_object_info.vy_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_vy_is_in_range(r1_obj31_b.radar1_obj31_vy);
              all_object_info.d_length_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_d_length_decode(r1_obj31_b.radar1_obj31_d_length);
              all_object_info.d_length_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_d_length_is_in_range(
                      r1_obj31_b.radar1_obj31_d_length);
              all_object_info.dz_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_dz_decode(r1_obj31_b.radar1_obj31_dz);
              all_object_info.dz_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_dz_is_in_range(r1_obj31_b.radar1_obj31_dz);
              all_object_info.moving_state_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_moving_state_decode(
                      r1_obj31_b.radar1_obj31_moving_state);
              all_object_info.moving_state_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_moving_state_is_in_range(
                      r1_obj31_b.radar1_obj31_moving_state);
              all_object_info.dx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_dx_sigma_decode(r1_obj31_b.radar1_obj31_dx_sigma);
              all_object_info.dx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_dx_sigma_is_in_range(
                      r1_obj31_b.radar1_obj31_dx_sigma);
              all_object_info.vx_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_vx_sigma_decode(r1_obj31_b.radar1_obj31_vx_sigma);
              all_object_info.vx_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_vx_sigma_is_in_range(
                      r1_obj31_b.radar1_obj31_vx_sigma);
              all_object_info.dy_sigma_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_dy_sigma_decode(r1_obj31_b.radar1_obj31_dy_sigma);
              all_object_info.dy_sigma_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_dy_sigma_is_in_range(
                      r1_obj31_b.radar1_obj31_dy_sigma);
              all_object_info.w_class_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_w_class_decode(r1_obj31_b.radar1_obj31_w_class);
              all_object_info.w_class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_w_class_is_in_range(
                      r1_obj31_b.radar1_obj31_w_class);
              all_object_info.class_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_class_decode(r1_obj31_b.radar1_obj31_class);
              all_object_info.class_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_class_is_in_range(r1_obj31_b.radar1_obj31_class);
              all_object_info.mess_bconsist_bit_decoded =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_mess_bconsist_bit_encode(
                      r1_obj31_b.radar1_obj31_mess_bconsist_bit);
              all_object_info.mess_bconsist_bit_is_in_range =
                  bosch_xgu_corner_radar_radar1_obj31_b_radar1_obj31_mess_bconsist_bit_is_in_range(
                      r1_obj31_b.radar1_obj31_mess_bconsist_bit);
              break;
          }
          all_object_info.timestamp = time;
          all_object_info.radar_number = radar_num;
          all_object_info.object_number = obj_num;
          memcpy(serialized_all_object_info, &radar_info, sizeof(all_object_info));
          (raw_obj_data_msg.obj_info)
              .insert((raw_obj_data_msg.obj_info).begin(), std::begin(serialized_all_object_info),
                      std::end(serialized_all_object_info));
          raw_obj_data_pub.publish(raw_obj_data_msg);
          break;
      }
    }
    canBusOff(hnd);
    canClose(hnd);

    ros::spinOnce();
  }
  return 0;
}
