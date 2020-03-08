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
