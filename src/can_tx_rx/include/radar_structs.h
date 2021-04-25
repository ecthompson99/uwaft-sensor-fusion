#include <canlib.h>
#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"
#include "ros/time.h"

#include "radar.h"
#include "sensor_diag.h"

#include "common/drive_ctrl_input_msg.h"
#include "common/radar_object_data.h"
#include "common/sensor_diagnostic_flag_CH2.h"

#define MESSAGE_BUFFER_SIZE 1000
#define TOPIC_TX "Front_Radar_CAN_Rx" // ouput radar info
#define TOPIC_DIAG "sensor_diagnostic_flags"
#define TOPIC_RX "drive_ctrl_input" // listen from vehicle speed
#define CH2_SERVICE "sensor_diagnostic_CH2"
#define SIZE_OF_MSG 8 

class Radar_RX{
  public:
    ros::NodeHandle* node_handle;
    ros::Publisher rad_pub;
    ros::Publisher diag_pub; 
    ros::Subscriber veh_sub;
    ros::ServiceClient client_ch2;

    Radar_RX(ros::NodeHandle* node_handle);
    
    struct dynamic_vehicle_info {
      double vehicle_speed;
      double steering_angle;
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
    void get_nums(long int id, int &case_n, int &radar_n, int &frame_n, int &obj_n, int &target_obj_n,
                  int channel_number);
    double signals_in_range(double val, bool cond);
    void get_static_veh_info(radar_input_mount_info_t &in_mount_info, radar_input_wheel_info_t &in_wheel_info, radar_input_veh_dim_t &in_veh_dim, int radar_num);
    void get_dynamic_veh_info(radar_input_veh_dyn_data_t &in_veh_dyn);
    uint8_t crc8bit_calculation(uint8_t can1670signals[7], int f_len);
    void clear_classes(common::radar_object_data &radar_obj, common::sensor_diagnostic_data_msg &diag_data,     Radar_RX::radar_diagnostic_response &diag_response, Radar_RX::radar_information &radar_info,Radar_RX::target_tracking_info &target_info, Radar_RX::object_tracking_info &object_info, uint8_t &tc_check, uint8_t &mc_check);
    void drive_ctrl_callback(const common::drive_ctrl_input_msg& recvd_data);
    Radar_RX::dynamic_vehicle_info vehicle_data;
};

