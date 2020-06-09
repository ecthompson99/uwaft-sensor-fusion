#include <canlib.h>

#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/ext_log_data.c"
#include "can_tx_rx/ext_log_data.h"

#include "common/mobileye_object_data_msg.h"

static const uint16_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;

void get_nums(int id, uint8_t &case_n) {
  if(id >=1824 && id <=1830){
    case_n = 1; //Traffic Sensor 
  } else if(id >= 1849 && id <= 1876 && id % 3 == 1){
    case_n = 2; //Obstacle A Frame
  } else if(id >= 1850 && id <= 1877 && id % 3 == 2){
    case_n = 3; //Obstacle B Frame
  } else if(id >= 1851 && id <= 1878 && id % 3 == 0){
    case_n = 4; //Obstacle C Frame
  } else{
    case_n = 0; 
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  ros::Publisher raw_obj_data_pub = can_tx_rx_CH4_handle.advertise<common::mobileye_object_data_msg>(
      "mobileye_object_data", TX_RX_MESSAGE_BUFFER_SIZE);

  common::mobileye_object_data_msg obj_data;

  canHandle hnd;

  canInitializeLibrary();

  hnd = canOpenChannel(3, canOPEN_EXCLUSIVE);

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
  uint8_t size_of_msg = 8;
  uint8_t frame_num = 0; //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
  //uint8_t obj_num = -1; //0 to 9 is accepted *Not sure how this will be tracked*
  uint8_t can_data[8] = {0};
  uint8_t serialized_all_object_info[sizeof(all_object_info)];
  
  all_object_info.channel_number = 4; 

  int unpack_return = -1;  // 0 is successful, negative error code

  while (ros::ok()) {
    canStatus stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);

    if (canOK == stat) {
      get_nums(id, frame_num);

      switch(frame_num){
        case 1:
          ext_log_data_tsr_t data_tsr; 
          unpack_return = ext_log_data_tsr_unpack(can_data,size_of_msg);

          all_object_info.no_entry_vision_status_decoded = ext_log_data_tsr_no_entry_vision_status_decode(data_tsr.no_entry_vision_status);
          all_object_info.no_entry_vision_status_is_in_range = ext_log_data_tsr_no_entry_vision_status_is_in_range(data_tsr.no_entry_vision_status);

          all_object_info.filter_type_decoded = ext_log_data_tsr_filter_type_decode(data_tsr.filter_type);
          all_object_info.filter_type_is_in_range = ext_log_data_tsr_filter_type_is_in_range(data_tsr.filter_type);

          all_object_info.sign_position_z_decoded = ext_log_data_tsr_sign_position_z_decode(data_tsr.sign_position_z);
          all_object_info.sign_position_z_is_in_range = ext_log_data_tsr_sign_position_z_is_in_range(data_tsr.sign_position_z);

          all_object_info.sign_position_y_decoded = ext_log_data_tsr_sign_position_y_decode(data_tsr.sign_position_y);
          all_object_info.sign_position_y_is_in_range = ext_log_data_tsr_sign_position_y_is_in_range(data_tsr.sign_position_y);

          all_object_info.sign_position_x_decoded = ext_log_data_tsr_sign_position_x_decode(data_tsr.sign_position_x);
          all_object_info.sign_position_x_is_in_range = ext_log_data_tsr_sign_position_x_is_in_range(data_tsr.sign_position_x);
          
          all_object_info.vision_only_supp_sign_type_decoded = ext_log_data_tsr_vision_only_supp_sign_type_decode(data_tsr.vision_only_supp_sign_type);
          all_object_info.vision_only_supp_sign_type_is_in_range = ext_log_data_tsr_vision_only_supp_sign_type_is_in_range(data_tsr.vision_only_supp_sign_type);

          all_object_info.vision_only_sign_type_decoded = ext_log_data_tsr_vision_only_sign_type_decode(data_tsr.vision_only_sign_type);
          all_object_info.vision_only_sign_type_is_in_range = ext_log_data_tsr_vision_only_sign_type_is_in_range(data_tsr.vision_only_sign_type);

        break;
        case 2:
          ext_log_data_obstacle_data_a_t obj_frame_a; 
          unpack_return = ext_log_data_obstacle_data_a_unpack(&obj_frame_a,can_data,size_of_msg); 
          
          all_object_info.Cut_in_and_Out_decoded = ext_log_data_obstacle_data_a_cut_in_and_out_decode(obj_frame_a.cut_in_and_out);
          all_object_info.Cut_in_and_Out_is_in_range = ext_log_data_obstacle_data_a_cut_in_and_out_is_in_range(obj_frame_a.cut_in_and_out);

          all_object_info.blinker_info_decoded = ext_log_data_obstacle_data_a_blinker_info_decode(obj_frame_a.blinker_info);
          all_object_info.blinker_info_is_in_range = ext_log_data_obstacle_data_a_blinker_info_is_in_range(obj_frame_a.blinker_info);

          all_object_info.obstacle_valid_decoded = ext_log_data_obstacle_data_a_obstacle_valid_decode(obj_frame_a.obstacle_valid);
          all_object_info.obstacle_valid_is_in_range = ext_log_data_obstacle_data_a_obstacle_valid_is_in_range(obj_frame_a.obstacle_valid);

          all_object_info.obstacle_brake_lights_decoded = ext_log_data_obstacle_data_a_obstacle_brake_lights_decode(obj_frame_a.obstacle_brake_lights);
          all_object_info.obstacle_brake_lights_is_in_range = ext_log_data_obstacle_data_a_obstacle_brake_lights_is_in_range(obj_frame_a.obstacle_brake_lights);

          all_object_info.obstacle_status_decoded = ext_log_data_obstacle_data_a_obstacle_status_decode(obj_frame_a.obstacle_status);
          all_object_info.obstacle_status_is_in_range = ext_log_data_obstacle_data_a_obstacle_status_is_in_range(obj_frame_a.obstacle_status);

          all_object_info.obstacle_type_decoded = ext_log_data_obstacle_data_a_obstacle_type_decode(obj_frame_a.obstacle_type);
          all_object_info.obstacle_type_is_in_range = ext_log_data_obstacle_data_a_obstacle_type_is_in_range(obj_frame_a.obstacle_type);

          all_object_info.obstacle_vel_x_decoded = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(obj_frame_a.obstacle_vel_x);
          all_object_info.obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(obj_frame_a.obstacle_vel_x);

          all_object_info.obstacle_pos_y_decoded = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(obj_frame_a.obstacle_pos_y);
          all_object_info.obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(obj_frame_a.obstacle_pos_y);

          all_object_info.obstacle_pos_x_decoded = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(obj_frame_a.obstacle_pos_x);
          all_object_info.obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(obj_frame_a.obstacle_pos_x);

          all_object_info.obstacle_type_decoded = ext_log_data_obstacle_data_a_obstacle_type_decode(obj_frame_a.obstacle_type);
          all_object_info.obstacle_type_is_in_range = ext_log_data_obstacle_data_a_obstacle_type_x_is_in_range(obj_frame_a.obstacle_type);
        
        break; 
        case 3:
          ext_log_data_obstacle_data_b_t obj_frame_b; 
          unpack_return = ext_log_data_obstacle_data_b_unpack(&obj_frame_b,can_data,size_of_msg); 

          all_object_info.matched_radar_id_decoded = ext_log_data_obstacle_data_b_matched_radar_id_decode(obj_frame_b.matched_radar_id);
          all_object_info.matched_radar_id_is_in_range = ext_log_data_obstacle_data_b_matched_radar_id_is_in_range(obj_frame_b.matched_radar_id);

          all_object_info.radar_match_confidence_decoded = ext_log_data_obstacle_data_b_radar_match_confidence_decode(obj_frame_b.radar_match_confidence);
          all_object_info.radar_match_confidence_is_in_range = ext_log_data_obstacle_data_b_radar_match_confidence_is_in_range(obj_frame_b.radar_match_confidence);

          all_object_info.obstacle_radar_vel_x_decoded = ext_log_data_obstacle_data_b_obstacle_radar_vel_x_decode(obj_frame_b.obstacle_radar_vel_x);
          all_object_info.obstacle_radar_vel_x_is_in_range = ext_log_data_obstacle_data_b_obstacle_radar_vel_x_is_in_range(obj_frame_b.obstacle_radar_vel_x);

          all_object_info.radar_pos_x_decoded = ext_log_data_obstacle_data_b_radar_pos_x_decode(obj_frame_b.radar_pos_x);
          all_object_info.radar_pos_x_is_in_range = ext_log_data_obstacle_data_b_radar_pos_x_is_in_range(obj_frame_b.radar_pos_x);

          all_object_info.cipv_flag_decoded = ext_log_data_obstacle_data_b_cipv_flag_decode(obj_frame_b.cipv_flag);
          all_object_info.cipv_flag_is_in_range = ext_log_data_obstacle_data_b_cipv_flag_is_in_range(obj_frame_b. cipv_flag);
          
          all_object_info.obstacle_lane_decoded = ext_log_data_obstacle_data_b_obstacle_lane_decode(obj_frame_b.obstacle_lane);
          all_object_info.obstacle_lane_is_in_range = ext_log_data_obstacle_data_b_obstacle_lane_is_in_range(obj_frame_b. obstacle_lane);

          all_object_info.obstacle_age_decoded = ext_log_data_obstacle_data_b_obstacle_age_decode(obj_frame_b.obstacle_age);
          all_object_info.obstacle_age_is_in_range = ext_log_data_obstacle_data_b_obstacle_age_is_in_range(obj_frame_b. obstacle_age);

          all_object_info.obstacle_width_decoded = ext_log_data_obstacle_data_b_obstacle_width_decode(obj_frame_b.obstacle_width);
          all_object_info.obstacle_width_is_in_range = ext_log_data_obstacle_data_b_obstacle_width_is_in_range(obj_frame_b. obstacle_width);

          all_object_info.obstacle_length_decoded = ext_log_data_obstacle_data_b_obstacle_length_decode(obj_frame_b.obstacle_length);
          all_object_info.obstacle_length_is_in_range = ext_log_data_obstacle_data_b_obstacle_length_is_in_range(obj_frame_b.obstacle_length);
        
        break; 
        case 4:
          ext_log_data_obstacle_data_c_t obj_frame_c; 
          unpack_return = ext_log_data_obstacle_data_c_unpack(&obj_frame_c,can_data,size_of_msg);

          all_object_info.object_accel_x_decoded = ext_log_data_obstacle_data_c_object_accel_x_decode(obj_frame_c.object_accel_x);
          all_object_info.object_accel_x_is_in_range = ext_log_data_obstacle_data_c_object_accel_x_is_in_range(obj_frame_c.object_accel_x);

          all_object_info.obstacle_angle_decoded = ext_log_data_obstacle_data_c_obstacle_angle_decode(obj_frame_c.obstacle_angle);
          all_object_info.obstacle_angle_is_in_range = ext_log_data_obstacle_data_c_obstacle_angle_is_in_range(obj_frame_c.obstacle_angle);

          all_object_info.obstacle_replaced_decoded = ext_log_data_obstacle_data_c_obstacle_replaced_decode(obj_frame_c.obstacle_replaced);
          all_object_info.obstacle_replaced_is_in_range = ext_log_data_obstacle_data_c_obstacle_replaced_is_in_range(obj_frame_c.obstacle_replaced);

          all_object_info.obstacle_scale_change_decoded = ext_log_data_obstacle_data_c_obstacle_scale_change_decode(obj_frame_c.obstacle_scale_change);
          all_object_info.obstacle_scale_change_is_in_range = ext_log_data_obstacle_data_c_obstacle_scale_change_is_in_range(obj_frame_c.obstacle_scale_change);

          all_object_info.obstacle_angle_rate_decoded = ext_log_data_obstacle_data_c_obstacle_angle_rate_decode(obj_frame_c.obstacle_angle_rate);
          all_object_info.obstacle_angle_rate_is_in_range = ext_log_data_obstacle_data_c_obstacle_angle_rate_is_in_range(obj_frame_c.obstacle_angle_rate);

        break; 
      }
    }

    ros::spinOnce();
  }
  
  canBusOff(hnd);
  canClose(hnd);

  return 0;
}

