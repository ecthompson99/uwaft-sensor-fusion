#include <canlib.h>
#include "ext_log_data.h"
#include "mobileye_struct.h"
#define canDRIVER_SILENT 1
#define SIZE_OF_MSG 8
// #define canOPEN_EXCLUSIVE 0x0008

static void check(char* id, canStatus stat) {
  if (stat != canOK) {
    char buf[50];
    buf[0] = '\0';
    // Retreive informational text about the status code
    canGetErrorText(stat, buf, sizeof(buf));
    printf("%s: failed, stat=%d (%s)\n", id, (int)stat, buf);
  }
}  // check

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_tx_rx_CH4");
    ros::NodeHandle can_tx_rx_CH4_handle;

    Mobileye_RX mobeye_rx = Mobileye_RX(&can_tx_rx_CH4_handle);
    SensorDiagnostics sens_diag = SensorDiagnostics(&can_tx_rx_CH4_handle);

    common::mobileye_object_data me_obj; //Processed values for object detection
    common::raw_lane_data me_raw_lane; //Processed lane values
    common::sensor_diagnostic_data_msg diag_data; // sensor diagnostics

    Mobileye_RX::mobileye_object mobileye_obj;
    Mobileye_RX::mobileye_lane mobileye_lane;
    Mobileye_RX::mobileye_diagnostics mobileye_diag;

    ros::Time mem1 = ros::Time::now();

    mobileye_obj.channel_number = 3; 

    long int id;
    unsigned int dlc; 
    unsigned int flag; 
    unsigned long time;
    uint8_t case_num = 0;
    int obj_num = -1; // 0 to 10 is valid
    uint8_t can_data[8] = {0};

    int unpack_return = -1;  // 0 is successful, negative error code

    canInitializeLibrary();
    canHandle hnd;
    canStatus stat;

    hnd = canOpenChannel(mobileye_obj.channel_number, canOPEN_EXCLUSIVE);
    std::cout << "CN: " << +mobileye_obj.channel_number << std::endl;
    if (hnd < 0) {
      char msg[64];
      canGetErrorText((canStatus)hnd, msg, sizeof(msg));
      fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
      exit(1);
    }
    stat = canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_SILENT);
    stat = canBusOn(hnd);

    while (ros::ok()) {
      ros::Time now = ros::Time::now();

      if (now.toSec() - mem1.toSec() > 0.5) {  // 0.1
        obj_num = 0;                           // for testing purposes (see mobileye_struct.h)

        // Obstacle Frame A
        stat = canReadSpecific(hnd, 0x739, &can_data, &dlc, &flag, &time);
        if (stat == canOK) {
          ext_log_data_obstacle_data_a_t frame_a_unpacked;
          unpack_return = ext_log_data_obstacle_data_a_unpack(&frame_a_unpacked, can_data, SIZE_OF_MSG);

          mobileye_obj.obstacle_cut_in_cut_out_decode =
              ext_log_data_obstacle_data_a_cut_in_and_out_decode(frame_a_unpacked.cut_in_and_out);
          mobileye_obj.obstacle_cut_in_cut_out_is_in_range =
              ext_log_data_obstacle_data_a_cut_in_and_out_is_in_range(frame_a_unpacked.cut_in_and_out);

          mobileye_obj.obstacle_valid_decode =
              ext_log_data_obstacle_data_a_obstacle_valid_decode(frame_a_unpacked.obstacle_valid);
          mobileye_obj.obstacle_valid_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_valid_is_in_range(frame_a_unpacked.obstacle_valid);

          mobileye_obj.obstacle_status_decode =
              ext_log_data_obstacle_data_a_obstacle_status_decode(frame_a_unpacked.obstacle_status);
          mobileye_obj.obstacle_status_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_status_is_in_range(frame_a_unpacked.obstacle_status);

          mobileye_obj.obstacle_type_decode =
              ext_log_data_obstacle_data_a_obstacle_type_decode(frame_a_unpacked.obstacle_type);
          mobileye_obj.obstacle_type_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_type_is_in_range(frame_a_unpacked.obstacle_type);

          mobileye_obj.obstacle_vel_x_decode =
              ext_log_data_obstacle_data_a_obstacle_vel_x_decode(frame_a_unpacked.obstacle_vel_x);
          mobileye_obj.obstacle_vel_x_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(frame_a_unpacked.obstacle_vel_x);

          mobileye_obj.obstacle_pos_y_decode =
              ext_log_data_obstacle_data_a_obstacle_pos_y_decode(frame_a_unpacked.obstacle_pos_y);
          mobileye_obj.obstacle_pos_y_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(frame_a_unpacked.obstacle_pos_y);

          mobileye_obj.obstacle_pos_x_decode =
              ext_log_data_obstacle_data_a_obstacle_pos_x_decode(frame_a_unpacked.obstacle_pos_x);
          mobileye_obj.obstacle_pos_x_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(frame_a_unpacked.obstacle_pos_x);

          mobileye_obj.obstacle_id_decode =
              ext_log_data_obstacle_data_a_obstacle_id_decode(frame_a_unpacked.obstacle_id);
          mobileye_obj.obstacle_id_is_in_range =
              ext_log_data_obstacle_data_a_obstacle_id_is_in_range(frame_a_unpacked.obstacle_id);

          me_obj.me_cut_in_cut_out[obj_num] = mobeye_rx.signal_in_range(
              mobileye_obj.obstacle_cut_in_cut_out_decode, mobileye_obj.obstacle_cut_in_cut_out_is_in_range);
          me_obj.me_valid[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_valid_decode, mobileye_obj.obstacle_valid_is_in_range);
          me_obj.me_status[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_status_decode, mobileye_obj.obstacle_status_is_in_range);
          me_obj.me_type[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_type_decode, mobileye_obj.obstacle_type_is_in_range);
          me_obj.me_id[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_id_decode, mobileye_obj.obstacle_id_is_in_range);
          me_obj.me_vx[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_vel_x_decode, mobileye_obj.obstacle_vel_x_is_in_range);
          me_obj.me_dy[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_pos_y_decode, mobileye_obj.obstacle_pos_y_is_in_range);
          me_obj.me_dx[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_pos_x_decode, mobileye_obj.obstacle_pos_x_is_in_range);

          std::cout << "Case 1 time: " << +time << std::endl;
          me_obj.me_timestamp = time;
        }

        // Obstacle frame B
        stat = canReadSpecific(hnd, 0x73A, &can_data, &dlc, &flag, &time);
        if (stat == canOK) {
          ext_log_data_obstacle_data_b_t frame_b_unpacked;
          unpack_return = ext_log_data_obstacle_data_b_unpack(&frame_b_unpacked, can_data, SIZE_OF_MSG);

          mobileye_obj.obstacle_lane_decode =
              ext_log_data_obstacle_data_b_obstacle_lane_decode(frame_b_unpacked.obstacle_lane);
          mobileye_obj.obstacle_lane_is_in_range =
              ext_log_data_obstacle_data_b_obstacle_lane_is_in_range(frame_b_unpacked.obstacle_lane);

          mobileye_obj.obstacle_cipv_flag_decode =
              ext_log_data_obstacle_data_b_cipv_flag_decode(frame_b_unpacked.cipv_flag);
          mobileye_obj.obstacle_cipv_flag_is_in_range =
              ext_log_data_obstacle_data_b_cipv_flag_is_in_range(frame_b_unpacked.cipv_flag);

          mobileye_obj.obstacle_age_decode =
              ext_log_data_obstacle_data_b_obstacle_age_decode(frame_b_unpacked.obstacle_age);
          mobileye_obj.obstacle_age_is_in_range =
              ext_log_data_obstacle_data_b_obstacle_age_is_in_range(frame_b_unpacked.obstacle_age);

          me_obj.me_lane[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_lane_decode, mobileye_obj.obstacle_lane_is_in_range);
          me_obj.me_cipv_flag[obj_num] = mobeye_rx.signal_in_range(mobileye_obj.obstacle_cipv_flag_decode,
                                                                   mobileye_obj.obstacle_cipv_flag_is_in_range);
          me_obj.me_age[obj_num] =
              mobeye_rx.signal_in_range(mobileye_obj.obstacle_age_decode, mobileye_obj.obstacle_age_is_in_range);
          std::cout << "Case 2" << std::endl;
        }

        // Obstacle frame C
        stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);
        if ((stat == canOK) && (id >= 0x73B)) {
          ext_log_data_obstacle_data_c_t frame_c_unpacked;
          unpack_return = ext_log_data_obstacle_data_c_unpack(&frame_c_unpacked, can_data, SIZE_OF_MSG);

          mobileye_obj.obstacle_accel_x_decode =
              ext_log_data_obstacle_data_c_object_accel_x_decode(frame_c_unpacked.object_accel_x);
          mobileye_obj.obstacle_accel_x_is_in_range =
              ext_log_data_obstacle_data_c_object_accel_x_is_in_range(frame_c_unpacked.object_accel_x);

          me_obj.me_ax[obj_num] = mobeye_rx.signal_in_range(mobileye_obj.obstacle_accel_x_decode,
                                                            mobileye_obj.obstacle_accel_x_is_in_range);
          std::cout << "Case 3" << std::endl;
        }

        // Left Lane Frame A
        stat = canReadSpecific(hnd, 0x766, &can_data, &dlc, &flag, &time);
        if (stat == canOK) {
          ext_log_data_lka_left_lane_a_t left_a_unpacked;
          unpack_return = ext_log_data_lka_left_lane_a_unpack(&left_a_unpacked, can_data, SIZE_OF_MSG);

          mobileye_lane.left_lane_type_decode =
              ext_log_data_lka_left_lane_a_lane_type_decode(left_a_unpacked.lane_type);
          mobileye_lane.left_lane_type_is_in_range =
              ext_log_data_lka_left_lane_a_lane_type_is_in_range(left_a_unpacked.lane_type);

          mobileye_lane.left_quality_decode = ext_log_data_lka_left_lane_a_quality_decode(left_a_unpacked.quality);
          mobileye_lane.left_quality_is_in_range =
              ext_log_data_lka_left_lane_a_quality_is_in_range(left_a_unpacked.quality);

          mobileye_lane.left_position_decode = ext_log_data_lka_left_lane_a_position_decode(left_a_unpacked.position);
          mobileye_lane.left_position_is_in_range =
              ext_log_data_lka_left_lane_a_position_is_in_range(left_a_unpacked.position);

          mobileye_lane.left_curvature_decode =
              ext_log_data_lka_left_lane_a_curvature_decode(left_a_unpacked.curvature);
          mobileye_lane.left_curvature_is_in_range =
              ext_log_data_lka_left_lane_a_curvature_is_in_range(left_a_unpacked.curvature);

          mobileye_lane.left_curvature_derivative_decode =
              ext_log_data_lka_left_lane_a_curvature_derivative_decode(left_a_unpacked.curvature_derivative);
          mobileye_lane.left_curvature_derivative_is_in_range =
              ext_log_data_lka_left_lane_a_curvature_derivative_is_in_range(left_a_unpacked.curvature_derivative);

          me_raw_lane.me_lane_position_L =
              mobeye_rx.signal_in_range(mobileye_lane.left_position_decode, mobileye_lane.left_position_is_in_range);
          me_raw_lane.me_lane_type_L =
              mobeye_rx.signal_in_range(mobileye_lane.left_lane_type_decode, mobileye_lane.left_lane_type_is_in_range);
          me_raw_lane.me_lane_quality_L =
              mobeye_rx.signal_in_range(mobileye_lane.left_quality_decode, mobileye_lane.left_quality_is_in_range);
          me_raw_lane.me_lane_curvature_L =
              mobeye_rx.signal_in_range(mobileye_lane.left_curvature_decode, mobileye_lane.left_curvature_is_in_range);
          me_raw_lane.me_lane_curvature_derivative_L = mobeye_rx.signal_in_range(
              mobileye_lane.left_curvature_derivative_decode, mobileye_lane.left_curvature_derivative_is_in_range);

          diag_data.quality_L =
              mobeye_rx.signal_in_range(mobileye_lane.left_quality_decode, mobileye_lane.left_quality_is_in_range);
          std::cout << "Case 4" << std::endl;
        }

        // Left Lane Frame B
        stat = canReadSpecific(hnd, 0x767, &can_data, &dlc, &flag, &time);
        if (stat == canOK) {
          ext_log_data_lka_left_lane_b_t left_b_unpacked;
          unpack_return = ext_log_data_lka_left_lane_b_unpack(&left_b_unpacked, can_data, SIZE_OF_MSG);

          mobileye_lane.left_heading_angle_decode =
              ext_log_data_lka_left_lane_b_heading_angle_decode(left_b_unpacked.heading_angle);
          mobileye_lane.left_heading_angle_is_in_range =
              ext_log_data_lka_left_lane_b_heading_angle_is_in_range(left_b_unpacked.heading_angle);

          me_raw_lane.me_lane_heading_angle_L = mobeye_rx.signal_in_range(mobileye_lane.left_heading_angle_decode,
                                                                          mobileye_lane.left_heading_angle_is_in_range);
          std::cout << "Case 5" << std::endl;
        }

        // Right Lane Frame A
        stat = canReadSpecific(hnd, 0x768, &can_data, &dlc, &flag, &time);
        if (stat == canOK) {
          ext_log_data_lka_right_lane_a_t right_a_unpacked;
          unpack_return = ext_log_data_lka_right_lane_a_unpack(&right_a_unpacked, can_data, SIZE_OF_MSG);

          mobileye_lane.right_lane_type_decode =
              ext_log_data_lka_right_lane_a_lane_type_decode(right_a_unpacked.lane_type);
          mobileye_lane.right_lane_type_is_in_range =
              ext_log_data_lka_right_lane_a_lane_type_is_in_range(right_a_unpacked.lane_type);

          mobileye_lane.right_quality_decode = ext_log_data_lka_right_lane_a_quality_decode(right_a_unpacked.quality);
          mobileye_lane.right_quality_is_in_range =
              ext_log_data_lka_right_lane_a_quality_is_in_range(right_a_unpacked.quality);

          mobileye_lane.right_position_decode =
              ext_log_data_lka_right_lane_a_position_decode(right_a_unpacked.position);
          mobileye_lane.right_position_is_in_range =
              ext_log_data_lka_right_lane_a_position_is_in_range(right_a_unpacked.position);

          mobileye_lane.right_curvature_decode =
              ext_log_data_lka_right_lane_a_curvature_decode(right_a_unpacked.curvature);
          mobileye_lane.right_curvature_is_in_range =
              ext_log_data_lka_right_lane_a_curvature_is_in_range(right_a_unpacked.curvature);

          mobileye_lane.right_curvature_derivative_decode =
              ext_log_data_lka_right_lane_a_curvature_derivative_decode(right_a_unpacked.curvature_derivative);
          mobileye_lane.right_curvature_derivative_is_in_range =
              ext_log_data_lka_right_lane_a_curvature_derivative_is_in_range(right_a_unpacked.curvature_derivative);

          me_raw_lane.me_lane_type_R = mobeye_rx.signal_in_range(mobileye_lane.right_lane_type_decode,
                                                                 mobileye_lane.right_lane_type_is_in_range);
          me_raw_lane.me_lane_quality_R =
              mobeye_rx.signal_in_range(mobileye_lane.right_quality_decode, mobileye_lane.right_quality_is_in_range);
          me_raw_lane.me_lane_position_R =
              mobeye_rx.signal_in_range(mobileye_lane.right_position_decode, mobileye_lane.right_position_is_in_range);
          me_raw_lane.me_lane_curvature_R = mobeye_rx.signal_in_range(mobileye_lane.right_curvature_decode,
                                                                      mobileye_lane.right_curvature_is_in_range);
          me_raw_lane.me_lane_curvature_derivative_R = mobeye_rx.signal_in_range(
              mobileye_lane.right_curvature_derivative_decode, mobileye_lane.right_curvature_derivative_is_in_range);

          diag_data.quality_R =
              mobeye_rx.signal_in_range(mobileye_lane.right_quality_decode, mobileye_lane.right_quality_is_in_range);
          std::cout << "Case 6" << std::endl;
        }

        // Right Lane Frame B
        stat = canReadSpecific(hnd, 0x769, &can_data, &dlc, &flag, &time);
        if (stat == canOK) {
          ext_log_data_lka_right_lane_b_t right_b_unpacked;
          unpack_return = ext_log_data_lka_right_lane_b_unpack(&right_b_unpacked, can_data, SIZE_OF_MSG);

          mobileye_lane.right_heading_angle_decode =
              ext_log_data_lka_right_lane_b_heading_angle_decode(right_b_unpacked.heading_angle);
          mobileye_lane.right_heading_angle_is_in_range =
              ext_log_data_lka_right_lane_b_heading_angle_is_in_range(right_b_unpacked.heading_angle);

          me_raw_lane.me_lane_heading_angle_R = mobeye_rx.signal_in_range(
              mobileye_lane.right_heading_angle_decode, mobileye_lane.right_heading_angle_is_in_range);
          std::cout << "Case 7" << std::endl;
        }

        // mobileye diagnostics 1792
        stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);
        if ((stat == canOK) && (id >= 0x700) && (id <= 0x736)) {
          ext_log_data_aws_display_t aws_display_unpacked;
          unpack_return = ext_log_data_aws_display_unpack(&aws_display_unpacked, can_data, SIZE_OF_MSG);

          mobileye_diag.headway_valid_decode =
              ext_log_data_aws_display_headway_valid_decode(aws_display_unpacked.headway_valid);
          mobileye_diag.headway_valid_is_in_range =
              ext_log_data_aws_display_headway_valid_is_in_range(aws_display_unpacked.headway_valid);

          mobileye_diag.failsafe_decode = ext_log_data_aws_display_failsafe_decode(aws_display_unpacked.failsafe);
          mobileye_diag.failsafe_is_in_range =
              ext_log_data_aws_display_failsafe_is_in_range(aws_display_unpacked.failsafe);

          mobileye_diag.maintenance_decode =
              ext_log_data_aws_display_maintenance_decode(aws_display_unpacked.maintenance);
          mobileye_diag.maintenance_is_in_range =
              ext_log_data_aws_display_maintenance_is_in_range(aws_display_unpacked.maintenance);

          diag_data.headway_valid =
              mobeye_rx.signal_in_range(mobileye_diag.headway_valid_decode, mobileye_diag.headway_valid_is_in_range);
          diag_data.maintenance =
              mobeye_rx.signal_in_range(mobileye_diag.maintenance_decode, mobileye_diag.maintenance_is_in_range);
          diag_data.failsafe =
              mobeye_rx.signal_in_range(mobileye_diag.failsafe_decode, mobileye_diag.failsafe_is_in_range);
          diag_data.timestamp = time;
          std::cout << "Case 8" << std::endl;
          std::cout << "maintenance: " << +diag_data.maintenance << std::endl;
        }

        // validate Mobileye
        ros::ServiceClient client_ch4;
        common::sensor_diagnostic_flag_CH4 srv_ch4;

        srv_ch4.request.mobileye = sens_diag.validate_mobileye(diag_data);
        if (srv_ch4.request.mobileye) {
            std::cout << "Valid Ch4 service call" << std::endl;
        }
        else{
        std::cout << "Invalid Ch4 service call" << std::endl;
        }

        mobeye_rx.mob_pub_obj.publish(me_obj);
        mobeye_rx.mob_pub_lane.publish(me_raw_lane);
        mobeye_rx.diag_pub.publish(diag_data);

        mem1 = now;
      }
      ros::spinOnce();
    }
    canBusOff(hnd);
    canClose(hnd);
    return 0;
}