#include "radar_structs.h"
#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8
#include <ros/ros.h>

Radar_RX::Radar_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
    rad_pub = node_handle->advertise<common::radar_object_data>(TOPIC_RX,MESSAGE_BUFFER_SIZE);
    diag_pub = node_handle->advertise<common::sensor_diagnostic_data_msg>(TOPIC_DIAG, MESSAGE_BUFFER_SIZE);
};
void Radar_RX::get_nums(long int id, int &case_n, int &radar_n, int &frame_n, int &obj_n, int &target_obj_n,
                        int channel_number) {
  if (id == 1985 || id == 1958 || id == 1879 || id == 1957) {
    case_n = 1;  // diag responses and requests
  } else if (id > 1604 && id < 1659) {
    case_n = 2;  // target A and B frames (?) the IDs are incorrectly calculated from the dbc
  } else if (id == 1665 || id == 1667 || id == 1280 || id == 1282 || id == 1670 || id == 1672) {
    case_n = 3;  // ender, starter, and statuses messages
  } else if (id > 1284 && id < 1597) {
    case_n = 4;  // radar A and B object frames
  } else {
    case_n = 0;  // faulted
  }

  // default these values should be set to -1 (0 is used)
  obj_n = -1;
  target_obj_n = -1;
  radar_n = -1;
  frame_n = -1;

  switch (case_n) {
    case 1:  // diag responses and requests
      if (channel_number == 1) {
        radar_n = 3;  // front radar
      } else if (id == 1985 || id == 1879) {
        radar_n = 1;  // right corner radar
      } else if (id == 1958 || id == 1957) {
        radar_n = 2;  // left corner radar
      }
      break;

    case 2:  // target A and B frames
      if (channel_number == 1) {
        radar_n = 3;  // front radar
      } else if (id % 10 == 5 || id % 10 == 6) {
        radar_n = 1;  // radar 1 in dbc (all ids for targets end with a 5 or a 6)
      } else if (id % 10 == 7 || id % 10 == 8) {
        radar_n = 2;  // radar 2 in dbc
      }

      if (id % 10 == 5 || id % 10 == 7) {
        frame_n = 1;  // a frame in dbc (all ids for targets end with a 5 if they are radar 1, or 7 if they are  radar
                      // 2)
      } else if (id % 10 == 6 || id % 10 == 8) {
        frame_n = 2;  // frame b in dbc
      }

      target_obj_n = (id - 1600 - (id % 10)) / 10;  // takes the target object number based on the defined id

      break;

    case 3:  // ender, starter, and statuses messages
      if (channel_number == 1) {
        radar_n = 3;  // front radar
      } else if (id == 1665 || id == 1280 || id == 1670) {
        radar_n = 1;  // radar 1 in dbc
      } else if (id == 1667 || id == 1282 || id == 1672) {
        radar_n = 2;  // radar 2 in dbc
      }
      break;

    case 4:  // radar A and B object frames
      if (channel_number == 1) {
        radar_n = 3;  // front radar
      } else if (id % 10 == 5 || id % 10 == 6) {
        radar_n = 1;  // radar 1 in dbc (all ids follow the same convention as target messages)
      } else if (id % 10 == 7 || id % 10 == 8) {
        radar_n = 2;  // radar 2 in dbc
      }

      if (id % 10 == 5 || id % 10 == 7) {
        frame_n = 1;  // a frame in dbc (all ids follow the same convention as target messages)
      } else if (id % 10 == 6 || id % 10 == 8) {
        frame_n = 2;  // b frame in dbc
      }

      obj_n = (id - 1280 - (id % 10)) / 10;  // takes the tracked object number based on the defined id

      break;
  }
};
void Radar_RX::get_static_veh_info(radar_input_mount_info_t &in_mount_info, radar_input_veh_dyn_data_t &in_veh_dyn,
                                   radar_input_wheel_info_t &in_wheel_info, radar_input_veh_dim_t &in_veh_dim,
                                   int radar_num) {
  // Mount Info
  float latsensor_tocenter;
  float longsensor_torear;
  float sensor_height;
  bool sensor_orient;
  float sensor_angle;

  // Vehicle dynamics Info
  float str_ang = 0;
  float prnd = 3;      // default enumeration, drive
  bool wheelslip = 0;  // default don't send information
  float v_ego = 0;     // debug set to 10, should be 0
  bool v_stand = 0;
  bool use_str_ang = 1;  // debug 0, should be 1
  float yawrate = 0;

  float wheelbase = 2.863;       // distance from front to rear axles [m]
  float trackwidth = 1.681;      // distance from right and left wheel [m]
  float strwhlang_ratio = 15.1;  // ratio of steering wheel to wheels turning

  // vehicle dimensions
  float veh_maxwidth = 2.158;  // max width of the vehicle [m]
  float veh_minwidth = 1.948;  // min width of the vehicle [m]

  // these two parameters definitely look wrong
  float frontbump_pos = 3.881;  // longitudinal position of front bumper wrt sensor [m] ???
  float rearbump_pos = 0.964;   // longitudinal position of rear bumper wrt sensor [m]   ???

  switch (radar_num) {
    case 1:  // right corner radar
      latsensor_tocenter = 0.885;
      longsensor_torear = 3.35;
      sensor_height = 0.673;
      sensor_orient = 0;
      sensor_angle = -0.785398;
      break;
    case 2:  // left corner radar
      latsensor_tocenter = 0.885;
      longsensor_torear = 3.37;
      sensor_height = 0.681;
      sensor_orient = 1;
      sensor_angle = 0.785398;
      break;
    case 3:  // front radar

      // mounting information
      latsensor_tocenter = 0;
      longsensor_torear = 3.863;  // 3.784;
      sensor_height = 0.370;      // 0.558;
      sensor_orient = 0;
      sensor_angle = 0;
      break;
  }

  in_mount_info.ri_mi_lat_sensor_mount_to_center =
      radar_input_mount_info_ri_mi_lat_sensor_mount_to_center_encode(latsensor_tocenter);
  in_mount_info.ri_mi_long_sensor_mount_to_rear_axle =
      radar_input_mount_info_ri_mi_long_sensor_mount_to_rear_axle_encode(longsensor_torear);
  in_mount_info.ri_mi_sensor_height = radar_input_mount_info_ri_mi_sensor_height_encode(sensor_height);
  in_mount_info.ri_mi_sensor_orientation = radar_input_mount_info_ri_mi_sensor_orientation_encode(sensor_orient);
  in_mount_info.ri_mi_sensor_mount_angle = radar_input_mount_info_ri_mi_sensor_mount_angle_encode(sensor_angle);

  in_veh_dyn.ri_veh_steer_angle = radar_input_veh_dyn_data_ri_veh_steer_angle_encode(str_ang);
  in_veh_dyn.ri_veh_velocity = radar_input_veh_dyn_data_ri_veh_velocity_encode(v_ego);
  // in_veh_dyn.ri_veh_velocity = radar_input_veh_dyn_data_ri_veh_velocity_encode(10);
  in_veh_dyn.ri_veh_use_steer_angle = radar_input_veh_dyn_data_ri_veh_use_steer_angle_encode(use_str_ang);
  // in_veh_dyn.ri_veh_use_steer_angle = radar_input_veh_dyn_data_ri_veh_use_steer_angle_encode(1);
  in_veh_dyn.ri_veh_standstill = radar_input_veh_dyn_data_ri_veh_standstill_encode(v_stand);  // use_str_ang
  // in_veh_dyn.ri_veh_standstill = radar_input_veh_dyn_data_ri_veh_standstill_encode(1);
  in_veh_dyn.ri_veh_yaw_rate = radar_input_veh_dyn_data_ri_veh_yaw_rate_encode(yawrate);
  in_veh_dyn.ri_veh_any_wheel_slip_event =
      radar_input_veh_dyn_data_ri_veh_any_wheel_slip_event_encode(wheelslip);  // prnd
  // in_veh_dyn.ri_veh_any_wheel_slip_event = radar_input_veh_dyn_data_ri_veh_any_wheel_slip_event_encode(3);
  in_veh_dyn.ri_veh_prndstat = radar_input_veh_dyn_data_ri_veh_prndstat_encode(prnd);  // wheelslip

  in_wheel_info.ri_wi_wheel_base = radar_input_wheel_info_ri_wi_wheel_base_encode(wheelbase);
  in_wheel_info.ri_wi_track_width = radar_input_wheel_info_ri_wi_track_width_encode(trackwidth);
  in_wheel_info.ri_wi_steering_angle_ratio = radar_input_wheel_info_ri_wi_steering_angle_ratio_encode(strwhlang_ratio);

  in_veh_dim.ri_vd_max_width = radar_input_veh_dim_ri_vd_max_width_encode(veh_maxwidth);
  in_veh_dim.ri_vd_min_width = radar_input_veh_dim_ri_vd_min_width_encode(veh_minwidth);
  in_veh_dim.ri_vd_long_front_bumper_pos = radar_input_veh_dim_ri_vd_long_front_bumper_pos_encode(frontbump_pos);
  in_veh_dim.ri_vd_long_rear_bumper_pos = radar_input_veh_dim_ri_vd_long_rear_bumper_pos_encode(rearbump_pos);
};
double Radar_RX::signals_in_range(double val, bool cond) { return (cond) ? (val) : 0; };
uint8_t Radar_RX::crc8bit_calculation(uint8_t *can1670signals, int f_len) {
  uint8_t crc = 0xFF;
  // std::cout << "Msg" << +can1670signals << std::endl;

  for (int index = 0; index < f_len; index++) {
    crc ^= can1670signals[index];  // Assign data to CRC
    // std::cout << "Index" << +index << "---" << +crc << std::endl;
    for (int bitIndex = 0; bitIndex < 8; bitIndex++) {  // Loop through 8 bits
      // std::cout << "bitIndex" << +bitIndex << std::endl;
      if ((crc & 0x80) != 0) {  // 0x80 == 128 == 10000000
        crc = (crc << 1);
        crc ^= (0x1D);  // 0x1D == 29
                        // std::cout << "A" << +crc << std::endl;
      } else {
        crc = (crc << 1);
        // std::cout << "B" << +crc << std::endl;
      }
    }
    // std::cout << "Indexxx" << +index << "---" << +crc << std::endl;
  }
  // std::cout << "Last one" << +crc << std::endl;
  crc = ~crc;
  // std::cout << "Last two" << +crc << std::endl;
  return crc;
};
void Radar_RX::clear_classes(common::radar_object_data &radar_obj, common::sensor_diagnostic_data_msg &diag_data,
                             Radar_RX::radar_diagnostic_response &diag_response,
                             Radar_RX::radar_information &radar_info, Radar_RX::target_tracking_info &target_info,
                             Radar_RX::object_tracking_info &object_info, uint8_t &tc_check, uint8_t &mc_check) {
  // clean up class message each cycle with blank defaults
  common::radar_object_data blank_radar;
  common::sensor_diagnostic_data_msg blank_diag;
  Radar_RX::radar_diagnostic_response blank_diag_response;
  Radar_RX::radar_information blank_info;
  Radar_RX::target_tracking_info blank_target;
  Radar_RX::object_tracking_info blank_object;

  radar_obj = blank_radar;
  diag_data = blank_diag;

  diag_response = blank_diag_response;
  radar_info = blank_info;
  radar_info.channel_number = 1;
  target_info = blank_target;
  object_info = blank_object;

  tc_check = 0;
  mc_check = 0;
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH2");
  ros::NodeHandle can_tx_rx_CH2_handle;

  Radar_RX rad_rx = Radar_RX(&can_tx_rx_CH2_handle);
  SensorDiagnostics sens_diag = SensorDiagnostics(&can_tx_rx_CH2_handle);

  common::radar_object_data radar_obj;
  common::sensor_diagnostic_data_msg diag_data;

  Radar_RX::radar_diagnostic_response diag_response;
  Radar_RX::radar_information radar_info;
  Radar_RX::target_tracking_info target_info;
  Radar_RX::object_tracking_info object_info;

  radar_input_veh_dyn_data_t in_veh_dyn;
  radar_input_veh_dim_t in_veh_dim;
  radar_input_wheel_info_t in_wheel_info;
  radar_input_mount_info_t in_mount_info;

  in_mount_info.ri_mi_mc = 0;
  in_veh_dyn.ri_veh_mc = 0;
  in_veh_dim.ri_vd_mc = 0;
  in_wheel_info.ri_wi_mc = 0;

  ros::Time mem1 = ros::Time::now();
  ros::Time mem2 = ros::Time::now();
  ros::Time mem3 = ros::Time::now();

  radar_info.channel_number = 1;  //1 for ch2 and 2 for ch3

  long int id;
  unsigned int dlc;
  unsigned int flag;
  unsigned long time;
  uint8_t can_data[8] = {0};
  int case_num = 0;
  int radar_num = 0;           // 1 or 2 = valid
  int frame_num = 0;           // 1 = Frame_A, 2 = Frame_B, 3 = general, other = error
  int obj_num = -1;            // 0 to 31 = valid
  int target_object_num = -1;  // 0 to 5 = valid
  int unpack_return = -1;  // 0 is successful, negative error code

  // radar counters, one for each radar
  uint8_t tc_check = 0;
  uint8_t mc_check = 0;

  bool pub_data = false;  // 0 if the node has not receieved a starter bit, otherwise 1

  canInitializeLibrary();
  canHandle hnd;
  canStatus stat;

  hnd = canOpenChannel(radar_info.channel_number, canOPEN_EXCLUSIVE);
  if (hnd < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
  canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  canBusOn(hnd);
  std::cout << "Set up CAN Parameters, starting to read in loop" << std::endl;
  while (ros::ok()) {
    ros::Time now = ros::Time::now();

    rad_rx.get_static_veh_info(in_mount_info, in_veh_dyn, in_wheel_info, in_veh_dim, radar_num);

    size_t size1 = 8u;
    size_t size2 = 5u;

    if ((now.toSec() - mem3.toSec()) > 1) {
      uint16_t int16_2bit;  // not used anywhere

      uint8_t in_mount_signals[7];
      in_mount_signals[0] = (in_mount_info.ri_mi_lat_sensor_mount_to_center << 1u) +
                            (in_mount_info.ri_mi_long_sensor_mount_to_rear_axle >> 8u);
      in_mount_signals[1] = (in_mount_info.ri_mi_long_sensor_mount_to_rear_axle & 0xFF);
      in_mount_signals[2] = in_mount_info.ri_mi_sensor_height;
      in_mount_signals[3] =
          (in_mount_info.ri_mi_sensor_orientation << 7u) + (in_mount_info.ri_mi_sensor_mount_angle >> 8u);
      in_mount_signals[4] = (in_mount_info.ri_mi_sensor_mount_angle & 0xFF);
      in_mount_signals[5] = {};
      in_mount_signals[6] = in_mount_info.ri_mi_mc;
      // std::cout << "Mount" << std::endl;
      in_mount_info.ri_mi_crc = rad_rx.crc8bit_calculation(in_mount_signals, 7);

      uint8_t radar_can_msg_mount[8] = {0};
      struct radar_input_mount_info_t *mount_info = &in_mount_info;
      radar_input_mount_info_pack(radar_can_msg_mount, mount_info, size1);
      switch (radar_num) {
        case 1:
          canWrite(hnd, 490, radar_can_msg_mount, SIZE_OF_MSG, canOPEN_EXCLUSIVE);
          break;
        case 2:
          canWrite(hnd, 492, radar_can_msg_mount, SIZE_OF_MSG, canOPEN_EXCLUSIVE);
          break;
        case 3:
          canWrite(hnd, 490, radar_can_msg_mount, SIZE_OF_MSG, canOPEN_EXCLUSIVE);
          break;
      }

      uint8_t in_wheel_signals[4];
      in_wheel_signals[0] = in_wheel_info.ri_wi_wheel_base;
      in_wheel_signals[1] = in_wheel_info.ri_wi_track_width;
      in_wheel_signals[2] = in_wheel_info.ri_wi_steering_angle_ratio;
      in_wheel_signals[3] = in_wheel_info.ri_wi_mc;
      // std::cout << "In Wheel" << std::endl;
      in_wheel_info.ri_wi_crc = rad_rx.crc8bit_calculation(in_wheel_signals, 4);

      uint8_t radar_can_msg_wheel[5] = {0};
      struct radar_input_wheel_info_t *wheel_info = &in_wheel_info;
      radar_input_wheel_info_pack(radar_can_msg_wheel, wheel_info, size2);
      canWrite(hnd, 202, radar_can_msg_wheel, 5, canOPEN_EXCLUSIVE);

      uint8_t in_veh_dim_signals[7];
      in_veh_dim_signals[0] = in_veh_dim.ri_vd_max_width;
      in_veh_dim_signals[1] = in_veh_dim.ri_vd_min_width;
      in_veh_dim_signals[2] = (in_veh_dim.ri_vd_long_front_bumper_pos >> 8u);
      in_veh_dim_signals[3] = (in_veh_dim.ri_vd_long_front_bumper_pos & 0xFF);
      in_veh_dim_signals[4] = (in_veh_dim.ri_vd_long_rear_bumper_pos >> 8u);
      in_veh_dim_signals[5] = (in_veh_dim.ri_vd_long_rear_bumper_pos & 0xFF);
      in_veh_dim_signals[6] = in_veh_dim.ri_vd_mc;
      in_veh_dim_signals[6] = in_veh_dim.ri_vd_mc;
      in_veh_dim_signals[6] = in_veh_dim.ri_vd_mc;
      // std::cout << "Vehicle Dim" << std::endl;
      in_veh_dim.ri_vd_crc = rad_rx.crc8bit_calculation(in_veh_dim_signals, 7);

      uint8_t radar_can_msg_veh_dim[8] = {0};
      struct radar_input_veh_dim_t *veh_dim = &in_veh_dim;
      radar_input_veh_dim_pack(radar_can_msg_veh_dim, veh_dim, size1);
      canWrite(hnd, 201, radar_can_msg_veh_dim, SIZE_OF_MSG, canOPEN_EXCLUSIVE);

      if (in_mount_info.ri_mi_mc > 15) {
        in_mount_info.ri_mi_mc = 0;
        in_veh_dim.ri_vd_mc = 0;
        in_wheel_info.ri_wi_mc = 0;
      } else {
        in_mount_info.ri_mi_mc = in_mount_info.ri_mi_mc + 1;
        in_veh_dim.ri_vd_mc = in_veh_dim.ri_vd_mc + 1;
        in_wheel_info.ri_wi_mc = in_wheel_info.ri_wi_mc + 1;
      }
      mem3 = now;
    }

    if ((now.toSec() - mem2.toSec()) > 0.02) {  // 0.02) {

      uint8_t in_veh_dyn_signals[7];
      in_veh_dyn_signals[0] = (in_veh_dyn.ri_veh_steer_angle >> 8u);
      in_veh_dyn_signals[1] = (in_veh_dyn.ri_veh_steer_angle & 0xFF);
      in_veh_dyn_signals[2] = (in_veh_dyn.ri_veh_velocity >> 8u);
      in_veh_dyn_signals[3] = (in_veh_dyn.ri_veh_velocity & 0xFF);
      in_veh_dyn_signals[4] = (in_veh_dyn.ri_veh_use_steer_angle << 7u) + (in_veh_dyn.ri_veh_standstill << 6u) +
                              (in_veh_dyn.ri_veh_yaw_rate >> 8u);
      in_veh_dyn_signals[5] = (in_veh_dyn.ri_veh_yaw_rate & 0xFF);
      in_veh_dyn_signals[6] =
          (in_veh_dyn.ri_veh_mc << 4u) + (in_veh_dyn.ri_veh_any_wheel_slip_event << 3u) + in_veh_dyn.ri_veh_prndstat;
      // std::cout << "Dynamics" << std::endl;
      in_veh_dyn.ri_veh_crc = rad_rx.crc8bit_calculation(in_veh_dyn_signals, 7);

      uint8_t radar_can_msg_veh_dyn[8] = {0};
      struct radar_input_veh_dyn_data_t *veh_dyn = &in_veh_dyn;
      radar_input_veh_dyn_data_pack(radar_can_msg_veh_dyn, veh_dyn, size1);
      canWrite(hnd, 200, radar_can_msg_veh_dyn, SIZE_OF_MSG, canOPEN_EXCLUSIVE);

      if (in_veh_dyn.ri_veh_mc > 15) {
        in_veh_dyn.ri_veh_mc = 0;
      } else {
        in_veh_dyn.ri_veh_mc = in_veh_dyn.ri_veh_mc + 1;
      }

      mem2 = now;
    }

    if (now.toSec() - mem1.toSec() > 0.1) {  // try commenting out

      stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);
      if (stat == canOK) {
        rad_rx.get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num, radar_info.channel_number);
        // // Left corner radar = radar_1 and right corner radar = radar_2
        // // // front radar = 3
        // std::cout << "ID, Case, Radar, Frame, Obj, Target_Obj" << std::endl;
        // std::cout << "ID" << +id << std::endl;
        // std::cout << "Case" << +case_num << std::endl;
        // std::cout << "Radar Number" << +radar_num << std::endl;
        // std::cout << "Frame Number" << +frame_num << std::endl;
        std::cout << "Object Number" << +obj_num << std::endl;
        // std::cout << "Target Object Number" << +target_object_num << std::endl;

        if (id == 1280 || id == 1282) {
          pub_data = true;
          rad_rx.clear_classes(radar_obj, diag_data, diag_response, radar_info, target_info, object_info, tc_check,
                               mc_check);
        }
        switch (case_num) {
          case 1:  // diag responses
            radar_diag_response_t r_diag_response_obj;
            unpack_return = radar_diag_response_unpack(&r_diag_response_obj, can_data, SIZE_OF_MSG);
            diag_response.diagnostic_decode =
                radar_diag_response_r_diag_response_decode(r_diag_response_obj.r_diag_response);
            diag_response.diagnostic_is_in_range =
                radar_diag_response_r_diag_response_is_in_range(r_diag_response_obj.r_diag_response);
            diag_response.timestamp = time;
            diag_response.radar_number = radar_num;

            diag_data.radar_diag_response =
                rad_rx.signals_in_range(diag_response.diagnostic_decode, diag_response.diagnostic_is_in_range);
            break;
          case 2:  // target tracking (not in use currently)
            switch (frame_num) {
              case 1:  // a frame
                radar_target_a_t r_target_a;
                unpack_return = radar_target_a_unpack(&r_target_a, can_data, SIZE_OF_MSG);
                target_info.target_dx_decode = radar_target_a_radar_target_dx_decode(r_target_a.radar_target_dx);
                target_info.target_dx_is_in_range =
                    radar_target_a_radar_target_dx_is_in_range(r_target_a.radar_target_dx);

                target_info.target_vx_decode = radar_target_a_radar_target_vx_decode(r_target_a.radar_target_vx);
                target_info.target_vx_is_in_range =
                    radar_target_a_radar_target_vx_is_in_range(r_target_a.radar_target_vx);

                target_info.target_dy_decode = radar_target_a_radar_target_dy_decode(r_target_a.radar_target_dy);
                target_info.target_dy_is_in_range =
                    radar_target_a_radar_target_dy_is_in_range(r_target_a.radar_target_dy);

                target_info.target_w_exist_decode =
                    radar_target_a_radar_target_w_exist_decode(r_target_a.radar_target_w_exist);
                target_info.target_w_exist_is_in_range =
                    radar_target_a_radar_target_w_exist_is_in_range(r_target_a.radar_target_w_exist);

                target_info.target_ax_decode = radar_target_a_radar_target_ax_decode(r_target_a.radar_target_ax);
                target_info.target_ax_is_in_range =
                    radar_target_a_radar_target_ax_is_in_range(r_target_a.radar_target_ax);

                target_info.target_w_obstacle_decode =
                    radar_target_a_radar_target_w_obstacle_decode(r_target_a.radar_target_w_obstacle);
                target_info.target_w_obstacle_is_in_range =
                    radar_target_a_radar_target_w_obstacle_is_in_range(r_target_a.radar_target_w_obstacle);

                target_info.target_flag_valid_decode =
                    radar_target_a_radar_target_flag_valid_decode(r_target_a.radar_target_flag_valid);
                target_info.target_flag_valid_is_in_range =
                    radar_target_a_radar_target_flag_valid_is_in_range(r_target_a.radar_target_flag_valid);

                target_info.target_w_non_obstacle_decode =
                    radar_target_a_radar_target_w_non_obstacle_decode(r_target_a.radar_target_w_non_obstacle);
                target_info.target_w_non_obstacle_is_in_range =
                    radar_target_a_radar_target_w_non_obstacle_is_in_range(r_target_a.radar_target_w_non_obstacle);

                target_info.target_flag_meas_decode =
                    radar_target_a_radar_target_flag_meas_decode(r_target_a.radar_target_flag_meas);
                target_info.target_flag_meas_is_in_range =
                    radar_target_a_radar_target_flag_meas_is_in_range(r_target_a.radar_target_flag_meas);

                target_info.target_flag_hist_decode =
                    radar_target_a_radar_target_flag_hist_decode(r_target_a.radar_target_flag_hist);
                target_info.target_flag_hist_is_in_range =
                    radar_target_a_radar_target_flag_hist_is_in_range(r_target_a.radar_target_flag_hist);

                target_info.target_mess_aconsist_bit_decode =
                    radar_target_a_radar_target_mess_aconsist_bit_decode(r_target_a.radar_target_mess_aconsist_bit);
                target_info.target_mess_aconsist_bit_is_in_range =
                    radar_target_a_radar_target_mess_aconsist_bit_is_in_range(
                        r_target_a.radar_target_mess_aconsist_bit);
                break;
              case 2:
                radar_target_b_t r_target_b;
                unpack_return = radar_target_b_unpack(&r_target_b, can_data, SIZE_OF_MSG);
                target_info.target_vy_decode = radar_target_b_radar_target_vy_decode(r_target_b.radar_target_vy);
                target_info.target_vy_is_in_range =
                    radar_target_b_radar_target_vy_is_in_range(r_target_b.radar_target_vy);

                target_info.target_d_length_decode =
                    radar_target_b_radar_target_d_length_decode(r_target_b.radar_target_d_length);
                target_info.target_d_length_is_in_range =
                    radar_target_b_radar_target_d_length_is_in_range(r_target_b.radar_target_d_length);

                target_info.target_dz_decode = radar_target_b_radar_target_dz_decode(r_target_b.radar_target_dz);
                target_info.target_dz_is_in_range =
                    radar_target_b_radar_target_dz_is_in_range(r_target_b.radar_target_dz);

                target_info.target_moving_state_decode =
                    radar_target_b_radar_target_moving_state_decode(r_target_b.radar_target_moving_state);
                target_info.target_moving_state_is_in_range =
                    radar_target_b_radar_target_moving_state_is_in_range(r_target_b.radar_target_moving_state);

                target_info.target_dx_sigma_decode =
                    radar_target_b_radar_target_dx_sigma_decode(r_target_b.radar_target_dx_sigma);
                target_info.target_dx_sigma_is_in_range =
                    radar_target_b_radar_target_dx_sigma_decode(r_target_b.radar_target_dx_sigma);

                target_info.target_vx_sigma_decode =
                    radar_target_b_radar_target_vx_sigma_decode(r_target_b.radar_target_vx_sigma);
                target_info.target_vx_sigma_is_in_range =
                    radar_target_b_radar_target_vx_sigma_is_in_range(r_target_b.radar_target_vx_sigma);

                target_info.target_ax_sigma_decode =
                    radar_target_b_radar_target_ax_sigma_decode(r_target_b.radar_target_ax_sigma);
                target_info.target_ax_sigma_is_in_range =
                    radar_target_b_radar_target_ax_sigma_is_in_range(r_target_b.radar_target_ax_sigma);

                target_info.target_dy_sigma_decode =
                    radar_target_b_radar_target_dy_sigma_decode(r_target_b.radar_target_dy_sigma);
                target_info.target_dy_sigma_is_in_range =
                    radar_target_b_radar_target_dy_sigma_is_in_range(r_target_b.radar_target_dy_sigma);

                target_info.target_w_class_decode =
                    radar_target_b_radar_target_w_class_decode(r_target_b.radar_target_w_class);
                target_info.target_w_class_is_in_range =
                    radar_target_b_radar_target_class_is_in_range(r_target_b.radar_target_w_class);

                target_info.target_class_decode =
                    radar_target_b_radar_target_class_decode(r_target_b.radar_target_class);
                target_info.target_class_is_in_range =
                    radar_target_b_radar_target_class_is_in_range(r_target_b.radar_target_class);

                target_info.target_dx_rear_end_loss_decode =
                    radar_target_b_radar_target_dx_rear_end_loss_decode(r_target_b.radar_target_dx_rear_end_loss);
                target_info.target_dx_rear_end_loss_is_in_range =
                    radar_target_b_radar_target_dx_rear_end_loss_is_in_range(r_target_b.radar_target_dx_rear_end_loss);

                target_info.target_mess_bconsist_bit_decode =
                    radar_target_b_radar_target_mess_bconsist_bit_decode(r_target_b.radar_target_mess_bconsist_bit);
                target_info.target_mess_bconsist_bit_is_in_range =
                    radar_target_b_radar_target_mess_bconsist_bit_is_in_range(
                        r_target_b.radar_target_mess_bconsist_bit);
                break;
            }
            break;
          case 3:                            // enders, starters, or statuses
            if (id == 1665 || id == 1667) {  // enders
              radar_object_ender_t r_ender;
              unpack_return = radar_object_ender_unpack(&r_ender, can_data, SIZE_OF_MSG);

              radar_info.radar_timestamp_decode = radar_object_ender_radar_timestamp_decode(r_ender.radar_timestamp);
              radar_info.radar_timestamp_is_in_range =
                  radar_object_ender_radar_timestamp_is_in_range(r_ender.radar_timestamp);

              radar_info.tc_counter_decode = radar_object_ender_radar_tc_counter_decode(r_ender.radar_tc_counter);
              radar_info.tc_counter_is_in_range =
                  radar_object_ender_radar_tc_counter_is_in_range(r_ender.radar_tc_counter);

              radar_info.obj_ender_consist_bit_decode =
                  radar_object_ender_radar_mess_ender_consist_bit_decode(r_ender.radar_mess_ender_consist_bit);
              radar_info.obj_ender_consist_bit_is_in_range =
                  radar_object_ender_radar_mess_ender_consist_bit_is_in_range(r_ender.radar_mess_ender_consist_bit);

              radar_info.packet_checksum_decode =
                  radar_object_ender_radar_packet_checksum_decode(r_ender.radar_packet_checksum);
              radar_info.packet_checksum_is_in_range =
                  radar_object_ender_radar_packet_checksum_is_in_range(r_ender.radar_packet_checksum);

              radar_obj.radar_timestamp =
                  rad_rx.signals_in_range(radar_info.radar_timestamp_decode, radar_info.radar_timestamp_is_in_range);

              diag_data.timestamp =
                  rad_rx.signals_in_range(radar_info.radar_timestamp_decode, radar_info.radar_timestamp_is_in_range);
              diag_data.radar_tc_counter =
                  rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);
              diag_data.radar_mess_ender_consist_bit = rad_rx.signals_in_range(
                  radar_info.obj_ender_consist_bit_decode, radar_info.obj_ender_consist_bit_is_in_range);
              diag_data.radar_packet_checksum =
                  rad_rx.signals_in_range(radar_info.packet_checksum_decode, radar_info.packet_checksum_is_in_range);
            } else if (id == 1280 || id == 1282) {  // starters
              radar_object_starter_t r_starter;
              unpack_return = radar_object_starter_unpack(&r_starter, can_data, SIZE_OF_MSG);

              radar_info.veh_psi_dt_decode = radar_object_starter_radar_veh_psi_dt_decode(r_starter.radar_veh_psi_dt);
              radar_info.veh_psi_dt_is_in_range =
                  radar_object_starter_radar_veh_psi_dt_is_in_range(r_starter.radar_veh_psi_dt);

              radar_info.veh_v_ego_decode = radar_object_starter_radar_veh_v_ego_decode(r_starter.radar_veh_v_ego);
              radar_info.veh_v_ego_is_in_range =
                  radar_object_starter_radar_veh_v_ego_is_in_range(r_starter.radar_veh_v_ego);

              radar_info.veh_a_ego_decode = radar_object_starter_radar_veh_a_ego_decode(r_starter.radar_veh_a_ego);
              radar_info.veh_a_ego_is_in_range =
                  radar_object_starter_radar_veh_a_ego_is_in_range(r_starter.radar_veh_a_ego);

              radar_info.veh_slip_angle_decode =
                  radar_object_starter_radar_veh_slip_angle_decode(r_starter.radar_veh_slip_angle);
              radar_info.veh_slip_angle_is_in_range =
                  radar_object_starter_radar_veh_slip_angle_is_in_range(r_starter.radar_veh_slip_angle);

              radar_info.mess_starter_consist_bit_decode =
                  radar_object_starter_radar_mess_starter_consist_bit_decode(r_starter.radar_mess_starter_consist_bit);
              radar_info.mess_starter_consist_bit_is_in_range =
                  radar_object_starter_radar_mess_starter_consist_bit_is_in_range(
                      r_starter.radar_mess_starter_consist_bit);

              radar_obj.veh_psi_dot =
                  rad_rx.signals_in_range(radar_info.veh_psi_dt_decode, radar_info.veh_psi_dt_is_in_range);
              radar_obj.veh_v_ego =
                  rad_rx.signals_in_range(radar_info.veh_v_ego_decode, radar_info.veh_v_ego_is_in_range);
              radar_obj.veh_a_ego =
                  rad_rx.signals_in_range(radar_info.veh_a_ego_decode, radar_info.veh_a_ego_is_in_range);
              radar_obj.veh_slip_angle =
                  rad_rx.signals_in_range(radar_info.veh_slip_angle_decode, radar_info.veh_slip_angle_is_in_range);

              diag_data.radar_mess_starter_consist_bit = rad_rx.signals_in_range(
                  radar_info.mess_starter_consist_bit_decode, radar_info.mess_starter_consist_bit_is_in_range);
            } else if (id == 1670 || id == 1672) {  // statuses

              radar_status_t r_status;
              unpack_return = radar_status_unpack(&r_status, can_data, SIZE_OF_MSG);

              radar_info.itc_info_decode = radar_status_r_stat_itc_info_decode(r_status.r_stat_itc_info);
              radar_info.itc_info_is_in_range = radar_status_r_stat_itc_info_is_in_range(r_status.r_stat_itc_info);

              radar_info.sgu_fail_decode = radar_status_r_stat_sgu_fail_decode(r_status.r_stat_sgu_fail);
              radar_info.sgu_fail_is_in_range = radar_status_r_stat_sgu_fail_is_in_range(r_status.r_stat_sgu_fail);

              radar_info.hw_fail_decode = radar_status_r_stat_hw_fail_decode(r_status.r_stat_hw_fail);
              radar_info.hw_fail_is_in_range = radar_status_r_stat_hw_fail_is_in_range(r_status.r_stat_hw_fail);

              radar_info.horizontal_misalignment_decode =
                  radar_status_r_stat_horizontal_misalignment_decode(r_status.r_stat_horizontal_misalignment);
              radar_info.horizontal_misalignment_is_in_range =
                  radar_status_r_stat_horizontal_misalignment_is_in_range(r_status.r_stat_horizontal_misalignment);

              radar_info.absorption_blindness_decode =
                  radar_status_r_stat_absorption_blindness_decode(r_status.r_stat_absorption_blindness);
              radar_info.absorption_blindness_is_in_range =
                  radar_status_r_stat_absorption_blindness_is_in_range(r_status.r_stat_absorption_blindness);

              radar_info.distortion_blindness_decode =
                  radar_status_r_stat_distortion_blindness_decode(r_status.r_stat_distortion_blindness);
              radar_info.distortion_blindness_is_in_range =
                  radar_status_r_stat_distortion_blindness_is_in_range(r_status.r_stat_distortion_blindness);

              radar_info.mc_decode = radar_status_r_stat_mc_decode(r_status.r_stat_mc);
              radar_info.mc_is_in_range = radar_status_r_stat_mc_is_in_range(r_status.r_stat_mc);

              radar_info.crc_decode = radar_status_r_stat_crc_decode(r_status.r_stat_crc);
              radar_info.crc_is_in_range = radar_status_r_stat_crc_is_in_range(r_status.r_stat_crc);

              diag_data.r_stat_itc_info =
                  rad_rx.signals_in_range(radar_info.itc_info_decode, radar_info.itc_info_is_in_range);
              diag_data.r_stat_sgu_fail =
                  rad_rx.signals_in_range(radar_info.sgu_fail_decode, radar_info.sgu_fail_is_in_range);
              diag_data.r_stat_hw_fail =
                  rad_rx.signals_in_range(radar_info.hw_fail_decode, radar_info.hw_fail_is_in_range);
              diag_data.r_stat_horizontal_misalignment = rad_rx.signals_in_range(
                  radar_info.horizontal_misalignment_decode, radar_info.horizontal_misalignment_is_in_range);
              diag_data.r_stat_absorption_blindness = rad_rx.signals_in_range(
                  radar_info.absorption_blindness_decode, radar_info.absorption_blindness_is_in_range);
              diag_data.r_stat_distortion_blindness = rad_rx.signals_in_range(
                  radar_info.distortion_blindness_decode, radar_info.distortion_blindness_is_in_range);
              diag_data.r_stat_mc = rad_rx.signals_in_range(radar_info.mc_decode, radar_info.mc_is_in_range);
              diag_data.r_stat_crc = rad_rx.signals_in_range(radar_info.crc_decode, radar_info.crc_is_in_range);
              diag_data.tc_check = tc_check;
              diag_data.mc_check = mc_check;

              if (!(tc_check + 0x1 == 256)) {
                tc_check = tc_check + 0x1;
              }

              if (!(mc_check + 0x1 == 16)) {
                mc_check = mc_check + 0x1;
              }
            }

            break;
          case 4:  // object tracking
            switch (frame_num) {
              case 1:  // a frame
                radar_obj_a_t r_object_a;
                unpack_return = radar_obj_a_unpack(&r_object_a, can_data, SIZE_OF_MSG);
                object_info.dx_decode = radar_obj_a_radar_obj_dx_decode(r_object_a.radar_obj_dx);
                object_info.dx_is_in_range = radar_obj_a_radar_obj_dx_is_in_range(r_object_a.radar_obj_dx);

                object_info.vx_decode = radar_obj_a_radar_obj_vx_decode(r_object_a.radar_obj_vx);
                object_info.vx_is_in_range = radar_obj_a_radar_obj_vx_is_in_range(r_object_a.radar_obj_vx);

                object_info.dy_decode = radar_obj_a_radar_obj_dy_decode(r_object_a.radar_obj_dy);
                object_info.dy_is_in_range = radar_obj_a_radar_obj_dy_is_in_range(r_object_a.radar_obj_dy);

                object_info.w_exist_decode = radar_obj_a_radar_obj_w_exist_decode(r_object_a.radar_obj_w_exist);
                object_info.w_exist_is_in_range =
                    radar_obj_a_radar_obj_w_exist_is_in_range(r_object_a.radar_obj_w_exist);

                object_info.ax_decode = radar_obj_a_radar_obj_ax_decode(r_object_a.radar_obj_ax);
                object_info.ax_is_in_range = radar_obj_a_radar_obj_ax_is_in_range(r_object_a.radar_obj_ax);

                object_info.w_obstacle_decode =
                    radar_obj_a_radar_obj_w_obstacle_decode(r_object_a.radar_obj_w_obstacle);
                object_info.w_obstacle_is_in_range =
                    radar_obj_a_radar_obj_w_obstacle_is_in_range(r_object_a.radar_obj_w_obstacle);

                object_info.flag_valid_decode =
                    radar_obj_a_radar_obj_flag_valid_decode(r_object_a.radar_obj_flag_valid);
                object_info.flag_valid_is_in_range =
                    radar_obj_a_radar_obj_flag_valid_is_in_range(r_object_a.radar_obj_flag_valid);

                object_info.w_non_obstacle_decode =
                    radar_obj_a_radar_obj_w_non_obstacle_decode(r_object_a.radar_obj_w_non_obstacle);
                object_info.w_non_obstacle_is_in_range =
                    radar_obj_a_radar_obj_w_non_obstacle_is_in_range(r_object_a.radar_obj_w_non_obstacle);

                object_info.flag_meas_decode = radar_obj_a_radar_obj_flag_meas_decode(r_object_a.radar_obj_flag_meas);
                object_info.flag_meas_is_in_range =
                    radar_obj_a_radar_obj_flag_meas_is_in_range(r_object_a.radar_obj_flag_meas);

                object_info.flag_hist_decode = radar_obj_a_radar_obj_flag_hist_decode(r_object_a.radar_obj_flag_hist);
                object_info.flag_hist_is_in_range =
                    radar_obj_a_radar_obj_flag_hist_is_in_range(r_object_a.radar_obj_flag_hist);

                object_info.mess_aconsist_bit_decode =
                    radar_obj_a_radar_obj_mess_aconsist_bit_decode(r_object_a.radar_obj_mess_aconsist_bit);
                object_info.mess_aconsist_bit_is_in_range =
                    radar_obj_a_radar_obj_mess_aconsist_bit_is_in_range(r_object_a.radar_obj_mess_aconsist_bit);

                radar_obj.radar_dx[obj_num] =
                    rad_rx.signals_in_range(object_info.dx_decode, object_info.dx_is_in_range);
                radar_obj.radar_vx[obj_num] =
                    rad_rx.signals_in_range(object_info.vx_decode, object_info.vx_is_in_range);
                radar_obj.radar_dy[obj_num] =
                    rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);
                radar_obj.radar_w_exist[obj_num] =
                    rad_rx.signals_in_range(object_info.w_exist_decode, object_info.w_exist_is_in_range);
                radar_obj.radar_ax[obj_num] =
                    rad_rx.signals_in_range(object_info.ax_decode, object_info.ax_is_in_range);
                radar_obj.radar_w_obstacle[obj_num] =
                    rad_rx.signals_in_range(object_info.w_obstacle_decode, object_info.w_obstacle_is_in_range);
                radar_obj.radar_flag_valid[obj_num] =
                    rad_rx.signals_in_range(object_info.flag_valid_decode, object_info.flag_valid_is_in_range);
                radar_obj.radar_w_non_obstacle[obj_num] =
                    rad_rx.signals_in_range(object_info.w_non_obstacle_decode, object_info.w_non_obstacle_is_in_range);
                radar_obj.flag_meas[obj_num] =
                    rad_rx.signals_in_range(object_info.flag_meas_decode, object_info.flag_meas_is_in_range);
                radar_obj.flag_hist[obj_num] =
                    rad_rx.signals_in_range(object_info.flag_hist_decode, object_info.flag_hist_is_in_range);

                diag_data.radar_mess_aconsist_bit = rad_rx.signals_in_range(object_info.mess_aconsist_bit_decode,
                                                                            object_info.mess_aconsist_bit_is_in_range);

                break;
              case 2:
                radar_obj_b_t r_object_b;
                unpack_return = radar_obj_b_unpack(&r_object_b, can_data, SIZE_OF_MSG);
                object_info.vy_decode = radar_obj_b_radar_obj_vy_decode(r_object_b.radar_obj_vy);
                object_info.vy_is_in_range = radar_obj_b_radar_obj_vy_is_in_range(r_object_b.radar_obj_vy);

                object_info.d_length_decode = radar_obj_b_radar_obj_d_length_decode(r_object_b.radar_obj_d_length);
                object_info.d_length_is_in_range =
                    radar_obj_b_radar_obj_d_length_is_in_range(r_object_b.radar_obj_d_length);

                object_info.dz_decode = radar_obj_b_radar_obj_dz_decode(r_object_b.radar_obj_dz);
                object_info.dz_is_in_range = radar_obj_b_radar_obj_dz_is_in_range(r_object_b.radar_obj_dz);

                object_info.moving_state_decode =
                    radar_obj_b_radar_obj_moving_state_decode(r_object_b.radar_obj_moving_state);
                object_info.moving_state_is_in_range =
                    radar_obj_b_radar_obj_moving_state_is_in_range(r_object_b.radar_obj_moving_state);

                object_info.dx_sigma_decode = radar_obj_b_radar_obj_dx_sigma_decode(r_object_b.radar_obj_dx_sigma);
                object_info.dx_sigma_is_in_range =
                    radar_obj_b_radar_obj_dx_sigma_is_in_range(r_object_b.radar_obj_dx_sigma);

                object_info.vx_sigma_decode = radar_obj_b_radar_obj_vx_sigma_decode(r_object_b.radar_obj_vx_sigma);
                object_info.vx_sigma_is_in_range = radar_obj_b_radar_obj_vx_sigma_decode(r_object_b.radar_obj_vx_sigma);

                object_info.ax_sigma_decode = radar_obj_b_radar_obj_ax_sigma_decode(r_object_b.radar_obj_ax_sigma);
                object_info.ax_sigma_is_in_range =
                    radar_obj_b_radar_obj_ax_sigma_is_in_range(r_object_b.radar_obj_ax_sigma);

                object_info.dy_sigma_decode = radar_obj_b_radar_obj_dy_sigma_decode(r_object_b.radar_obj_dy_sigma);
                object_info.dy_sigma_is_in_range =
                    radar_obj_b_radar_obj_dy_sigma_is_in_range(r_object_b.radar_obj_dy_sigma);

                object_info.w_class_decode = radar_obj_b_radar_obj_w_class_decode(r_object_b.radar_obj_w_class);
                object_info.w_class_is_in_range =
                    radar_obj_b_radar_obj_w_class_is_in_range(r_object_b.radar_obj_w_class);

                object_info.class_decode = radar_obj_b_radar_obj_class_decode(r_object_b.radar_obj_class);
                object_info.class_is_in_range = radar_obj_b_radar_obj_class_is_in_range(r_object_b.radar_obj_class);

                object_info.dx_rear_end_loss_decode =
                    radar_obj_b_radar_obj_dx_rear_end_loss_decode(r_object_b.radar_obj_dx_rear_end_loss);
                object_info.dx_rear_end_loss_is_in_range =
                    radar_obj_b_radar_obj_dx_rear_end_loss_is_in_range(r_object_b.radar_obj_dx_rear_end_loss);

                object_info.mess_bconsist_bit_decode =
                    radar_obj_b_radar_obj_mess_bconsist_bit_decode(r_object_b.radar_obj_mess_bconsist_bit);
                object_info.mess_bconsist_bit_is_in_range =
                    radar_obj_b_radar_obj_mess_bconsist_bit_is_in_range(r_object_b.radar_obj_mess_bconsist_bit);

                radar_obj.radar_vy[obj_num] =
                    rad_rx.signals_in_range(object_info.vy_decode, object_info.vy_is_in_range);
                radar_obj.d_length[obj_num] =
                    rad_rx.signals_in_range(object_info.d_length_decode, object_info.d_length_is_in_range);
                radar_obj.radar_dz[obj_num] =
                    rad_rx.signals_in_range(object_info.dz_decode, object_info.dz_is_in_range);
                radar_obj.moving_state[obj_num] =
                    rad_rx.signals_in_range(object_info.moving_state_decode, object_info.moving_state_is_in_range);
                radar_obj.radar_dx_sigma[obj_num] =
                    rad_rx.signals_in_range(object_info.dx_sigma_decode, object_info.dx_sigma_is_in_range);
                radar_obj.radar_vx_sigma[obj_num] =
                    rad_rx.signals_in_range(object_info.vx_sigma_decode, object_info.vx_sigma_is_in_range);
                radar_obj.radar_dy_sigma[obj_num] =
                    rad_rx.signals_in_range(object_info.dy_sigma_decode, object_info.dy_sigma_is_in_range);
                radar_obj.radar_ax_sigma[obj_num] =
                    rad_rx.signals_in_range(object_info.ax_sigma_decode, object_info.ax_sigma_is_in_range);
                radar_obj.radar_w_class[obj_num] =
                    rad_rx.signals_in_range(object_info.w_class_decode, object_info.w_class_is_in_range);
                radar_obj.radar_obj_class[obj_num] =
                    rad_rx.signals_in_range(object_info.class_decode, object_info.class_is_in_range);
                radar_obj.dx_rear_loss[obj_num] = rad_rx.signals_in_range(object_info.dx_rear_end_loss_decode,
                                                                          object_info.dx_rear_end_loss_is_in_range);

                diag_data.radar_mess_bconsist_bit = rad_rx.signals_in_range(object_info.mess_bconsist_bit_decode,
                                                                            object_info.mess_bconsist_bit_is_in_range);

                break;
            }

            object_info.timestamp = time;
            object_info.radar_number = radar_num;
            object_info.object_number = obj_num;
            radar_obj.radar_timestamp = time;
            radar_obj.radar_num = radar_num;
            break;
            break;
        }
        if (pub_data &&
            (id == 1667 || id == 1665)) {  // message must end with the ender bit and have started with an end bit

          // validate radar, using service calls for specific channels
          ros::ServiceClient client_ch2;
          common::sensor_diagnostic_flag_CH2 srv_ch2;

          srv_ch2.request.front_radar = sens_diag.validate_radar(diag_data);
          if (srv_ch2.request.front_radar) {
            std::cout << "Valid Ch2" << std::endl;
          } else {
            std::cout << "Invalid Ch2" << std::endl;
          }

          if (!client_ch2.call(srv_ch2)) {
            std::cout << "SERVICE REQUEST CHANNEL 2 FRONT RADAR FAILED" << std::endl;
          }

          // publish here
          rad_rx.rad_pub.publish(radar_obj);
          rad_rx.diag_pub.publish(diag_data);

          rad_rx.clear_classes(radar_obj, diag_data, diag_response, radar_info, target_info, object_info, tc_check,
                               mc_check);

          // resetting for next data cluster
          pub_data = false;
        }
        std::cout << "Publishing" << std::endl;
        rad_rx.rad_pub.publish(radar_obj);
        rad_rx.diag_pub.publish(diag_data);

        rad_rx.clear_classes(radar_obj, diag_data, diag_response, radar_info, target_info, object_info, tc_check,
                             mc_check);
      }

      mem1 = now;
    }

    ros::spinOnce();
    }

    canBusOff(hnd);
    canClose(hnd);
    return 0;
}