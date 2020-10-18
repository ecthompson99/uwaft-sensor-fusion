#include "mobileye_struct.h"
#include "ext_log_data.h"
#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8 

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;
  ros::NodeHandle diag_handle; 

  Mobileye_RX mobeye_rx = Mobileye_RX(&can_tx_rx_CH4_handle); 
  SensorDiagnostics sens_diag = SensorDiagnostics(&diag_handle);

  common::mobileye_object_data obj_data; //Processed values for object detection
  common::raw_lane_data raw_lane; 
  Mobileye_RX::mobileye_object mobileye_obj;
  canHandle hnd;
  canInitializeLibrary();

  mobileye_obj.channel_number = 3; 
  
  hnd = canOpenChannel(mobileye_obj.channel_number, canOPEN_ACCEPT_VIRTUAL); 

  if (hnd < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }
  canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  canBusOn(hnd);
  int frame_num = 0;
  int unpack_return = -1;  // 0 is successful, negative error code 

  while (ros::ok()) {
    canStatus stat; 
    stat = canRead(hnd, &mobileye_obj.id, &mobileye_obj.can_data, &mobileye_obj.dlc, &mobileye_obj.flag, &mobileye_obj.time_stamp);
    std::cout << "Stat: " << stat << std::endl;   
    if(canOK==stat){
      frame_num = mobeye_rx.get_nums(mobileye_obj); //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
    }
    switch(frame_num){
        case 2:
          {
            ext_log_data_obstacle_data_a_t frame_a_unpacked;
            unpack_return = ext_log_data_obstacle_data_a_unpack(&frame_a_unpacked,mobileye_obj.can_data,SIZE_OF_MSG); 

            mobileye_obj.obstacle_id_decode = ext_log_data_obstacle_data_a_obstacle_id_decode(frame_a_unpacked.obstacle_id);
            mobileye_obj.obstacle_id_is_in_range = ext_log_data_obstacle_data_a_obstacle_id_is_in_range(frame_a_unpacked.obstacle_id);
            obj_data.ObjNum = mobeye_rx.signal_in_range(mobileye_obj.obstacle_id_decode, mobileye_obj.obstacle_id_is_in_range);

            mobileye_obj.obstacle_vel_x_decode = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(frame_a_unpacked.obstacle_vel_x);
            mobileye_obj.obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(frame_a_unpacked.obstacle_vel_x);
            obj_data.MeVx = mobeye_rx.signal_in_range(mobileye_obj.obstacle_vel_x_decode, mobileye_obj.obstacle_vel_x_is_in_range); 

            mobileye_obj.obstacle_pos_y_decode = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(frame_a_unpacked.obstacle_pos_y);
            mobileye_obj.obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(frame_a_unpacked.obstacle_pos_y);
            obj_data.MeDx = mobeye_rx.signal_in_range(mobileye_obj.obstacle_pos_y_decode, mobileye_obj.obstacle_pos_y_is_in_range); 

            mobileye_obj.obstacle_pos_x_decode = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(frame_a_unpacked.obstacle_pos_x);
            mobileye_obj.obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(frame_a_unpacked.obstacle_pos_x);
            obj_data.MeDy = mobeye_rx.signal_in_range(mobileye_obj.obstacle_pos_x_decode, mobileye_obj.obstacle_pos_x_is_in_range);

            obj_data.MeTimestamp = mobileye_obj.time_stamp;

            std::cout << "Velocity: " << obj_data.MeVx << std::endl;
            std::cout << "Longitudinal Distance: " << obj_data.MeDx << std::endl;
            std::cout << "Lateral Distance: " << obj_data.MeDy << std::endl;
            std::cout << "Message sent at " << obj_data.MeTimestamp << std::endl; 
            
            break; 
          }          
        case 3:
          {
            ext_log_data_obstacle_data_b_t frame_b_unpacked;
            unpack_return = ext_log_data_obstacle_data_b_unpack(&frame_b_unpacked,mobileye_obj.can_data,SIZE_OF_MSG);

            mobileye_obj.obstacle_lane_decode = ext_log_data_obstacle_data_b_obstacle_lane_decode(frame_b_unpacked.obstacle_lane);
            mobileye_obj.obstacle_lane_decode_is_in_range = ext_log_data_obstacle_data_b_obstacle_lane_is_in_range(frame_b_unpacked.obstacle_lane);
            obj_data.MeLane = mobeye_rx.signal_in_range(mobileye_obj.obstacle_lane_decode, mobileye_obj.obstacle_lane_decode_is_in_range);
            break;
          }
        case 4:
          {
            ext_log_data_obstacle_data_c_t frame_c_unpacked; 
            unpack_return = ext_log_data_obstacle_data_c_unpack(&frame_c_unpacked,mobileye_obj.can_data,SIZE_OF_MSG);
            break;
          }
        case 5:
          {
            ext_log_data_lka_left_lane_a_t left_a_unpacked; 
            unpack_return = ext_log_data_lka_left_lane_a_unpack(&left_a_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);

            mobileye_obj.left_lane_type_decode =  ext_log_data_lka_left_lane_a_lane_type_decode(left_a_unpacked.lane_type); 
            mobileye_obj.left_lane_type_is_in_range =  ext_log_data_lka_left_lane_a_lane_type_is_in_range(left_a_unpacked.lane_type); 
            raw_lane.Lane_Type_L = mobeye_rx.signal_in_range(mobileye_obj.left_lane_type_decode, mobileye_obj.left_lane_type_is_in_range);
            
            mobileye_obj.left_quality_decode =  ext_log_data_lka_left_lane_a_quality_decode(left_a_unpacked.quality); 
            mobileye_obj.left_quality_is_in_range =  ext_log_data_lka_left_lane_a_quality_is_in_range(left_a_unpacked.quality); 
            raw_lane.Qual_L = mobeye_rx.signal_in_range(mobileye_obj.left_quality_decode, mobileye_obj.left_quality_is_in_range);

            mobileye_obj.left_position_decode =  ext_log_data_lka_left_lane_a_position_decode(left_a_unpacked.position); 
            mobileye_obj.left_position_is_in_range =  ext_log_data_lka_left_lane_a_position_is_in_range(left_a_unpacked.position); 
            raw_lane.LatPos_L = mobeye_rx.signal_in_range(mobileye_obj.left_position_decode, mobileye_obj.left_position_is_in_range);

            mobileye_obj.left_curvature_decode =  ext_log_data_lka_left_lane_a_curvature_decode(left_a_unpacked.curvature); 
            mobileye_obj.left_curvature_is_in_range =  ext_log_data_lka_left_lane_a_curvature_is_in_range(left_a_unpacked.curvature); 
            raw_lane.Curv_L = mobeye_rx.signal_in_range(mobileye_obj.left_curvature_decode, mobileye_obj.left_curvature_is_in_range);

            mobileye_obj.left_curvature_derivative_decode =  ext_log_data_lka_left_lane_a_curvature_derivative_decode(left_a_unpacked.curvature_derivative); 
            mobileye_obj.left_curvature_derivative_is_in_range =  ext_log_data_lka_left_lane_a_curvature_derivative_is_in_range(left_a_unpacked.curvature_derivative); 
            raw_lane.Curv_Deriv_L = mobeye_rx.signal_in_range(mobileye_obj.left_curvature_derivative_decode, mobileye_obj.left_curvature_derivative_is_in_range);
            
            std::cout << "Left Lane Type: " << raw_lane.Lane_Type_L << std::endl;
            std::cout << "Left Quality: " << raw_lane.Qual_L << std::endl;
            std::cout << std::setprecision(7) << "Left Position: " << raw_lane.LatPos_L << std::endl;
            std::cout << std::setprecision(7) << "Left Curvature " << raw_lane.Curv_L << std::endl; 
            std::cout << std::setprecision(7) << "Left Curvature Derivative: " << raw_lane.Curv_Deriv_L << std::endl;

            break; 
          }
        case 6:
          {
            ext_log_data_lka_left_lane_b_t left_b_unpacked; 
            unpack_return = ext_log_data_lka_left_lane_b_unpack(&left_b_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);

            mobileye_obj.left_heading_angle_decode =  ext_log_data_lka_left_lane_b_heading_angle_decode(left_b_unpacked.heading_angle); 
            mobileye_obj.left_heading_angle_is_in_range =  ext_log_data_lka_left_lane_b_heading_angle_is_in_range(left_b_unpacked.heading_angle); 
            raw_lane.Head_Ang_L = mobeye_rx.signal_in_range(mobileye_obj.left_heading_angle_decode, mobileye_obj.left_heading_angle_is_in_range);
            
            std::cout << std::setprecision(7) << "Left Heading Angle: " << raw_lane.Head_Ang_L << std::endl;

            break;
          }
        case 7:
        {
          ext_log_data_lka_right_lane_a_t right_a_unpacked;
          unpack_return = ext_log_data_lka_right_lane_a_unpack(&right_a_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);

          mobileye_obj.right_lane_type_decode =  ext_log_data_lka_right_lane_a_lane_type_decode(right_a_unpacked.lane_type); 
          mobileye_obj.right_lane_type_is_in_range =  ext_log_data_lka_right_lane_a_lane_type_is_in_range(right_a_unpacked.lane_type); 
          raw_lane.Lane_Type_R = mobeye_rx.signal_in_range(mobileye_obj.right_lane_type_decode, mobileye_obj.right_lane_type_is_in_range);
          
          mobileye_obj.right_quality_decode =  ext_log_data_lka_right_lane_a_quality_decode(right_a_unpacked.quality); 
          mobileye_obj.right_quality_is_in_range =  ext_log_data_lka_right_lane_a_quality_is_in_range(right_a_unpacked.quality); 
          raw_lane.Qual_R = mobeye_rx.signal_in_range(mobileye_obj.right_quality_decode, mobileye_obj.right_quality_is_in_range);

          mobileye_obj.right_position_decode =  ext_log_data_lka_right_lane_a_position_decode(right_a_unpacked.position); 
          mobileye_obj.right_position_is_in_range =  ext_log_data_lka_right_lane_a_position_is_in_range(right_a_unpacked.position); 
          raw_lane.LatPos_R = mobeye_rx.signal_in_range(mobileye_obj.right_position_decode, mobileye_obj.right_position_is_in_range);

          mobileye_obj.right_curvature_decode =  ext_log_data_lka_right_lane_a_curvature_decode(right_a_unpacked.curvature); 
          mobileye_obj.right_curvature_is_in_range =  ext_log_data_lka_right_lane_a_curvature_is_in_range(right_a_unpacked.curvature); 
          raw_lane.Curv_R = mobeye_rx.signal_in_range(mobileye_obj.right_curvature_decode, mobileye_obj.right_curvature_is_in_range);

          mobileye_obj.right_curvature_derivative_decode =  ext_log_data_lka_right_lane_a_curvature_derivative_decode(right_a_unpacked.curvature_derivative); 
          mobileye_obj.right_curvature_derivative_is_in_range =  ext_log_data_lka_right_lane_a_curvature_derivative_is_in_range(right_a_unpacked.curvature_derivative); 
          raw_lane.Curv_Deriv_R = mobeye_rx.signal_in_range(mobileye_obj.right_curvature_derivative_decode, mobileye_obj.right_curvature_derivative_is_in_range);
          
          std::cout << "Right Lane Type: " << raw_lane.Lane_Type_R << std::endl;
          std::cout << "Right Quality: " << raw_lane.Qual_R << std::endl;
          std::cout << std::setprecision(7) << "Right Position: " << raw_lane.LatPos_R << std::endl;
          std::cout << std::setprecision(7) << "Right Curvature: " << raw_lane.Curv_R << std::endl; 
          std::cout << std::setprecision(7) << "Right Curvature Derivative: " << raw_lane.Curv_Deriv_R << std::endl;
          break; 
        }
      case 8:
        {
          ext_log_data_lka_right_lane_b_t right_b_unpacked;  
          unpack_return = ext_log_data_lka_right_lane_b_unpack(&right_b_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);

          mobileye_obj.right_heading_angle_decode =  ext_log_data_lka_right_lane_b_heading_angle_decode(right_b_unpacked.heading_angle); 
          mobileye_obj.right_heading_angle_is_in_range =  ext_log_data_lka_right_lane_b_heading_angle_is_in_range(right_b_unpacked.heading_angle); 
          raw_lane.Head_Ang_R = mobeye_rx.signal_in_range(mobileye_obj.right_heading_angle_decode, mobileye_obj.right_heading_angle_is_in_range);

          break;
        }
      }
      ros::spinOnce();
      ros::Duration(0.5).sleep();
    }
  canBusOff(hnd);
  canClose(hnd);

  return 0;
}

