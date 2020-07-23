#include "can_tx_rx/mobileye_struct.h"
#define TX_RX_MESSAGE_BUFFER_SIZE 1000
#define canDRIVER_NORMAL 4
#define TOPIC_AD "Mobileye_CAN_Rx"
#define SIZE_OF_MSG 8 

Mobileye_RX::Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
  //sub = node_handle->subscribe(UNIT_TEST_SUBSCRIBER,TX_RX_MESSAGE_BUFFER_SIZE, &Mobileye_RX::sub_callback, this);
  mob_pub = node_handle->advertise<common::mobileye_object_data>("Topic",TX_RX_MESSAGE_BUFFER_SIZE);
}

uint8_t Mobileye_RX::get_nums(mobileye_object mobileye_obj) {
  if(mobileye_obj.id >=1824 && mobileye_obj.id <=1830){
    return 1; //Traffic Sensor 
  } else if(mobileye_obj.id >= 1849 && mobileye_obj.id <= 1876 && mobileye_obj.id % 3 == 1){
    return 2; //Obstacle A Frame
  } else if(mobileye_obj.id >= 1850 && mobileye_obj.id <= 1877 && mobileye_obj.id % 3 == 2){
    return 3; //Obstacle B Frame
  } else if(mobileye_obj.id >= 1851 && mobileye_obj.id <= 1878 && mobileye_obj.id % 3 == 0){
    return 4; //Obstacle C Frame
  } else if(mobileye_obj.id == 1894){
    return 5; //LKA Left Lane Frame A 
  } else if(mobileye_obj.id == 1895){
    return 6; //LKA Left Lane Frame B 
  } else if(mobileye_obj.id == 1896){
    return 7; //LKA Right Lane Frame A 
  } else if(mobileye_obj.id == 1897){
    return 8; //LKA Right Lane Frame B 
  } else{
    return 0; 
  }
}

double Mobileye_RX::signal_in_range(double val, bool cond){
    return (cond) ? (val) : 0; 
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  Mobileye_RX mobeye_rx = Mobileye_RX(&can_tx_rx_CH4_handle); 
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
  while (ros::ok()) {
    canStatus stat; 
    stat = canRead(hnd, &mobileye_obj.id, &mobileye_obj.can_data, &mobileye_obj.dlc, &mobileye_obj.flag, &mobileye_obj.time_stamp);
    if(canOK==stat){
      frame_num = mobeye_rx.get_nums(mobileye_obj); //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
    }
    switch(frame_num){
        case 2:
          {
            int ext_log_data_obstacle_data_a_unpack_status = ext_log_data_obstacle_data_a_unpack(&mobeye_rx.frame_a_unpacked,mobileye_obj.can_data,SIZE_OF_MSG); 
            mobileye_obj.obstacle_vel_x_decode = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(mobeye_rx.frame_a_unpacked.obstacle_vel_x);
            mobileye_obj.obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(mobeye_rx.frame_a_unpacked.obstacle_vel_x);
            obj_data.MeVx = mobeye_rx.signal_in_range(mobileye_obj.obstacle_vel_x_decode, mobileye_obj.obstacle_vel_x_is_in_range); 

            mobileye_obj.obstacle_pos_y_decode = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(mobeye_rx.frame_a_unpacked.obstacle_pos_y);
            mobileye_obj.obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(mobeye_rx.frame_a_unpacked.obstacle_pos_y);
            obj_data.MeDx = mobeye_rx.signal_in_range(mobileye_obj.obstacle_pos_y_decode, mobileye_obj.obstacle_pos_y_is_in_range); 

            mobileye_obj.obstacle_pos_x_decode = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(mobeye_rx.frame_a_unpacked.obstacle_pos_x);
            mobileye_obj.obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(mobeye_rx.frame_a_unpacked.obstacle_pos_x);
            obj_data.MeDy = mobeye_rx.signal_in_range(mobileye_obj.obstacle_pos_x_decode, mobileye_obj.obstacle_pos_x_is_in_range);

            obj_data.MeTimestamp = mobileye_obj.time_stamp;
            /*
            std::cout << "Velocity: " << obj_data.MeVx << std::endl;
            std::cout << "Longitudinal Distance: " << obj_data.MeDx << std::endl;
            std::cout << "Lateral Distance: " << obj_data.MeDy << std::endl;
            std::cout << "Message sent at " << obj_data.MeTimestamp << std::endl; 
            */
            break; 
          }          
        case 3:
          {
            int ext_log_data_obstacle_data_b_unpack_status = ext_log_data_obstacle_data_b_unpack(&mobeye_rx.frame_b_unpacked,mobileye_obj.can_data,SIZE_OF_MSG); 
            break;
          }
        case 4:
          {
            int ext_log_data_obstacle_data_c_unpack_status = ext_log_data_obstacle_data_c_unpack(&mobeye_rx.frame_c_unpacked,mobileye_obj.can_data,SIZE_OF_MSG);
            break;
          }
        case 5:
          {
            int ext_log_data_lka_left_lane_a_unpack_status = ext_log_data_lka_left_lane_a_unpack(&mobeye_rx.left_a_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);
            std::cout << "Lane Type Encoded: " << mobeye_rx.left_a_unpacked.lane_type << std::endl; 
            mobileye_obj.left_lane_type_decode =  ext_log_data_lka_left_lane_a_lane_type_decode(mobeye_rx.left_a_unpacked.lane_type); 
            mobileye_obj.left_lane_type_is_in_range =  ext_log_data_lka_left_lane_a_lane_type_is_in_range(mobeye_rx.left_a_unpacked.lane_type); 
            raw_lane.Lane_Type_L = mobeye_rx.signal_in_range(mobileye_obj.left_lane_type_decode, mobileye_obj.left_lane_type_is_in_range);
            
            mobileye_obj.left_quality_decode =  ext_log_data_lka_left_lane_a_quality_decode(mobeye_rx.left_a_unpacked.quality); 
            mobileye_obj.left_quality_is_in_range =  ext_log_data_lka_left_lane_a_quality_is_in_range(mobeye_rx.left_a_unpacked.quality); 
            raw_lane.Qual_L = mobeye_rx.signal_in_range(mobileye_obj.left_quality_decode, mobileye_obj.left_quality_is_in_range);

            mobileye_obj.left_position_decode =  ext_log_data_lka_left_lane_a_position_decode(mobeye_rx.left_a_unpacked.position); 
            mobileye_obj.left_position_is_in_range =  ext_log_data_lka_left_lane_a_position_is_in_range(mobeye_rx.left_a_unpacked.position); 
            raw_lane.LatPos_L = mobeye_rx.signal_in_range(mobileye_obj.left_position_decode, mobileye_obj.left_position_is_in_range);

            mobileye_obj.left_curvature_decode =  ext_log_data_lka_left_lane_a_curvature_decode(mobeye_rx.left_a_unpacked.curvature); 
            mobileye_obj.left_curvature_is_in_range =  ext_log_data_lka_left_lane_a_curvature_is_in_range(mobeye_rx.left_a_unpacked.curvature); 
            raw_lane.Curv_L = mobeye_rx.signal_in_range(mobileye_obj.left_curvature_decode, mobileye_obj.left_curvature_is_in_range);

            mobileye_obj.left_curvature_derivative_decode =  ext_log_data_lka_left_lane_a_curvature_derivative_decode(mobeye_rx.left_a_unpacked.curvature_derivative); 
            mobileye_obj.left_curvature_derivative_is_in_range =  ext_log_data_lka_left_lane_a_curvature_derivative_is_in_range(mobeye_rx.left_a_unpacked.curvature_derivative); 
            raw_lane.Curv_Deriv_L = mobeye_rx.signal_in_range(mobileye_obj.left_curvature_derivative_decode, mobileye_obj.left_curvature_derivative_is_in_range);
            
            std::cout << "Left Lane Type: " << raw_lane.Lane_Type_L << std::endl;
            /*
            std::cout << "Left Quality: " << raw_lane.Qual_L << std::endl;
            std::cout << "Left Position: " << raw_lane.LatPos_L << std::endl;
            std::cout << "Left Curvature " << raw_lane.Curv_L << std::endl; 
            std::cout << "Left Curvature Derivative: " << raw_lane.Curv_Deriv_L << std::endl;
            */
            break; 
          }
        case 6:
          {
            int ext_log_data_lka_left_lane_b_unpack_status = ext_log_data_lka_left_lane_b_unpack(&mobeye_rx.left_b_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);
            mobileye_obj.left_heading_angle_decode =  ext_log_data_lka_left_lane_b_heading_angle_decode(mobeye_rx.left_b_unpacked.heading_angle); 
            mobileye_obj.left_heading_angle_is_in_range =  ext_log_data_lka_left_lane_b_heading_angle_is_in_range(mobeye_rx.left_b_unpacked.heading_angle); 
            raw_lane.Head_Ang_L = mobeye_rx.signal_in_range(mobileye_obj.left_heading_angle_decode, mobileye_obj.left_heading_angle_is_in_range);
            
            //std::cout << "Left Heading Angle: " << raw_lane.Head_Ang_L << std::endl;

            break;
          }
        case 7:
        {
          int ext_log_data_lka_right_lane_a_unpack_status = ext_log_data_lka_right_lane_a_unpack(&mobeye_rx.right_a_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);
          mobileye_obj.right_lane_type_decode =  ext_log_data_lka_right_lane_a_lane_type_decode(mobeye_rx.right_a_unpacked.lane_type); 
          mobileye_obj.right_lane_type_is_in_range =  ext_log_data_lka_right_lane_a_lane_type_is_in_range(mobeye_rx.right_a_unpacked.lane_type); 
          raw_lane.Lane_Type_R = mobeye_rx.signal_in_range(mobileye_obj.right_lane_type_decode, mobileye_obj.right_lane_type_is_in_range);
          
          mobileye_obj.right_quality_decode =  ext_log_data_lka_right_lane_a_quality_decode(mobeye_rx.right_a_unpacked.quality); 
          mobileye_obj.right_quality_is_in_range =  ext_log_data_lka_right_lane_a_quality_is_in_range(mobeye_rx.right_a_unpacked.quality); 
          raw_lane.Qual_R = mobeye_rx.signal_in_range(mobileye_obj.right_quality_decode, mobileye_obj.right_quality_is_in_range);

          mobileye_obj.right_position_decode =  ext_log_data_lka_right_lane_a_position_decode(mobeye_rx.right_a_unpacked.position); 
          mobileye_obj.right_position_is_in_range =  ext_log_data_lka_right_lane_a_position_is_in_range(mobeye_rx.right_a_unpacked.position); 
          raw_lane.LatPos_R = mobeye_rx.signal_in_range(mobileye_obj.right_position_decode, mobileye_obj.right_position_is_in_range);

          mobileye_obj.right_curvature_decode =  ext_log_data_lka_right_lane_a_curvature_decode(mobeye_rx.right_a_unpacked.curvature); 
          mobileye_obj.right_curvature_is_in_range =  ext_log_data_lka_right_lane_a_curvature_is_in_range(mobeye_rx.right_a_unpacked.curvature); 
          raw_lane.Curv_R = mobeye_rx.signal_in_range(mobileye_obj.right_curvature_decode, mobileye_obj.right_curvature_is_in_range);

          mobileye_obj.right_curvature_derivative_decode =  ext_log_data_lka_right_lane_a_curvature_derivative_decode(mobeye_rx.right_a_unpacked.curvature_derivative); 
          mobileye_obj.right_curvature_derivative_is_in_range =  ext_log_data_lka_right_lane_a_curvature_derivative_is_in_range(mobeye_rx.right_a_unpacked.curvature_derivative); 
          raw_lane.Curv_Deriv_R = mobeye_rx.signal_in_range(mobileye_obj.right_curvature_derivative_decode, mobileye_obj.right_curvature_derivative_is_in_range);
          /*
          std::cout << "Right Lane Type: " << raw_lane.Lane_Type_R << std::endl;
          std::cout << "Right Quality: " << raw_lane.Qual_R << std::endl;
          std::cout << "Right Position: " << raw_lane.LatPos_R << std::endl;
          std::cout << "Right Curvature: " << raw_lane.Curv_R << std::endl; 
          std::cout << "Right Curvature Derivative: " << raw_lane.Curv_Deriv_R << std::endl;
          */
          break; 
        }
      case 8:
        {
          int ext_log_data_lka_right_lane_b_unpack_status = ext_log_data_lka_right_lane_b_unpack(&mobeye_rx.right_b_unpacked, mobileye_obj.can_data,SIZE_OF_MSG);
          mobileye_obj.right_heading_angle_decode =  ext_log_data_lka_right_lane_b_heading_angle_decode(mobeye_rx.right_b_unpacked.heading_angle); 
          mobileye_obj.right_heading_angle_is_in_range =  ext_log_data_lka_right_lane_b_heading_angle_is_in_range(mobeye_rx.right_b_unpacked.heading_angle); 
          raw_lane.Head_Ang_R = mobeye_rx.signal_in_range(mobileye_obj.right_heading_angle_decode, mobileye_obj.right_heading_angle_is_in_range);
          
          //std::cout << "Right Heading Angle: " << raw_lane.Head_Ang_R << std::endl;

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

