#include "can_tx_rx/mobileye_struct.h"
//add the structs to the header file
static const uint16_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;

Mobileye_RX::Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
  sub = node_handle->subscribe(UNIT_TEST_SUBSCRIBER,TX_RX_MESSAGE_BUFFER_SIZE, &Mobileye_RX::sub_callback, this);
  pub = node_handle->advertise();
}

void Mobileye_RX::get_nums(mobileye_object mobileye_obj, int id, uint8_t &case_n) {
  if(mobileye_obj.id >=1824 && mobileye_obj.id <=1830){
    case_n = 1; //Traffic Sensor 
  } else if(mobileye_obj.id >= 1849 && mobileye_obj.id <= 1876 && mobileye_obj.id % 3 == 1){
    case_n = 2; //Obstacle A Frame
  } else if(mobileye_obj.id >= 1850 && mobileye_obj.id <= 1877 && mobileye_obj.id % 3 == 2){
    case_n = 3; //Obstacle B Frame
  } else if(mobileye_obj.id >= 1851 && mobileye_obj.id <= 1878 && mobileye_obj.id % 3 == 0){
    case_n = 4; //Obstacle C Frame
  } else{
    case_n = 0; 
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  Mobileye_RX mobeye_rx = Mobileye_RX(&can_tx_rx_CH4_handle); 

  //ros::Publisher raw_obj_data_pub = can_tx_rx_CH4_handle.advertise<common::mobileye_object_data_msg>(
  //    "mobileye_object_data", TX_RX_MESSAGE_BUFFER_SIZE);

  common::mobileye_object_data_msg obj_data;

  int hnd;

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

  long int id[3];
  //unsigned int dlc;
  //unsigned int flag;
  unsigned long time;
  uint8_t size_of_msg = 8;
  uint8_t frame_num[3] = {0}; //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
  uint8_t obj_num = -1; //0 to 63 is accepted
  uint8_t can_data[8] = {0};
  uint8_t serialized_all_object_info[sizeof(all_object_info)];
  
  all_object_info.channel_number = 3; 

  int unpack_return = -1;  // 0 is successful, negative error code

  while (ros::ok()) {
    //Check the validity of all three frames simultaneously
    canStatus statA = canRead(hnd, &id[0], &can_data, &dlc, &flag, &time);
    canStatus statB = canRead(hnd, &id[1], &can_data, &dlc, &flag, &time); 
    canStatus statC = canRead(hnd, &id[2], &can_data, &dlc, &flag, &time);

    if (canOK == statA && canOK == statB && canOK == statC) {
      get_nums(id[0], &frame_num[0]);
      get_nums(id[1], &frame_num[1]);
      get_nums(id[2], &frame_num[2]);

      for(int i = 0; i< sizeof(frame_num); i++){
        switch(frame_num){
          case 1:
            ext_log_data_tsr_t data_tsr; 
            unpack_return = ext_log_data_tsr_unpack(can_data,size_of_msg);

          break;
          case 2:
            ext_log_data_obstacle_data_a_t obj_frame_a; 
            unpack_return = ext_log_data_obstacle_data_a_unpack(&obj_frame_a,can_data,size_of_msg); 

            all_object_info.obstacle_vel_x_decoded = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(obj_frame_a.obstacle_vel_x);
            all_object_info.obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(obj_frame_a.obstacle_vel_x);

            all_object_info.obstacle_pos_y_decoded = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(obj_frame_a.obstacle_pos_y);
            all_object_info.obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(obj_frame_a.obstacle_pos_y);

            all_object_info.obstacle_pos_x_decoded = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(obj_frame_a.obstacle_pos_x);
            all_object_info.obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(obj_frame_a.obstacle_pos_x);

          break; 
          case 3:
            ext_log_data_obstacle_data_b_t obj_frame_b; 
            unpack_return = ext_log_data_obstacle_data_b_unpack(&obj_frame_b,can_data,size_of_msg); 

            all_object_info.obstacle_radar_vel_x_decoded = ext_log_data_obstacle_data_b_obstacle_radar_vel_x_decode(obj_frame_b.obstacle_radar_vel_x);
            all_object_info.obstacle_radar_vel_x_is_in_range = ext_log_data_obstacle_data_b_obstacle_radar_vel_x_is_in_range(obj_frame_b.obstacle_radar_vel_x);

            all_object_info.radar_pos_x_decoded = ext_log_data_obstacle_data_b_radar_pos_x_decode(obj_frame_b.radar_pos_x);
            all_object_info.radar_pos_x_is_in_range = ext_log_data_obstacle_data_b_radar_pos_x_is_in_range(obj_frame_b.radar_pos_x);
            
            all_object_info.obstacle_lane_decoded = ext_log_data_obstacle_data_b_obstacle_lane_decode(obj_frame_b.obstacle_lane);
            all_object_info.obstacle_lane_is_in_range = ext_log_data_obstacle_data_b_obstacle_lane_is_in_range(obj_frame_b. obstacle_lane);
          
          break; 
          case 4:
            ext_log_data_obstacle_data_c_t obj_frame_c; 
            unpack_return = ext_log_data_obstacle_data_c_unpack(&obj_frame_c,can_data,size_of_msg);

            all_object_info.object_accel_x_decoded = ext_log_data_obstacle_data_c_object_accel_x_decode(obj_frame_c.object_accel_x);
            all_object_info.object_accel_x_is_in_range = ext_log_data_obstacle_data_c_object_accel_x_is_in_range(obj_frame_c.object_accel_x);
          //process information if valid
          //if in range, use value, if not, remove the data (0)
          //create the processed struct based on the above criteria 
          //send that to the subscriber
          break; 
        }
      }
    }

    ros::spinOnce();
  }
  
  canBusOff(hnd);
  canClose(hnd);

  return 0;
}

