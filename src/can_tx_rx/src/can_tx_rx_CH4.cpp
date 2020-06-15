#include "can_tx_rx/mobileye_struct.h"

Mobileye_RX::Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
  sub = node_handle->subscribe(UNIT_TEST_SUBSCRIBER,TX_RX_MESSAGE_BUFFER_SIZE, &Mobileye_RX::sub_callback, this);
  pub = node_handle->advertise<&Mobileye_RX::>;
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

  double Mobileye_RX::signal_in_range(double val, bool cond){
    return (cond) ? (val) : 0; 
  }

void sub_callback(const common::mobileye_object_data_msg& output_obj){

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  Mobileye_RX mobeye_rx = Mobileye_RX(&can_tx_rx_CH4_handle); 

  common::mobileye_object_data_msg[3] obj_data; //one for each message (Frame A, B,C)

  canInitializeLibrary();

  uint8_t frame_num[3] = {0}; //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
  //uint8_t obj_num = -1; //0 to 63 is accepted
  
  mobileye_obj.channel_number = 3; 

  hnd = canOpenChannel(mobileye_obj.channel_number, sum(mobileye_obj.flag));

  if (hnd < 0) {
    char msg[64];
    canGetErrorText((canStatus)hnd, msg, sizeof(msg));
    fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
    exit(1);
  }

  canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
  canSetBusOutputControl(hnd, canDRIVER_NORMAL);
  canBusOn(hnd);

  while (ros::ok()) {
    //Check the validity of all three frames simultaneously
    canStatus[3] stat; 
    for (int index= 0; index < sizeof(stat); index++){
      stat[index] = canRead(hnd, &mobileye_obj[index].id, &mobileye_obj[index].can_data, &mobileye_obj[index].dlc, &mobileye_obj[index].flag, mobileye_obj[index].time_stamp);
    }
    
    if (canOK == stat[0] && canOK == stat[1] && canOK == stat[2]) {
      for(int index=0; index<sizeof(stat);index++){
        get_nums(id[index], &frame_num[index]);
      }

      for(int index = 0; index < sizeof(frame_num); index++){
        switch(frame_num[index]){
          case 1:
            ext_log_data_tsr_t data_tsr; 
            unpack_return = ext_log_data_tsr_unpack(&data_tsr, mobileye_obj[index].can_data, mobileye_obj[index].dlc);
          break;
          case 2:
            ext_log_data_obstacle_data_a_t obj_frame_a; 
            unpack_return = ext_log_data_obstacle_data_a_unpack(&obj_frame_a,mobileye_obj[index].can_data,mobileye_obj[index].dlc); 

            mobileye_obj[index].obstacle_vel_x_decode = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(obj_frame_a.obstacle_vel_x);
            mobileye_obj[index].obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(obj_frame_a.obstacle_vel_x);
            obj_data.MeVx = &Mobileye_RX::signal_in_range(mobileye_obj[index].obstacle_vel_x_decode, mobileye_obj[index].obstacle_vel_x_is_in_range); 

            mobileye_obj[index].object_pos_y_decode = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(obj_frame_a.obstacle_pos_y);
            mobileye_obj[index].obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(obj_frame_a.obstacle_pos_y);
            obj_data.MeDx = &Mobileye_RX::signal_in_range(mobileye_obj[index].obstacle_pos_y_decode, mobileye_obj[index].obstacle_pos_y_is_in_range); 

            mobileye_obj[index].obstacle_pos_x_decode = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(obj_frame_a.obstacle_pos_x);
            mobileye_obj[index].obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(obj_frame_a.obstacle_pos_x);
            obj_data.MeDy = &Mobileye_RX::signal_in_range(mobileye_obj[index].obstacle_pos_x_decode, mobileye_obj[index].obstacle_pos_x_is_in_range);

            obj_data.MeTimestamp = mobileye_obj[index].time_stamp;
          break; 
          case 3:
            ext_log_data_obstacle_data_b_t obj_frame_b; 
            unpack_return = ext_log_data_obstacle_data_b_unpack(&obj_frame_b,mobileye_obj[index].can_data,mobileye_obj[index].dlc); 

            //all_object_info.obstacle_radar_vel_x_decoded = ext_log_data_obstacle_data_b_obstacle_radar_vel_x_decode(obj_frame_b.obstacle_radar_vel_x);
            //all_object_info.obstacle_radar_vel_x_is_in_range = ext_log_data_obstacle_data_b_obstacle_radar_vel_x_is_in_range(obj_frame_b.obstacle_radar_vel_x);

            //all_object_info.radar_pos_x_decoded = ext_log_data_obstacle_data_b_radar_pos_x_decode(obj_frame_b.radar_pos_x);
            //all_object_info.radar_pos_x_is_in_range = ext_log_data_obstacle_data_b_radar_pos_x_is_in_range(obj_frame_b.radar_pos_x);
            
            //all_object_info.obstacle_lane_decoded = ext_log_data_obstacle_data_b_obstacle_lane_decode(obj_frame_b.obstacle_lane);
            //all_object_info.obstacle_lane_is_in_range = ext_log_data_obstacle_data_b_obstacle_lane_is_in_range(obj_frame_b. obstacle_lane);
          
          break; 
          case 4:
            ext_log_data_obstacle_data_c_t obj_frame_c; 
            unpack_return = ext_log_data_obstacle_data_c_unpack(&obj_frame_c,mobileye_obj[index].can_data,mobileye_obj[index].dlc);

            //all_object_info.object_accel_x_decoded = ext_log_data_obstacle_data_c_object_accel_x_decode(obj_frame_c.object_accel_x);
            //all_object_info.object_accel_x_is_in_range = ext_log_data_obstacle_data_c_object_accel_x_is_in_range(obj_frame_c.object_accel_x);
            //process information if valid
            //if in range, use value, if not, remove the data (0)
            //create the processed struct based on the above criteria 
            //send that to the subscriber
          break; 
          std::cout >> "Hey a loop finished";
        }
      }
    }

    ros::spinOnce();
  }
  
  canBusOff(hnd);
  canClose(hnd);

  return 0;
}

