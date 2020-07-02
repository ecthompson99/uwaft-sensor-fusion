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
  common::mobileye_object_data obj_data; //Processed values 
  Mobileye_RX::mobileye_object mobileye_obj[3]; //one for each message (Frame A, B,C)
  canHandle hnd;
  canInitializeLibrary();

  
  //uint8_t obj_num = -1; //0 to 63 is accepted
  for (int index = 0; index < 3;index++){
    mobileye_obj[index].channel_number = 3; 
  }
  

  //hnd = canOpenChannel(mobileye_obj[0].channel_number, mobileye_obj[0].flag+mobileye_obj[1].flag+mobileye_obj[2].flag);
  //marked for failure 
  //canOPEN_ACCEPT_VIRTUAL
  //canOPEN_EXCLUSIVE
  hnd = canOpenChannel(mobileye_obj[0].channel_number,canOPEN_ACCEPT_VIRTUAL); 

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
    //Check the validity of all three frames simultaneously
    canStatus stat; 
    int index = 0; 
    stat = canRead(hnd, &mobileye_obj[index].id, &mobileye_obj[index].can_data, &mobileye_obj[index].dlc, &mobileye_obj[index].flag, &mobileye_obj[index].time_stamp);
    if(canOK==stat){
      frame_num = mobeye_rx.get_nums(mobileye_obj[index]); //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
      //std::cout << "Frame Num = " << frame_num << std::endl; 
      if(frame_num-2 != index){
        frame_num = 0; //If not in the proper order, will automatically ignore the response (A-> B -> C)
      }
      else if(index != 0){
        if(mobileye_obj[index].id-mobileye_obj[index-1].id==1){
          frame_num = 0; 
        }//Based on mobileye id, the new mobileye obj num should be the same if it is 1 less 

      }
    }
    switch(frame_num){
        case 1://currently not in use 
          {
            int ext_log_data_tsr_t_unpack_status = ext_log_data_tsr_unpack(&mobeye_rx.frame_tsr_unpacked, mobileye_obj[index].can_data, SIZE_OF_MSG);
            break;
          }
        case 2:
          {
            int ext_log_data_obstacle_data_a_unpack_status = ext_log_data_obstacle_data_a_unpack(&mobeye_rx.frame_a_unpacked,mobileye_obj[index].can_data,SIZE_OF_MSG); 
            mobileye_obj[index].obstacle_vel_x_decode = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(mobeye_rx.frame_a_unpacked.obstacle_vel_x);
            mobileye_obj[index].obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(mobeye_rx.frame_a_unpacked.obstacle_vel_x);
            obj_data.MeVx = mobeye_rx.signal_in_range(mobileye_obj[index].obstacle_vel_x_decode, mobileye_obj[index].obstacle_vel_x_is_in_range); 

            mobileye_obj[index].obstacle_pos_y_decode = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(mobeye_rx.frame_a_unpacked.obstacle_pos_y);
            mobileye_obj[index].obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(mobeye_rx.frame_a_unpacked.obstacle_pos_y);
            obj_data.MeDx = mobeye_rx.signal_in_range(mobileye_obj[index].obstacle_pos_y_decode, mobileye_obj[index].obstacle_pos_y_is_in_range); 

            mobileye_obj[index].obstacle_pos_x_decode = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(mobeye_rx.frame_a_unpacked.obstacle_pos_x);
            mobileye_obj[index].obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(mobeye_rx.frame_a_unpacked.obstacle_pos_x);
            obj_data.MeDy = mobeye_rx.signal_in_range(mobileye_obj[index].obstacle_pos_x_decode, mobileye_obj[index].obstacle_pos_x_is_in_range);

            obj_data.MeTimestamp = mobileye_obj[index].time_stamp;

            std::cout << obj_data.MeVx << std::endl;
            std::cout << obj_data.MeDx << std::endl;
            std::cout << obj_data.MeDy << std::endl;
            std::cout <<obj_data.MeTimestamp << std::endl; 
            
            break; 
          }          
        case 3:
          {
            int ext_log_data_obstacle_data_b_unpack_status = ext_log_data_obstacle_data_b_unpack(&mobeye_rx.frame_b_unpacked,mobileye_obj[index].can_data,SIZE_OF_MSG); 
            break;
          }
        case 4:
          {
            int ext_log_data_obstacle_data_c_unpack_status = ext_log_data_obstacle_data_c_unpack(&mobeye_rx.frame_c_unpacked,mobileye_obj[index].can_data,SIZE_OF_MSG);
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

