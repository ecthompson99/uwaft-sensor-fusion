#include "can_tx_rx/mobileye_struct.h"
#define TX_RX_MESSAGE_BUFFER_SIZE 1000
#define canDRIVER_NORMAL 4
#define TOPIC_AD "Mobileye_CAN_Rx"
#define SIZE_OF_MSG 8 

Mobileye_RX::Mobileye_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
  //sub = node_handle->subscribe(UNIT_TEST_SUBSCRIBER,TX_RX_MESSAGE_BUFFER_SIZE, &Mobileye_RX::sub_callback, this);
  mob_pub = node_handle->advertise<common::mobileye_obj_data>(TOPIC_AD,10);
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

void sub_callback(const common::mobileye_object_data& output_obj){

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "can_tx_rx_CH4");
  ros::NodeHandle can_tx_rx_CH4_handle;

  Mobileye_RX mobeye_rx = Mobileye_RX(&can_tx_rx_CH4_handle); 
  common::mobileye_object_data obj_data; //Processed values 
  Mobileye_RX::mobileye_object mobileye_obj[3]; //one for each message (Frame A, B,C)
  Mobileye_RX mobileye = Mobileye_RX(); 
  canHandle hnd;
  canInitializeLibrary();

  
  //uint8_t obj_num = -1; //0 to 63 is accepted
  
  mobileye_obj.channel_number = 3; 

  hnd = canOpenChannel(mobileye_obj.channel_number, mobileye_obj[0].flag+mobileye_obj[1].flag+mobileye_obj[2].flag);

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
    canStatus stat; 
    for (int index= 0; index < 3; index++){
      stat = canRead(hnd, &mobileye_obj[index].id, &mobileye_obj[index].can_data, &mobileye_obj[index].dlc, &mobileye_obj[index].flag, &mobileye_obj[index].time_stamp);
      if(canOK==stat){
        int frame_num = get_nums(mobileye_obj[index]); //1 = TRS 2 = Frame A, 3 = Frame B, 4 = Frame C, others = error 
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
              int ext_log_data_tsr_t_unpack_status = ext_log_data_tsr_unpack(&mobileye.frame_tsr_unpacked, mobileye_obj[index].can_data, SIZE_OF_MSG);
              break;
            }
          case 2:
            {
              int ext_log_data_obstacle_data_a_unpack_status = ext_log_data_obstacle_data_a_unpack(&mobileye.frame_a_unpacked,mobileye_obj[index].can_data,SIZE_OF_MSG); 
              mobileye_obj[index].obstacle_vel_x_decode = ext_log_data_obstacle_data_a_obstacle_vel_x_decode(mobileye.frame_a_unpacked.obstacle_vel_x);
              mobileye_obj[index].obstacle_vel_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_vel_x_is_in_range(mobileye.frame_a_unpacked.obstacle_vel_x);
              obj_data.MeVx = Mobileye_RX::signal_in_range(mobileye_obj[index].obstacle_vel_x_decode, mobileye_obj[index].obstacle_vel_x_is_in_range); 

              mobileye_obj[index].object_pos_y_decode = ext_log_data_obstacle_data_a_obstacle_pos_y_decode(&mobileye.frame_a_unpacked.obstacle_pos_y);
              mobileye_obj[index].obstacle_pos_y_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_y_is_in_range(mobileye.frame_a_unpacked.obstacle_pos_y);
              obj_data.MeDx = Mobileye_RX::signal_in_range(mobileye_obj[index].obstacle_pos_y_decode, mobileye_obj[index].obstacle_pos_y_is_in_range); 

              mobileye_obj[index].obstacle_pos_x_decode = ext_log_data_obstacle_data_a_obstacle_pos_x_decode(&mobileye.frame_a_unpacked.obstacle_pos_x);
              mobileye_obj[index].obstacle_pos_x_is_in_range = ext_log_data_obstacle_data_a_obstacle_pos_x_is_in_range(mobileye.frame_a_unpacked.obstacle_pos_x);
              obj_data.MeDy = Mobileye_RX::signal_in_range(mobileye_obj[index].obstacle_pos_x_decode, mobileye_obj[index].obstacle_pos_x_is_in_range);

              obj_data.MeTimestamp = mobileye_obj[index].time_stamp;
              break; 
            }          
          case 3:
            {
              int ext_log_data_obstacle_data_b_unpack_status = ext_log_data_obstacle_data_b_unpack(mobileye.frame_b_unpacked,mobileye_obj[index].can_data,SIZE_OF_MSG); 
              break;
            }
          case 4:
            {
              int ext_log_data_obstacle_data_c_unpack_status = ext_log_data_obstacle_data_c_unpack(mobileye.frame_c_unpacked,mobileye_obj[index].can_data,SIZE_OF_MSG);
              break;
            }
            //if in range, use value, if not, remove the data (0)
            //create the processed struct based on the above criteria 
            //send that to the subscriber
       }   
      //std::cout >> "Hey a loop finished";
      }
      ros::spinOnce();     
    }
  canBusOff(hnd);
  canClose(hnd);

  return 0;
}

