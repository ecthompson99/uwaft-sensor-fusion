#include "cav_pcm_csw_struct.h"

void CAV_PCM_TX_RX::get_nums(int id, uint8_t &case_num){
    if(id == CAN_message.drive_ctrl_msg1){
        case_num = 1; //drive_ctrl message related (1st pcm to cav message from pcm, activation messages from csw)
    }
    else if(id == CAN_message.pcm_to_cav2){
        case_num = 2; //unused 2nd pcm to cav message, and will be ignored (potential for diagnostics later on)
    }
    else if(id == CAN_message.pcm_to_cav3){
        case_num = 3; //unused 3rd pcm to cav message, and will be ignored (potential for diagnostics later on)
    }
}
void CAV_PCM_TX_RX::can_callback(const common::can_comms_data_msg& recvd_data){
    cav_out.long_accel = recvd_data.long_accel;
    cav_out.lcc_steer = recvd_data.lcc_steer;
    cav_out.acc_valid = recvd_data.acc_valid;
    cav_out.aeb_valid = recvd_data.aeb_valid;
    cav_out.lcc_valid = recvd_data.lcc_valid;
    cav_out.acc_fault = recvd_data.acc_fault;
    cav_out.aeb_fault = recvd_data.aeb_fault;
    cav_out.lcc_fault = recvd_data.lcc_fault;
    cav_out.front_radar_fault = recvd_data.front_radar_fault;
    cav_out.left_radar_fault = recvd_data.left_radar_fault;
    cav_out.right_radar_fault = recvd_data.right_radar_fault;
    cav_out.mobileye_fault = recvd_data.mobileye_fault;
    cav_out.cav_rolling_counter =  recvd_data.alive_rolling_counter;

    cav_out.lat_pv++;
    cav_out.long_pv++;
    cav_out.lat_rc++;
    cav_out.long_rc++;

    // Reset PV and RC
    if (cav_out.lat_pv >= 2147483647u){
        cav_out.lat_pv = 0;
    }
    if (cav_out.long_pv >= 2147483647u){
        cav_out.long_pv = 0;
    }
    if (cav_out.lat_rc >= 3u){
        cav_out.lat_rc = 0;
    }
    if (cav_out.long_rc >= 3u){
        cav_out.long_rc = 0;
    }

}
