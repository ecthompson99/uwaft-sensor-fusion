#include "cav_pcm_csw_struct.h"
#include "cav_pcm_csw.h"

#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8
#define TOPIC_TX "CAN comms data"

CAV_PCM_TX_RX::CAV_PCM_TX_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
    drive_ctrl_pub = node_handle->advertise<common::drive_ctrl_input_msg>(TOPIC_RX,MESSAGE_BUFFER_SIZE);
    can_comms_sub = node_handle ->subscribe(TOPIC_TX, MESSAGE_BUFFER_SIZE, &CAV_PCM_TX_RX::can_callback, this);
    cav_out = {0};
};
double CAV_PCM_TX_RX::signals_in_range(double val, bool cond){
    return (cond) ? (val) : 0;
};
void CAV_PCM_TX_RX::get_nums(int id, uint8_t &case_num){
    if(id==1072){
        case_num = 1; //drive_ctrl message related (1st pcm to cav message from pcm, activation messages from csw)
    }
    else if(id==1073){
        case_num = 2; //unused 2nd pcm to cav message, and will be ignored (potential for diagnostics later on)
    }
    else if(id==1074){
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
}

int main(int argc, char **argv){
    ros::init(argc, argv, "can_tx_rx_CH1");
    ros::NodeHandle can_tx_rx_CH1_handle;
    
    CAV_PCM_TX_RX cav_rx = CAV_PCM_TX_RX(&can_tx_rx_CH1_handle);
    
    common::drive_ctrl_input_msg drive_ctrl;
    common::can_comms_data_msg can_comms; 

    CAV_PCM_TX_RX::cav_pcm_csw_in cav_in;

    cav_in.channel_number = 1; 

    long int id;
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;
    uint8_t case_num = 0;
    uint8_t can_data[8] = {0};

    int unpack_return = -1;  // 0 is successful, negative error code

    canInitializeLibrary();
    canHandle hnd;
    canStatus stat;

    hnd = canOpenChannel(cav_in.channel_number, canOPEN_ACCEPT_VIRTUAL);
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
        stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);
        cav_rx.get_nums(id, case_num);
        if (canOK == stat) {
            switch(case_num){
                case 1:
                    emc_pcm_cav_interface_pcm_to_cav_1_t pcm1;
                    unpack_return = emc_pcm_cav_interface_pcm_to_cav_1_unpack(&pcm1, can_data, SIZE_OF_MSG);
                    
                    cav_in.pcm_rc1_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_to_cav1_rc_decode(pcm1.pcm_to_cav1_rc);
                    cav_in.pcm_rc1_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_to_cav1_rc_is_in_range(pcm1.pcm_to_cav1_rc);

                    cav_in.aeb_allowed_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_aeb_allowed_decode(pcm1.pcm_aeb_allowed);
                    cav_in.aeb_allowed_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_aeb_allowed_is_in_range(pcm1.pcm_aeb_allowed);

                    cav_in.acc_allowed_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_acc_allowed_decode(pcm1.pcm_acc_allowed);
                    cav_in.acc_allowed_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_acc_allowed_is_in_range(pcm1.pcm_acc_allowed);

                    cav_in.lcc_allowed_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_lcc_allowed_decode(pcm1.pcm_lcc_allowed);
                    cav_in.lcc_allowed_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_lcc_allowed_is_in_range(pcm1.pcm_lcc_allowed);

                    cav_in.veh_spd_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_veh_spd_decode(pcm1.pcm_veh_spd);
                    cav_in.veh_spd_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_veh_spd_is_in_range(pcm1.pcm_veh_spd);

                    cav_in.hsc_alive_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_hsc_alive_decode(pcm1.pcm_hsc_alive);
                    cav_in.hsc_alive_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_hsc_alive_is_in_range(pcm1.pcm_hsc_alive);

                    cav_in.str_ang_decode = emc_pcm_cav_interface_pcm_to_cav_1_pcm_str_ang_decode(pcm1.pcm_str_ang);
                    cav_in.str_ang_is_in_range = emc_pcm_cav_interface_pcm_to_cav_1_pcm_str_ang_is_in_range(pcm1.pcm_str_ang);

                    drive_ctrl.aeb_allowed = cav_rx.signals_in_range(cav_in.aeb_allowed_decode, cav_in.aeb_allowed_is_in_range);
                    drive_ctrl.acc_allowed = cav_rx.signals_in_range(cav_in.acc_allowed_decode, cav_in.acc_allowed_is_in_range);
                    drive_ctrl.lcc_allowed = cav_rx.signals_in_range(cav_in.lcc_allowed_decode, cav_in.lcc_allowed_is_in_range);
                    drive_ctrl.alive_rolling_counter_MABx = cav_rx.signals_in_range(cav_in.hsc_alive_decode, cav_in.hsc_alive_is_in_range);
                    drive_ctrl.veh_spd = cav_rx.signals_in_range(cav_in.veh_spd_decode, cav_in.veh_spd_is_in_range);
                    drive_ctrl.str_ang = cav_rx.signals_in_range(cav_in.str_ang_decode, cav_in.str_ang_is_in_range);

                    cav_rx.drive_ctrl_pub.publish(drive_ctrl);
                    break;
                case 2:
                    emc_pcm_cav_interface_pcm_to_cav_2_t pcm2; 
                    cav_in.pcm_rc2_decode = emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_decode(pcm2.pcm_to_cav2_rc);
                    cav_in.pcm_rc2_is_in_range = emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_is_in_range(pcm2.pcm_to_cav2_rc);
                    break;
                case 3:
                    emc_pcm_cav_interface_pcm_to_cav_3_t pcm3; 
                    cav_in.pcm_rc3_decode = emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_decode(pcm3.pcm_to_cav3_rc);
                    cav_in.pcm_rc3_is_in_range = emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_is_in_range(pcm3.pcm_to_cav3_rc);
                    break;
            }
        }
            size_t size = 8u;
            
            struct emc_pcm_cav_interface_cav_lateral_ctrl_t lateral_ctrl; 
            struct emc_pcm_cav_interface_cav_long_ctrl_t long_ctrl; 

            lateral_ctrl.cav_lcc_valid = emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_valid_encode(cav_rx.cav_out.lcc_valid);
            lateral_ctrl.cav_lcc_str_ang = emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_str_ang_encode(cav_rx.cav_out.lcc_steer);
            lateral_ctrl.cav_cav_sys_alive = emc_pcm_cav_interface_cav_lateral_ctrl_cav_cav_sys_alive_encode(cav_rx.cav_out.cav_rolling_counter);
            
            long_ctrl.cav_aeb_valid = emc_pcm_cav_interface_cav_long_ctrl_cav_aeb_valid_encode(cav_rx.cav_out.aeb_valid);
            long_ctrl.cav_acc_valid = emc_pcm_cav_interface_cav_long_ctrl_cav_acc_valid_encode(cav_rx.cav_out.acc_valid);
            long_ctrl.cav_long_accel = emc_pcm_cav_interface_cav_long_ctrl_cav_long_accel_encode(cav_rx.cav_out.long_accel);
            
            uint8_t lateral_ctrl_msg[8] = {0};
            uint8_t long_ctrl_msg[8] = {0};

            struct emc_pcm_cav_interface_cav_lateral_ctrl_t *lateral_ctrl_struct = &lateral_ctrl;
            struct emc_pcm_cav_interface_cav_long_ctrl_t *long_ctrl_struct = &long_ctrl;

            emc_pcm_cav_interface_cav_long_ctrl_pack(long_ctrl_msg,long_ctrl_struct, size); 
            emc_pcm_cav_interface_cav_lateral_ctrl_pack(lateral_ctrl_msg,lateral_ctrl_struct, size);

            canWrite(hnd, 1040, lateral_ctrl_msg, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
            canWrite(hnd, 1056, long_ctrl_msg, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL); 

        ros::spinOnce();
    }
    canBusOff(hnd);
    canClose(hnd);
    return 0;
}