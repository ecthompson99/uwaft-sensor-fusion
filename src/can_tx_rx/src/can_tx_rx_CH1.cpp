#include "cav_pcm_csw_struct.h"
#include "cav_pcm_csw.h"
#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8
#define TOPIC_TX "can_comms_data"

CAV_PCM_TX_RX::CAV_PCM_TX_RX(ros::NodeHandle* node_handle) : node_handle(node_handle){
    drive_ctrl_pub = node_handle->advertise<common::drive_ctrl_input_msg>(TOPIC_RX,MESSAGE_BUFFER_SIZE);
    can_comms_sub = node_handle ->subscribe(TOPIC_TX, MESSAGE_BUFFER_SIZE, &CAV_PCM_TX_RX::can_callback, this);
    cav_out = {0};
};

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
    canHandle hnd;
    canStatus stat;
    CAN_Helper::initialize_can(cav_in.channel_number, hnd, stat);
    
    while (ros::ok()) {
        /**
        hnd: handle of open circuit
        id: ptr to buffer that receives CAN identifier
        can_data: ptr to the buffer which receives the message data
        dlc: ptr to buffer that receives message len
        flag: ptr to buffer that receives message flag
        time: ptr to buffer that receives message timestamp
        **/
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

                    drive_ctrl.aeb_allowed = CAN_Helper::signal_in_range(cav_in.aeb_allowed_decode, cav_in.aeb_allowed_is_in_range);
                    drive_ctrl.acc_allowed = CAN_Helper::signal_in_range(cav_in.acc_allowed_decode, cav_in.acc_allowed_is_in_range);
                    drive_ctrl.lcc_allowed = CAN_Helper::signal_in_range(cav_in.lcc_allowed_decode, cav_in.lcc_allowed_is_in_range);
                    drive_ctrl.alive_rolling_counter_MABx = CAN_Helper::signal_in_range(cav_in.hsc_alive_decode, cav_in.hsc_alive_is_in_range);
                    drive_ctrl.veh_spd = CAN_Helper::signal_in_range(cav_in.veh_spd_decode, cav_in.veh_spd_is_in_range);
                    drive_ctrl.str_ang = CAN_Helper::signal_in_range(cav_in.str_ang_decode, cav_in.str_ang_is_in_range);
                    
                    // std::cout << "\nPCM input--- ";
                    // std::cout << "PCM to CAV RC1: " << cav_in.pcm_rc1_decode << std::endl;
                    // std::cout << "LCC Allowed: " << cav_in.lcc_allowed_decode << std::endl;
                    // std::cout << "ACC Allowed: " << cav_in.acc_allowed_decode << std::endl;
                    // std::cout << "AEB Allowed: " << cav_in.aeb_allowed_decode << std::endl;
                    // std::cout << "Vehicle speed: " << cav_in.veh_spd_decode << std::endl;
                    // std::cout << "Steering angle: " << cav_in.str_ang_decode << std::endl;
                    // std::cout << "PCM HSC alive: " << cav_in.hsc_alive_decode << std::endl;

                    cav_rx.drive_ctrl_pub.publish(drive_ctrl);
                    break;
                case 2:
                    emc_pcm_cav_interface_pcm_to_cav_2_t pcm2; 
                    cav_in.pcm_rc2_decode = emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_decode(pcm2.pcm_to_cav2_rc);
                    cav_in.pcm_rc2_is_in_range = emc_pcm_cav_interface_pcm_to_cav_2_pcm_to_cav2_rc_is_in_range(pcm2.pcm_to_cav2_rc);
                    
                    // std::cout << "PCM to CAV RC2: " << cav_in.pcm_rc2_decode << std::endl;

                    break;
                case 3:
                    emc_pcm_cav_interface_pcm_to_cav_3_t pcm3; 
                    cav_in.pcm_rc3_decode = emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_decode(pcm3.pcm_to_cav3_rc);
                    cav_in.pcm_rc3_is_in_range = emc_pcm_cav_interface_pcm_to_cav_3_pcm_to_cav3_rc_is_in_range(pcm3.pcm_to_cav3_rc);
                    
                    // std::cout << "PCM to CAV RC3: " << cav_in.pcm_rc3_decode << std::endl;

                    break;
            }
        }
            size_t size = 8u;
            
            struct emc_pcm_cav_interface_cav_lateral_ctrl_t lateral_ctrl; 
            struct emc_pcm_cav_interface_cav_long_ctrl_t long_ctrl; 

            lateral_ctrl.cav_lcc_valid = emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_valid_encode(cav_rx.cav_out.lcc_valid);
            lateral_ctrl.cav_lcc_str_ang = emc_pcm_cav_interface_cav_lateral_ctrl_cav_lcc_str_ang_encode(cav_rx.cav_out.lcc_steer);
            lateral_ctrl.cav_cav_sys_alive = emc_pcm_cav_interface_cav_lateral_ctrl_cav_cav_sys_alive_encode(cav_rx.cav_out.cav_rolling_counter);
            lateral_ctrl.cav_to_pcm_lat_pv = emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_pv_encode(cav_rx.cav_out.lat_pv);
            lateral_ctrl.cav_to_pcm_lat_rc = emc_pcm_cav_interface_cav_lateral_ctrl_cav_to_pcm_lat_rc_encode(cav_rx.cav_out.lat_rc);

            long_ctrl.cav_aeb_valid = emc_pcm_cav_interface_cav_long_ctrl_cav_aeb_valid_encode(cav_rx.cav_out.aeb_valid);
            long_ctrl.cav_acc_valid = emc_pcm_cav_interface_cav_long_ctrl_cav_acc_valid_encode(cav_rx.cav_out.acc_valid);
            long_ctrl.cav_long_accel = emc_pcm_cav_interface_cav_long_ctrl_cav_long_accel_encode(cav_rx.cav_out.long_accel);
            long_ctrl.cav_to_pcm_long_pv = emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_pv_encode(cav_rx.cav_out.long_pv);
            long_ctrl.cav_to_pcm_long_rc = emc_pcm_cav_interface_cav_long_ctrl_cav_to_pcm_long_rc_encode(cav_rx.cav_out.long_rc);

            uint8_t lateral_ctrl_msg[8] = {0};
            uint8_t long_ctrl_msg[8] = {0};

            struct emc_pcm_cav_interface_cav_lateral_ctrl_t *lateral_ctrl_struct = &lateral_ctrl;
            struct emc_pcm_cav_interface_cav_long_ctrl_t *long_ctrl_struct = &long_ctrl;

            emc_pcm_cav_interface_cav_long_ctrl_pack(long_ctrl_msg,long_ctrl_struct, size); 
            emc_pcm_cav_interface_cav_lateral_ctrl_pack(lateral_ctrl_msg,lateral_ctrl_struct, size);

            canWrite(hnd, 1040, lateral_ctrl_msg, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
            canWrite(hnd, 1056, long_ctrl_msg, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
            
            ros::Rate(10).sleep();

        ros::spinOnce();
    }
    canBusOff(hnd);
    canClose(hnd);
    return 0;
}