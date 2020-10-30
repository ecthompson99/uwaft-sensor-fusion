#include "radar_structs.h"
#include "radar.h"
#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8 

void clear_classes(common::radar_object_data &radar_obj, common::sensor_diagnostic_data_msg &diag_data,     Radar_RX::radar_diagnostic_response &diag_response, Radar_RX::radar_information &radar_info,Radar_RX::target_tracking_info &target_info, Radar_RX::object_tracking_info &object_info, uint8_t &tc_check, uint8_t &mc_check){
    //clean up class message each cycle with blank defaults
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
    target_info = blank_target;
    object_info = blank_object;

    tc_check = 0;
    mc_check = 0;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_tx_rx_CH3");
    ros::NodeHandle can_tx_rx_CH3_handle;

    Radar_RX rad_rx = Radar_RX(&can_tx_rx_CH3_handle);
    SensorDiagnostics sens_diag = SensorDiagnostics(&can_tx_rx_CH3_handle);

    common::radar_object_data radar_obj; 
    common::sensor_diagnostic_data_msg diag_data; 

    Radar_RX::radar_diagnostic_response diag_response; // decoding
    Radar_RX::radar_information radar_info; // decoding
    Radar_RX::target_tracking_info target_info; // decoding
    Radar_RX::object_tracking_info object_info;  // decoding

    radar_input_veh_dyn_data_t in_veh_dyn; 
    radar_input_veh_dim_t in_veh_dim; 
    radar_input_wheel_info_t in_wheel_info; 
    radar_input_mount_info_t in_mount_info; 
    uint8_t counter = 0; 

    radar_info.channel_number = 2;
    object_info.channel_number = 2;

    long int id;
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;
    int case_num = 0;
    int radar_num = 0;           // 1 or 2 = valid
    int frame_num = 0;           // 1 = Frame_A, 2 = Frame_B, 3 = general, other = error
    int obj_num = -1;            // 0 to 31 = valid
    int target_object_num = -1;  // 0 to 5 = valid
    uint8_t can_data[8] = {0};

    int unpack_return = -1;  // 0 is successful, negative error code

    // for each radar
    uint8_t tc_check = 0;
    uint8_t mc_check  = 0;

    bool cycle_check = false; // false if not receive starting bit

    canInitializeLibrary();
    canHandle hnd;
    canStatus stat;
    
    hnd = canOpenChannel(radar_info.channel_number, canOPEN_ACCEPT_VIRTUAL);
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
        if (canOK == stat) {
            // Left corner radar = radar_1 and right corner radar = radar_2
            Radar_RX::get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num, radar_info.channel_number+1);
            std::cout << "ID, Case, Radar, Frame, Obj, Target_Obj" << std::endl;
            std::cout << +id << std::endl;
            std::cout << +case_num << std::endl;
            std::cout << +radar_num << std::endl;
            std::cout << +frame_num << std::endl;
            std::cout << +obj_num << std::endl;
            std::cout << +target_object_num << std::endl;

            // check for starter msg
            if (id == 1280 || id ==1282){
                cycle_check = true;
            }

            switch (case_num) {
                case 1://diag responses
                    radar_diag_response_t r_diag_response_obj;
                    unpack_return = radar_diag_response_unpack(&r_diag_response_obj, can_data, SIZE_OF_MSG);
                    diag_response.diagnostic_decode = radar_diag_response_r_diag_response_decode(r_diag_response_obj.r_diag_response);
                    diag_response.diagnostic_is_in_range = radar_diag_response_r_diag_response_is_in_range(r_diag_response_obj.r_diag_response);

                    diag_response.timestamp = time;
                    diag_response.radar_number = radar_num; // front, left, right

                    diag_data.radar_diag_response = rad_rx.signals_in_range(diag_response.diagnostic_decode, diag_response.diagnostic_is_in_range);  

                    break;

                case 2: //target tracking (NOT CURRENTLY IN USE)
                    switch(frame_num){
                        case 1:// Target A frame
                            radar_target_a_t r_target_a; 
                            unpack_return = radar_target_a_unpack(&r_target_a, can_data, SIZE_OF_MSG);
                            target_info.target_dx_decode = radar_target_a_radar_target_dx_decode(r_target_a.radar_target_dx);
                            target_info.target_dx_is_in_range = radar_target_a_radar_target_dx_is_in_range(r_target_a.radar_target_dx);

                            target_info.target_vx_decode = radar_target_a_radar_target_vx_decode(r_target_a.radar_target_vx);
                            target_info.target_vx_is_in_range = radar_target_a_radar_target_vx_is_in_range(r_target_a.radar_target_vx);

                            target_info.target_dy_decode = radar_target_a_radar_target_dy_decode(r_target_a.radar_target_dy);
                            target_info.target_dy_is_in_range = radar_target_a_radar_target_dy_is_in_range(r_target_a.radar_target_dy);

                            target_info.target_w_exist_decode = radar_target_a_radar_target_w_exist_decode(r_target_a.radar_target_w_exist);
                            target_info.target_w_exist_is_in_range = radar_target_a_radar_target_w_exist_is_in_range(r_target_a.radar_target_w_exist);

                            target_info.target_ax_decode = radar_target_a_radar_target_ax_decode(r_target_a.radar_target_ax);
                            target_info.target_ax_is_in_range = radar_target_a_radar_target_ax_is_in_range(r_target_a.radar_target_ax);

                            target_info.target_w_obstacle_decode = radar_target_a_radar_target_w_obstacle_decode(r_target_a.radar_target_w_obstacle);
                            target_info.target_w_obstacle_is_in_range = radar_target_a_radar_target_w_obstacle_is_in_range(r_target_a.radar_target_w_obstacle);

                            target_info.target_flag_valid_decode = radar_target_a_radar_target_flag_valid_decode(r_target_a.radar_target_flag_valid);
                            target_info.target_flag_valid_is_in_range = radar_target_a_radar_target_flag_valid_is_in_range(r_target_a.radar_target_flag_valid);

                            target_info.target_w_non_obstacle_decode = radar_target_a_radar_target_w_non_obstacle_decode(r_target_a.radar_target_w_non_obstacle);
                            target_info.target_w_non_obstacle_is_in_range = radar_target_a_radar_target_w_non_obstacle_is_in_range(r_target_a.radar_target_w_non_obstacle);
                            
                            target_info.target_flag_meas_decode = radar_target_a_radar_target_flag_meas_decode(r_target_a.radar_target_flag_meas);
                            target_info.target_flag_meas_is_in_range = radar_target_a_radar_target_flag_meas_is_in_range(r_target_a.radar_target_flag_meas);

                            target_info.target_flag_hist_decode = radar_target_a_radar_target_flag_hist_decode(r_target_a.radar_target_flag_hist);
                            target_info.target_flag_hist_is_in_range = radar_target_a_radar_target_flag_hist_is_in_range(r_target_a.radar_target_flag_hist);

                            target_info.target_mess_aconsist_bit_decode = radar_target_a_radar_target_mess_aconsist_bit_decode(r_target_a.radar_target_mess_aconsist_bit);
                            target_info.target_mess_aconsist_bit_is_in_range = radar_target_a_radar_target_mess_aconsist_bit_is_in_range(r_target_a.radar_target_mess_aconsist_bit); 

                            break;
                        case 2: // Target B frame
                            radar_target_b_t r_target_b;
                            unpack_return = radar_target_b_unpack(&r_target_b, can_data, SIZE_OF_MSG);
                            target_info.target_vy_decode = radar_target_b_radar_target_vy_decode(r_target_b.radar_target_vy);
                            target_info.target_vy_is_in_range = radar_target_b_radar_target_vy_is_in_range(r_target_b.radar_target_vy);

                            target_info.target_d_length_decode = radar_target_b_radar_target_d_length_decode(r_target_b.radar_target_d_length);
                            target_info.target_d_length_is_in_range = radar_target_b_radar_target_d_length_is_in_range(r_target_b.radar_target_d_length);

                            target_info.target_dz_decode = radar_target_b_radar_target_dz_decode(r_target_b.radar_target_dz);
                            target_info.target_dz_is_in_range = radar_target_b_radar_target_dz_is_in_range(r_target_b.radar_target_dz);

                            target_info.target_moving_state_decode = radar_target_b_radar_target_moving_state_decode(r_target_b.radar_target_moving_state);
                            target_info.target_moving_state_is_in_range = radar_target_b_radar_target_moving_state_is_in_range(r_target_b.radar_target_moving_state);

                            target_info.target_dx_sigma_decode = radar_target_b_radar_target_dx_sigma_decode(r_target_b.radar_target_dx_sigma);
                            target_info.target_dx_sigma_is_in_range = radar_target_b_radar_target_dx_sigma_decode(r_target_b.radar_target_dx_sigma);

                            target_info.target_vx_sigma_decode = radar_target_b_radar_target_vx_sigma_decode(r_target_b.radar_target_vx_sigma);
                            target_info.target_vx_sigma_is_in_range = radar_target_b_radar_target_vx_sigma_is_in_range(r_target_b.radar_target_vx_sigma);

                            target_info.target_ax_sigma_decode = radar_target_b_radar_target_ax_sigma_decode(r_target_b.radar_target_ax_sigma);
                            target_info.target_ax_sigma_is_in_range = radar_target_b_radar_target_ax_sigma_is_in_range(r_target_b.radar_target_ax_sigma);

                            target_info.target_dy_sigma_decode = radar_target_b_radar_target_dy_sigma_decode(r_target_b.radar_target_dy_sigma);
                            target_info.target_dy_sigma_is_in_range = radar_target_b_radar_target_dy_sigma_is_in_range(r_target_b.radar_target_dy_sigma);

                            target_info.target_w_class_decode = radar_target_b_radar_target_w_class_decode(r_target_b.radar_target_w_class);
                            target_info.target_w_class_is_in_range = radar_target_b_radar_target_class_is_in_range(r_target_b.radar_target_w_class);

                            target_info.target_class_decode = radar_target_b_radar_target_class_decode(r_target_b.radar_target_class);
                            target_info.target_class_is_in_range = radar_target_b_radar_target_class_is_in_range(r_target_b.radar_target_class);

                            target_info.target_dx_rear_end_loss_decode = radar_target_b_radar_target_dx_rear_end_loss_decode(r_target_b.radar_target_dx_rear_end_loss);
                            target_info.target_dx_rear_end_loss_is_in_range = radar_target_b_radar_target_dx_rear_end_loss_is_in_range(r_target_b.radar_target_dx_rear_end_loss);

                            target_info.target_mess_bconsist_bit_decode = radar_target_b_radar_target_mess_bconsist_bit_decode(r_target_b.radar_target_mess_bconsist_bit);
                            target_info.target_mess_bconsist_bit_is_in_range = radar_target_b_radar_target_mess_bconsist_bit_is_in_range(r_target_b.radar_target_mess_bconsist_bit);

                            break;
                    }
                    
                    break;
                case 3://enders, starters, or statuses
                    if(id==1665||id==1667){//enders
                        radar_object_ender_t r_ender; 
                        unpack_return = radar_object_ender_unpack(&r_ender, can_data, SIZE_OF_MSG);

                        radar_info.radar_timestamp_decode = radar_object_ender_radar_timestamp_decode(r_ender.radar_timestamp);
                        radar_info.radar_timestamp_is_in_range = radar_object_ender_radar_timestamp_is_in_range(r_ender.radar_timestamp);
                        
                        radar_info.tc_counter_decode = radar_object_ender_radar_tc_counter_decode(r_ender.radar_tc_counter);
                        radar_info.tc_counter_is_in_range = radar_object_ender_radar_tc_counter_is_in_range(r_ender.radar_tc_counter);

                        radar_info.obj_ender_consist_bit_decode = radar_object_ender_radar_mess_ender_consist_bit_decode(r_ender.radar_mess_ender_consist_bit);
                        radar_info.obj_ender_consist_bit_is_in_range = radar_object_ender_radar_mess_ender_consist_bit_is_in_range(r_ender.radar_mess_ender_consist_bit);

                        radar_info.packet_checksum_decode = radar_object_ender_radar_packet_checksum_decode(r_ender.radar_packet_checksum);
                        radar_info.packet_checksum_is_in_range = radar_object_ender_radar_packet_checksum_is_in_range(r_ender.radar_packet_checksum);

                        radar_obj.radar_timestamp = rad_rx.signals_in_range(radar_info.radar_timestamp_decode, radar_info.radar_timestamp_is_in_range);

                        diag_data.timestamp = rad_rx.signals_in_range(radar_info.radar_timestamp_decode, radar_info.radar_timestamp_is_in_range);
                        diag_data.radar_tc_counter = rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);
                        diag_data.radar_mess_ender_consist_bit = rad_rx.signals_in_range(radar_info.obj_ender_consist_bit_decode, radar_info.obj_ender_consist_bit_is_in_range); 
                        diag_data.radar_packet_checksum = rad_rx.signals_in_range(radar_info.packet_checksum_decode, radar_info.packet_checksum_is_in_range);
                   
                    }
                    else if(id==1280||id==1282){//starters 
                        radar_object_starter_t r_starter; 
                        unpack_return = radar_object_starter_unpack(&r_starter, can_data, SIZE_OF_MSG);
                        
                        radar_info.veh_psi_dt_decode = radar_object_starter_radar_veh_psi_dt_decode(r_starter.radar_veh_psi_dt);
                        radar_info.veh_psi_dt_is_in_range = radar_object_starter_radar_veh_psi_dt_is_in_range(r_starter.radar_veh_psi_dt);

                        radar_info.veh_v_ego_decode = radar_object_starter_radar_veh_v_ego_decode(r_starter.radar_veh_v_ego);
                        radar_info.veh_v_ego_is_in_range = radar_object_starter_radar_veh_v_ego_is_in_range(r_starter.radar_veh_v_ego);
                        
                        radar_info.veh_a_ego_decode = radar_object_starter_radar_veh_a_ego_decode(r_starter.radar_veh_a_ego);
                        radar_info.veh_a_ego_is_in_range = radar_object_starter_radar_veh_a_ego_is_in_range(r_starter.radar_veh_a_ego);

                        radar_info.veh_slip_angle_decode = radar_object_starter_radar_veh_slip_angle_decode(r_starter.radar_veh_slip_angle);
                        radar_info.veh_slip_angle_is_in_range = radar_object_starter_radar_veh_slip_angle_is_in_range(r_starter.radar_veh_slip_angle);

                        radar_info.mess_starter_consist_bit_decode = radar_object_starter_radar_mess_starter_consist_bit_decode(r_starter.radar_mess_starter_consist_bit);
                        radar_info.mess_starter_consist_bit_is_in_range = radar_object_starter_radar_mess_starter_consist_bit_is_in_range(r_starter.radar_mess_starter_consist_bit);

                        radar_obj.veh_psi_dot = rad_rx.signals_in_range(radar_info.veh_psi_dt_decode, radar_info.veh_psi_dt_is_in_range);
                        radar_obj.veh_v_ego = rad_rx.signals_in_range(radar_info.veh_v_ego_decode, radar_info.veh_v_ego_is_in_range);
                        radar_obj.veh_a_ego = rad_rx.signals_in_range(radar_info.veh_a_ego_decode, radar_info.veh_a_ego_is_in_range);
                        radar_obj.veh_slip_angle = rad_rx.signals_in_range(radar_info.veh_slip_angle_decode, radar_info.veh_slip_angle_is_in_range);

                        diag_data.radar_mess_starter_consist_bit = rad_rx.signals_in_range(radar_info.mess_starter_consist_bit_decode, radar_info.mess_starter_consist_bit_is_in_range);
                    }

                    else if(id==1670||id==1672){//statuses
                        radar_status_t r_status;
                        unpack_return = radar_status_unpack(&r_status, can_data, SIZE_OF_MSG);
                        
                        radar_info.itc_info_decode = radar_status_r_stat_itc_info_decode(r_status.r_stat_itc_info);
                        radar_info.itc_info_is_in_range = radar_status_r_stat_itc_info_is_in_range(r_status.r_stat_itc_info);

                        radar_info.sgu_fail_decode = radar_status_r_stat_sgu_fail_decode(r_status.r_stat_sgu_fail);
                        radar_info.sgu_fail_is_in_range = radar_status_r_stat_sgu_fail_is_in_range(r_status.r_stat_sgu_fail);

                        radar_info.hw_fail_decode = radar_status_r_stat_hw_fail_decode(r_status.r_stat_hw_fail);
                        radar_info.hw_fail_is_in_range = radar_status_r_stat_hw_fail_is_in_range(r_status.r_stat_hw_fail);

                        radar_info.horizontal_misalignment_decode = radar_status_r_stat_horizontal_misalignment_decode(r_status.r_stat_horizontal_misalignment);
                        radar_info.horizontal_misalignment_is_in_range = radar_status_r_stat_horizontal_misalignment_is_in_range(r_status.r_stat_horizontal_misalignment);

                        radar_info.absorption_blindness_decode = radar_status_r_stat_absorption_blindness_decode(r_status.r_stat_absorption_blindness);
                        radar_info.absorption_blindness_is_in_range = radar_status_r_stat_absorption_blindness_is_in_range(r_status.r_stat_absorption_blindness);

                        radar_info.distortion_blindness_decode = radar_status_r_stat_distortion_blindness_decode(r_status.r_stat_distortion_blindness);
                        radar_info.distortion_blindness_is_in_range = radar_status_r_stat_distortion_blindness_is_in_range(r_status.r_stat_distortion_blindness);

                        radar_info.mc_decode = radar_status_r_stat_mc_decode(r_status.r_stat_mc);
                        radar_info.mc_is_in_range = radar_status_r_stat_mc_is_in_range(r_status.r_stat_mc);

                        radar_info.crc_decode = radar_status_r_stat_crc_decode(r_status.r_stat_crc);
                        radar_info.crc_is_in_range = radar_status_r_stat_crc_is_in_range(r_status.r_stat_crc);

                        diag_data.r_stat_itc_info = rad_rx.signals_in_range(radar_info.itc_info_decode, radar_info.itc_info_is_in_range);
                        diag_data.r_stat_sgu_fail = rad_rx.signals_in_range(radar_info.sgu_fail_decode, radar_info.sgu_fail_is_in_range);
                        diag_data.r_stat_hw_fail = rad_rx.signals_in_range(radar_info.hw_fail_decode, radar_info.hw_fail_is_in_range);
                        diag_data.r_stat_horizontal_misalignment = rad_rx.signals_in_range(radar_info.horizontal_misalignment_decode, radar_info.horizontal_misalignment_is_in_range);
                        diag_data.r_stat_absorption_blindness = rad_rx.signals_in_range(radar_info.absorption_blindness_decode, radar_info.absorption_blindness_decode);
                        diag_data.r_stat_distortion_blindness = rad_rx.signals_in_range(radar_info.distortion_blindness_decode, radar_info.distortion_blindness_is_in_range);
                        diag_data.r_stat_mc = rad_rx.signals_in_range(radar_info.mc_decode, radar_info.mc_is_in_range);
                        diag_data.r_stat_crc = rad_rx.signals_in_range(radar_info.crc_decode, radar_info.crc_is_in_range);                        
                        diag_data.tc_check = tc_check; 
                        diag_data.mc_check = mc_check; 

                        Radar_RX::set_tc_mc(diag_data);
                   
                    }

                    break;
                case 4://object tracking 
                    switch(frame_num){
                        case 1://Tracking A frame
                            radar_obj_a_t r_object_a; 
                            unpack_return = radar_obj_a_unpack(&r_object_a, can_data, SIZE_OF_MSG);
                            object_info.dx_decode = radar_obj_a_radar_obj_dx_decode(r_object_a.radar_obj_dx);
                            object_info.dx_is_in_range = radar_obj_a_radar_obj_dx_is_in_range(r_object_a.radar_obj_dx);

                            object_info.vx_decode = radar_obj_a_radar_obj_vx_decode(r_object_a.radar_obj_vx);
                            object_info.vx_is_in_range = radar_obj_a_radar_obj_vx_is_in_range(r_object_a.radar_obj_vx);

                            object_info.dy_decode = radar_obj_a_radar_obj_dy_decode(r_object_a.radar_obj_dy);
                            object_info.dy_is_in_range = radar_obj_a_radar_obj_dy_is_in_range(r_object_a.radar_obj_dy);

                            object_info.w_exist_decode = radar_obj_a_radar_obj_w_exist_decode(r_object_a.radar_obj_w_exist);
                            object_info.w_exist_is_in_range = radar_obj_a_radar_obj_w_exist_is_in_range(r_object_a.radar_obj_w_exist);

                            object_info.ax_decode = radar_obj_a_radar_obj_ax_decode(r_object_a.radar_obj_ax);
                            object_info.ax_is_in_range = radar_obj_a_radar_obj_ax_is_in_range(r_object_a.radar_obj_ax);

                            object_info.w_obstacle_decode = radar_obj_a_radar_obj_w_obstacle_decode(r_object_a.radar_obj_w_obstacle);
                            object_info.w_obstacle_is_in_range = radar_obj_a_radar_obj_w_obstacle_is_in_range(r_object_a.radar_obj_w_obstacle);

                            object_info.flag_valid_decode = radar_obj_a_radar_obj_flag_valid_decode(r_object_a.radar_obj_flag_valid);
                            object_info.flag_valid_is_in_range = radar_obj_a_radar_obj_flag_valid_is_in_range(r_object_a.radar_obj_flag_valid);

                            object_info.w_non_obstacle_decode = radar_obj_a_radar_obj_w_non_obstacle_decode(r_object_a.radar_obj_w_non_obstacle);
                            object_info.w_non_obstacle_is_in_range = radar_obj_a_radar_obj_w_non_obstacle_is_in_range(r_object_a.radar_obj_w_non_obstacle);
                            
                            object_info.flag_meas_decode = radar_obj_a_radar_obj_flag_meas_decode(r_object_a.radar_obj_flag_meas);
                            object_info.flag_meas_is_in_range = radar_obj_a_radar_obj_flag_meas_is_in_range(r_object_a.radar_obj_flag_meas);

                            object_info.flag_hist_decode = radar_obj_a_radar_obj_flag_hist_decode(r_object_a.radar_obj_flag_hist);
                            object_info.flag_hist_is_in_range = radar_obj_a_radar_obj_flag_hist_is_in_range(r_object_a.radar_obj_flag_hist);

                            object_info.mess_aconsist_bit_decode = radar_obj_a_radar_obj_mess_aconsist_bit_decode(r_object_a.radar_obj_mess_aconsist_bit);
                            object_info.mess_aconsist_bit_is_in_range = radar_obj_a_radar_obj_mess_aconsist_bit_is_in_range(r_object_a.radar_obj_mess_aconsist_bit);

                            radar_obj.radar_dx[obj_num] = rad_rx.signals_in_range(object_info.dx_decode, object_info.dx_is_in_range);
                            radar_obj.radar_vx[obj_num] = rad_rx.signals_in_range(object_info.vx_decode, object_info.vx_is_in_range);
                            radar_obj.radar_dy[obj_num] = rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);
                            radar_obj.radar_w_exist[obj_num] = rad_rx.signals_in_range(object_info.w_exist_decode, object_info.w_exist_is_in_range);
                            radar_obj.radar_ax[obj_num] = rad_rx.signals_in_range(object_info.ax_decode, object_info.ax_is_in_range);
                            radar_obj.radar_w_obstacle[obj_num] = rad_rx.signals_in_range(object_info.w_obstacle_decode, object_info.w_obstacle_is_in_range);
                            radar_obj.radar_flag_valid[obj_num] = rad_rx.signals_in_range(object_info.flag_valid_decode, object_info.flag_valid_is_in_range);
                            radar_obj.radar_w_non_obstacle[obj_num] = rad_rx.signals_in_range(object_info.w_non_obstacle_decode, object_info.w_non_obstacle_is_in_range);
                            radar_obj.flag_meas[obj_num] = rad_rx.signals_in_range(object_info.flag_meas_decode, object_info.flag_meas_is_in_range);
                            radar_obj.flag_hist[obj_num] = rad_rx.signals_in_range(object_info.flag_hist_decode, object_info.flag_hist_is_in_range);

                            diag_data.radar_mess_aconsist_bit = rad_rx.signals_in_range(object_info.mess_aconsist_bit_decode, object_info.mess_aconsist_bit_is_in_range);
                            
                            break;
                        case 2: // Packet B
                            radar_obj_b_t r_object_b;
                            unpack_return = radar_obj_b_unpack(&r_object_b, can_data, SIZE_OF_MSG);
                            object_info.vy_decode = radar_obj_b_radar_obj_vy_decode(r_object_b.radar_obj_vy);
                            object_info.vy_is_in_range = radar_obj_b_radar_obj_vy_is_in_range(r_object_b.radar_obj_vy);

                            object_info.d_length_decode = radar_obj_b_radar_obj_d_length_decode(r_object_b.radar_obj_d_length);
                            object_info.d_length_is_in_range = radar_obj_b_radar_obj_d_length_is_in_range(r_object_b.radar_obj_d_length);

                            object_info.dz_decode = radar_obj_b_radar_obj_dz_decode(r_object_b.radar_obj_dz);
                            object_info.dz_is_in_range = radar_obj_b_radar_obj_dz_is_in_range(r_object_b.radar_obj_dz);

                            object_info.moving_state_decode = radar_obj_b_radar_obj_moving_state_decode(r_object_b.radar_obj_moving_state);
                            object_info.moving_state_is_in_range = radar_obj_b_radar_obj_moving_state_is_in_range(r_object_b.radar_obj_moving_state);

                            object_info.dx_sigma_decode = radar_obj_b_radar_obj_dx_sigma_decode(r_object_b.radar_obj_dx_sigma);
                            object_info.dx_sigma_is_in_range = radar_obj_b_radar_obj_dx_sigma_is_in_range(r_object_b.radar_obj_dx_sigma);

                            object_info.vx_sigma_decode = radar_obj_b_radar_obj_vx_sigma_decode(r_object_b.radar_obj_vx_sigma);
                            object_info.vx_sigma_is_in_range = radar_obj_b_radar_obj_vx_sigma_decode(r_object_b.radar_obj_vx_sigma);

                            object_info.ax_sigma_decode = radar_obj_b_radar_obj_ax_sigma_decode(r_object_b.radar_obj_ax_sigma);
                            object_info.ax_sigma_is_in_range = radar_obj_b_radar_obj_ax_sigma_is_in_range(r_object_b.radar_obj_ax_sigma);

                            object_info.dy_sigma_decode = radar_obj_b_radar_obj_dy_sigma_decode(r_object_b.radar_obj_dy_sigma);
                            object_info.dy_sigma_is_in_range = radar_obj_b_radar_obj_dy_sigma_is_in_range(r_object_b.radar_obj_dy_sigma);

                            object_info.w_class_decode = radar_obj_b_radar_obj_w_class_decode(r_object_b.radar_obj_w_class);
                            object_info.w_class_is_in_range = radar_obj_b_radar_obj_w_class_is_in_range(r_object_b.radar_obj_w_class);

                            object_info.class_decode = radar_obj_b_radar_obj_class_decode(r_object_b.radar_obj_class);
                            object_info.class_is_in_range = radar_obj_b_radar_obj_class_is_in_range(r_object_b.radar_obj_class);

                            object_info.dx_rear_end_loss_decode = radar_obj_b_radar_obj_dx_rear_end_loss_decode(r_object_b.radar_obj_dx_rear_end_loss);
                            object_info.dx_rear_end_loss_is_in_range = radar_obj_b_radar_obj_dx_rear_end_loss_is_in_range(r_object_b.radar_obj_dx_rear_end_loss);

                            object_info.mess_bconsist_bit_decode = radar_obj_b_radar_obj_mess_bconsist_bit_decode(r_object_b.radar_obj_mess_bconsist_bit);
                            object_info.mess_bconsist_bit_is_in_range = radar_obj_b_radar_obj_mess_bconsist_bit_is_in_range(r_object_b.radar_obj_mess_bconsist_bit);

                            radar_obj.radar_vy[obj_num] = rad_rx.signals_in_range(object_info.vy_decode, object_info.vy_is_in_range);
                            radar_obj.d_length[obj_num] = rad_rx.signals_in_range(object_info.d_length_decode, object_info.d_length_is_in_range);
                            radar_obj.radar_dz[obj_num] = rad_rx.signals_in_range(object_info.dz_decode, object_info.dz_is_in_range);
                            radar_obj.moving_state[obj_num] = rad_rx.signals_in_range(object_info.moving_state_decode, object_info.moving_state_is_in_range);
                            radar_obj.radar_dx_sigma[obj_num] = rad_rx.signals_in_range(object_info.dx_sigma_decode, object_info.dx_sigma_is_in_range);
                            radar_obj.radar_vx_sigma[obj_num] = rad_rx.signals_in_range(object_info.vx_sigma_decode, object_info.vx_sigma_is_in_range);
                            radar_obj.radar_dy_sigma[obj_num] = rad_rx.signals_in_range(object_info.dy_sigma_decode, object_info.dy_sigma_is_in_range);
                            radar_obj.radar_ax_sigma[obj_num] = rad_rx.signals_in_range(object_info.ax_sigma_decode, object_info.ax_sigma_is_in_range);
                            radar_obj.radar_w_class[obj_num] = rad_rx.signals_in_range(object_info.w_class_decode, object_info.w_class_is_in_range);
                            radar_obj.radar_obj_class[obj_num] = rad_rx.signals_in_range(object_info.class_decode, object_info.class_is_in_range);
                            radar_obj.dx_rear_loss[obj_num] = rad_rx.signals_in_range(object_info.dx_rear_end_loss_decode, object_info.dx_rear_end_loss_is_in_range);

                            diag_data.radar_mess_bconsist_bit = rad_rx.signals_in_range(object_info.mess_bconsist_bit_decode, object_info.mess_bconsist_bit_is_in_range);

                            break;
                    }

                    radar_obj.radar_timestamp = time;
                    radar_obj.radar_num = radar_num;

                    object_info.timestamp = time;
                    object_info.radar_number = radar_num;
                    object_info.object_number = obj_num;

                    break;
                break;
            }
            // check ender message
            if (cycle_check && (id == 1667 || id == 1665)){ // starter bit condition passed

                // service call to validate radars
                ros::ServiceClient client_ch3;
                common::sensor_diagnostic_flag_CH3 srv_ch3_left;
                common::sensor_diagnostic_flag_CH3 srv_ch3_right;

                if (radar_num == 1) // left radar
                {
                    srv_ch3_left.request.left_corner_radar = sens_diag.validate_radar(diag_data);
                    if (srv_ch3_left.request.left_corner_radar){
                        std::cout << "Valid Ch3 left corner" << std::endl;
                    }
                    else{
                        std::cout << "Invalid Ch3 service call for left radar" << std::endl;
                    }
                }
                
                else if (radar_num == 2) // right radar
                {
                    srv_ch3_left.request.right_corner_radar = sens_diag.validate_radar(diag_data);
                    if (srv_ch3_right.request.right_corner_radar){
                        std::cout << "Valid Ch3 right corner" << std::endl;
                    }
                    else{
                        std::cout << "Invalid Ch3 service call for right radar" << std::endl;
                    }
                }

                // publish here
                rad_rx.rad_pub.publish(radar_obj);
                rad_rx.diag_pub.publish(diag_data);

                clear_classes(radar_obj, diag_data,diag_response,radar_info,target_info, object_info, tc_check, mc_check);
                cycle_check = false; // reset for next data cluster
            }
            //Tx node 
            uint8_t radar_veh_dyn[8] =  {0};
            uint8_t radar_veh_dim[8] = {0};
            uint8_t radar_wheel_info[8] = {0};
            size_t size = 8u; 

            rad_rx.get_static_veh_info(in_mount_info, in_veh_dyn, in_wheel_info, in_veh_dim, radar_num);
            
            //rolling counter additions
            in_mount_info.ri_mi_mc  = radar_input_mount_info_ri_mi_mc_encode(counter);
            in_veh_dyn.ri_veh_mc = radar_input_veh_dyn_data_ri_veh_mc_encode(counter);
            in_veh_dim.ri_vd_mc = radar_input_veh_dim_ri_vd_mc_encode(counter);
            in_wheel_info.ri_wi_mc = radar_input_wheel_info_ri_wi_mc_encode(counter);

            //goal: ensure that there are 7 entries in the unsigned 8 bit integer, grouping smaller 1-4 bits signals or breaking up 16 bit signals as necessary
            //to do: research if crc requires bits to be properly ordered as shown in dbc and if there is an easier way to do so
            uint16_t int16_2bit;
            
            uint8_t in_mount_signals[7];

            int16_2bit = ((uint16_t)in_mount_info.ri_mi_lat_sensor_mount_to_center << 8) | in_mount_info.ri_mi_long_sensor_mount_to_rear_axle;
            in_mount_signals[0] = static_cast<uint8_t>((int16_2bit & 0xFF00) >> 8);
            in_mount_signals[1] = static_cast<uint8_t>(int16_2bit & 0x00FF);
            in_mount_signals[2] = in_mount_info.ri_mi_sensor_height;
            in_mount_signals[3] = in_mount_info.ri_mi_sensor_orientation;
            int16_2bit = ((uint16_t)in_mount_info.ri_mi_sensor_mount_angle << 8) | in_mount_info.ri_mi_sensor_mount_angle;
            in_mount_signals[4] = static_cast<uint8_t>((int16_2bit & 0xFF00) >> 8);
            in_mount_signals[5] = static_cast<uint8_t>(int16_2bit & 0x00FF);
            in_mount_signals[6] = in_mount_info.ri_mi_mc;

            uint8_t in_veh_dyn_signals[7];
            in_veh_dyn_signals[0] = in_veh_dyn.ri_veh_steer_angle;
            in_veh_dyn_signals[1] = in_veh_dyn.ri_veh_velocity;
            in_veh_dyn_signals[2] = in_veh_dyn.ri_veh_standstill;
            in_veh_dyn_signals[3] = in_veh_dyn.ri_veh_use_steer_angle;
            in_veh_dyn_signals[4] = in_veh_dyn.ri_veh_yaw_rate;
            in_veh_dyn_signals[5] = in_veh_dyn.ri_veh_prndstat+in_veh_dyn.ri_veh_any_wheel_slip_event;
            in_veh_dyn_signals[6] = in_veh_dyn.ri_veh_mc;

            uint8_t in_wheel_signals[7];
            in_wheel_signals[0] = in_wheel_info.ri_wi_wheel_base;
            in_wheel_signals[1] = in_wheel_info.ri_wi_track_width;
            in_wheel_signals[2] = in_wheel_info.ri_wi_steering_angle_ratio;
            in_wheel_signals[3] = in_wheel_info.ri_wi_mc;

            uint8_t in_veh_dim_signals[7];
            in_veh_dim_signals[0] = in_veh_dim.ri_vd_max_width;
            in_veh_dim_signals[1] = in_veh_dim.ri_vd_min_width;
            in_veh_dim_signals[2] = in_veh_dim.ri_vd_long_front_bumper_pos;
            in_veh_dim_signals[3] = in_veh_dim.ri_vd_long_rear_bumper_pos;
            in_veh_dim_signals[4] = in_veh_dim.ri_vd_mc;

            in_mount_info.ri_mi_crc = rad_rx.crc8bit_calculation(in_mount_signals);
            in_veh_dyn.ri_veh_crc = rad_rx.crc8bit_calculation(in_veh_dyn_signals);
            in_wheel_info.ri_wi_crc = rad_rx.crc8bit_calculation(in_wheel_signals);
            in_veh_dim.ri_vd_crc = rad_rx.crc8bit_calculation(in_veh_dim_signals);

            uint8_t radar_can_msg_mount[8] = {0};
            uint8_t radar_can_msg_veh_dyn[8] = {0};
            uint8_t radar_can_msg_wheel[8] = {0};
            uint8_t radar_can_msg_veh_dim[8] = {0};

            struct radar_input_veh_dyn_data_t *veh_dyn = &in_veh_dyn; 
            struct radar_input_veh_dim_t *veh_dim = &in_veh_dim; 
            struct radar_input_wheel_info_t *wheel_info = &in_wheel_info; 
            struct radar_input_mount_info_t *mount_info = &in_mount_info; 

            //radar_input_mount_info_pack(radar_can_msg_mount, in_mount_info, size);
            radar_input_veh_dyn_data_pack(radar_can_msg_veh_dyn, veh_dyn, size);
            //radar_input_wheel_info_pack(radar_can_msg_wheel, in_wheel_info, size);
            //radar_input_veh_dim_pack(radar_can_msg_veh_dim, in_veh_dim, size);

            
            canWrite(hnd, 200, radar_can_msg_veh_dyn, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
            canWrite(hnd, 201, radar_can_msg_veh_dim, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
            canWrite(hnd, 202, radar_can_msg_wheel, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);

            switch(radar_num){
                case 1:
                    canWrite(hnd, 490, radar_can_msg_mount, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
                    break;
                case 2:
                    canWrite(hnd, 492, radar_can_msg_mount, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
                    break;
                case 3:
                    canWrite(hnd, 490, radar_can_msg_mount, SIZE_OF_MSG, canOPEN_ACCEPT_VIRTUAL);
                    break;
            }
            if(counter>15){
                counter = 0;
            }
            else{
                counter++;
            }
       }
        ros::spinOnce();
    }
    canBusOff(hnd);
    canClose(hnd);
  return 0;
}