#include "radar_structs.h"
#include "radar.h"
#define canDRIVER_NORMAL 4
#define SIZE_OF_MSG 8 

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_tx_rx_CH2");
    ros::NodeHandle can_tx_rx_CH2_handle;
    //ros::NodeHandle diag_handle; 

    Radar_RX rad_rx = Radar_RX(&can_tx_rx_CH2_handle);
    SensorDiagnostics sens_diag = SensorDiagnostics(&can_tx_rx_CH2_handle);

    Radar_RX::radar_diagnostic_response diag_response;
    Radar_RX::radar_information radar_info;
    Radar_RX::target_tracking_info target_info;
    Radar_RX::object_tracking_info object_info; 

    diag_response.channel_number = 1;
    radar_info.channel_number = 1;
    target_info.channel_number = 1;
    object_info.channel_number = 1;

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
        
        //Rx node 
        stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);
        if (canOK == stat) {
            // Left corner radar = radar_1 and right corner radar = radar_2
            Radar_RX::get_nums(id, case_num, radar_num, frame_num, obj_num, target_object_num);
            std::cout << "ID, Case, Radar, Frame, Obj, Target_Obj" << std::endl;
            std::cout << +id << std::endl;
            std::cout << +case_num << std::endl;
            std::cout << +radar_num << std::endl;
            std::cout << +frame_num << std::endl;
            std::cout << +obj_num << std::endl;
            std::cout << +target_object_num << std::endl;

            switch (case_num) {
                case 1://diag responses
                    radar_diag_response_t r_diag_response_obj;
                    unpack_return =
                        radar_diag_response_unpack(&r_diag_response_obj, can_data, SIZE_OF_MSG);
                    diag_response.diagnostic_decode = radar_diag_response_r_diag_response_decode(
                        r_diag_response_obj.r_diag_response);
                    diag_response.diagnostic_is_in_range =
                        radar_diag_response_r_diag_response_is_in_range(
                        r_diag_response_obj.r_diag_response);
                    break;
                    diag_response.timestamp = time;
                    diag_response.radar_number = radar_num;

                case 2: //target tracking (not in use currently)
                    switch(frame_num){
                        case 1://a frame
                            radar_target_a_t r_target_a; 
                            unpack_return = 
                                radar_target_a_unpack(&r_target_a, can_data, SIZE_OF_MSG);
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
                        case 2:
                            radar_target_b_t r_target_b;
                            unpack_return = 
                                radar_target_b_unpack(&r_target_b, can_data, SIZE_OF_MSG);
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
                        unpack_return = 
                            radar_object_ender_unpack(&r_ender, can_data, SIZE_OF_MSG);

                        radar_info.tc_counter_decode = radar_object_ender_radar_tc_counter_decode(r_ender.radar_tc_counter);
                        radar_info.tc_counter_is_in_range = radar_object_ender_radar_tc_counter_is_in_range(r_ender.radar_tc_counter);
                        //diag_data.radar_tc_counter = rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);

                        radar_info.obj_ender_consist_bit_decode = radar_object_ender_radar_mess_ender_consist_bit_decode(r_ender.radar_mess_ender_consist_bit);
                        radar_info.obj_ender_consist_bit_is_in_range = radar_object_ender_radar_mess_ender_consist_bit_is_in_range(r_ender.radar_mess_ender_consist_bit);
                        //diag_data.radar_mess_ender_consist_bit = rad_rx.signals_in_range(radar_info.obj_ender_consist_bit_decode, radar_info.obj_ender_consist_bit_is_in_range);

                        radar_info.packet_checksum_encoded = radar_object_ender_radar_packet_checksum_decode(r_ender.radar_packet_checksum);
                        radar_info.packet_checksum_is_in_range = radar_object_ender_radar_packet_checksum_is_in_range(r_ender.radar_packet_checksum);
                        //diag_data.radar_packet_checksum = rad_rx.signals_in_range(radar_info.packet_checksum_encoded, radar_info.packet_checksum_is_in_range);
                    }
                    else if(id==1280||id==1282){//starters 
                        radar_object_starter_t r_starter; 
                        unpack_return = 
                            radar_object_starter_unpack(&r_starter, can_data, SIZE_OF_MSG);
                        
                        radar_info.veh_psi_dt_decode = radar_object_starter_radar_veh_psi_dt_decode(r_starter.radar_veh_psi_dt);
                        radar_info.veh_psi_dt_is_in_range = radar_object_starter_radar_veh_psi_dt_is_in_range(r_starter.radar_veh_psi_dt);
                        //diag_data.radar_tc_counter = rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);

                        radar_info.veh_v_ego_decode = radar_object_starter_radar_veh_v_ego_decode(r_starter.radar_veh_v_ego);
                        radar_info.veh_v_ego_is_in_range = radar_object_starter_radar_veh_v_ego_is_in_range(r_starter.radar_veh_v_ego);
                        //diag_data.radar_tc_counter = rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);
                        
                        radar_info.veh_a_ego_decode = radar_object_starter_radar_veh_a_ego_decode(r_starter.radar_veh_a_ego);
                        radar_info.veh_a_ego_is_in_range = radar_object_starter_radar_veh_a_ego_is_in_range(r_starter.radar_veh_a_ego);
                        //diag_data.radar_tc_counter = rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);

                        radar_info.veh_slip_angle_decode = radar_object_starter_radar_veh_slip_angle_decode(r_starter.radar_veh_slip_angle);
                        radar_info.veh_slip_angle_is_in_range = radar_object_starter_radar_veh_slip_angle_is_in_range(r_starter.radar_veh_slip_angle);
                        //diag_data.radar_tc_counter = rad_rx.signals_in_range(radar_info.tc_counter_decode, radar_info.tc_counter_is_in_range);

                        radar_info.mess_starter_consist_bit_decode = radar_object_starter_radar_mess_starter_consist_bit_decode(r_starter.radar_mess_starter_consist_bit);
                        radar_info.mess_starter_consist_bit_is_in_range = radar_object_starter_radar_mess_starter_consist_bit_is_in_range(r_starter.radar_mess_starter_consist_bit);
                        //diag_data.radar_mess_starter_consist_bit = rad_rx.signals_in_range(radar_info.mess_starter_consist_bit_decode, radar_info.mess_starter_consist_bit_is_in_range);
                    }
                    else if(id==1670||id==1672){//statuses

                        radar_status_t r_status;
                        unpack_return = 
                            radar_status_unpack(&r_status, can_data, SIZE_OF_MSG);
                        
                        radar_info.itc_info_decode = radar_status_r_stat_itc_info_decode(r_status.r_stat_itc_info);
                        radar_info.itc_info_is_in_range = radar_status_r_stat_itc_info_is_in_range(r_status.r_stat_itc_info);
                        //diag_data.r_stat_itc_info = rad_rx.signals_in_range(radar_info.itc_info_decode, radar_info.itc_info_is_in_range);

                        radar_info.sgu_fail_decode = radar_status_r_stat_sgu_fail_decode(r_status.r_stat_sgu_fail);
                        radar_info.sgu_fail_is_in_range = radar_status_r_stat_sgu_fail_is_in_range(r_status.r_stat_sgu_fail);
                        //diag_data.r_stat_sgu_fail = rad_rx.signals_in_range(radar_info.sgu_fail_decode, radar_info.sgu_fail_is_in_range);

                        radar_info.hw_fail_decode = radar_status_r_stat_hw_fail_decode(r_status.r_stat_hw_fail);
                        radar_info.hw_fail_is_in_range = radar_status_r_stat_hw_fail_is_in_range(r_status.r_stat_hw_fail);
                        //diag_data.r_stat_hw_fail = rad_rx.signals_in_range(radar_info.hw_fail_decode, radar_info.hw_fail_is_in_range);

                        radar_info.horizontal_misalignment_decode = radar_status_r_stat_horizontal_misalignment_decode(r_status.r_stat_horizontal_misalignment);
                        radar_info.horizontal_misalignment_is_in_range = radar_status_r_stat_horizontal_misalignment_is_in_range(r_status.r_stat_horizontal_misalignment);
                        //diag_data.r_stat_horizontal_misalignment = rad_rx.signals_in_range(radar_info.horizontal_misalignment_decode, radar_info.horizontal_misalignment_is_in_range);

                        radar_info.absorption_blindness_decode = radar_status_r_stat_absorption_blindness_decode(r_status.r_stat_absorption_blindness);
                        radar_info.absorption_blindness_is_in_range = radar_status_r_stat_absorption_blindness_is_in_range(r_status.r_stat_absorption_blindness);
                        //diag_data.r_stat_absorption_blindness = rad_rx.signals_in_range(radar_info.absorption_blindness_decode, radar_info.absorption_blindness_decode);

                        radar_info.distortion_blindness_decode = radar_status_r_stat_distortion_blindness_decode(r_status.r_stat_distortion_blindness);
                        radar_info.distortion_blindness_is_in_range = radar_status_r_stat_distortion_blindness_is_in_range(r_status.r_stat_distortion_blindness);
                        //diag_data.r_stat_distortion_blindness = rad_rx.signals_in_range(radar_info.distortion_blindness_decode, radar_info.distortion_blindness_is_in_range);

                        radar_info.mc_decode = radar_status_r_stat_mc_decode(r_status.r_stat_mc);
                        radar_info.mc_is_in_range = radar_status_r_stat_mc_is_in_range(r_status.r_stat_mc);
                        //diag_data.r_stat_mc = rad_rx.signals_in_range(radar_info.mc_decode, radar_info.mc_is_in_range);

                        radar_info.crc_decode = radar_status_r_stat_crc_decode(r_status.r_stat_crc);
                        radar_info.crc_is_in_range = radar_status_r_stat_crc_is_in_range(r_status.r_stat_crc);
                        //diag_data.r_stat_crc = rad_rx.signals_in_range(radar_info.crc_decode, radar_info.crc_is_in_range);                        
                    }
                    radar_info.timestamp = time;
                    radar_info.radar_number = radar_num;

                    //diag_data.timestamp = time;
                    //diag_data.radar_number = radar_num;
                    break;
                case 4://object tracking 
                    switch(frame_num){
                        case 1://a frame
                            radar_obj_a_t r_object_a; 
                            unpack_return = 
                                radar_obj_a_unpack(&r_object_a, can_data, SIZE_OF_MSG);
                            object_info.dx_decode = radar_obj_a_radar_obj_dx_decode(r_object_a.radar_obj_dx);
                            object_info.dx_is_in_range = radar_obj_a_radar_obj_dx_is_in_range(r_object_a.radar_obj_dx);
                            //radar_obj.RadarDx = rad_rx.signals_in_range(object_info.dx_decode, object_info.dx_is_in_range);

                            object_info.vx_decode = radar_obj_a_radar_obj_vx_decode(r_object_a.radar_obj_vx);
                            object_info.vx_is_in_range = radar_obj_a_radar_obj_vx_is_in_range(r_object_a.radar_obj_vx);
                            //radar_obj.RadarVx = rad_rx.signals_in_range(object_info.vx_decode, object_info.vx_is_in_range);

                            object_info.dy_decode = radar_obj_a_radar_obj_dy_decode(r_object_a.radar_obj_dy);
                            object_info.dy_is_in_range = radar_obj_a_radar_obj_dy_is_in_range(r_object_a.radar_obj_dy);
                            //radar_obj.RadarDy = rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);

                            object_info.w_exist_decode = radar_obj_a_radar_obj_w_exist_decode(r_object_a.radar_obj_w_exist);
                            object_info.w_exist_is_in_range = radar_obj_a_radar_obj_w_exist_is_in_range(r_object_a.radar_obj_w_exist);
                            //radar_obj.w_exist = rad_rx.signals_in_range(object_info.w_exist_decode, object_info.w_exist_is_in_range);

                            object_info.ax_decode = radar_obj_a_radar_obj_ax_decode(r_object_a.radar_obj_ax);
                            object_info.ax_is_in_range = radar_obj_a_radar_obj_ax_is_in_range(r_object_a.radar_obj_ax);
                            //radar_obj.RadarAx = rad_rx.signals_in_range(object_info.ax_decode, object_info.ax_is_in_range);

                            object_info.w_obstacle_decode = radar_obj_a_radar_obj_w_obstacle_decode(r_object_a.radar_obj_w_obstacle);
                            object_info.w_obstacle_is_in_range = radar_obj_a_radar_obj_w_obstacle_is_in_range(r_object_a.radar_obj_w_obstacle);
                            //radar_obj.w_obstacle = rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);

                            object_info.flag_valid_decode = radar_obj_a_radar_obj_flag_valid_decode(r_object_a.radar_obj_flag_valid);
                            object_info.flag_valid_is_in_range = radar_obj_a_radar_obj_flag_valid_is_in_range(r_object_a.radar_obj_flag_valid);
                            //radar_obj.flag_valid = rad_rx.signals_in_range(object_info.flag_valid_decode , object_info.flag_valid_is_in_range);

                            object_info.w_non_obstacle_decode = radar_obj_a_radar_obj_w_non_obstacle_decode(r_object_a.radar_obj_w_non_obstacle);
                            object_info.w_non_obstacle_is_in_range = radar_obj_a_radar_obj_w_non_obstacle_is_in_range(r_object_a.radar_obj_w_non_obstacle);
                            //radar_obj.w_non_obstacle = rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);
                            
                            object_info.flag_meas_decode = radar_obj_a_radar_obj_flag_meas_decode(r_object_a.radar_obj_flag_meas);
                            object_info.flag_meas_is_in_range = radar_obj_a_radar_obj_flag_meas_is_in_range(r_object_a.radar_obj_flag_meas);
                            //radar_obj.w_non_obstacle = rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);

                            object_info.flag_hist_decode = radar_obj_a_radar_obj_flag_hist_decode(r_object_a.radar_obj_flag_hist);
                            object_info.flag_hist_is_in_range = radar_obj_a_radar_obj_flag_hist_is_in_range(r_object_a.radar_obj_flag_hist);
                            //radar_obj.w_non_obstacle = rad_rx.signals_in_range(object_info.dy_decode, object_info.dy_is_in_range);

                            object_info.mess_aconsist_bit_decode = radar_obj_a_radar_obj_mess_aconsist_bit_decode(r_object_a.radar_obj_mess_aconsist_bit);
                            object_info.mess_aconsist_bit_is_in_range = radar_obj_a_radar_obj_mess_aconsist_bit_is_in_range(r_object_a.radar_obj_mess_aconsist_bit);
                            //diag_data.radar_mess_aconsist_bit = rad_rx.signals_in_range(object_info.mess_aconsist_bit_decode,object_info.mess_aconsist_bit_is_in_range);
                            break;
                        case 2:
                            radar_obj_b_t r_object_b;
                            unpack_return = 
                                radar_obj_b_unpack(&r_object_b, can_data, SIZE_OF_MSG);
                            object_info.vy_decode = radar_obj_b_radar_obj_vy_decode(r_object_b.radar_obj_vy);
                            object_info.vy_is_in_range = radar_obj_b_radar_obj_vy_is_in_range(r_object_b.radar_obj_vy);
                            //radar_obj.RadarVy = rad_rx.signals_in_range(object_info.vy_decode, object_info.vy_is_in_range);

                            object_info.d_length_decode = radar_obj_b_radar_obj_d_length_decode(r_object_b.radar_obj_d_length);
                            object_info.d_length_is_in_range = radar_obj_b_radar_obj_d_length_is_in_range(r_object_b.radar_obj_d_length);
                            //radar_obj. = rad_rx.signals_in_range(object_info.d_length_decode, object_info.d_length_is_in_range);

                            object_info.dz_decode = radar_obj_b_radar_obj_dz_decode(r_object_b.radar_obj_dz);
                            object_info.dz_is_in_range = radar_obj_b_radar_obj_dz_is_in_range(r_object_b.radar_obj_dz);
                            //radar_obj.RadarVy = rad_rx.signals_in_range(object_info.vy_decode, object_info.vy_is_in_range);

                            object_info.moving_state_decode = radar_obj_b_radar_obj_moving_state_decode(r_object_b.radar_obj_moving_state);
                            object_info.moving_state_is_in_range = radar_obj_b_radar_obj_moving_state_is_in_range(r_object_b.radar_obj_moving_state);
                            //radar_obj. = rad_rx.signals_in_range(object_info.d_length_decode, object_info.d_length_is_in_range);

                            object_info.dx_sigma_decode = radar_obj_b_radar_obj_dx_sigma_decode(r_object_b.radar_obj_dx_sigma);
                            object_info.dx_sigma_is_in_range = radar_obj_b_radar_obj_dx_sigma_is_in_range(r_object_b.radar_obj_dx_sigma);
                            //radar_obj.RadarDxSigma = rad_rx.signals_in_range(object_info.dx_sigma_decode, object_info.dx_sigma_is_in_range);

                            object_info.vx_sigma_decode = radar_obj_b_radar_obj_vx_sigma_decode(r_object_b.radar_obj_vx_sigma);
                            object_info.vx_sigma_is_in_range = radar_obj_b_radar_obj_vx_sigma_decode(r_object_b.radar_obj_vx_sigma);
                            //radar_obj.RadarVxSigma = rad_rx.signals_in_range(object_info.vx_sigma_decode, object_info.vx_sigma_is_in_range);

                            object_info.ax_sigma_decode = radar_obj_b_radar_obj_ax_sigma_decode(r_object_b.radar_obj_ax_sigma);
                            object_info.ax_sigma_is_in_range = radar_obj_b_radar_obj_ax_sigma_is_in_range(r_object_b.radar_obj_ax_sigma);
                            //radar_obj.RadarAxSigma = rad_rx.signals_in_range(object_info.ax_sigma_decode, object_info.ax_sigma_is_in_range);

                            object_info.dy_sigma_decode = radar_obj_b_radar_obj_dy_sigma_decode(r_object_b.radar_obj_dy_sigma);
                            object_info.dy_sigma_is_in_range = radar_obj_b_radar_obj_dy_sigma_is_in_range(r_object_b.radar_obj_dy_sigma);
                            //radar_obj.RadarDySigma = rad_rx.signals_in_range(object_info.dy_sigma_decode, object_info.dy_sigma_is_in_range);

                            object_info.w_class_decode = radar_obj_b_radar_obj_w_class_decode(r_object_b.radar_obj_w_class);
                            object_info.w_class_is_in_range = radar_obj_b_radar_obj_w_class_is_in_range(r_object_b.radar_obj_w_class);
                            //radar_obj.RadarDySigma = rad_rx.signals_in_range(object_info.dy_sigma_decode, object_info.dy_sigma_is_in_range);

                            object_info.class_decode = radar_obj_b_radar_obj_class_decode(r_object_b.radar_obj_class);
                            object_info.class_is_in_range = radar_obj_b_radar_obj_class_is_in_range(r_object_b.radar_obj_class);
                            //radar_obj.RadarDySigma = rad_rx.signals_in_range(object_info.dy_sigma_decode, object_info.dy_sigma_is_in_range);

                            object_info.dx_rear_end_loss_decode = radar_obj_b_radar_obj_dx_rear_end_loss_decode(r_object_b.radar_obj_dx_rear_end_loss);
                            object_info.dx_rear_end_loss_is_in_range = radar_obj_b_radar_obj_dx_rear_end_loss_is_in_range(r_object_b.radar_obj_dx_rear_end_loss);
                            //radar_obj.RadarDySigma = rad_rx.signals_in_range(object_info.dy_sigma_decode, object_info.dy_sigma_is_in_range);

                            object_info.mess_bconsist_bit_decode = radar_obj_b_radar_obj_mess_bconsist_bit_decode(r_object_b.radar_obj_mess_bconsist_bit);
                            object_info.mess_bconsist_bit_is_in_range = radar_obj_b_radar_obj_mess_bconsist_bit_is_in_range(r_object_b.radar_obj_mess_bconsist_bit);
                            //diag_data.radar_mess_bconsist_bit = rad_rx.signals_in_range(object_info.mess_bconsist_bit_decode,object_info.mess_bconsist_bit_is_in_range);
                            break;
                    }
                    object_info.timestamp = time;
                    object_info.radar_number = radar_num;
                    object_info.object_number = obj_num;

                    //radar_obj.RadarTimestamp = time;
                    //radar_obj.RadarNum = radar_num;
                    //radar_obj.ObjNum = obj_num;

                    //diag_data.timestamp = time;
                    //diag_data.radar_number = radar_num;

                    //std::cout << "Radar Time: " << +radar_obj.RadarTimestamp << std::endl;
                    //std::cout << "Radar Number: " << +radar_obj.RadarNum << std::endl; 
                    //std::cout << "Radar Obj Number: " << +radar_obj.ObjNum << std::endl;
                    break;
                break;
                if(case_num==3){
                    if(id == 1667 || id== 1665){//message must end with the ender bit
                        common::radar_object_data radar_obj; 
                        common::sensor_diagnostic_data_msg diag_data;
                        
                        //assign values here
                        //diag_data.radar_mess_aconsist_bit = rad_rx.signals_in_range(target_info.target_mess_aconsist_bit_decode,target_info.target_mess_aconsist_bit_is_in_range);
                        
                        //validate radar

                        //publish here
                        //rad_rx.rad_pub.publish(radar_obj);
                        //rad_rx.diag_pub.publish(diag_data);
                        
                        //clean up signals 
                        Radar_RX::radar_diagnostic_response blank_diag;
                        Radar_RX::radar_information blank_info;
                        Radar_RX::target_tracking_info blank_target;
                        Radar_RX::object_tracking_info blank_object; 

                        diag_response = blank_diag;
                        radar_info = blank_info;
                        target_info = blank_target;
                        object_info = blank_object; 
                    }
                }
      }
        //Tx node 
    }
    canBusOff(hnd);
    canClose(hnd);
    
    ros::spinOnce();
  }
  return 0;
}