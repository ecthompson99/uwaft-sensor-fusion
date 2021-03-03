#include "radar_structs.h"

void Radar_RX::get_nums(int id, int &case_n, int &radar_n, int &frame_n, int &obj_n, int &target_obj_n, int channel_number) {
    if (id == CAN_message.diag_response1 || id == CAN_message.diag_response2 || id == CAN_message.diag_request1 || id == CAN_message.diag_request2) {
        case_n = 1; //diag responses and requests
    } else if (id > CAN_message.target_AB1 && id < CAN_message.target_AB2) {
        case_n = 2; //target A and B frames (?) the IDs are incorrectly calculated from the dbc 
    } else if (id == CAN_message.ender1 || id == CAN_message.ender2 || id == CAN_message.starter1 || id == CAN_message.starter2 || id == CAN_message.status1 || id == CAN_message.status2) {
        case_n = 3; //ender, starter, and statuses messages
    } else if (id > CAN_message.radar_AB1 && id < CAN_message.radar_AB2) {
        case_n = 4; //radar A and B object frames 
    } else {
        case_n = 0; //faulted 
    }
    
    //default these values should be set to -1 (0 is used)
    obj_n = -1;
    target_obj_n = -1; 
    radar_n = -1;
    frame_n = -1; 

    switch (case_n) {
        case 1: //diag responses and requests
        if(channel_number == 2){
            radar_n = 3; //front radar
        }
        else if (id == CAN_message.diag_response1 || id == CAN_message.diag_request1) {
            radar_n = 1;//right corner radar  
        } else if (id == CAN_message.diag_response2 || id == CAN_message.diag_request2){
            radar_n = 2;//left corner radar
        }
        break;

        case 2: //target A and B frames 
        if(channel_number == 2){
            radar_n = 3; //front radar
        }
        else if (id % 10 == 5 || id % 10 == 6) {
            radar_n = 1;//radar 1 in dbc (all ids for targets end with a 5 or a 6)
        } else if (id %10 == 7 || id % 10 == 8){
            radar_n = 2; //radar 2 in dbc 
        }

        if (id % 10 == 5 || id % 10 == 7) {
            frame_n = 1; //a frame in dbc (all ids for targets end with a 5 if they are radar 1, or 7 if they are  radar 2)
        } else if (id %10 == 6 || id % 10 == 8){
            frame_n = 2; //frame b in dbc 
        }

        target_obj_n = (id - 1600 - (id % 10)) / 10; //takes the target object number based on the defined id 

        break;

        case 3: //ender, starter, and statuses messages
        if(channel_number == 2){
            radar_n = 3; //front radar
        }
        else if (id == CAN_message.ender1 || id == CAN_message.starter1 || id == CAN_message.status1) {
            radar_n = 1; //radar 1 in dbc 1670
        } else if (id == CAN_message.ender2 || id == CAN_message.starter2 || id == CAN_message.status2){
            radar_n = 2; //radar 2 in dbc 
        }
        break;

        case 4://radar A and B object frames 
        if(channel_number == 2){
            radar_n = 3; //front radar
        }
        if (id % 10 == 5 || id % 10 == 6) {
            radar_n = 1; //radar 1 in dbc (all ids follow the same convention as target messages)
        } else if (id %10 == 7 || id % 10 == 8){
            radar_n = 2; //radar 2 in dbc 
        }

        if (id % 10 == 5 || id % 10 == 7) {
            frame_n = 1; //a frame in dbc (all ids follow the same convention as target messages)
        } else  if(id % 10 == 6 || id % 10 == 8){
            frame_n = 2; //b frame in dbc 
        }

        obj_n = (id - CAN_message.starter1 - (id % 10)) / 10; //takes the tracked object number based on the defined id 

        break;
    }
};

void Radar_RX::get_static_veh_info(radar_input_mount_info_t &in_mount_info, radar_input_veh_dyn_data_t &in_veh_dyn, radar_input_wheel_info_t &in_wheel_info, radar_input_veh_dim_t &in_veh_dim, int radar_num){
    //Mount Info
    float latsensor_tocenter; 
    float longsensor_torear; 
    float sensor_height; 
    bool sensor_orient; 
    float sensor_angle; 

    //Vehicle dynamics Info
    float str_ang = 0; 
    float prnd = 3; //default enumeration, drive
    bool wheelslip = 0; //default don't send information 
    float v_ego = 0; 
    bool v_stand = 0;
    bool use_str_ang = 1; 
    float yawrate = 0;

    float wheelbase = 2.863; //distance from front to rear axles [m]
    float trackwidth = 1.681; //distance from right and left wheel [m]
    float strwhlang_ratio = 15.1; //ratio of steering wheel to wheels turning

    //vehicle dimensions
    float veh_maxwidth = 2.158; //max width of the vehicle [m]
    float veh_minwidth = 1.948; //min width of the vehicle [m]

    //these two parameters definitely look wrong
    float frontbump_pos = 3.881; //longitudinal position of front bumper wrt sensor [m] ???
    float rearbump_pos = 0.964; //longitudinal position of rear bumper wrt sensor [m]   ???

    switch(radar_num){
    case 1: //right corner radar
        latsensor_tocenter = 0.885; 
        longsensor_torear = 3.35; 
        sensor_height = 0.673;
        sensor_orient = 0; 
        sensor_angle = -0.785398;
        break;
    case 2: //left corner radar
        latsensor_tocenter = 0.885; 
        longsensor_torear = 3.37; 
        sensor_height = 0.681;
        sensor_orient = 1; 
        sensor_angle = 0.785398;
        break;
    case 3: //front radar
        
        //mounting information 
        latsensor_tocenter = 0;
        longsensor_torear = 3.784; 
        sensor_height = 0.558;
        sensor_orient = 0;
        sensor_angle = 0;
        break;
    }

    in_mount_info.ri_mi_lat_sensor_mount_to_center = radar_input_mount_info_ri_mi_lat_sensor_mount_to_center_encode(latsensor_tocenter);
    in_mount_info.ri_mi_long_sensor_mount_to_rear_axle = radar_input_mount_info_ri_mi_long_sensor_mount_to_rear_axle_encode(longsensor_torear);
    in_mount_info.ri_mi_sensor_height = radar_input_mount_info_ri_mi_sensor_height_encode(sensor_height);
    in_mount_info.ri_mi_sensor_orientation = radar_input_mount_info_ri_mi_sensor_orientation_encode(sensor_orient);
    in_mount_info.ri_mi_sensor_mount_angle = radar_input_mount_info_ri_mi_sensor_mount_angle_encode(sensor_angle);

    in_veh_dyn.ri_veh_steer_angle = radar_input_veh_dyn_data_ri_veh_steer_angle_encode(str_ang);
    in_veh_dyn.ri_veh_velocity = radar_input_veh_dyn_data_ri_veh_velocity_encode(v_ego);
    in_veh_dyn.ri_veh_use_steer_angle = radar_input_veh_dyn_data_ri_veh_use_steer_angle_encode(use_str_ang);
    in_veh_dyn.ri_veh_standstill = radar_input_veh_dyn_data_ri_veh_standstill_encode(use_str_ang);
    in_veh_dyn.ri_veh_yaw_rate = radar_input_veh_dyn_data_ri_veh_yaw_rate_encode(yawrate);
    in_veh_dyn.ri_veh_any_wheel_slip_event = radar_input_veh_dyn_data_ri_veh_any_wheel_slip_event_encode(prnd);
    in_veh_dyn.ri_veh_prndstat = radar_input_veh_dyn_data_ri_veh_prndstat_encode(wheelslip);

    in_wheel_info.ri_wi_wheel_base = radar_input_wheel_info_ri_wi_wheel_base_encode(wheelbase);
    in_wheel_info.ri_wi_track_width = radar_input_wheel_info_ri_wi_track_width_encode(trackwidth);
    in_wheel_info.ri_wi_steering_angle_ratio = radar_input_wheel_info_ri_wi_steering_angle_ratio_encode(strwhlang_ratio);

    in_veh_dim.ri_vd_max_width = radar_input_veh_dim_ri_vd_max_width_encode(veh_maxwidth);
    in_veh_dim.ri_vd_min_width = radar_input_veh_dim_ri_vd_min_width_encode(veh_minwidth);
    in_veh_dim.ri_vd_long_front_bumper_pos = radar_input_veh_dim_ri_vd_long_front_bumper_pos_encode(frontbump_pos);
    in_veh_dim.ri_vd_long_rear_bumper_pos = radar_input_veh_dim_ri_vd_long_rear_bumper_pos_encode(rearbump_pos);
};

uint8_t Radar_RX::crc8bit_calculation(uint8_t can1670signals[7], int f_len) {
    uint8_t crc = 0xFF;

    for (int index = 0; index < f_len; index++) {
        crc ^= can1670signals[index];  // Assign data to CRC

        for (int bitIndex = 0; bitIndex < 8; bitIndex++) {  // Loop through 8 bits
        if ((crc & 0x80 != 0)) {
            crc = (crc << 1);
            crc ^= 0x1D;
        } else {
            crc = (crc << 1);
        }
        }
    }
    crc = ~crc;
    return crc;
};

void Radar_RX::clear_classes(common::radar_object_data &radar_obj, common::sensor_diagnostic_data_msg &diag_data,     Radar_RX::radar_diagnostic_response &diag_response, Radar_RX::radar_information &radar_info,Radar_RX::target_tracking_info &target_info, Radar_RX::object_tracking_info &object_info, uint8_t &tc_check, uint8_t &mc_check){
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
};
