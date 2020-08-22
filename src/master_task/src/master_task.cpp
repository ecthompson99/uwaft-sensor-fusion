#include "master_task.h"

MasterTask::MasterTask(ros::NodeHandle* nodeHandle) : nh(nodeHandle) {
  drive_ctrl_sub =
      nh->subscribe("drive_control_input", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::drive_ctrl_msg_callback, this);
  acc_sub = 
      nh->subscribe("acc_output", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::acc_output_msg_callback, this);
  aeb_sub = 
      nh->subscribe("aeb_output", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::aeb_output_msg_callback, this);
  lcc_sub = 
      nh->subscribe("lcc_output", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::lcc_output_msg_callback, this);
  sensor_diagnostic_CH2_server = 
      nh->advertiseService("sensor_diagnostic_CH2", &MasterTask::sensor_diagnostic_callback_CH2, this);
  sensor_diagnostic_CH3_server = 
      nh->advertiseService("sensor_diagnostic_CH3", &MasterTask::sensor_diagnostic_callback_CH3, this);
  sensor_diagnostic_CH4_server = 
      nh->advertiseService("sensor_diagnostic_CH4", &MasterTask::sensor_diagnostic_callback_CH4, this);
  
  master_task_pub = nh->advertise<common::can_comms_data_msg>("can_comms_data", MASTER_MESSAGE_BUFFER_SIZE);
}

MasterTask::~MasterTask() {}

void MasterTask::publish_can_comms_msg() { master_task_pub.publish(can_comms_msg); }

void MasterTask::drive_ctrl_msg_callback(const common::drive_ctrl_input_msg& drive_ctrl_msg) 
{
    ACC_ACTIVATION = drive_ctrl_msg.acc_activation;
    AEB_ACTIVATION = drive_ctrl_msg.aeb_activation;
    LCC_ACTIVATION = drive_ctrl_msg.lcc_activation;

    ACC_ALLOWED = drive_ctrl_msg.acc_allowed;
    AEB_ALLOWED = drive_ctrl_msg.aeb_allowed;
    LCC_ALLOWED = drive_ctrl_msg.lcc_allowed;

    alive_rolling_counter_MABx = drive_ctrl_msg.alive_rolling_counter_MABx;
    alive_rolling_counter_Jetson = drive_ctrl_msg.alive_rolling_counter_Jetson;

    acc_speed_set_point = drive_ctrl_msg.acc_speed_set_point;
    acc_gap_level = drive_ctrl_msg.acc_gap_level;
    VEHICLE_SPEED = drive_ctrl_msg.veh_spd;
    STEERING_ANGLE = drive_ctrl_msg.str_ang;
}

void MasterTask::acc_output_msg_callback(const common::acc_output_msg& acc_msg)
{
    ACC_FAULT = acc_msg.acc_fault;
    ACC_ACCEL = acc_msg.acc_accel;
    can_comms_msg.acc_fault = acc_msg.acc_fault;
}

void MasterTask::aeb_output_msg_callback(const common::aeb_output_msg& aeb_msg)
{
    AEB_FAULT = aeb_msg.aeb_fault;
    AEB_ENGAGED = aeb_msg.aeb_engaged;
    AEB_ACCEL = aeb_msg.aeb_accel;
    can_comms_msg.aeb_fault = aeb_msg.aeb_fault;
}

void MasterTask::lcc_output_msg_callback(const common::lcc_output_msg& lcc_msg)
{
    LCC_FAULT = lcc_msg.lcc_fault;
    LCC_STEER = lcc_msg.lcc_steer;
    can_comms_msg.lcc_fault = lcc_msg.lcc_fault;
}

bool MasterTask::sensor_diagnostic_callback_CH2(common::sensor_diagnostic_flag_CH2::Request &req_CH2, common::sensor_diagnostic_flag_CH2::Response &res_CH2)
{
    FRONT_RADAR = req_CH2.front_radar;
    can_comms_msg.front_radar_fault = !(req_CH2.front_radar);
    return true;
}

bool MasterTask::sensor_diagnostic_callback_CH3(common::sensor_diagnostic_flag_CH3::Request &req_CH3, common::sensor_diagnostic_flag_CH3::Response &res_CH3)
{
    LEFT_CORNER_RADAR = req_CH3.left_corner_radar;
    RIGHT_CORNER_RADAR = req_CH3.right_corner_radar;
    can_comms_msg.left_radar_fault = !(req_CH3.left_corner_radar);
    can_comms_msg.right_radar_fault = !(req_CH3.right_corner_radar);
    return true;
}

bool MasterTask::sensor_diagnostic_callback_CH4(common::sensor_diagnostic_flag_CH4::Request &req_CH4, common::sensor_diagnostic_flag_CH4::Response &res_CH4)
{
    MOBILEYE = req_CH4.mobileye;
    can_comms_msg.mobileye_fault = !(req_CH4.mobileye);
    return true;
}
common::can_comms_data_msg MasterTask::get_can_comms_msg() { return can_comms_msg; }


void MasterTask::CAV_1_5() {
    if (!MOBILEYE) {
        can_comms_msg.acc_valid = 0;
        can_comms_msg.aeb_valid = 0;
        can_comms_msg.lcc_valid = 0;
    }
}

void MasterTask::CAV_1_6() {
    if (!FRONT_RADAR) {
        can_comms_msg.acc_valid = 0;
        can_comms_msg.aeb_valid = 0;
    }
}

void MasterTask::ACC_1_1() {
    //buffer_time = 0.2 s
    if (!ACC_ACTIVATION || !ACC_ALLOWED || ACC_FAULT || !FRONT_RADAR || !LEFT_CORNER_RADAR || !RIGHT_CORNER_RADAR || !MOBILEYE)
    {
        // std::cout << "acc activation " << ACC_ACTIVATION << " acc allowed " << ACC_ALLOWED << " acc fault " << ACC_FAULT << " FR " << FRONT_RADAR << " LR " << LEFT_CORNER_RADAR << " RR " << RIGHT_CORNER_RADAR << " ME " << MOBILEYE << std::endl;
        ros::WallTime curr = ros::WallTime::now();
        if (initial_OFF_request) {
            std::cout << "initial request" << std::endl;
            begin1 = ros::WallTime::now();
            initial_OFF_request = false;
        }
        else if ((curr.toSec() - begin1.toSec()) >= buffer_time) {
            std::cout << "ACC_1.1 long_accel = 0" << std::endl;
            can_comms_msg.long_accel = 0;
        }
    }
    else    initial_OFF_request = true; 
}

void MasterTask::ACC_15() {
    if (LCC_STEER > 30) {
        can_comms_msg.acc_valid = 0;
        std::cout << "ACC_15 acc_valid = 0" << std::endl;
    }
}

void MasterTask::ACC_17() {
    if (!ACC_ACTIVATION) {
        ros::WallTime curr = ros::WallTime::now();
        if (initial_OFF_request_2) {
            begin3 = ros::WallTime::now();
            initial_OFF_request_2 = false;
        }
        else if ((curr.toSec() - begin3.toSec()) >= 0.200) {
            can_comms_msg.acc_valid = 0;
            std::cout << "ACC_17 acc_valid = 0" << std::endl;
        }
    }
    else    initial_OFF_request_2 = true;
}

void MasterTask::ACC_20() {
    if (!ACC_ALLOWED || !ACC_ACTIVATION) {
        can_comms_msg.acc_valid = 0;
        std::cout << "ACC_20 acc_valid = 0" << std::endl;
    }
}

void MasterTask::ACC_4() {
    if (!AEB_ALLOWED || !AEB_ACTIVATION || AEB_FAULT) {
        can_comms_msg.acc_valid = 0;
        std::cout << "ACC_4 acc_valid = 0" << std::endl;
    }
}

void MasterTask::AEB_13() {
    if (AEB_ENGAGED) {
        can_comms_msg.long_accel = AEB_ACCEL;
        std::cout << "AEB_13 long_accel = " << AEB_ACCEL << std::endl;
    }
    else {
        can_comms_msg.long_accel = ACC_ACCEL;
        std::cout << "AEB_13 long_accel = " << ACC_ACCEL << std::endl;
    }
}

void MasterTask::AEB_22() {
    if (VEHICLE_SPEED < 15) {
        can_comms_msg.aeb_valid = 0;
        std::cout << "AEB_22 aeb_valid = 0" << std::endl;
    }    
}

void MasterTask::AEB_24() {
    if (AEB_ACTIVATION && AEB_ALLOWED) {
        ros::WallTime curr = ros::WallTime::now();
        if (initial_OFF_request_5) {
            begin6 = ros::WallTime::now();
            initial_OFF_request_5 = false;
        }
        else if ((curr.toSec() - begin6.toSec()) >= 0.500) {
            if (can_comms_msg.aeb_valid) {
                can_comms_msg.aeb_valid = 1;
                std::cout << "AEB_24 aeb_valid = 1" << std::endl;
            }            
        }
    }
    else    initial_OFF_request_5 = true;
}

void MasterTask::AEB_26() {
    if (!AEB_ALLOWED || !AEB_ACTIVATION) {
        can_comms_msg.aeb_valid = 0;
        std::cout << "AEB_26 aeb_valid = 0" << std::endl;
    }  
}

void MasterTask::CAV_2_2() {
    if (abs(AEB_ACCEL) > 4.9)   can_comms_msg.long_accel = -4.9;
}

void MasterTask::LCC_1() {
    if (LCC_STEER > 30) {
        can_comms_msg.lcc_steer = 30;
        std::cout << "LCC_1 lcc_steer = 30" << std::endl;
    }
    else if (LCC_STEER < -30) {
        can_comms_msg.lcc_steer = -30;
        std::cout << "LCC_1 lcc_steer = -30" << std::endl;
    }
}

void MasterTask::LCC_10() {
    if (LCC_ACTIVATION && LCC_ALLOWED) {
        ros::WallTime curr = ros::WallTime::now();
        if (initial_OFF_request_6) {
            begin7 = ros::WallTime::now();
            initial_OFF_request_6 = false;
        }
        else if ((curr.toSec() - begin7.toSec()) >= 0.500) {
            if (can_comms_msg.lcc_valid) {
                can_comms_msg.lcc_valid = 1;
                std::cout << "LCC_10 lcc_valid = 1" << std::endl;
            }
            
        }
    }
    else    initial_OFF_request_6 = true;
}

void MasterTask::LCC_11() {
    if (!LCC_ACTIVATION || !LCC_ALLOWED) {
        ros::WallTime curr = ros::WallTime::now();
        if (initial_OFF_request_7) {
            begin8 = ros::WallTime::now();
            initial_OFF_request_7 = false;
        }
        else if ((curr.toSec() - begin8.toSec()) >= 0.200) {
            can_comms_msg.lcc_valid = 0;
            std::cout << "LCC_11 lcc_valid = 0" << std::endl;
        }
    }
    else    initial_OFF_request_7 = true;
}

void MasterTask::LCC_3() {
    curr_vel = VEHICLE_SPEED;
    ros::WallTime curr_time = ros::WallTime::now();
    ego_accel = (curr_vel - prev_vel)/(curr_time.toSec() - prev_time.toSec());
    if (abs(ego_accel) > 2.5 || abs(ACC_ACCEL) > 2.5 || abs(AEB_ACCEL) > 2.5) {
        can_comms_msg.lcc_valid = 0;
        std::cout << "LCC_3 lcc_valid = 0" << std::endl;
    }
    else
    {
        if (can_comms_msg.lcc_valid) {
            can_comms_msg.lcc_valid = 1;
            std::cout << "LCC_3 lcc_valid = 1" << std::endl;
        }
        
    }
    prev_time = curr_time;
    prev_vel = curr_vel;
}