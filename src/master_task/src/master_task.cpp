#include "master_task.h"

MasterTask::MasterTask(ros::NodeHandle* nodeHandle) : nh(nodeHandle) {
  drive_ctrl_sub =
      nh->subscribe("drive_ctrl_input", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::drive_ctrl_msg_callback, this);
  acc_sub = 
      nh->subscribe("acc_output_msg", MASTER_MESSAGE_BUFFER_SIZE, &MasterTask::acc_output_msg_callback, this);
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

// need to make sure this callback function is called at least every 50 ms because rolling counter functions
// are called from here
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

    INT_2();
    // INT_7(); // Commented out for now since we dont have Jetson sending us messages
    // put other functions here if you want to unit test them
    // publish_can_comms_msg(); // uncomment if unit testing the above functions
    // ros::spinOnce();
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

bool MasterTask::sensor_diagnostic_callback_CH2(common::sensor_diagnostic_flag_CH2::Request& req_CH2,
                                                common::sensor_diagnostic_flag_CH2::Response& /*res_CH2*/) {
  FRONT_RADAR = req_CH2.front_radar;
  can_comms_msg.front_radar_fault = !(req_CH2.front_radar);
  return true;
}

bool MasterTask::sensor_diagnostic_callback_CH3(common::sensor_diagnostic_flag_CH3::Request& req_CH3,
                                                common::sensor_diagnostic_flag_CH3::Response& /*res_CH3*/) {
  // Note: since we aren't using corner radars at the moment for testing, set these values to true
  // LEFT_CORNER_RADAR = req_CH3.left_corner_radar;
  // RIGHT_CORNER_RADAR = req_CH3.right_corner_radar;
  LEFT_CORNER_RADAR = true;
  RIGHT_CORNER_RADAR = true;
  can_comms_msg.left_radar_fault = !(req_CH3.left_corner_radar);
  can_comms_msg.right_radar_fault = !(req_CH3.right_corner_radar);
  return true;
}

bool MasterTask::sensor_diagnostic_callback_CH4(common::sensor_diagnostic_flag_CH4::Request& req_CH4,
                                                common::sensor_diagnostic_flag_CH4::Response& /*res_CH4*/) {
  MOBILEYE = req_CH4.mobileye;
  can_comms_msg.mobileye_fault = !(req_CH4.mobileye);
  return true;
}
common::can_comms_data_msg MasterTask::get_can_comms_msg() { return can_comms_msg; }

void MasterTask::ACC_1_1() {
    //buffer_time = 0.2 s
    if (!ACC_ACTIVATION || !ACC_ALLOWED || ACC_FAULT || !FRONT_RADAR || !LEFT_CORNER_RADAR || !RIGHT_CORNER_RADAR || !MOBILEYE)
    {
        ros::SteadyTime curr = ros::SteadyTime::now();
        if (initial_OFF_request) {
          begin1 = ros::SteadyTime::now();
          initial_OFF_request = false;
        }
        else if ((curr.toSec() - begin1.toSec()) >= buffer_time) {
            can_comms_msg.long_accel = 0;
        }
    }
    else {
      initial_OFF_request = true;
    } 
}

void MasterTask::ACC_4() {
    if (!AEB_ALLOWED || !AEB_ACTIVATION || AEB_FAULT) {
        can_comms_msg.acc_valid = 0;
    }
}

void MasterTask::ACC_15() {
    if (LCC_STEER > 15) {
        can_comms_msg.acc_valid = 0;
    }
}

void MasterTask::ACC_16() {
    if (ACC_ACTIVATION && ACC_ALLOWED) {
      ros::SteadyTime curr = ros::SteadyTime::now();
      if (initial_OFF_request_8) {
        begin6 = ros::SteadyTime::now();
        initial_OFF_request_8 = false;
      } else if ((curr - begin9) >= ros::WallDuration(0.5)) {
        if (can_comms_msg.acc_valid) {
          can_comms_msg.acc_valid = 1;
        }
      }
    }
    else {
      initial_OFF_request_8 = true;
    }
}

void MasterTask::ACC_17() {
    if (!ACC_ACTIVATION) {
      ros::SteadyTime curr = ros::SteadyTime::now();
      if (initial_OFF_request_2) {
        begin3 = ros::SteadyTime::now();
        initial_OFF_request_2 = false;
      } else if ((curr - begin3) >= ros::WallDuration(0.200)) {
        can_comms_msg.acc_valid = 0;
      }
    }
    else{
      initial_OFF_request_2 = true;
    }  
}

void MasterTask::ACC_18() {
  // arbitrary vars for time in ms:
  // signal error message sent to the Jetson + ACC disabled if, within period_max,
  // signal (ACC_activatioin) is in the opposite state for more than opposite_max*10 ms cumulatively
  ros::SteadyTime curr = ros::SteadyTime::now();
  // number of ms that have passed this period
  // first truncate milliseconds so we're able to put it into a long
  unsigned long long currms = (curr.toNSec() / 1000000);
  unsigned long long period_start_ms = (acc_period_start.toNSec() / 1000000);
  unsigned long long time_passed_in_period = currms - period_start_ms;
  unsigned long long ten_sec_ms = (ten_sec_start_acc.toNSec() / 1000000);
  unsigned long long time_passed_since_10ms = currms - ten_sec_ms;

  if (can_comms_msg.acc_fault){
    can_comms_msg.acc_valid = 0;
  } 
  if (time_passed_since_10ms >= 10) {
    if ((ACC_ALLOWED != can_comms_msg.acc_valid)) {
      opposite_counter += 1;
    }
    ten_sec_start_acc = curr;
  }
  // check if counter is too high
  if (opposite_counter >= OPPOSITE_MAX) {
    can_comms_msg.acc_valid = 0;
    can_comms_msg.acc_fault = 1;
  }
  // check if period is over
  if (time_passed_in_period > PERIOD_MAX && !can_comms_msg.acc_fault) {  // reset period
    acc_period_start = curr;
    opposite_counter = 0;
  }
}

void MasterTask::ACC_20() {
    if (!ACC_ALLOWED || !ACC_ACTIVATION) {
        can_comms_msg.acc_valid = 0;
    }
}

void MasterTask::AEB_13() {
    if (AEB_ENGAGED) {
        can_comms_msg.long_accel = AEB_ACCEL;
    }
    else {
        can_comms_msg.long_accel = ACC_ACCEL;
    }
}

void MasterTask::AEB_22() {
  if (VEHICLE_SPEED < MAX_STEERING_ANGLE) {
      can_comms_msg.aeb_valid = 0;
  }    
}

void MasterTask::AEB_24() {
    if (AEB_ACTIVATION && AEB_ALLOWED) {
      ros::SteadyTime curr = ros::SteadyTime::now();
      if (initial_OFF_request_5) {
        begin6 = ros::SteadyTime::now();
        initial_OFF_request_5 = false;
      } else if ((curr - begin6) >= ros::WallDuration(0.500)) {
        if (can_comms_msg.aeb_valid) {
          can_comms_msg.aeb_valid = 1;
        }
      }
    }
    else {
      initial_OFF_request_5 = true;
    }
}

void MasterTask::AEB_26() {
    if (!AEB_ALLOWED || !AEB_ACTIVATION) {
        can_comms_msg.aeb_valid = 0;
    }  
}

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

void MasterTask::CAV_2_2() {
    if (abs(AEB_ACCEL) > 4.9) {
      can_comms_msg.long_accel = -4.9;
    }
}

void MasterTask::INT_1() {
    if (!(can_comms_msg.alive_rolling_counter == 15)) {
      can_comms_msg.alive_rolling_counter += 1;
    }
        
    else {
      can_comms_msg.alive_rolling_counter = 0;
    }
}

void MasterTask::INT_2() {
  ros::SteadyTime curr = ros::SteadyTime::now();
  if (!first_RC && (alive_rolling_counter_MABx >= 0 && alive_rolling_counter_MABx <= 15) &&
      ((alive_rolling_counter_MABx - prev_rolling_counter_MABx == 1) ||
       (alive_rolling_counter_MABx - prev_rolling_counter_MABx == -15))) {
    if (initial_HSC_alive) {  // check if rolling counter requirements are satisfied for the first time after start
                              // up/last fault;
      initial_HSC_alive = false;
      rc_time_init = ros::SteadyTime::now();
      can_comms_msg.acc_valid = 0;
      can_comms_msg.aeb_valid = 0;
      can_comms_msg.lcc_valid = 0;
      can_comms_msg.long_accel = 0;
      can_comms_msg.lcc_steer = 0;
    } else {
      if ((curr - rc_time_init) > ros::WallDuration(0.100)) {  // if rolling counter has been valid for more than 100 ms
        can_comms_msg.acc_valid = 1;
        can_comms_msg.aeb_valid = 1;
        can_comms_msg.lcc_valid = 1;
        can_comms_msg.long_accel = 1;
        can_comms_msg.lcc_steer = 1;
        prev_time_correct_mabx = curr;
      }
    }
  } else if (curr - prev_time_correct_mabx < ros::WallDuration(0.050)) {  // 50 ms buffer for rc to be incorrect
    can_comms_msg.acc_valid = 1;
    can_comms_msg.aeb_valid = 1;
    can_comms_msg.lcc_valid = 1;
    can_comms_msg.long_accel = 1;
    can_comms_msg.lcc_steer = 1;
  } else {
    initial_HSC_alive = true;
    can_comms_msg.acc_valid = 0;
    can_comms_msg.aeb_valid = 0;
    can_comms_msg.lcc_valid = 0;
    can_comms_msg.long_accel = 0;
    can_comms_msg.lcc_steer = 0;
  }
  first_RC = false;
  prev_rolling_counter_MABx = alive_rolling_counter_MABx;
  prev_time_rc = curr;
}

void MasterTask::INT_7() {
  ros::SteadyTime curr = ros::SteadyTime::now();

  if (!first_RC_Jetson && (alive_rolling_counter_Jetson >= 0 && alive_rolling_counter_Jetson <= 15) &&
      ((alive_rolling_counter_Jetson - prev_rolling_counter_Jetson == 1) ||
       (alive_rolling_counter_Jetson - prev_rolling_counter_Jetson == -15))) {
    if (initial_Jetson_alive) {  // check if rolling counter requirements are satisfied for the first time after start
                                 // up/last fault;
      initial_Jetson_alive = false;
      rc_time_init = ros::SteadyTime::now();
      can_comms_msg.acc_valid = 0;
      can_comms_msg.aeb_valid = 0;
      can_comms_msg.lcc_valid = 0;
      can_comms_msg.long_accel = 0;
      can_comms_msg.lcc_steer = 0;
    } else {
      if ((curr - rc_time_init_jetson) > ros::WallDuration(0.100)) {  // if rolling counter has been valid for more than 100 ms
        // don't set sensors to true so that we don't overwrite HSC rolling counter function
        prev_time_correct_jetson = curr;
      }
    }
  } else if (curr.toSec() - prev_time_correct_jetson.toSec() < 0.050) {  // 50 ms buffer for rc to be incorrect
  } else {
    initial_Jetson_alive = true;
    can_comms_msg.acc_valid = 0;
    can_comms_msg.aeb_valid = 0;
    can_comms_msg.lcc_valid = 0;
    can_comms_msg.long_accel = 0;
    can_comms_msg.lcc_steer = 0;
  }
  first_RC_Jetson = false;
  prev_rolling_counter_Jetson = alive_rolling_counter_Jetson;
  prev_time_rc_jetson = curr;
}

void MasterTask::LCC_1() {
    if (LCC_STEER > 30) {
        can_comms_msg.lcc_steer = 30;
    }
    else if (LCC_STEER < -30) {
        can_comms_msg.lcc_steer = -30;
    }
}

void MasterTask::LCC_3() {
    curr_vel = VEHICLE_SPEED;
    ros::SteadyTime curr_time = ros::SteadyTime::now();
    ego_accel = (curr_vel - prev_vel)/(curr_time.toSec() - prev_time.toSec());
    if (abs(ego_accel) > 2.5 || abs(ACC_ACCEL) > 2.5 || abs(AEB_ACCEL) > 2.5) {
        can_comms_msg.lcc_valid = 0;
    }
    else
    {
      if (can_comms_msg.lcc_valid) {
          can_comms_msg.lcc_valid = 1;
      }
    }
    prev_time = curr_time;
    prev_vel = curr_vel;
}

void MasterTask::LCC_10() {
    if (LCC_ACTIVATION && LCC_ALLOWED) {
      ros::SteadyTime curr = ros::SteadyTime::now();
      if (initial_OFF_request_6) {
        begin7 = ros::SteadyTime::now();
        initial_OFF_request_6 = false;
      } else if ((curr - begin7) >= ros::WallDuration(0.500)) {
        if (can_comms_msg.lcc_valid) {
          can_comms_msg.lcc_valid = 1;
        }
      }
    }
    else {
      initial_OFF_request_6 = true;
    }
}

void MasterTask::LCC_11() {
    if (!LCC_ACTIVATION || !LCC_ALLOWED) {
      ros::SteadyTime curr = ros::SteadyTime::now();
      if (initial_OFF_request_7) {
        begin8 = ros::SteadyTime::now();
        initial_OFF_request_7 = false;
      } else if ((curr - begin8) >= ros::WallDuration(0.200)) {
        can_comms_msg.lcc_valid = 0;
      }
    }
    else {
      initial_OFF_request_7 = true;
    }
}

void MasterTask::LCC_12() {
  ros::SteadyTime curr = ros::SteadyTime::now();
  // number of ms that have passed this period
  // first truncate milliseconds so we're able to put it into a long
  unsigned long long currms = (curr.toNSec() / 1000000);
  unsigned long long period_start_ms = (lcc_period_start.toNSec() / 1000000);
  unsigned long long time_passed_in_period = currms - period_start_ms;
  unsigned long long tenSecms = (ten_sec_start_lcc.toNSec() / 1000000);
  unsigned long long time_passed_since_10ms = currms - tenSecms;

  if (can_comms_msg.lcc_fault){
    can_comms_msg.lcc_valid = 0;
  } 
  if (time_passed_since_10ms >= 10) {
    if ((LCC_ALLOWED != can_comms_msg.lcc_valid)) {
      opposite_counter_lcc += 1;
    }
    ten_sec_start_lcc = curr;
  }
  // check if counter is too high
  if (opposite_counter_lcc >= OPPOSITE_MAX) {
    ROS_INFO("lcc values were opposite for too long");
    can_comms_msg.lcc_valid = 0;
    can_comms_msg.lcc_fault = 1;
  }
  // check if period is over
  if (time_passed_in_period > PERIOD_MAX && !can_comms_msg.lcc_fault) {  // reset period
    ROS_INFO("lcc period is over");
    lcc_period_start = curr;
    opposite_counter_lcc = 0;
  }
}

// checks that the rolling counter functions have been called within the last 50 ms
// otherwise, turn all signals to null
// Since the rolling counter functions are called only in msg callbacks, we must also check
// that they are called frequently
void MasterTask::check_rolling_counters_called() {
  ros::SteadyTime curr = ros::SteadyTime::now();
  // Commenting this out for now since Jetson is not sending us any messages for Year 3
  // if (((curr - prev_time_rc) > ros::WallDuration(0.050)) || ((curr - prev_time_rc_jetson) > ros::WallDuration(0.050))) {
  if ((curr - prev_time_rc) > ros::WallDuration(0.050)) {

    // std::cout << "HSC / Jetson time out" << std::endl;
    initial_HSC_alive = true;
    can_comms_msg.acc_valid = 0;
    can_comms_msg.aeb_valid = 0;
    can_comms_msg.lcc_valid = 0;
    can_comms_msg.long_accel = 0;
    can_comms_msg.lcc_steer = 0;
  }
}