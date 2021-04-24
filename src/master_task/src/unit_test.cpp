#include "master_task.h"
#include "ros/ros.h"
#include <vector>
#include <ros/console.h> // logs
#include <iostream>
using namespace std;

// all tests defined in master task unit test Confluence page

ros::Publisher drive_crtl_pub;
ros::Publisher acc_pub;
ros::Publisher aeb_pub;
ros::Publisher lcc_pub;
ros::Subscriber can_comms_data;

// can_comms callback vars
double LONG_ACCEL;
double LCC_STEER;
bool ACC_VALID;
bool LCC_VALID;
bool AEB_VALID;
bool ACC_FAULT;
bool LCC_FAULT;
bool AEB_FAULT;
bool FRONT_RADAR_FAULT;
bool LEFT_RADAR_FAULT;
bool RIGHT_RADAR_FAULT;
bool MOBILEEYE_FAULT;
unsigned long int ALIVE_ROLLING_COUNTER;

struct SensorStruct // fake input given to master task for testing
{
  bool acc_activation = 1;
  bool aeb_activation = 1;
  bool lcc_activation = 1;

  bool acc_allowed = 1;
  bool aeb_allowed = 1;
  bool lcc_allowed = 1;

  unsigned long int alive_rolling_counter_MABx = 0;
  unsigned long int alive_rolling_counter_Jetson = 0;

  double acc_speed_set_point = 30.5;
  unsigned int acc_gap_level = 2;
  double veh_spd = 25.5;
  double str_ang = 10.5;

  bool acc_fault = false;
  double acc_accel = 2;

  bool aeb_fault = false;
  bool aeb_engaged = true;
  double aeb_accel = -2;

  bool lcc_fault = false;
  double lcc_steer = 2.5;

  unsigned long msgTime; // amount of time the sensors should be set to these values
};

void can_comms_callback(const common::can_comms_data_msg& can_comms_data) 
{
  LONG_ACCEL = can_comms_data.long_accel;
  LCC_STEER = can_comms_data.lcc_steer;
  ACC_VALID = can_comms_data.acc_valid;
  LCC_VALID = can_comms_data.lcc_valid;
  AEB_VALID = can_comms_data.aeb_valid;
  ACC_FAULT = can_comms_data.acc_fault;
  LCC_FAULT = can_comms_data.lcc_fault;
  AEB_FAULT = can_comms_data.aeb_fault;
  FRONT_RADAR_FAULT = can_comms_data.front_radar_fault;
  LEFT_RADAR_FAULT = can_comms_data.left_radar_fault;
  RIGHT_RADAR_FAULT = can_comms_data.right_radar_fault;
  ALIVE_ROLLING_COUNTER = can_comms_data.alive_rolling_counter;
  MOBILEEYE_FAULT = can_comms_data.mobileye_fault;
}

void checkExpectedOutput(ros::NodeHandle* n, const common::can_comms_data_msg& expectedOutput){
  if (LONG_ACCEL != expectedOutput.long_accel) std::cout << "long_accel " << LONG_ACCEL << ", expected " << expectedOutput.long_accel << std::endl;
  if (LCC_STEER != expectedOutput.lcc_steer) std::cout << "lcc_steer " << LCC_STEER << ", expected " << expectedOutput.lcc_steer << endl;
  if (ACC_VALID != expectedOutput.acc_valid) std::cout << "acc_valid " << ACC_VALID << ", expected " << expectedOutput.acc_valid << endl;
  if (AEB_VALID != expectedOutput.aeb_valid) std::cout << "aeb_valid " << AEB_VALID << ", expected " << expectedOutput.aeb_valid << endl;
  if (LCC_VALID != expectedOutput.lcc_valid) std::cout << "lcc_valid " << LCC_VALID << ", expected " << expectedOutput.lcc_valid << endl;
  if (ACC_FAULT != expectedOutput.acc_fault) std::cout << "acc_fault " << ACC_FAULT << ", expected " << expectedOutput.acc_fault << endl;
  if (AEB_FAULT != expectedOutput.aeb_fault) std::cout << "aeb_fault " << AEB_FAULT << ", expected " << expectedOutput.aeb_fault << endl;
  if (LCC_FAULT != expectedOutput.lcc_fault) std::cout << "lcc_fault " << LCC_FAULT << ", expected " << expectedOutput.lcc_fault << endl;
  if (FRONT_RADAR_FAULT != expectedOutput.front_radar_fault) std::cout << "front_radar_fault " << FRONT_RADAR_FAULT << ", expected " << expectedOutput.front_radar_fault << endl;
  if (LEFT_RADAR_FAULT != expectedOutput.left_radar_fault) std::cout << "left_radar_fault " << LEFT_RADAR_FAULT << ", expected " << expectedOutput.left_radar_fault << endl;
  if (RIGHT_RADAR_FAULT != expectedOutput.right_radar_fault) std::cout << "right_radar_fault " << RIGHT_RADAR_FAULT << ", expected " << expectedOutput.right_radar_fault << endl;
  if (MOBILEEYE_FAULT != expectedOutput.mobileye_fault) std::cout << "mobileye_fault " << MOBILEEYE_FAULT << ", expected " << expectedOutput.mobileye_fault << endl;
}

bool checkExpectedOutputBool(ros::NodeHandle* n, const common::can_comms_data_msg& expectedOutput){
  return (ACC_VALID == expectedOutput.acc_valid && ACC_FAULT == expectedOutput.acc_fault && 
  LCC_VALID == expectedOutput.lcc_valid && LCC_FAULT == expectedOutput.lcc_fault);
}

// sends test sensor values specified in SensorStruct
void sendMsg(ros::NodeHandle* n, struct SensorStruct input){
  common::drive_ctrl_input_msg drive_ctrl_msg;
  common::acc_output_msg acc_msg;
  common::aeb_output_msg aeb_msg;
  common::lcc_output_msg lcc_msg;

  drive_ctrl_msg.acc_activation = input.acc_activation;
  drive_ctrl_msg.aeb_activation = input.aeb_activation;
  drive_ctrl_msg.lcc_activation = input.lcc_activation;
  drive_ctrl_msg.acc_allowed = input.acc_allowed;
  drive_ctrl_msg.aeb_allowed = input.aeb_allowed;
  drive_ctrl_msg.lcc_allowed = input.lcc_allowed;
  drive_ctrl_msg.acc_gap_level = input.acc_gap_level;
  drive_ctrl_msg.str_ang = input.str_ang;
  drive_ctrl_msg.acc_speed_set_point = input.acc_speed_set_point;
  drive_ctrl_msg.veh_spd = input.veh_spd;

  acc_msg.acc_fault = input.acc_fault;
  acc_msg.acc_accel = input.acc_accel;

  aeb_msg.aeb_fault = input.aeb_fault;
  aeb_msg.aeb_engaged = input.aeb_engaged;
  aeb_msg.aeb_accel = input.aeb_accel;

  lcc_msg.lcc_fault = input.lcc_fault;
  lcc_msg.lcc_steer = input.lcc_steer;

  drive_ctrl_msg.alive_rolling_counter_MABx = input.alive_rolling_counter_MABx;
  drive_ctrl_msg.alive_rolling_counter_Jetson = input.alive_rolling_counter_Jetson;

  drive_crtl_pub.publish(drive_ctrl_msg);
  acc_pub.publish(acc_msg);
  aeb_pub.publish(aeb_msg);
  lcc_pub.publish(lcc_msg);
}

// sends input messages to master task for time specified in SensorStruct and checks output
// assumes HSC rolling counter and Jetson rolling counter are functioning properly
// badRC == 0 both rc's working, badRC == 1 neither rc's working, badRC == 2 only HSC working, badRC == 3 only Jetson working
void runTest(ros::NodeHandle* n, struct SensorStruct* input, common::can_comms_data_msg expectedOutput, int badRC){
  ros::Rate rate(200);
  ros::SteadyTime start = ros::SteadyTime::now();
  unsigned long timePassed = 0;
  int count = input->alive_rolling_counter_MABx;
  
  while (timePassed < input->msgTime && ros::ok()){   
    count += 1; 
    // publish messages
    // increment rolling counters
    //cout << input->alive_rolling_counter_MABx << endl;
    if (!badRC || badRC == 2){
      input->alive_rolling_counter_MABx = count % 16;
    }
    if (!badRC || badRC == 3){
      input->alive_rolling_counter_Jetson = count % 16;
    }
    sendMsg(n, *input);
    
    // check output
    // dont check expected output for 20 ms after changing input since there's a delay in receiving messages
    if (timePassed > 20){
      checkExpectedOutput(n, expectedOutput);
    }

    ros::spinOnce();
    rate.sleep();
    ros::SteadyTime curr = ros::SteadyTime::now();
    timePassed = ((curr.toNSec() - start.toNSec()) / 1000000) % 1000000000000000000;
  }
}

// hsc alive rolling counter tests
// runs with either both rc's working normally or both running incorrectly
void INT_2(ros::NodeHandle* n, int testNum){
  if (testNum == 1){
    cout << "INT_2 test 1" << endl;
    //cout << "First 100 ms, working normally" << endl;
    struct SensorStruct ss;
    ss.msgTime = 110;
    common::can_comms_data_msg eo;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0);
    //cout << "Next 1000 ms, working normally" << endl;
    ss.msgTime = 1000;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
  }
  else if (testNum == 2){
    cout << "INT_2 test 2" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 80;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0);
    
    ss.msgTime = 1000;
    runTest(n, &ss, eo, 1);
  }
  else if (testNum == 3){
    cout << "INT_2 test 3" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 110;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0); // buffer
    ss.msgTime = 250;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
    // stops working
    ss.msgTime = 50;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;    
    runTest(n, &ss, eo, 1); // buffer
    ss.msgTime = 1000;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 1);
  }
  else if (testNum == 4){
    cout << "INT_2 test 4" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 500;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;

    runTest(n, &ss, eo, 1);

    ss.msgTime = 80;
    runTest(n, &ss, eo, 0);

    ss.msgTime = 1000;
    runTest(n, &ss, eo, 1);
  }
  else if (testNum == 5){
    cout << "INT_2 test 5" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 500;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 1);
    ss.msgTime = 110;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 900;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
  }
  else if (testNum == 6){
    cout << "INT_2 test 6" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 110;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 30;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 1);
    ss.msgTime = 900;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
  }
}

void INT_7(ros::NodeHandle* n, int testNum){
  if (testNum == 1){ 
    cout << "INT_7 test 1" << endl;
    struct SensorStruct ss;
    ss.msgTime = 1000;
    common::can_comms_data_msg eo;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 3);
  }
  else if (testNum == 2){ 
    cout << "INT_7 test 2" << endl;
    struct SensorStruct ss;
    ss.msgTime = 1000;
    common::can_comms_data_msg eo;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 3);
  }
  else if (testNum == 3){
    cout << "INT_7 test 3" << endl;
    struct SensorStruct ss;
    ss.msgTime = 110;
    common::can_comms_data_msg eo;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0); // buffer
    ss.msgTime = 200;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 50;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 2); // buffer
    ss.msgTime = 1000;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 2);
  }
  else if (testNum == 4){
    cout << "INT_7 test 4" << endl;
    struct SensorStruct ss;
    ss.msgTime = 110;
    common::can_comms_data_msg eo;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 200;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 50;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 3); // buffer
    ss.msgTime = 1000;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 3);
  }
  else if (testNum == 5){
    cout << "INT_7 test 5" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 300;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 2);
    ss.msgTime = 110;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 900;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
  }
  else if (testNum == 6){
    cout << "INT_7 test 6" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 300;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 3);
    ss.msgTime = 110;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 900;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
  }
  else if (testNum == 7){
    cout << "INT_7 test 7" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 300;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 3);
    ss.msgTime = 110;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 300;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 50;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 2); // buffer
    ss.msgTime = 1000;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 2);
  }
  else if (testNum == 8){
    cout << "INT_7 test 8" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 300;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 2);
    ss.msgTime = 110;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 300;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 50;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 3); // buffer
    ss.msgTime = 1000;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 3);
  }
  else if (testNum == 9){
    cout << "INT_7 test 9" << endl;
    struct SensorStruct ss;
    common::can_comms_data_msg eo;
    ss.msgTime = 110;
    eo.acc_valid = 0;
    eo.aeb_valid = 0;
    eo.lcc_valid = 0;
    eo.long_accel = 0;
    eo.lcc_steer = 0;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 200;
    eo.acc_valid = 1;
    eo.aeb_valid = 1;
    eo.lcc_valid = 1;
    eo.long_accel = 1;
    eo.lcc_steer = 1;
    runTest(n, &ss, eo, 0);
    ss.msgTime = 35;
    runTest(n, &ss, eo, 3); // buffer
    ss.msgTime = 35;
    runTest(n, &ss, eo, 2);
  }
}


// returns fraction of time that test wasn't working for
pair<int,int> runFractionTest(ros::NodeHandle* n, struct SensorStruct* input, common::can_comms_data_msg expectedOutput, int badRC){
  ros::Rate rate(200);
  ros::SteadyTime start = ros::SteadyTime::now();
  unsigned long timePassed = 0;
  int count = input->alive_rolling_counter_MABx;
  int notworking = 0;
  int working = 0;
  
  while (timePassed < input->msgTime && ros::ok()){   
    count += 1; 
    // publish messages
    // increment rolling counters
    //cout << input->alive_rolling_counter_MABx << endl;
    if (!badRC || badRC == 2){
      input->alive_rolling_counter_MABx = count % 16;
    }
    if (!badRC || badRC == 3){
      input->alive_rolling_counter_Jetson = count % 16;
    }
    sendMsg(n, *input);
    
    // check output
    // dont check expected output for 20 ms after changing input since there's a delay in receiving messages
    if (timePassed > 20){
      if (checkExpectedOutputBool(n, expectedOutput))
        working += 1;
      else notworking += 1;
    }

    ros::spinOnce();
    rate.sleep();
    ros::SteadyTime curr = ros::SteadyTime::now();
    timePassed = ((curr.toNSec() - start.toNSec()) / 1000000) % 1000000000000000000;
  }
  return pair<int,int>(working,working+notworking);
}

void ACC_18(ros::NodeHandle* n, int testNum){
  cout << "Setting up ACC_18 test..." << endl;
  // first 100 ms will result in false acc_valid since there's a 100 ms buffer in 
  // the rolling counter functions, so we will run regularly for 130 ms 
  // before each test
  // we DONT care what this section outputs
  common::can_comms_data_msg eo;
  struct SensorStruct ss;
  eo.aeb_valid = 0;
  eo.lcc_valid = 0;
  eo.long_accel = 0;
  eo.lcc_steer = 0;
  ss.msgTime = 110;
  eo.acc_valid = 0;
  eo.acc_fault = 0;
  runFractionTest(n, &ss, eo, 0);
  eo.aeb_valid = 1;
  eo.lcc_valid = 1;
  eo.long_accel = 1;
  eo.lcc_steer = 1;
  eo.acc_fault = 0;
  eo.acc_valid = 1;
  if (testNum == 1){ 
    pair<int,int> fraction;
    cout << "ACC_18 test 1" << endl;
    ss.msgTime = 2000;
    ss.acc_allowed = 1;
    fraction = runFractionTest(n, &ss, eo, 0);
    cout << fraction.first << " out of " << fraction.second << endl;
  }
  else if (testNum == 2){ 
    pair<int,int> fraction;
    cout << "ACC_18 test 2" << endl;
    ss.msgTime = 2000;
    ss.acc_allowed = 0;
    pair<int, int> testRes = runFractionTest(n, &ss, eo, 0);
    fraction.first += testRes.first;
    fraction.second += testRes.second;
    cout << fraction.first << " out of " << fraction.second << endl;
  }
  else if (testNum == 3){ 
    bool correctAtLeastOnce = false;
    cout << "ACC_18 test 3" << endl;
    pair<int,int> fraction;
    pair<int, int> testRes;
    for (int x=0;x<12;x+=1){
      ss.msgTime = 50;
      ss.acc_allowed = 0;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
      ss.msgTime = 50;
      ss.acc_allowed = 1;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
    }
    cout << fraction.first << " out of " << fraction.second << endl;
  }
  else if (testNum == 4){ 
    cout << "ACC_18 test 4" << endl;
    pair<int,int> fraction;
    pair<int, int> testRes;
    for (int x=0;x<48;x+=1){
      ss.msgTime = 30;
      ss.acc_allowed = 0;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
      ss.msgTime = 70;
      ss.acc_allowed = 1;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
    }
    cout << fraction.first << " out of " << fraction.second << endl;
  }
}

void LCC_12(ros::NodeHandle* n, int testNum){
  cout << "Setting up LCC_12 test..." << endl;
  common::can_comms_data_msg eo;
  struct SensorStruct ss;
  eo.aeb_valid = 0;
  eo.acc_valid = 0;
  eo.long_accel = 0;
  eo.lcc_steer = 0;
  ss.msgTime = 110;
  eo.lcc_valid = 0;
  eo.lcc_fault = 0;
  runFractionTest(n, &ss, eo, 0);
  eo.aeb_valid = 1;
  eo.acc_valid = 1;
  eo.long_accel = 1;
  eo.lcc_steer = 1;
  eo.lcc_fault = 0;
  eo.lcc_valid = 1;
  if (testNum == 1){ 
    pair<int,int> fraction;
    cout << "LCC_12 test 1" << endl;
    ss.msgTime = 2000;
    ss.lcc_allowed = 1;
    fraction = runFractionTest(n, &ss, eo, 0);
    cout << fraction.first << " out of " << fraction.second << endl;
  }
  else if (testNum == 2){ 
    pair<int,int> fraction;
    cout << "LCC_12 test 2" << endl;
    ss.msgTime = 2000;
    ss.lcc_allowed = 0;
    pair<int, int> testRes = runFractionTest(n, &ss, eo, 0);
    fraction.first += testRes.first;
    fraction.second += testRes.second;
    cout << fraction.first << " out of " << fraction.second << endl;
  }
  else if (testNum == 3){ 
    bool correctAtLeastOnce = false;
    cout << "LCC_12 test 3" << endl;
    pair<int,int> fraction;
    pair<int, int> testRes;
    for (int x=0;x<12;x+=1){
      ss.msgTime = 50;
      ss.lcc_allowed = 0;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
      ss.msgTime = 50;
      ss.lcc_allowed = 1;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
    }
    cout << fraction.first << " out of " << fraction.second << endl;
  }
  else if (testNum == 4){ 
    cout << "LCC_12 test 4" << endl;
    pair<int,int> fraction;
    pair<int, int> testRes;
    for (int x=0;x<48;x+=1){
      ss.msgTime = 30;
      ss.lcc_allowed = 0;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
      ss.msgTime = 70;
      ss.lcc_allowed = 1;
      testRes = runFractionTest(n, &ss, eo, 0);
      fraction.first += testRes.first;
      fraction.second += testRes.second;
    }
    cout << fraction.first << " out of " << fraction.second << endl;
  }
}

int main(int argc, char **argv)
  {
    ros::init(argc, argv, "master_task_unit_test");
    
    ros::NodeHandle n;
    ros::ServiceClient client2 = n.serviceClient<common::sensor_diagnostic_flag_CH2>("sensor_diagnostic_CH2");
    ros::ServiceClient client3 = n.serviceClient<common::sensor_diagnostic_flag_CH3>("sensor_diagnostic_CH3");
    ros::ServiceClient client4 = n.serviceClient<common::sensor_diagnostic_flag_CH4>("sensor_diagnostic_CH4");

    common::sensor_diagnostic_flag_CH2 srv2;
    common::sensor_diagnostic_flag_CH3 srv3;
    common::sensor_diagnostic_flag_CH4 srv4;

    srv2.request.front_radar = true;
    srv3.request.left_corner_radar = true;
    srv3.request.right_corner_radar = true;
    srv4.request.mobileye = true;

    drive_crtl_pub = n.advertise<common::drive_ctrl_input_msg>("drive_control_input", 1000);
    acc_pub = n.advertise<common::acc_output_msg>("acc_output", 1000);
    aeb_pub = n.advertise<common::aeb_output_msg>("aeb_output", 1000);
    lcc_pub = n.advertise<common::lcc_output_msg>("lcc_output", 1000); 
    can_comms_data = n.subscribe("can_comms_data", 1000, can_comms_callback);

    // // INT-2 test / INT-7 test 9
    // ros::Duration(1.0).sleep();
    // INT_2(&n, 1);
    // ros::Duration(1.0).sleep();
    // INT_2(&n, 2);
    // ros::Duration(1.0).sleep();
    // INT_2(&n, 3);
    // ros::Duration(1.0).sleep();
    // INT_2(&n, 4);
    // ros::Duration(1.0).sleep();
    // INT_2(&n, 5);
    // ros::Duration(1.0).sleep();
    // INT_2(&n, 6);

    // INT-7 tests 1-8
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 1);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 2);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 3);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 4);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 5);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 6);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 7);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 8);
    // ros::Duration(1.0).sleep();
    // INT_7(&n, 9);

    // ACC-18 & LCC-12 tests
    // run these one at a time
    // have acc_18() in master task callback function
    // ACC_18(&n, 1);
    // ACC_18(&n, 2);
    // ACC_18(&n, 3);
    // ACC_18(&n, 4);
    // have both acc_18() and lcc_12 in master task callback function
    // LCC_12(&n, 1);
    // LCC_12(&n, 2);
    // LCC_12(&n, 3);
    // LCC_12(&n, 4);

    // std::cout << "check 1" << std::endl;
    // if (client2.call(srv2))
    // {
    //   ROS_INFO("srv2 successfully called!");
    // }
    // else
    // {
    //   ROS_ERROR("Failed to call service srv2");
    //   return 1;
    // }

    // if (client3.call(srv3))
    // {
    //   ROS_INFO("srv3 successfully called!");
    // }
    // else
    // {
    //   ROS_ERROR("Failed to call service srv3");
    //   return 1;
    // }


    // if (client4.call(srv4))
    // {
    //   ROS_INFO("srv4 successfully called!");
    // }
    // else
    // {
    //   ROS_ERROR("Failed to call service srv4");
    //   return 1;
    // }
      
  //ACC_17(&n);
   return 1;
 }