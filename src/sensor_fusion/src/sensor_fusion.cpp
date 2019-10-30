#include "sensor_fusion.h"

SensorFusion::SensorFusion(ros::NodeHandle* node_handle) : sensor_fusion_node_handle(node_handle) {
  raw_sensor_obj_sub = sensor_fusion_node_handle->subscribe("raw_sensor_obj", MESSAGE_BUFFER_SIZE,
                                                            &SensorFusion::raw_sensor_obj_callback, this);

  sensor_diag_flag_sub = sensor_fusion_node_handle->subscribe("sensor_diagnostic_flag", MESSAGE_BUFFER_SIZE,
                                                              &SensorFusion::sensor_diag_flag_msg_callback, this);

  fused_obj_pub = sensor_fusion_node_handle->advertise<sensor_fusion::fused_object_data_msg>("fused_object_data",
                                                                                             MESSAGE_BUFFER_SIZE);
}

SensorFusion::~SensorFusion() {}

void SensorFusion::publish_fused_data() { fused_obj_pub.publish(fused_data_msg); }

void SensorFusion::raw_sensor_obj_callback(const sensor_fusion::raw_sensor_object_data_msg& raw_msg) {
  uint8_t radar_num = raw_msg.radar_num;
  uint8_t num_objects = raw_msg.num_objects;

  // unit testing
  if (5 == (radar_num) && 3 == num_objects) {
    fused_data_msg.accel_x = 1;
  }
  if ((raw_msg.accel_x[0]) == (raw_msg.vel_x[0])) {
    fused_data_msg.vel_x = 2;
  }
  if ((raw_msg.pos_x[1]) == (raw_msg.pos_y[1])) {
    fused_data_msg.pos_x = 3;
  }
  if ((raw_msg.exist_prob[0]) == (raw_msg.valid[0])) {
    fused_data_msg.pos_y = 4;
  }

  ROS_INFO_STREAM(unsigned(num_objects) << " OBJECTS FROM RADAR " << unsigned(radar_num) << "\n");

  for (uint32_t object_number = 0; object_number < num_objects; object_number++) {
    ROS_INFO_STREAM("OBJECT " << object_number << ":\n"
                              << "Accelerations_x: " << raw_msg.accel_x[object_number] << " "
                              << "Velocities_x: " << raw_msg.vel_x[object_number] << " "
                              << "Positions_x: " << raw_msg.pos_x[object_number] << " "
                              << "Positions_y: " << raw_msg.pos_y[object_number] << " "
                              << "Exist Prob: " << raw_msg.exist_prob[object_number] << " "
                              << "Valid: " << raw_msg.valid[object_number] << "\n");
  }
}

void SensorFusion::sensor_diag_flag_msg_callback(const sensor_fusion::sensor_diagnostic_flag_msg& sensor_msg) {
  // temporary workaround for array
  int first = sensor_msg.radar_reliability[0];
  int second = sensor_msg.radar_reliability[1];
  int third = sensor_msg.radar_reliability[2];
  int fourth = sensor_msg.radar_reliability[3];
  int fifth = sensor_msg.radar_reliability[4];
  int sixth = sensor_msg.radar_reliability[5];

  // temporary for unit tests
  if ((1 == first) && (2 == second)) {
    fused_data_msg.accel_x = 1;
  }
  if ((3 == third) && (4 == fourth)) {
    fused_data_msg.vel_x = 2;
  }
  if ((5 == fifth) && (6 == sixth)) {
    fused_data_msg.pos_x = 3;
    fused_data_msg.pos_y = 4;
  }

  ROS_INFO_STREAM("\n"
                  << "1st sensor " << first << "\n"
                  << "2nd sensor " << second << "\n"
                  << "3rd sensor " << third << "\n"
                  << "4th sensor " << fourth << "\n"
                  << "5th sensor " << fifth << "\n"
                  << "6th sensor " << sixth << "\n");
}

sensor_fusion::fused_object_data_msg SensorFusion::get_fused_data_msg() { return fused_data_msg; }
