#ifndef __ENV_STATE_H__
#define __ENV_STATE_H__

#include "ros/ros.h"
#include "object_state.h"
#include "common/filtered_object_msg.h" // sub
#include "common/tracked_output_msg.h"  // pub
#include "common/target_output_msg.h"  // pub
#include "common/binary_class_msg.h"
#include "sensor_fusion/env_state_srv.h"  //service

#include <vector>

static const uint8_t MESSAGE_BUFFER_SIZE = 10;

class EnvironmentState {
  public:        
    EnvironmentState(ros::NodeHandle* env_state_node_handle);
    virtual ~EnvironmentState();
    void publish_target_obj();
    void publish_tracked_obj();
    void publish_binary_class();
    void filtered_object_callback(const common::filtered_object_msg& filtered_msg);
    common::tracked_output_msg get_tracked_output_msg();
    common::target_output_msg get_target_output_msg();

    void add_object(const ObjectState& tracked_msg);
    void update_object(const ObjectState& tracked_msg, size_t index);
    void check_timestamp(const ObjectState& tracked_msg);
    void update_env_state(const ObjectState& tracked_msg); 
    // void find_target_object(const ObjectState& tracked_msg);
    void find_target_object();

    bool env_state_srv_callback(sensor_fusion::env_state_srv::Request &req, sensor_fusion::env_state_srv::Response &res);
    bool changed_lane(int target_lane);

    std::vector<ObjectState> trackedObjects;
    ObjectState targetObjectsInLanes[3];
    double global_clk;
    double prev_time[3] = {-1,-1,-1};
    double prev_time_target = -1;

  private:
    ros::NodeHandle* env_state_node_handle;
    ros::Subscriber filtered_object_sub;
    ros::Publisher tracked_obj_pub;
    ros::Publisher target_obj_pub;
    ros::Publisher binary_class_pub;
    common::tracked_output_msg tracked_output_msg;
    common::target_output_msg target_output_msg;
    ros::ServiceServer service;
};

#endif  // __ENV_STATE_H__
