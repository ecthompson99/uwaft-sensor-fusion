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
// #define COUNTER_LIM 2
#define UPDATE_TOL 5.00
#define MAX_OBJ 32

class EnvironmentState {
  public:        
    EnvironmentState(ros::NodeHandle* env_state_node_handle);
    virtual ~EnvironmentState();
    void publish_target_obj(); /*!< Publishes single target object in ego lane. */
    void publish_tracked_obj(); /*!< Publishes target object in adjacent and ego lanes. */
    void publish_all_tracked_obj(); /*!< Publishes all currently tracked objects. */
    void publish_binary_class();
    void filtered_object_callback(const common::filtered_object_msg& filtered_msg); /*!< Called when KF publishes to filtered_object topic. */
    common::tracked_output_msg get_tracked_output_msg();
    common::target_output_msg get_target_output_msg();

    void add_object(const ObjectState& tracked_msg); /*!< Add object to tracked objects vector. */
    void update_object(const ObjectState& tracked_msg, size_t index); /*!< Update tracked object msg at specfied index. */
    void check_tracked_time();  /*!< Check when object was last seen. If over threshold, erase. */
    void update_env_state(const ObjectState& tracked_msg);  /*!< Update env state vector depending on object match or not. */
    // void find_target_object(const ObjectState& tracked_msg);
    void find_target_object();  /*!< Assigns target object in each lane based on dx or ID match. */
    void reset_tracks();

    bool env_state_srv_callback(sensor_fusion::env_state_srv::Request &req, sensor_fusion::env_state_srv::Response &res);
    bool changed_lane(int target_lane);

    std::vector<ObjectState> trackedObjects;
    ObjectState targetObjectsInLanes[3];
    double global_clk;
    double prev_time[3] = {-1,-1,-1};
    double prev_time_target = -1;
    // double counter = 0;
    ros::Time last_msg_ros_timestamp;
    double last_msg_timestamp;

   private:
    ros::NodeHandle* env_state_node_handle;
    ros::Subscriber filtered_object_sub;
    ros::Publisher tracked_obj_pub;
    ros::Publisher all_tracked_obj_pub;
    ros::Publisher target_obj_pub;
    ros::Publisher binary_class_pub;
    common::tracked_output_msg tracked_output_msg;
    common::target_output_msg target_output_msg;
    common::target_output_msg all_tracked_output_msg;
    ros::ServiceServer service;
};

#endif  // __ENV_STATE_H__
