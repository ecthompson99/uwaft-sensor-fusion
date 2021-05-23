#include "env_state.h"
#include <cinttypes>

EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
  filtered_object_sub = env_state_node_handle->subscribe("filtered_obj", MESSAGE_BUFFER_SIZE, &EnvironmentState::filtered_object_callback, this);
  tracked_obj_pub = env_state_node_handle->advertise<common::tracked_output_msg>("tracked_obj", MESSAGE_BUFFER_SIZE);
  all_tracked_obj_pub = env_state_node_handle->advertise<common::target_output_msg>("all_tracked_obj", MESSAGE_BUFFER_SIZE);
  target_obj_pub = env_state_node_handle->advertise<common::target_output_msg>("target_output", MESSAGE_BUFFER_SIZE);
  binary_class_pub = env_state_node_handle->advertise<common::binary_class_msg>("binary_class", MESSAGE_BUFFER_SIZE);
  service = env_state_node_handle->advertiseService("env_service_topic", &EnvironmentState::env_state_srv_callback, this);

  global_clk = 0;
  trackedObjects.reserve(MAX_OBJ); 	// reserve memory for vector (MAX_OBJ)

  EnvironmentState::reset_tracks();
}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::reset_tracks() { 
  ObjectState default_obj;
  // Initialize target obj in each lane
  targetObjectsInLanes[0] = default_obj;
  targetObjectsInLanes[1] = default_obj;
  targetObjectsInLanes[2] = default_obj;
  trackedObjects.clear();
  printf("Tracks reset\n");
}

void EnvironmentState::publish_target_obj() { // from left to right

  // ROS_INFO_STREAM("counter" << counter);
  // ROS_INFO_STREAM("a" << targetObjectsInLanes[0].get_obj_timestamp());
  // ROS_INFO_STREAM("b" << prev_time_target);
  // if ((targetObjectsInLanes[0].get_obj_timestamp() == prev_time_target)){

  // if (counter > COUNTER_LIM) {
  //   target_output_msg.obj_id = 0;
  //   target_output_msg.obj_dx = 255;
  //   target_output_msg.obj_lane = 0;
  //   target_output_msg.obj_vx = 0;
  //   target_output_msg.obj_dy = 0;
  //   target_output_msg.obj_ax = 0;
  //   target_output_msg.obj_path = 0;
  //   target_output_msg.obj_vy = 0;
  //   target_output_msg.obj_timestamp = 0;

  // } else {
    target_output_msg.obj_id = targetObjectsInLanes[0].get_obj_id();
    target_output_msg.obj_dx = targetObjectsInLanes[0].get_obj_dx();
    target_output_msg.obj_lane = targetObjectsInLanes[0].get_obj_lane(); // 1 to 3
    target_output_msg.obj_vx = targetObjectsInLanes[0].get_obj_vx();
    target_output_msg.obj_dy = targetObjectsInLanes[0].get_obj_dy();
    target_output_msg.obj_ax = targetObjectsInLanes[0].get_obj_ax();
    target_output_msg.obj_path = targetObjectsInLanes[0].get_obj_path();
    target_output_msg.obj_vy = targetObjectsInLanes[0].get_obj_vy();
    target_output_msg.obj_timestamp = targetObjectsInLanes[0].get_obj_timestamp();
    // target_output_msg.obj_track_num = static_cast<uint8_t>(targetObjectsInLanes[0].get_obj_lane() + 1);  // 1 to 3
  // }
  // prev_time_target = targetObjectsInLanes[0].get_obj_timestamp();
  target_obj_pub.publish(target_output_msg);
}

void EnvironmentState::publish_tracked_obj() { // from left to right
  for (size_t lane = 0; lane < 3; lane++) {
    // ROS_INFO_STREAM("counter: " << +counter);
    // if (counter > COUNTER_LIM) {
    //   tracked_output_msg.obj_id[lane] = 0;
    //   tracked_output_msg.obj_dx[lane] = 255;
    //   tracked_output_msg.obj_lane[lane] = 0;  // 0 to 2
    //   tracked_output_msg.obj_vx[lane] = 0;
    //   tracked_output_msg.obj_dy[lane] = 0;
    //   tracked_output_msg.obj_ax[lane] = 0;
    //   tracked_output_msg.obj_path[lane] = 0;
    //   tracked_output_msg.obj_vy[lane] = 0;
    //   tracked_output_msg.obj_timestamp[lane] = 0;
    //   //   // tracked_output_msg.obj_track_num[lane] = static_cast<uint8_t>(trackedObjects[lane].get_obj_lane() + 1);
    //   //   // 1 to 3
    // } else {
      tracked_output_msg.obj_id[lane] = targetObjectsInLanes[lane].get_obj_id();
      tracked_output_msg.obj_dx[lane] = targetObjectsInLanes[lane].get_obj_dx();
      tracked_output_msg.obj_lane[lane] = targetObjectsInLanes[lane].get_obj_lane();  // 0 to 2
      tracked_output_msg.obj_vx[lane] = targetObjectsInLanes[lane].get_obj_vx();
      tracked_output_msg.obj_dy[lane] = targetObjectsInLanes[lane].get_obj_dy();
      tracked_output_msg.obj_ax[lane] = targetObjectsInLanes[lane].get_obj_ax();
      tracked_output_msg.obj_path[lane] = targetObjectsInLanes[lane].get_obj_path();
      tracked_output_msg.obj_vy[lane] = targetObjectsInLanes[lane].get_obj_vy();
      tracked_output_msg.obj_timestamp[lane] = targetObjectsInLanes[lane].get_obj_timestamp();
      // tracked_output_msg.obj_track_num[lane] = static_cast<uint8_t>(trackedObjects[lane].get_obj_lane() + 1);  // 1 to 3
    // }
  }
  tracked_obj_pub.publish(tracked_output_msg);
}

void EnvironmentState::publish_all_tracked_obj() { 
  // If trackedObjects is empty or counter < limit, publish default
  // if (trackedObjects.size() >= 1 && counter <= COUNTER_LIM) {
  if (trackedObjects.size() >= 1) {
    // Publish all objects in trackedObjects
    for (size_t i = 0; i < trackedObjects.size(); i++) {
      all_tracked_output_msg.obj_id = trackedObjects[i].get_obj_id();
      all_tracked_output_msg.obj_dx = trackedObjects[i].get_obj_dx();
      all_tracked_output_msg.obj_lane = trackedObjects[i].get_obj_lane();  // 0 to 2
      all_tracked_output_msg.obj_vx = trackedObjects[i].get_obj_vx();
      all_tracked_output_msg.obj_dy = trackedObjects[i].get_obj_dy();
      all_tracked_output_msg.obj_ax = trackedObjects[i].get_obj_ax();
      all_tracked_output_msg.obj_path = trackedObjects[i].get_obj_path();
      all_tracked_output_msg.obj_vy = trackedObjects[i].get_obj_vy();
      all_tracked_output_msg.obj_timestamp = trackedObjects[i].get_obj_timestamp();
      
      all_tracked_obj_pub.publish(all_tracked_output_msg);
    }
  }
}

void EnvironmentState::filtered_object_callback(const common::filtered_object_msg& filtered_msg) {
  // counter = 0;
  ObjectState tracked_msg;
  tracked_msg.copy_info(filtered_msg); // copy constructor
  
  update_env_state(tracked_msg); // update id of objects in state vector
  last_msg_ros_timestamp = ros::Time::now(); // This is the ROS timestamp when last msg received from Kalman Filter 
  last_msg_timestamp = tracked_msg.get_obj_timestamp(); // This is the actual timestamp of the last msg received from KF
}

void EnvironmentState::publish_binary_class() {
  common::binary_class_msg out;
  for (auto i : trackedObjects) {
    out.dx.push_back(i.get_obj_dx());
    out.dy.push_back(i.get_obj_dy());
    out.vx.push_back(i.get_obj_vx());
    out.vy.push_back(i.get_obj_vy());
    out.timestamp.push_back(i.get_obj_timestamp());
  }
  out.global_clk = global_clk;
  binary_class_pub.publish(out);
}

common::target_output_msg EnvironmentState::get_target_output_msg() { return target_output_msg; }

common::tracked_output_msg EnvironmentState::get_tracked_output_msg() { return tracked_output_msg; }

void EnvironmentState::add_object(const ObjectState& tracked_msg) {
  trackedObjects.push_back(tracked_msg); 
}

void EnvironmentState::update_object(const ObjectState& tracked_msg, size_t index) {
  trackedObjects[index] = tracked_msg; 
}

void EnvironmentState::check_tracked_time() {
  for (size_t index = 0; index < trackedObjects.size(); index++) {
    // printf("Tracked obj id: %lu, time difference: %f\r\n", trackedObjects[index].get_obj_id(), last_msg_timestamp - trackedObjects[index].get_obj_timestamp());
    if ((last_msg_timestamp - trackedObjects[index].get_obj_timestamp()) > UPDATE_TOL) {  // more recent timestamps are larger
      trackedObjects.erase(trackedObjects.begin() + index);
      index--;
      printf("Erased object from trackedObjects\r\n");
    }
  }
}

void EnvironmentState::update_env_state(const ObjectState& tracked_msg) {
  bool found = 0;
  size_t index_found = 0;
  // printf("New msg timesatmp: %f\r\n", tracked_msg.get_obj_timestamp());

  for (size_t index = 0; index < trackedObjects.size(); index++) {
    // if object is found in the state vector (has been tracked), update it's ID
    if (tracked_msg.get_obj_id() == trackedObjects[index].get_obj_id()) {
      found = true;
      index_found = index;
    }
  }

  // if object has been tracked, update the object in the state vector
  if (found == true) {
    update_object(tracked_msg, index_found);
  }
  // if object has not been tracked, add the object to state vector
  else {
    add_object(tracked_msg);
  }
  for(size_t i = 0; i < trackedObjects.size(); i++){
    printf("Tracked id: %lu, timestamp: %f, dx: %f\n", trackedObjects[i].get_obj_id(), trackedObjects[i].get_obj_timestamp(), trackedObjects[i].get_obj_dx());
  }
}

// void EnvironmentState::find_target_object(const ObjectState& tracked_msg){
void EnvironmentState::find_target_object(){
  ObjectState empty_obj(0, 255, 0, 0, 0, 0, 0, 0, 0, 0);

  for (size_t i = 0; i < trackedObjects.size(); i++) {
    int tracked_lane = trackedObjects[i].get_obj_lane();
    // 1 = center lane, 2 = left lane, 3 = right lane
    // update target object if a new object is closer than current target in the same lane, or if the current target moves
    if (tracked_lane == 1) {
      if ((trackedObjects[i].get_obj_dx() <= targetObjectsInLanes[0].get_obj_dx()) || 
          (trackedObjects[i].get_obj_id() == targetObjectsInLanes[0].get_obj_id())) {
          targetObjectsInLanes[0] = trackedObjects[i];
      } else {
        targetObjectsInLanes[0] = empty_obj;
      }
    } else if (tracked_lane == 2) {
        if ((trackedObjects[i].get_obj_dx() <= targetObjectsInLanes[1].get_obj_dx()) ||
            (trackedObjects[i].get_obj_id() == targetObjectsInLanes[1].get_obj_id())) {
            targetObjectsInLanes[1] = trackedObjects[i];
        } else {
          targetObjectsInLanes[1] = empty_obj;
        }
    } else if (tracked_lane == 3) {
        if ((trackedObjects[i].get_obj_dx() <= targetObjectsInLanes[2].get_obj_dx()) ||
            (trackedObjects[i].get_obj_id() == targetObjectsInLanes[2].get_obj_id())) {
            targetObjectsInLanes[2] = trackedObjects[i];
        } else {
          targetObjectsInLanes[2] = empty_obj;
        }
    }
  }

}

bool EnvironmentState::env_state_srv_callback(sensor_fusion::env_state_srv::Request& /*req*/,
                                              sensor_fusion::env_state_srv::Response& res) {
  // breaking the objects stored in the vector into members and storing in multiple vectors for srv communication
  // index refers to specific objectNum in the original vector
  for (size_t i = 0; i < trackedObjects.size(); i++) {
    res.id.push_back(trackedObjects[i].get_obj_id());
    res.dx.push_back(trackedObjects[i].get_obj_dx());
    res.dy.push_back(trackedObjects[i].get_obj_dy());
    res.timestamp.push_back(trackedObjects[i].get_obj_timestamp());
    // printf("returned %f, %f for %lu\n", res.dx[i], res.dy[i], res.id[i]);
    // res.count.push_back(trackedObjects[i].get_obj_count());
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "env_state");
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state = EnvironmentState(&env_state_node_handle);
  ros::Time mem1 = ros::Time::now();
  ros::Time mem2 = ros::Time::now();

  while (ros::ok()) {
    ros::Time time_now = ros::Time::now();

    // Stores the time difference since last time a message was received. If Kalman Filter
    // has not sent a message in the past UPDATE_TOL, then reset everything to publish default values
    double update_time_difference = time_now.toSec() - env_state.last_msg_ros_timestamp.toSec();
    if (update_time_difference > UPDATE_TOL) {
      env_state.reset_tracks();
    }

    // While the env_state node is running, we should be checking validity of messages
    // to remove outdated objects as well as updating the 3 tracked obj and target obj to publish
    env_state.check_tracked_time(); 
    env_state.find_target_object(); 

    if (time_now.toSec() - mem1.toSec() > 0.01) {
      env_state.publish_target_obj();
      env_state.publish_tracked_obj();
      env_state.publish_all_tracked_obj();
      env_state.publish_binary_class();
      env_state.global_clk += 0.01;
      // env_state.counter = env_state.counter + 0.01;
      // env_state.counter = env_state.counter + (time_now.toSec() - mem1.toSec());
      mem1 = time_now;
    }

    ros::spinOnce();
  }
	// ros::spin();

  return 0;
}
