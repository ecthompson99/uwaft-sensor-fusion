#include "env_state.h"
#include <cinttypes>

#define TIMESTAMP_TOL 5 // 5 s tolerance used to determine outdated tracks
#define MAX_OBJ 32

EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
  filtered_object_sub = env_state_node_handle->subscribe("filtered_obj", MESSAGE_BUFFER_SIZE, &EnvironmentState::filtered_object_callback, this);
  tracked_obj_pub = env_state_node_handle->advertise<common::tracked_output_msg>("tracked_obj", MESSAGE_BUFFER_SIZE);
  target_obj_pub = env_state_node_handle->advertise<common::target_output_msg>("target_output", MESSAGE_BUFFER_SIZE);
  binary_class_pub = env_state_node_handle->advertise<common::binary_class_msg>("binary_class", MESSAGE_BUFFER_SIZE);
    
  global_clk = 0;
  trackedObjects.reserve(MAX_OBJ); 	// reserve memory for vector (MAX_OBJ)

  // initialize target object (default)
  ObjectState initialize_target(0, 255, 0, 0, 0, 0, 0, 0, 0, 0);
  
  // Initialize target obj in each lane
  targetObjectsInLanes[0] = initialize_target;
  targetObjectsInLanes[1] = initialize_target;
  targetObjectsInLanes[2] = initialize_target;

  service = env_state_node_handle->advertiseService("env_service_topic", &EnvironmentState::env_state_srv_callback, this);     
}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_target_obj() { // from left to right
  // if (targetObjectsInLanes[0].get_obj_timestamp() == prev_time_target)
  // {
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
    // if (targetObjectsInLanes[lane].get_obj_timestamp() == prev_time[lane])
    // {
    //   tracked_output_msg.obj_id[lane] = 0;
    //   tracked_output_msg.obj_dx[lane] = 0;
    //   tracked_output_msg.obj_lane[lane] = 0;  // 0 to 2
    //   tracked_output_msg.obj_vx[lane] = 0;
    //   tracked_output_msg.obj_dy[lane] = 0;
    //   tracked_output_msg.obj_ax[lane] = 0;
    //   tracked_output_msg.obj_path[lane] = 0;
    //   tracked_output_msg.obj_vy[lane] = 0;
    //   tracked_output_msg.obj_timestamp[lane] = 0;
    //   // tracked_output_msg.obj_track_num[lane] = static_cast<uint8_t>(trackedObjects[lane].get_obj_lane() + 1);  // 1 to 3
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
    // prev_time[lane] = targetObjectsInLanes[lane].get_obj_timestamp();
  }
  tracked_obj_pub.publish(tracked_output_msg);
}

void EnvironmentState::filtered_object_callback(const common::filtered_object_msg& filtered_msg) {
  ObjectState tracked_msg;
  tracked_msg.copy_info(filtered_msg); // copy constructor
  
  update_env_state(tracked_msg); // update id of objects in state vector

  check_timestamp(tracked_msg); // removes outdated state vector

  find_target_object(tracked_msg);     // fill array with target obj
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
  EnvironmentState::trackedObjects.push_back(tracked_msg); 
}

void EnvironmentState::update_object(const ObjectState& tracked_msg, size_t index) {
  EnvironmentState::trackedObjects[index] = tracked_msg; 
}

void EnvironmentState::check_timestamp(const ObjectState& tracked_msg) {

  bool indices[MAX_OBJ] = {0};
  int count = 1;

  for (size_t index = 0; index < EnvironmentState::trackedObjects.size(); index++) {
    if ((tracked_msg.get_obj_timestamp() - EnvironmentState::trackedObjects[index].get_obj_timestamp()) > TIMESTAMP_TOL) {  // assume more recent timestamps are larger
     indices[index + 1] = true;  // have to add 1 since vector .begin starts at 1
    }
  }

  for (auto temp = EnvironmentState::trackedObjects.begin(); temp != EnvironmentState::trackedObjects.end(); temp++){
    if (indices[count] == true){
      // iterator is decremented after it is passed to erase() but before erase() is executed
      EnvironmentState::trackedObjects.erase(temp--);
    }
    count++;
  }
}

void EnvironmentState::update_env_state(const ObjectState& tracked_msg) {
  
  bool found = false;
  size_t index_found = 0;

  for (size_t index = 0; index < EnvironmentState::trackedObjects.size(); index++) {
    // if object is found in the state vector (has been tracked), update it's ID
    if (tracked_msg.get_obj_id() == EnvironmentState::trackedObjects[index].get_obj_id()) {
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
  printf("%lu: %f %f\n", trackedObjects[0].get_obj_id(), trackedObjects[0].get_obj_dx(), trackedObjects[0].get_obj_dy());
}

void EnvironmentState::find_target_object(const ObjectState& tracked_msg){
    
  int tracked_lane = tracked_msg.get_obj_lane();
  ObjectState empty_obj(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

  // 1 = center lane, 2 = left lane, 3 = right lane
  // update target object if a new object is closer than current target or if the current target moves
  
  if (tracked_lane == 1) {
    if ((tracked_msg.get_obj_dx() <= targetObjectsInLanes[0].get_obj_dx()) || 
        (tracked_msg.get_obj_id() == targetObjectsInLanes[0].get_obj_id())) {
          targetObjectsInLanes[0] = tracked_msg;
    } else {
      targetObjectsInLanes[0] = empty_obj;
    }
  } else if (tracked_lane == 2) {
      if ((tracked_msg.get_obj_dx() <= targetObjectsInLanes[1].get_obj_dx()) ||
          (tracked_msg.get_obj_id() == targetObjectsInLanes[1].get_obj_id())) {
            targetObjectsInLanes[1] = tracked_msg;
      } else {
        targetObjectsInLanes[1] = empty_obj;
      }
  } else if (tracked_lane == 3) {
      if ((tracked_msg.get_obj_dx() <= targetObjectsInLanes[2].get_obj_dx()) ||
          (tracked_msg.get_obj_id() == targetObjectsInLanes[2].get_obj_id())) {
            targetObjectsInLanes[2] = tracked_msg;
      } else {
        targetObjectsInLanes[2] = empty_obj;
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
    printf("returned %f, %f for %lu\n", res.dx[i], res.dy[i], res.id[i]);
    // res.count.push_back(trackedObjects[i].get_obj_count());
  }
  return true;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "env_state");
  ros::NodeHandle env_state_node_handle;
  EnvironmentState env_state = EnvironmentState(&env_state_node_handle);
  ros::Time mem1 = ros::Time::now();

  while (ros::ok()) {
    ros::Time time_now = ros::Time::now();
    if (time_now.toSec() - mem1.toSec() > 0.1) {
      env_state.publish_target_obj();
      env_state.publish_tracked_obj();
      env_state.publish_binary_class();
      env_state.global_clk += 0.1;
    }
    ros::spinOnce();
  }
	// ros::spin();

  return 0;
}
