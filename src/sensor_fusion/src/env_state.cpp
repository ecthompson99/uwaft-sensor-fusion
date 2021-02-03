#include "env_state.h"
#include <cinttypes>

#define TIMESTAMP_TOL 5 // 5 s tolerance used to determine outdated tracks
#define MAX_OBJ 32


EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
	filtered_object_sub = env_state_node_handle->subscribe("filtered_obj", MESSAGE_BUFFER_SIZE, &EnvironmentState::filtered_object_callback, this);

	tracked_obj_pub = env_state_node_handle->advertise<common::tracked_output_msg>("tracked_obj", MESSAGE_BUFFER_SIZE);

	target_obj_pub = env_state_node_handle->advertise<common::target_output_msg>("target_obj", MESSAGE_BUFFER_SIZE);

	binary_class_pub = env_state_node_handle->advertise<common::binary_class_msg>("binary_class", MESSAGE_BUFFER_SIZE);
	
    global_clk = 0;

	trackedObjects.reserve(MAX_OBJ); 	// reserve memory for vector (MAX_OBJ)

	// initialize target object (default)
    ObjectState initialize_target(0, 255, 0, 0, 0, 0, 0, 0, 0, 0);

    target = initialize_target;
    service = env_state_node_handle->advertiseService("env_service_topic", &EnvironmentState::env_state_srv_callback, this);     
    target1 = 0;
    first_timestamp = 1;

}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_target_obj() {
    target_output_msg.obj_id = target.get_obj_id();
    target_output_msg.obj_dx = target.get_obj_dx();
    target_output_msg.obj_lane = target.get_obj_lane(); // 0 to 2
    target_output_msg.obj_vx = target.get_obj_vx();
    target_output_msg.obj_dy = target.get_obj_dy();
    target_output_msg.obj_ax = target.get_obj_ax();
    target_output_msg.obj_path = target.get_obj_path();
    target_output_msg.obj_vy = target.get_obj_vy();
    target_output_msg.obj_timestamp = target.get_obj_timestamp();
    target_output_msg.obj_track_num = static_cast<uint8_t>(target.get_obj_lane() + 1);  // 1 to 3

    target_obj_pub.publish(target_output_msg);
}

void EnvironmentState::publish_tracked_obj() { // from left to right
    for (size_t lane = 0; lane < trackedObjects.size(); lane++) {
        tracked_output_msg.obj_id = trackedObjects[lane].get_obj_id();
        tracked_output_msg.obj_dx = trackedObjects[lane].get_obj_dx();
        tracked_output_msg.obj_lane = trackedObjects[lane].get_obj_lane();  // 0 to 2
        tracked_output_msg.obj_vx = trackedObjects[lane].get_obj_vx();
        tracked_output_msg.obj_dy = trackedObjects[lane].get_obj_dy();
        tracked_output_msg.obj_ax = trackedObjects[lane].get_obj_ax();
        tracked_output_msg.obj_path = trackedObjects[lane].get_obj_path();
        tracked_output_msg.obj_vy = trackedObjects[lane].get_obj_vy();
        tracked_output_msg.obj_timestamp = trackedObjects[lane].get_obj_timestamp();
        tracked_output_msg.obj_track_num = static_cast<uint8_t>(trackedObjects[lane].get_obj_lane() + 1);  // 1 to 3
    }

    tracked_obj_pub.publish(tracked_output_msg);
}

void EnvironmentState::filtered_object_callback(const common::filtered_object_msg& filtered_msg) {

    ObjectState tracked_msg;
    tracked_msg.copy_info(filtered_msg); // copy constructor
    
    update_env_state(tracked_msg); // update id of objects in state vector

    check_timestamp(tracked_msg); // removes outdated state vector
  
    find_target_object(tracked_msg);     // fill array with target obj

	if (first_timestamp) {
		global_clk = filtered_msg.obj_timestamp;
		first_timestamp = 0;
	}
	else if (filtered_msg.obj_timestamp > global_clk + 0.1) {
		publish_binary_class(filtered_msg.obj_timestamp);
		publish_tracked_obj();
        publish_target_obj();
	}
    
}

void EnvironmentState::publish_binary_class(double t) {
	common::binary_class_msg out;
	for (auto i : trackedObjects) {
		out.dx.push_back(i.get_obj_dx());
		out.dy.push_back(i.get_obj_dy());
	}
	out.global_clk = global_clk;
	out.timestamp = t;
	binary_class_pub.publish(out);
	global_clk += 0.1;
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
	if (found == true)
		update_object(tracked_msg, index_found);

	// if object has not been tracked, add the object to state vector
	else
	    add_object(tracked_msg);

	printf("%lu: %f %f\n", trackedObjects[0].get_obj_id(), trackedObjects[0].get_obj_dx(), trackedObjects[0].get_obj_dy());
}

void EnvironmentState::find_target_object(const ObjectState& tracked_msg){
    
    int target_lane = target.get_obj_lane();
    int tracked_lane = tracked_msg.get_obj_lane();

    // first time fill target
    if (target1 = false){
        target = tracked_msg;
        target1 = true;
    }

    // We want the target to be directly ahead of us
    else if (tracked_lane == 1){
        if ((tracked_msg.get_obj_dx() <= target.get_obj_dx()) ||
            ((tracked_msg.get_obj_dx() > target.get_obj_dx()) && 
            (tracked_msg.get_obj_id() == target.get_obj_id()))){
                target = tracked_msg;
        }
    }
 

    // if don't find target_object, then will be default (set in constructor)
    else{}
    
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
   
    // while (ros::ok()) {
	// 	// env_state.publish_target_obj();
	// 	env_state.publish_tracked_obj();
	// 	ros::Rate(10).sleep();
	// 	ros::spinOnce();
    // }
	ros::spin();

	return 0;
}




