#include "env_state.h"
//#include "object_state.h"

#define TIMESTAMP_TOL 10 // 10 ms tolerance used to determine outdated tracks
#define NUM_OBJECTS 100

bool target1, target2, target3 = false;

EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
  filtered_object_sub = env_state_node_handle->subscribe("kf_dummy_data", MESSAGE_BUFFER_SIZE,
                                                            &EnvironmentState::filtered_object_callback, this);

  object_output_pub = env_state_node_handle->advertise<sensor_fusion::object_output_msg>("object_output",
                                                                                             MESSAGE_BUFFER_SIZE);

	// reserve memory for vector (NUM_OBJECTS)
	trackedObjects.reserve(NUM_OBJECTS);
	
	
  //my_service = env_state_node_handle->advertiseService("srv_env_state_topic", &EnvironmentState::env_state_vector_service_callback, this);     

}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_object_output() {
//  for (int index = 0; index < 3; index++) {
//     object_output_msg.obj_id = targetObjects[index].get_obj_id();
//     object_output_msg.obj_dx = targetObjects[index].get_obj_dx();
//     object_output_msg.obj_lane = targetObjects[index].get_obj_lane();
//     object_output_msg.obj_vx = targetObjects[index].get_obj_vx();
//     object_output_msg.obj_dy = targetObjects[index].get_obj_dy();
//     object_output_msg.obj_ax = targetObjects[index].get_obj_ax();
//     object_output_msg.obj_path = targetObjects[index].get_obj_path();
//     object_output_msg.obj_vy = targetObjects[index].get_obj_vy();
//     object_output_msg.obj_timestamp = targetObjects[index].get_obj_timestamp();
//     object_output_msg.object_track_num = targetObjects[index].get_obj_lane();

//     printf("Target Object %d \n", index);
//     printf("%d, %f, %d, %f, %f, %f, %d, %f, %f \n",
//     targetObjects[index].get_obj_id(), targetObjects[index].get_obj_dx(), targetObjects[index].get_obj_lane(), 
//     targetObjects[index].get_obj_vx(), targetObjects[index].get_obj_dy(), targetObjects[index].get_obj_ax(), 
//     targetObjects[index].get_obj_path(), targetObjects[index].get_obj_vy(), targetObjects[index].get_obj_timestamp());

//     printf("Object Output Msg\n");
//     printf("%d, %f, %d, %f, %f, %f, %d, %f, %f, %f \n",
//     object_output_msg.obj_id, object_output_msg.obj_dx, object_output_msg.obj_lane, 
//     object_output_msg.obj_vx, object_output_msg.obj_dy, object_output_msg.obj_ax, 
//     object_output_msg.obj_path, object_output_msg.obj_vy, object_output_msg.obj_timestamp, 
//     object_output_msg.object_track_num);
//    
//    object_output_pub.publish(object_output_msg);
//  }
}

void EnvironmentState::filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg) {
    // printf("Filtered Message:\n");
    // printf("%d, %f, %d, %f, %f, %f, %d, %f, %f \n",
    // filtered_msg.obj_id, filtered_msg.obj_dx, filtered_msg.obj_lane, 
    // filtered_msg.obj_vx, filtered_msg.obj_dy, filtered_msg.obj_ax, 
    // filtered_msg.obj_path, filtered_msg.obj_vy, filtered_msg.obj_timestamp);

    ObjectState tracked_msg;
    tracked_msg.copy_info(filtered_msg); // copy constructor

//     printf("Tracked Message:\n");
//     printf("%d, %f, %d, %f, %f, %f, %d, %f, %f \n",
//     tracked_msg.get_obj_id(), tracked_msg.get_obj_dx(), tracked_msg.get_obj_lane(), 
//     tracked_msg.get_obj_vx(), tracked_msg.get_obj_dy(), tracked_msg.get_obj_ax(), 
//     tracked_msg.get_obj_path(), tracked_msg.get_obj_vy(), tracked_msg.get_obj_timestamp());

    update_env_state(tracked_msg); // update id of objects in state vector

    check_timestamp(tracked_msg); // removes outdated state vector
  
    find_target_objects(tracked_msg);     // fill array with target objects
    
}


sensor_fusion::object_output_msg EnvironmentState::get_object_output_msg() { return object_output_msg; }

void EnvironmentState::add_object(const ObjectState& tracked_msg) {
  EnvironmentState::trackedObjects.push_back(tracked_msg); 
}

void EnvironmentState::update_object(const ObjectState& tracked_msg, int index) {
  EnvironmentState::trackedObjects[index] = tracked_msg; 
}

void EnvironmentState::check_timestamp(const ObjectState& tracked_msg) {

	bool indices[NUM_OBJECTS] = {0};
	int count = 1;
  
  for (int index = 0 ;index < EnvironmentState::trackedObjects.size(); index++){	
  	if ((tracked_msg.get_obj_timestamp() - EnvironmentState::trackedObjects[index].get_obj_timestamp()) > TIMESTAMP_TOL){ // assume more recent timestamps are larger
  		indices[index + 1] = true; // have to add 1 since vector .begin starts at 1
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
	int index_found = 0;

  for (int index = 0; index < EnvironmentState::trackedObjects.size(); index++){
    // if object is found in the state vector (has been tracked), update it's ID
    if (tracked_msg.get_obj_id() == EnvironmentState::trackedObjects[index].get_obj_id()){
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
			
}
void EnvironmentState::find_target_objects(const ObjectState& tracked_msg){
    
    int lane = tracked_msg.get_obj_lane();
		
		// first time fill target1 
		if (target1 == false && lane == 0){
			targetObjects[lane] = tracked_msg;
			target1 = true;
		}
		
		// first time fill target2
		else if (target2 == false && lane == 1){
			targetObjects[lane] = tracked_msg;
			target2 = true;
		}
			
		// first time fill target3
		else if (target3 == false && lane == 2){
			targetObjects[lane] = tracked_msg;
			target3 = true;
		}
		
   	else if ((tracked_msg.get_obj_dx() <= targetObjects[lane].get_obj_dx()) ||
         ((tracked_msg.get_obj_dx() > targetObjects[lane].get_obj_dx()) && 
         (tracked_msg.get_obj_id() == targetObjects[lane].get_obj_id()))){
      targetObjects[lane] = tracked_msg;
    }
    
    else{}
    
    //TESTING-------------------------------------------------------
//    printf("Target Object 1:\n");
//     printf("%d, %f, %d, %f, %f, %f, %d, %f, %f \n",
//     targetObjects[0].get_obj_id(), targetObjects[0].get_obj_dx(), targetObjects[0].get_obj_lane(), 
//     targetObjects[0].get_obj_vx(), targetObjects[0].get_obj_dy(), targetObjects[0].get_obj_ax(), 
//     targetObjects[0].get_obj_path(), targetObjects[0].get_obj_vy(), targetObjects[0].get_obj_timestamp());

}


// service callback
//bool EnvironmentState::env_state_vector_service_callback(sensor_fusion::env_state_srv::Request &req, sensor_fusion::env_state_srv::Response &res){
  //res.classObj = 3;
  
//  return true;
//}




