#include "env_state.h"
//#include "object_state.h"


#define TIMESTAMP_TOL 10 // 10 ms tolerance used to determine outdated tracks
#define NUM_OBJECTS 100

bool target1, target2, target3 = false;

EnvironmentState::EnvironmentState(ros::NodeHandle* node_handle) : env_state_node_handle(node_handle) {
  filtered_object_sub = env_state_node_handle->subscribe("kf_dummy_data", MESSAGE_BUFFER_SIZE,
                                                            &EnvironmentState::filtered_object_callback, this);

  target_obj_pub = env_state_node_handle->advertise<sensor_fusion::target_output_msg>("target_obj",
                                                                                             MESSAGE_BUFFER_SIZE);

	
	tracked_obj_pub = env_state_node_handle->advertise<sensor_fusion::tracked_output_msg>("tracked_obj",
                                                                                             MESSAGE_BUFFER_SIZE);


	trackedObjects.reserve(NUM_OBJECTS); 	// reserve memory for vector (NUM_OBJECTS)

	// initialize array of targetObjects
	ObjectState target_obj1(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ObjectState target_obj2(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
	ObjectState target_obj3(0, 0, 0, 0, 0, 0, 0, 0, 0, 0);

	targetObjects[0] = target_obj1;
	targetObjects[1] = target_obj2;
	targetObjects[2] = target_obj3;

  service = env_state_node_handle->advertiseService("env_service_topic", &EnvironmentState::env_state_srv_callback, this);     

}

EnvironmentState::~EnvironmentState() {}

void EnvironmentState::publish_target_obj() {

	//if (target1 && target2 && target3){ // all 3 targets are filled
			for (int lane = 0; lane < 3; lane++){

		   target_output_msg.obj_id = targetObjects[lane].get_obj_id();
		   target_output_msg.obj_dx = targetObjects[lane].get_obj_dx();
		   target_output_msg.obj_lane = targetObjects[lane].get_obj_lane(); // 0 to 2
		   target_output_msg.obj_vx = targetObjects[lane].get_obj_vx();
		   target_output_msg.obj_dy = targetObjects[lane].get_obj_dy();
		   target_output_msg.obj_ax = targetObjects[lane].get_obj_ax();
		   target_output_msg.obj_path = targetObjects[lane].get_obj_path();
		   target_output_msg.obj_vy = targetObjects[lane].get_obj_vy();
		   target_output_msg.obj_timestamp = targetObjects[lane].get_obj_timestamp();
		   target_output_msg.obj_track_num = targetObjects[lane].get_obj_lane() + 1; // 1 to 3


	//	 TESTING-------------------------------------------------------
		   printf("Target Output Msg\n");
		   printf("%d, %f, %d, %f, %f, %f, %d, %f, %f, %d \n",
		   target_output_msg.obj_id, target_output_msg.obj_dx, target_output_msg.obj_lane, 
		   target_output_msg.obj_vx, target_output_msg.obj_dy, target_output_msg.obj_ax, 
		   target_output_msg.obj_path, target_output_msg.obj_vy, target_output_msg.obj_timestamp, 
		   target_output_msg.obj_track_num);
	 

		  target_obj_pub.publish(target_output_msg);
		   
		}
	//}

}

void EnvironmentState::publish_tracked_obj() {

		if (trackedObjects.size() >=1 ){

			for (int lane = 0; lane < trackedObjects.size(); lane++){

		   tracked_output_msg.obj_id = trackedObjects[lane].get_obj_id();
		   tracked_output_msg.obj_dx = trackedObjects[lane].get_obj_dx();
		   tracked_output_msg.obj_lane = trackedObjects[lane].get_obj_lane(); // 0 to 2
		   tracked_output_msg.obj_vx = trackedObjects[lane].get_obj_vx();
		   tracked_output_msg.obj_dy = trackedObjects[lane].get_obj_dy();
		   tracked_output_msg.obj_ax = trackedObjects[lane].get_obj_ax();
		   tracked_output_msg.obj_path = trackedObjects[lane].get_obj_path();
		   tracked_output_msg.obj_vy = trackedObjects[lane].get_obj_vy();
		   tracked_output_msg.obj_timestamp = trackedObjects[lane].get_obj_timestamp();
		   tracked_output_msg.obj_track_num = trackedObjects[lane].get_obj_lane() + 1; // 1 to 3


	//	 TESTING-------------------------------------------------------
		   printf("Tracked Output Msg\n");
		   printf("%d, %f, %d, %f, %f, %f, %d, %f, %f, %d \n",
		   tracked_output_msg.obj_id, tracked_output_msg.obj_dx, tracked_output_msg.obj_lane, 
		   tracked_output_msg.obj_vx, tracked_output_msg.obj_dy, tracked_output_msg.obj_ax, 
		   tracked_output_msg.obj_path, tracked_output_msg.obj_vy, tracked_output_msg.obj_timestamp, 
		   tracked_output_msg.obj_track_num);
	 

		  tracked_obj_pub.publish(tracked_output_msg);
		   
		}

	}

}

void EnvironmentState::filtered_object_callback(const sensor_fusion::filtered_object_msg& filtered_msg) {
    // TESTING-------------------------------------------------------
    // printf("Filtered Message:\n");
    // printf("%d, %f, %d, %f, %f, %f, %d, %f, %f \n",
    // filtered_msg.obj_id, filtered_msg.obj_dx, filtered_msg.obj_lane, 
    // filtered_msg.obj_vx, filtered_msg.obj_dy, filtered_msg.obj_ax, 
    // filtered_msg.obj_path, filtered_msg.obj_vy, filtered_msg.obj_timestamp);

    ObjectState tracked_msg;
    tracked_msg.copy_info(filtered_msg); // copy constructor
    
//		 TESTING-------------------------------------------------------
//     printf("Tracked Message:\n");
//     printf("%d, %f, %d, %f, %f, %f, %d, %f, %f \n",
//     tracked_msg.get_obj_id(), tracked_msg.get_obj_dx(), tracked_msg.get_obj_lane(), 
//     tracked_msg.get_obj_vx(), tracked_msg.get_obj_dy(), tracked_msg.get_obj_ax(), 
//     tracked_msg.get_obj_path(), tracked_msg.get_obj_vy(), tracked_msg.get_obj_timestamp());

    
    update_env_state(tracked_msg); // update id of objects in state vector

    check_timestamp(tracked_msg); // removes outdated state vector
  
    find_target_objects(tracked_msg);     // fill array with target objects
    
}


sensor_fusion::target_output_msg EnvironmentState::get_target_output_msg() { return target_output_msg; }

sensor_fusion::tracked_output_msg EnvironmentState::get_tracked_output_msg() { return tracked_output_msg; }

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


bool EnvironmentState::env_state_srv_callback(sensor_fusion::env_state_srv::Request &req, sensor_fusion::env_state_srv::Response &res){

        // breaking the objects stored in the vector into members and storing in multiple vectors for srv communication
        // index refers to specific objectNum in the original vector
        for(int i = 0; i < trackedObjects.size(); i++){
            res.id.push_back(trackedObjects[i].get_obj_id());
            res.dx.push_back(trackedObjects[i].get_obj_dx());
            res.lane.push_back(trackedObjects[i].get_obj_lane());
            res.vx.push_back(trackedObjects[i].get_obj_vx());
            res.dy.push_back(trackedObjects[i].get_obj_dy());
            res.ax.push_back(trackedObjects[i].get_obj_ax());
            res.path.push_back(trackedObjects[i].get_obj_path());
            res.vy.push_back(trackedObjects[i].get_obj_vy());
            res.timestamp.push_back(trackedObjects[i].get_obj_timestamp());
            
            //res.count.push_back(trackedObjects[i].get_obj_count());
        }
        return true;
}






