#include "data_association.h"

DataAssociation::DataAssociation(ros::NodeHandle* node_handle) : node_handle(node_handle) {
    
    //sensor_data_obj_sub = node_handle->subscribe(SENSOR_DATA_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_data_obj_callback, this);
    std::cout << "data association class waiting and listening to " << SENSOR_DATA_TOPIC << " topic" << std::endl;
    sensor_data_obj_pub = sensor_fusion_node_handle->advertise<ObjectState>(KALMAN_FILTER_TOPIC, MESSAGE_BUFFER_SIZE);
}

void DataAssociation::delete_potential_objects() {
    std::cout << "deleting obj" << std::endl;
    for (auto obj : potential_objs) {
        // if (time.current - obj.timeAdded > 5) {
        if (true) {
            // remove obj from potential_objs
        }
    }
}

void publish_object_to_kf(ObjectState sensor_data) {
    sensor_data_obj_pub.publish(sensor_data);
}

bool is_mobile_eye_data(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {
    if(recvd_data.radar_dx)     //will this condition return true if it exists?
        return false;
    else
        return true;
}

ObjectState convert_radar_data(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {    
    return ObjectState(recvd_data.radar_dx, recvd_data.radar_vx, recvd_data.radar_dy, recvd_data.radar_vy, recvd_dat.radar_timestamp);
}

ObjectState convert_mobile_eye(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {
    //me_vy = someCalc;   //how will we do this?
    // maybe diff objectState constructor without a vy param?
    double me_vy;

    return ObjectState(recvd_data.me_dx, recvd_data.me_vx, recvd_data.me_dy, me_vy, recvd_dat.me_timestamp);

}

bool objects_match(ObjectState obj, ObjectState sensor_data) {  //both of type confirmedObjsContainer - post-conversion
    int dist = sqrt((pow(obj.x - sensor_data.x), 2) + (pow(obj.y - confirmedObject.y), 2));

    if(dist<=TOL){
        return true;
    } else {
        return false;
    } 
}


void DataAssociation::sensor_data_obj_callback(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {
    std::cout << "running checks for objects" << std::endl;

    ObjectState sensor_data;    //would be object of type confirmedObjsContainer

    vector<int> toRemove;   //builds up a vector of indicies to be removed because removing random indicies while iterating  a second time will skip
    
    if(is_mobile_eye_data(recvd_data)){
        sensor_data = convert_mobile_eye(recvd_data);
    } else {
        sensor_data = convert_radar_data(recvd_data);
    }

    //get environment state with service
    
    std::vector<ObjectState> envState;
    bool matched_obj = false;

    for (auto obj : envState) {
        if (objects_match(obj, sensor_data)) {      //if it's' placed in the confirmed container don't need to push_back to the temp vector
            publish_object_to_kf(sensor_data);  //match ID so the KF can compare this sensor_data to its' prediction
            matched_obj = true;
            break;
        }
    } 

    
    for (int i = 0; i < potential_objs.size(); i++) {  
        
        if (!matched_obj && objects_match(potential_objs[i], sensor_data)) {    //won't consider this possibility if already matched for that sensor_data
            potential_objs[i].count += 1;
            potential_objs[i].timestamp = std::chrono::system_clock::now();
            if (potential_objs[i].count > 5) {
                publish_object_to_kf(sensor_data);  //do we have to break once we publish?
                toRemove.emplace_back(i);
            } 
            matched_obj = true;           
        }
    
        if(std::chrono::system_clock::now() - potential_objs[i].timestamp > 5){ //need to check each iteration for if objects have been sitting without update
                toRemove.emplace_back(i);                                       //would we have to swap the objects(for timestamp) instead of just iterating count?
        }
    }

    if(!matched_obj)
        potential_objs.emplace_back(sensor_data);   //only add into potential tracks only after checking all potentials and seeing no match. 

    for(auto index : toRemove)
        potential_objs.erase(index);    //only at the end of all iterations do we erase the objects that were published or expired (timestamp)
    
}

