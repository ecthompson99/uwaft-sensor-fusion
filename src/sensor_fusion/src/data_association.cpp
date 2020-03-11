#include "data_association.h"
#include <algorithm>
#include <ctime>

DataAssociation::DataAssociation(ros::NodeHandle* node_handle) : node_handle(node_handle) {
    
    sensor_me_data_obj_sub = node_handle->subscribe(SENSOR_DATA_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_me_data_obj_callback, this);
    sensor_radar_data_obj_sub = node_handle->subscribe("sensor_data_topic", MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_radar_data_obj_callback, this);
    sensor_diag_sub = node_handle->subscribe(SENSOR_DIAG_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_diagnostics_callback, this);
    
    sensor_data_obj_pub = node_handle->advertise<sensor_fusion::fused_object_data_msg>(KALMAN_FILTER_TOPIC, 10);
}

bool olderThan(ObjectState obj) {
    int secondsToDelete = 5;

    // epoch seconds
    return (double)std::time(nullptr) - obj.timestamp > secondsToDelete;
}

void DataAssociation::delete_potential_objects() {
    for (int i = 0 ; i <= 10; i += 2) {
        // ObjectState obj = ObjectState(1, 2, 3, 4, (double)std::time(nullptr));

        // potential_objs.emplace_back(obj);
    }

    potential_objs.erase(std::remove_if(potential_objs.begin(), potential_objs.end(), olderThan), potential_objs.end());
}

sensor_fusion::fused_object_data_msg convert_object_state_to_fused_msg(ObjectState obj) {
    sensor_fusion::fused_object_data_msg x;
    return x;
}

void DataAssociation::publish_object_to_kf(ObjectState sensor_data) {
    // need to convert objectstate to fused object msg? gfdi
    sensor_fusion::fused_object_data_msg fused_data = convert_object_state_to_fused_msg(sensor_data);
    sensor_data_obj_pub.publish(fused_data);
}

bool is_mobile_eye_data(const sensor_fusion::radar_object_data& recvd_data) {
    if(recvd_data.radar_dx) {    //will this condition return true if it exists?
        return false;
    } else {
        return true;
    }
        
}

ObjectState convert_radar_data(const sensor_fusion::radar_object_data& recvd_data) {    
    return ObjectState(recvd_data.radar_dx, recvd_data.radar_vx, recvd_data.radar_dy, recvd_data.radar_vy, recvd_data.radar_timestamp);
}

ObjectState convert_mobile_eye(const sensor_fusion::mobileye_object_data& recvd_data) {
    //me_vy = someCalc;   //how will we do this?
    // maybe diff objectState constructor without a vy param?
    double me_vy;

    return ObjectState(recvd_data.me_dx, recvd_data.me_vx, recvd_data.me_dy, me_vy, recvd_data.me_timestamp);

}

bool DataAssociation::objects_match(ObjectState obj, ObjectState sensor_data) {  //both of type confirmedObjsContainer - post-conversion
    int dist = sqrt(pow((obj.dx - sensor_data.dx), 2) + (pow((obj.dy - sensor_data.dy), 2)));

    if(dist <= TOL){
        return true;
    } else {
        return false;
    } 
}


void DataAssociation::sensor_radar_data_obj_callback(const sensor_fusion::radar_object_data& recvd_data) {
    std::cout << "running checks for radar objects" << std::endl;

    ObjectState sensor_data = convert_radar_data(recvd_data);

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
            if (potential_objs[i].count > 5) {
                publish_object_to_kf(sensor_data);  //do we have to break once we publish?
                toRemove.emplace_back(i);
            } 
            matched_obj = true;           
        }
    }

    if(!matched_obj) {
        potential_objs.emplace_back(sensor_data);   //only add into potential tracks only after checking all potentials and seeing no match. 
    }

    for(int i =0; i<toRemove.size(); ++i) {
        potential_objs.erase(potential_objs.begin() + i);    //only at the end of all iterations do we erase the objects that were published or expired (timestamp)
    }

    //DO WE NEED TO EMPTY THE VECTOR
}

void DataAssociation::sensor_me_data_obj_callback(const sensor_fusion::mobileye_object_data& recvd_data) {
    std::cout << "running checks for me objects" << std::endl;

    ObjectState sensor_data = convert_mobile_eye(recvd_data);

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
            if (potential_objs[i].count > 5) {
                publish_object_to_kf(sensor_data);  //do we have to break once we publish?
                toRemove.emplace_back(i);
            } 
            matched_obj = true;           
        }
    }

    if(!matched_obj) {
        potential_objs.emplace_back(sensor_data);   //only add into potential tracks only after checking all potentials and seeing no match. 
    }

    for(int i =0; i<toRemove.size(); ++i) {
        potential_objs.erase(potential_objs.begin() + i);    //only at the end of all iterations do we erase the objects that were published or expired (timestamp)
    }

    //DO WE NEED TO EMPTY THE VECTOR
}

void DataAssociation::sensor_diagnostics_callback(const sensor_fusion::sensor_diagnostic_flag_msg& sensor_diag) {
    const uint8_t reliability_threshold = 10;

    for (int i = 0; i < 6; ++i) {
        if (sensor_diag.radar_reliability[i] > reliability_threshold) {
            std::cout << "reliability of " << unsigned(sensor_diag.radar_reliability[i]) << " is above realibility threshold of " << unsigned(reliability_threshold)<< std::endl;
        }
    }
}

