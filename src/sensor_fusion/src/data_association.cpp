#include "data_association.h"
#include <algorithm>
#include <ctime>

DataAssociation::DataAssociation(ros::NodeHandle* node_handle) : node_handle(node_handle) {
    
    client = node_handle->serviceClient<sensor_fusion::env_state_srv>("srv_env_state_topic");
    
    sensor_diag_sub = node_handle->subscribe(SENSOR_DIAG_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_diagnostics_callback, this);
    
    sensor_me_data_obj_sub = node_handle->subscribe(MOBILEYE_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_me_data_obj_callback, this);
    mock_me_pub = node_handle->advertise<sensor_fusion::mobileye_object_data>(MOBILEYE_TOPIC, 10);
    
    sensor_radar_data_obj_sub = node_handle->subscribe(RADAR_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_radar_data_obj_callback, this);
    mock_radar_pub = node_handle->advertise<sensor_fusion::radar_object_data>(RADAR_TOPIC, 10);    
    
    sensor_data_obj_pub = node_handle->advertise<sensor_fusion::fused_object_data_msg>(KALMAN_FILTER_TOPIC, 10);

    potential_objs.emplace_back(ObjectState(1000000, 696969, 420420, 1, std::time(nullptr)));
}

bool oldObject(ObjectState obj) {
    const int secondsToDelete = 5;

    // epoch seconds
    if ((double)std::time(nullptr) - obj.timestamp > secondsToDelete) { // works because we swap entire object including timestamp when match
        std::cout << "DELETED OBJ CUZ TOO OLD" << std::endl;
    }
    
    return (double)std::time(nullptr) - obj.timestamp > secondsToDelete;
}

void DataAssociation::delete_potential_objects() {
    potential_objs.erase(std::remove_if(potential_objs.begin(), potential_objs.end(), oldObject), potential_objs.end());
}

sensor_fusion::fused_object_data_msg convert_object_state_to_fused_msg(ObjectState obj) {
    sensor_fusion::fused_object_data_msg msg;
    // msg.x = obj.dx;
    return msg;
}

void DataAssociation::publish_object_to_kf(ObjectState sensor_data) {
    // need to convert objectstate to fused object msg? gfdi
    sensor_fusion::fused_object_data_msg fused_data = convert_object_state_to_fused_msg(sensor_data);
    sensor_data_obj_pub.publish(fused_data);
}

ObjectState convert_radar_data(const sensor_fusion::radar_object_data& recvd_data) {    
    return ObjectState(recvd_data.radar_dx, recvd_data.radar_vx, recvd_data.radar_dy, recvd_data.radar_vy, recvd_data.radar_timestamp);
}

ObjectState convert_mobile_eye(const sensor_fusion::mobileye_object_data& recvd_data) {
    double me_vy = 0.0;
    return ObjectState(recvd_data.me_dx, recvd_data.me_vx, recvd_data.me_dy, me_vy, recvd_data.me_timestamp);

}

bool DataAssociation::objects_match(ObjectState obj, ObjectState sensor_data) {  //both of type confirmedObjsContainer - post-conversion
    int dist = sqrt(pow((obj.dx - sensor_data.dx), 2) + (pow((obj.dy - sensor_data.dy), 2)));

    if (dist <= TOL) {
        std::cout << "objects match with a distance of " << dist << std::endl;
    }

    return dist <= TOL;
}


void DataAssociation::sensor_radar_data_obj_callback(const sensor_fusion::radar_object_data& recvd_data) {
    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;
    for (auto obj : potential_objs) {
        std::cout << "obj dx: " << obj.dx << " ";
        std::cout << "obj dy: " << obj.dy << " ";
        std::cout << "obj vx: " << obj.vx << " ";
        std::cout << "obj vy: " << obj.vy << " ";
        std::cout << "obj timestamp: " << (time_t)obj.timestamp << " ";
        std::cout << "obj count: " << obj.count;
        std::cout << std::endl;
    }
    std::cout << std::endl;

    std::cout << "Radar data: ";
    std::cout << "radar_dx: " << recvd_data.radar_dx << " ";
    std::cout << "radar_dy: " << recvd_data.radar_dy << " ";
    std::cout << "radar_vx: " << recvd_data.radar_vx << " ";
    std::cout << "radar_vy: " << recvd_data.radar_vy << " ";
    std::cout << "radar_timestamp: " << (time_t)recvd_data.radar_timestamp << " ";
    std::cout << std::endl << std::endl;


    ObjectState sensor_data = convert_radar_data(recvd_data);

    //get environment state with service
    
    sensor_fusion::env_state_srv service_call;    //service object
    
    if (client.call(service_call)){     //returns true if the service call went through, false is error
        
        std::vector<int> envState = service_call.response.env_state_vec_from_container;

        // std::vector<int> envState = service_call.response.env_state_vec_from_container;

        // if the object we received is already in the envState, send it to kf
        for (auto obj : envState) {
            if (objects_match(obj, sensor_data)) {      //if it's' placed in the confirmed container don't need to push_back to the temp vector
                publish_object_to_kf(sensor_data);  //match ID so the KF can compare this sensor_data to its' prediction
                return;
            }
        } 
    }

   
    for (int i = 0; i < potential_objs.size(); i++) {  
        if (objects_match(potential_objs[i], sensor_data)) {    //won't consider this possibility if already matched for that sensor_data

            // potential_objs[i].count += 1;
            int oldCount = potential_objs[i].count;

            potential_objs[i] = sensor_data;
            potential_objs[i].count = oldCount + 1;

            if (potential_objs[i].count > 5) {
                publish_object_to_kf(sensor_data);  //break once we publish
                potential_objs.erase (potential_objs.begin()+i);
                std::cout << "DELETED OBJ CUZ COUNT > 5" << std::endl;
            }

            return;
        }
    }

    std::cout << "added obj to potentials" << std::endl;
    potential_objs.emplace_back(sensor_data);
}

void DataAssociation::sensor_me_data_obj_callback(const sensor_fusion::mobileye_object_data& recvd_data) {
    std::cout << "Mobileye data: ";
    std::cout << "me_dx: " << recvd_data.me_dx << " ";
    std::cout << "me_dy: " << recvd_data.me_dy << " ";
    std::cout << "me_vx: " << recvd_data.me_vx << " ";
    std::cout << "me_timestamp: " << (time_t)recvd_data.me_timestamp << " ";
    std::cout << std::endl << std::endl;
    // return;
    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    ObjectState sensor_data = convert_mobile_eye(recvd_data);

    //get environment state with service
    
    std::vector<ObjectState> envState;

    // if the object we received is already in the envState, send it to kf
    for (auto obj : envState) {
        if (objects_match(obj, sensor_data)) {      //if it's' placed in the confirmed container don't need to push_back to the temp vector
            publish_object_to_kf(sensor_data);  //match ID so the KF can compare this sensor_data to its' prediction
            return;
        }
    } 

    // now check if it matches any of the potential objects

    for (int i = 0; i < potential_objs.size(); i++) {  
        if (objects_match(potential_objs[i], sensor_data)) {    //won't consider this possibility if already matched for that sensor_data
            int oldCount = potential_objs[i].count;
            int oldVy = potential_objs[i].vy;

            potential_objs[i] = sensor_data;
            
            potential_objs[i].vy = oldVy;
            potential_objs[i].count = oldCount + 1;

            if (potential_objs[i].count > 5) {
                publish_object_to_kf(sensor_data);  //do we have to break once we publish?
                potential_objs.erase (potential_objs.begin()+i);
                return;
            } 
        }
    }

    potential_objs.emplace_back(sensor_data);
}

void DataAssociation::sensor_diagnostics_callback(const sensor_fusion::sensor_diagnostic_flag_msg& sensor_diag) {
    const uint8_t reliability_threshold = 10;

    for (int i = 0; i < 6; ++i) {
        if (sensor_diag.radar_reliability[i] > reliability_threshold) {
            std::cout << "reliability of " << unsigned(sensor_diag.radar_reliability[i]) << " is above realibility threshold of " << unsigned(reliability_threshold)<< std::endl;
        }
    }
}

