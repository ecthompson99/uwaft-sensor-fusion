#include "data_association.h"

DataAssociation::DataAssociation(ros::NodeHandle* node_handle) : node_handle(node_handle) {
    
      sensor_data_obj_sub = node_handle->subscribe(SENSOR_DATA_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_data_obj_callback, this);
    std::cout << "data association class waiting and listening to " << SENSOR_DATA_TOPIC << " topic" << std::endl;
    //   sensor_data_obj_pub = sensor_fusion_node_handle->advertise<sensor_fusion::fused_object_data_msg>(KALMAN_FILTER_TOPIC, MESSAGE_BUFFER_SIZE);
}

void DataAssociation::delete_potential_objects() {
    // std::cout << "deleting obj" << std::endl;
    for (auto obj : potential_objs) {
        // if (time.current - obj.timeAdded > 5) {
        if (true) {
            // remove obj from potential_objs
        }
    }
}

void publish_object_to_kf(int sensor_data) {

}

bool objects_match(int obj, int sensor_data) {
    return false;
}

bool is_mobile_eye_data(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {
    return false;
}

int convert_mobile_eye(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {

}

int convert_radar_data(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {
    
}

void DataAssociation::sensor_data_obj_callback(const sensor_fusion::raw_sensor_object_data_msg& recvd_data) {
    std::cout << "running checks for objects" << std::endl;

    int sensor_data;    //would be object

    if(is_mobile_eye_data(recvd_data)){
        sensor_data = convert_mobile_eye(recvd_data);
    } else {
        sensor_data = convert_radar_data(recvd_data);
    }

    //get environment state with service
    
    std::vector<int> envState;
    bool matched_obj = false;

    for (auto obj : envState) {
        if (objects_match(obj, sensor_data)) {
            publish_object_to_kf(sensor_data);  //match ID 
            matched_obj = true;
            potential_objs.emplace_back(sensor_data);
            break;
        }

    }

    for (auto obj : potential_objs) {
        if (objects_match(obj, sensor_data)) {
            // obj.count += 1;
            // if (obj.count > 5) {
            //     publish_object_to_kf(sensor_data);
            // }
            matched_obj = true;
            break;
        }

    }

}
