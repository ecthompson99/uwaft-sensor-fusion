#include "data_association.h"
#include <algorithm>
#include <ctime>

double global_clk = 0;

DataAssociation::DataAssociation(ros::NodeHandle* node_handle) : node_handle(node_handle) {
    
    client = node_handle->serviceClient<sensor_fusion::env_state_srv>("env_service_topic");

    sensor_diag_sub = node_handle->subscribe(SENSOR_DIAG_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_diagnostics_callback, this);
    
    sensor_me_data_obj_sub = node_handle->subscribe(MOBILEYE_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_me_data_obj_callback, this);
    
    sensor_radar_data_obj_sub = node_handle->subscribe(RADAR_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_radar_data_obj_callback, this);
    
    radar_to_kf_pub = node_handle->advertise<sensor_fusion::associated_radar_msg>(KALMAN_FILTER_RADAR_TOPIC, 10);
    me_to_kf_pub = node_handle->advertise<sensor_fusion::associated_me_msg>(KALMAN_FILTER_ME_TOPIC, 10);

    next_id = 0;
}

// Not called
bool oldObject(ObjectState obj) {
    #define secondsToDelete 5

    // epoch seconds
    if (global_clk - obj.timestamp > secondsToDelete) { // works because we swap entire object including timestamp when match
        std::cout << "DELETED OBJ CUZ TOO OLD" << std::endl;
    }
    
    return global_clk - obj.timestamp > secondsToDelete;
}

// Not called
void DataAssociation::delete_potential_objects() {
    potential_objs.erase(std::remove_if(potential_objs.begin(), potential_objs.end(), oldObject), potential_objs.end());
}


bool DataAssociation::objects_match(ObjectState obj, double sensor_dx, double sensor_dy) {
    double dist = sqrt(pow((obj.dx - sensor_dx), 2) + (pow((obj.dy - sensor_dy), 2)));

    if (dist <= TOL) {
        std::cout << "objects match with a distance of " << dist << std::endl;
    }

    return dist <= TOL;
}


void DataAssociation::sensor_radar_data_obj_callback(const sensor_fusion::radar_object_data& recvd_data) {
    if (recvd_data.RadarDx < 7 && !recvd_data.RadarDy)
        return;
    global_clk = recvd_data.RadarTimestamp;

    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;
    // for (auto obj : potential_objs) {
    //     std::cout << "obj dx: " << obj.dx << " ";
    //     std::cout << "obj dy: " << obj.dy << " ";
    //     std::cout << "obj vx: " << obj.vx << " ";
    //     std::cout << "obj vy: " << obj.vy << " ";
    //     std::cout << "obj timestamp: " << (time_t)obj.timestamp << " ";
    //     std::cout << "obj count: " << obj.count;
    //     std::cout << std::endl;
    // }
    // std::cout << std::endl;

    // std::cout << "Radar data: ";
    // std::cout << "RadarDx: " << recvd_data.RadarDx << " ";
    // std::cout << "RadarDy: " << recvd_data.RadarDy << " ";
    // std::cout << "RadarVx: " << recvd_data.RadarVx << " ";
    // std::cout << "RadarVy: " << recvd_data.RadarVy << " ";
    // std::cout << "RadarTimestamp: " << (time_t)recvd_data.RadarTimestamp << " ";
    // std::cout << std::endl << std::endl;

    sensor_fusion::env_state_srv srv;
    std::vector<ObjectState> stateVector;

    if (client.call(srv)){
        std::cout<<"HEY service is called successfully\n";
        for(int i = 0; i < srv.response.id.size(); i++){
            
            ObjectState someObj(srv.response.id[i], srv.response.dx[i], srv.response.dy[i], srv.response.timestamp[i]);
        
            stateVector.push_back(someObj); 
        } 
        // std::cout << stateVector[0].id << std::endl;
        // std::cout << stateVector[2].dx << std::endl;

        for (auto obj : stateVector) {
            if (objects_match(obj, recvd_data.RadarDx, recvd_data.RadarDy)) {
                printf("%lu matched, sending now\n", obj.id);
                sensor_fusion::associated_radar_msg matched;
                matched.obj = recvd_data;
                matched.obj_id = obj.id;
                radar_to_kf_pub.publish(matched);
                return;
            }
        }     
        
    } else {
        ROS_ERROR("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?");
    }    

   // CHECK TEMP TRACKS
    for (int i = 0; i < potential_objs.size(); i++) {  
        if (objects_match(potential_objs[i], recvd_data.RadarDx, recvd_data.RadarDy)) {

            potential_objs[i].dx = recvd_data.RadarDx;
            potential_objs[i].dy = recvd_data.RadarDy;
            potential_objs[i].count++;

            if (potential_objs[i].count > POTENTIAL_THRESHOLD) {
                sensor_fusion::associated_radar_msg matched;
                matched.obj = recvd_data;
                matched.obj_id = next_id++;
                radar_to_kf_pub.publish(matched);
                potential_objs.erase (potential_objs.begin()+i);
                std::cout << "Publishing new object that passed threshold and removing from potential queue" << std::endl;
            }

            return;
        }
    }

    // ONLY OCCURS IF NOT IN CONFIRMED TRACKS OR TEMP TRACKS

    std::cout << "added obj to potentials" << std::endl;
    potential_objs.emplace_back(ObjectState(recvd_data.RadarDx, recvd_data.RadarDy));
}

void DataAssociation::sensor_me_data_obj_callback(const sensor_fusion::mobileye_object_data& recvd_data) {
    // std::cout << "Mobileye data: ";
    // std::cout << "MeDx: " << recvd_data.MeDx << " ";
    // std::cout << "MeDy: " << recvd_data.MeDy << " ";
    // std::cout << "MeVx: " << recvd_data.MeVx << " ";
    // std::cout << "MeTimestamp: " << (time_t)recvd_data.MeTimestamp << " ";
    // std::cout << std::endl << std::endl;
    if (!recvd_data.MeDx && !recvd_data.MeDy)
        return;
    global_clk = recvd_data.MeTimestamp;

    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    sensor_fusion::env_state_srv srv;
    std::vector<ObjectState> envState;

    if (client.call(srv)){
        std::cout<<"HEY me\n";
        for(int i = 0; i < srv.response.id.size(); i++){
            
            ObjectState someObj(srv.response.id[i], srv.response.dx[i], srv.response.dy[i], srv.response.timestamp[i]);
        
            envState.push_back(someObj); 
        }

        // if the object we received is already in the envState, send it to kf
        for (auto obj : envState) {
            if (objects_match(obj, recvd_data.MeDx, recvd_data.MeDy)) {
                sensor_fusion::associated_me_msg matched;
                matched.obj = recvd_data;
                matched.obj_id = obj.id;
                me_to_kf_pub.publish(matched);
                return;
            }
        }     
        
    } else {
        ROS_ERROR("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?");
    }

    // now check if it matches any of the potential objects

    for (int i = 0; i < potential_objs.size(); i++) {  
        if (objects_match(potential_objs[i], recvd_data.MeDx, recvd_data.MeDy)) {

            potential_objs[i].dx = recvd_data.MeDx;
            potential_objs[i].dy = recvd_data.MeDy;
            potential_objs[i].count++;

            if (potential_objs[i].count > POTENTIAL_THRESHOLD) {
                sensor_fusion::associated_me_msg matched;
                matched.obj = recvd_data;
                matched.obj_id = next_id++;
                me_to_kf_pub.publish(matched);
                potential_objs.erase (potential_objs.begin()+i);
                std::cout << "DELETED OBJ CUZ COUNT > 5" << std::endl;
            }
            return;
        }
    }

    potential_objs.emplace_back(ObjectState(recvd_data.MeDx, recvd_data.MeDy));
}

void DataAssociation::sensor_diagnostics_callback(const sensor_fusion::sensor_diagnostic_flag_msg& sensor_diag) {
    const uint8_t reliability_threshold = 10;

    for (int i = 0; i < 6; ++i) {
        if (sensor_diag.radar_reliability[i] > reliability_threshold) {
            std::cout << "reliability of " << unsigned(sensor_diag.radar_reliability[i]) << " is above realibility threshold of " << unsigned(reliability_threshold)<< std::endl;
        }
    }
}

int main(int argc, char** argv){
    ros::init(argc, argv, "data_association");
    ros::NodeHandle data_association_handle;
    DataAssociation data_assc = DataAssociation(&data_association_handle);
    while (ros::ok()) {
        data_assc.delete_potential_objects();
        ros::Rate(10).sleep();
        ros::spinOnce();
    }
    // ros::spin();
}
