#include "data_association.h"
#include <algorithm>
#include <ctime>

double global_clk = 0;

DataAssociation::DataAssociation(ros::NodeHandle* in_node_handle) : node_handle(in_node_handle) {
  client = node_handle->serviceClient<sensor_fusion::env_state_srv>("env_service_topic");

  sensor_me_data_obj_sub =
      node_handle->subscribe(MOBILEYE_TOPIC, MESSAGE_BUFFER_SIZE, &DataAssociation::sensor_me_data_obj_callback, this);

  sensor_front_radar_data_obj_sub = node_handle->subscribe(FRONT_RADAR_TOPIC, MESSAGE_BUFFER_SIZE,
                                                           &DataAssociation::sensor_radar_data_obj_callback, this);

  sensor_left_corner_radar_sub = node_handle->subscribe(LEFT_CORNER_RADAR_TOPIC, MESSAGE_BUFFER_SIZE,
                                                        &DataAssociation::sensor_radar_data_obj_callback, this);

  sensor_right_corner_radar_sub = node_handle->subscribe(RIGHT_CORNER_RADAR_TOPIC, MESSAGE_BUFFER_SIZE,
                                                         &DataAssociation::sensor_radar_data_obj_callback, this);

  radar_to_kf_pub = node_handle->advertise<common::associated_radar_msg>(KALMAN_FILTER_RADAR_TOPIC, 10);
  me_to_kf_pub = node_handle->advertise<common::associated_me_msg>(KALMAN_FILTER_ME_TOPIC, 10);

  next_id = 0;
}

// Not called
bool oldObject(ObjectState obj) {

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
    if (abs(sensor_dx - obj.dx) < DX_TOL && abs(sensor_dy - obj.dy) < DY_TOL) {
        printf("Object matched with dx of %f and dy of %f\n", sensor_dx - obj.dx, sensor_dy - obj.dy);
        return 1;
    }
    return 0;
}

bool DataAssociation::radar_match(ObjectState obj, double sensor_dx, double sensor_dy) {
    if (abs(sensor_dx - obj.dx) < 5 && abs(sensor_dy - obj.dy) < DY_TOL) {
        printf("Object matched with dx of %f and dy of %f\n", sensor_dx - obj.dx, sensor_dy - obj.dy);
        return 1;
    }
    return 0;
}


void DataAssociation::sensor_radar_data_obj_callback(const common::radar_object_data& recvd_data) {
    if (recvd_data.RadarDx < 1 || recvd_data.RadarDx > DX_RANGE || recvd_data.RadarDy < -DY_RANGE || recvd_data.RadarDy > DY_RANGE)
        return;
    double adjusted_dy = recvd_data.RadarDy - 1;
    global_clk = recvd_data.RadarTimestamp;

    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    sensor_fusion::env_state_srv srv;
    std::vector<ObjectState> stateVector;

    if (client.call(srv)){
        std::cout<<"HEY service is called successfully\n";
        for (size_t i = 0; i < srv.response.id.size(); i++) {
          ObjectState someObj(srv.response.id[i], srv.response.dx[i], srv.response.dy[i], srv.response.timestamp[i]);

          stateVector.push_back(someObj); 
        } 

        for (auto obj : stateVector) {
            if (radar_match(obj, recvd_data.RadarDx, adjusted_dy)) {
                printf("%lu matched, sending now\n", obj.id);
                common::associated_radar_msg matched;
                matched.obj = recvd_data;
                matched.obj.RadarDy = adjusted_dy;
                matched.obj_id = obj.id;
                radar_to_kf_pub.publish(matched);
                return;
            }
        }     
        
    } else {
        ROS_ERROR("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?");
    }    

   // CHECK TEMP TRACKS
    for (auto obj_iterator = potential_objs.begin(); obj_iterator != potential_objs.end(); obj_iterator++) {
      if (radar_match(*obj_iterator, recvd_data.RadarDx, adjusted_dy)) {
        obj_iterator->dx = recvd_data.RadarDx;
        obj_iterator->dy = adjusted_dy;
        obj_iterator->count++;

        if (obj_iterator->count > POTENTIAL_THRESHOLD) {
          common::associated_radar_msg matched;
          matched.obj = recvd_data;
          matched.obj_id = next_id++;
          radar_to_kf_pub.publish(matched);
          potential_objs.erase(obj_iterator);
          std::cout << "Publishing new object that passed threshold and removing from potential queue" << std::endl;
        }

        return;
      }
    }

    // ONLY OCCURS IF NOT IN CONFIRMED TRACKS OR TEMP TRACKS

    std::cout << "added obj to potentials" << std::endl;
    potential_objs.emplace_back(ObjectState(recvd_data.RadarDx, adjusted_dy));
}

void DataAssociation::sensor_me_data_obj_callback(const common::mobileye_object_data& recvd_data) {
    if (recvd_data.MeDx > DX_RANGE || recvd_data.MeDy < -DY_RANGE || recvd_data.MeDy > DY_RANGE)
        return;
    double adjusted_dy = recvd_data.MeDy - 2;
    global_clk = recvd_data.MeTimestamp;

    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    sensor_fusion::env_state_srv srv;
    std::vector<ObjectState> envState;

    if (client.call(srv)){
        std::cout<<"HEY me\n";
        for (size_t i = 0; i < srv.response.id.size(); i++) {
          ObjectState someObj(srv.response.id[i], srv.response.dx[i], srv.response.dy[i], srv.response.timestamp[i]);

          envState.push_back(someObj); 
        }

        // if the object we received is already in the envState, send it to kf
        for (auto obj : envState) {
            if (objects_match(obj, recvd_data.MeDx, adjusted_dy)) {
                common::associated_me_msg matched;
                matched.obj = recvd_data;
                matched.obj.MeDy = adjusted_dy;
                matched.obj_id = obj.id;
                me_to_kf_pub.publish(matched);
                return;
            }
        }     
        
    } else {
        ROS_ERROR("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?");
    }

    // now check if it matches any of the potential objects

    for (auto obj_iterator = potential_objs.begin(); obj_iterator != potential_objs.end(); obj_iterator++) {
      if (objects_match(*obj_iterator, recvd_data.MeDx, adjusted_dy)) {
        obj_iterator->dx = recvd_data.MeDx;
        obj_iterator->dy = adjusted_dy;
        obj_iterator->count++;

        if (obj_iterator->count > POTENTIAL_THRESHOLD) {
          common::associated_me_msg matched;
          matched.obj = recvd_data;
          matched.obj_id = next_id++;
          me_to_kf_pub.publish(matched);
          potential_objs.erase(obj_iterator);
          std::cout << "DELETED OBJ CUZ COUNT > 5" << std::endl;
        }
        return;
      }
    }

    potential_objs.emplace_back(ObjectState(recvd_data.MeDx, adjusted_dy));
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
