#include "data_association.h"
#include <algorithm>
#include <ctime>
#include <vector>


DataAssociation::DataAssociation(ros::NodeHandle* in_node_handle) : node_handle(in_node_handle) {
    client = node_handle->serviceClient<sensor_fusion::env_state_srv>(ENV_SERVICE);

    srv_ch2 = node_handle->advertiseService(CH2_SERVICE, &DataAssociation::sensor_diagnostic_callback_CH2, this);     
    srv_ch3 = node_handle->advertiseService(CH3_SERVICE, &DataAssociation::sensor_diagnostic_callback_CH3, this);     
    srv_ch4 = node_handle->advertiseService(CH4_SERVICE, &DataAssociation::sensor_diagnostic_callback_CH4, this);     

    sensor_me_data_obj_sub = node_handle->subscribe(MOBILEYE_TOPIC, MESSAGE_BUFFER_SIZE, 
                                                &DataAssociation::sensor_me_data_obj_callback, this);

    sensor_front_radar_data_obj_sub = node_handle->subscribe(RADAR_ONE_TOPIC, MESSAGE_BUFFER_SIZE,
                                                            &DataAssociation::sensor_radar_data_obj_callback, this);

    sensor_left_corner_radar_sub = node_handle->subscribe(RADAR_TWO_TOPIC, MESSAGE_BUFFER_SIZE,
                                                         &DataAssociation::sensor_radar_data_obj_callback, this);

    sensor_right_corner_radar_sub = node_handle->subscribe(RADAR_THREE_TOPIC, MESSAGE_BUFFER_SIZE,
                                                             &DataAssociation::sensor_radar_data_obj_callback, this);

    radar_to_kf_pub = node_handle->advertise<common::associated_radar_msg>(KALMAN_FILTER_RADAR_TOPIC, 10);
    me_to_kf_pub = node_handle->advertise<common::associated_me_msg>(KALMAN_FILTER_ME_TOPIC, 10);

    next_id = 0;
    global_clk = 0;

    filtered_me_obj = std::vector<MobileyeObject>(10);

    // diagnostics (assume true, service call if false)
    FRONT_RADAR = 1;
    RIGHT_CORNER_RADAR = 1;
    LEFT_CORNER_RADAR = 1;
    MOBILEYE = 1;
}

std::vector<RadarObject> DataAssociation::filter_radar(const common::radar_object_data& recvd_data){

    std::vector<RadarObject> filtered_radar_obj;

    // need an algorithm here that will filter out bad sensor readings
    // reduce object count from 10 objects

    size_t filtered_index = 0;
    for (size_t r_index = 0; r_index < RADAR_OBJ; r_index++){
        // dx and dy limits
        if (recvd_data.radar_dx[r_index] < MIN_DX || recvd_data.radar_dx[r_index] > MAX_DX
            || abs(recvd_data.radar_dy[r_index]) > DY_LIMIT) continue;

    // COMMENT OUT FOR SIMULATION
        // Stationary objects
        // if (((recvd_data.veh_v_ego + abs(recvd_data.radar_vx[r_index])) < VX_LIMIT) 
        //         || recvd_data.moving_state[r_index] == 3) continue;

        // Exist probability flag - needs more testing to confirm threshold
        if (recvd_data.radar_w_exist[r_index] < EXIST) continue;

        // Valid flag - 1 is valid
        if (recvd_data.radar_flag_valid[r_index] == 0) continue;

        // Measured and history flag - want history object only if measured
        if (recvd_data.flag_hist[r_index] == 1 && recvd_data.flag_meas[r_index] == 0) continue;

        // dLength - most likely an obj if it has length
        if (recvd_data.d_length[r_index] == 0) continue;
    

        // printf("Success filtering radar data: %f, %f, %f, %f", recvd_data.radar_dx[r_index], recvd_data.radar_dy[r_index], recvd_data.radar_vx[r_index], recvd_data.radar_vy[r_index]);

        RadarObject filtered_temp;

        filtered_temp.radar_dx = recvd_data.radar_dx[r_index];
        filtered_temp.radar_dy = recvd_data.radar_dy[r_index];
        filtered_temp.radar_vx = recvd_data.radar_vx[r_index];
        filtered_temp.radar_ax = recvd_data.radar_ax[r_index];
        filtered_temp.radar_dx_sigma = recvd_data.radar_dx_sigma[r_index];
        filtered_temp.radar_dy_sigma = recvd_data.radar_dy_sigma[r_index];
        filtered_temp.radar_vx_sigma = recvd_data.radar_vx_sigma[r_index];
        filtered_temp.radar_ax_sigma = recvd_data.radar_ax_sigma[r_index];
        filtered_temp.radar_w_exist = recvd_data.radar_w_exist[r_index];
        filtered_temp.radar_w_obstacle = recvd_data.radar_w_obstacle[r_index];
        filtered_temp.radar_flag_valid = recvd_data.radar_flag_valid[r_index]; // MUST HAVE
        filtered_temp.radar_w_non_obstacle = recvd_data.radar_w_non_obstacle[r_index];
        filtered_temp.flag_meas = recvd_data.flag_meas[r_index]; // MUST 
        filtered_temp.flag_hist = recvd_data.flag_hist[r_index];
        filtered_temp.d_length = recvd_data.d_length[r_index];
        filtered_temp.radar_dz = recvd_data.radar_dz[r_index];
        filtered_temp.moving_state = recvd_data.moving_state[r_index];
        filtered_temp.radar_w_class = recvd_data.radar_w_class[r_index];
        filtered_temp.radar_obj_class = recvd_data.radar_obj_class[r_index];
        filtered_temp.dx_rear_loss = recvd_data.dx_rear_loss[r_index];
        filtered_temp.radar_num = recvd_data.radar_num;
        filtered_temp.radar_timestamp = recvd_data.radar_timestamp;
        filtered_temp.radar_vy = recvd_data.radar_vy[r_index];

        filtered_radar_obj.push_back(filtered_temp);
    }
    return filtered_radar_obj;
}

std::vector<MobileyeObject> DataAssociation::filter_me(const common::mobileye_object_data& recvd_data){

    std::vector<MobileyeObject> filtered_me_obj;

    // need an algorithm here that will filter out bad sensor readings (for now just copy them over)
    // this will likely reduce object count from 32 objects every new message
    size_t filtered_index = 0;
    for (size_t me_index = 0; me_index < ME_OBJ; me_index++){
        // dx and dy threshold
        if (recvd_data.me_dx[me_index] < MIN_DX || recvd_data.me_dx[me_index] > MAX_DX
            || abs(recvd_data.me_dy[me_index] > DY_LIMIT) ) continue;

    //  COMMENT OUT FOR SIMULATION
        // Stationary objects - status: never moved
        if (recvd_data.me_status[me_index] == 1 || recvd_data.me_status[me_index] == 5) continue;
        
        // Valid objects
        if (recvd_data.me_valid[me_index] == 0) continue;
    

        MobileyeObject filtered_me_temp;

        filtered_me_temp.me_dx = recvd_data.me_dx[me_index];
        filtered_me_temp.me_dy = recvd_data.me_dy[me_index];
        filtered_me_temp.me_vx = recvd_data.me_vx[me_index];
        filtered_me_temp.me_ax = recvd_data.me_ax[me_index];
        filtered_me_temp.me_type = recvd_data.me_type[me_index];
        filtered_me_temp.me_status = recvd_data.me_status[me_index];
        filtered_me_temp.me_valid = recvd_data.me_valid[me_index];
        filtered_me_temp.me_cut_in_cut_out = recvd_data.me_cut_in_cut_out[me_index];
        filtered_me_temp.me_age = recvd_data.me_age[me_index];
        filtered_me_temp.me_lane = recvd_data.me_lane[me_index];
        filtered_me_temp.me_cipv_flag = recvd_data.me_cipv_flag[me_index];
        filtered_me_temp.me_timestamp = recvd_data.me_timestamp;
    
        filtered_me_obj.push_back(filtered_me_temp);
    }
    return filtered_me_obj;
}

void DataAssociation::pub_radar_signals(common::associated_radar_msg &associated_radar_msg, RadarObject &r_obj){
    associated_radar_msg.radar_dx = r_obj.radar_dx;
    associated_radar_msg.radar_dy = r_obj.radar_dy;
    associated_radar_msg.radar_vx = r_obj.radar_vx;
    associated_radar_msg.radar_vy = r_obj.radar_vy;
    associated_radar_msg.radar_ax = r_obj.radar_ax;
    associated_radar_msg.radar_dx_sigma = r_obj.radar_dx_sigma;
    associated_radar_msg.radar_dy_sigma = r_obj.radar_dy_sigma;
    associated_radar_msg.radar_vx_sigma = r_obj.radar_vx_sigma;
    associated_radar_msg.radar_ax_sigma = r_obj.radar_ax_sigma;
    associated_radar_msg.radar_timestamp = r_obj.radar_timestamp;
}

void DataAssociation::pub_me_signals(common::associated_me_msg &associated_me_msg, MobileyeObject &me_obj){
    associated_me_msg.me_dx = me_obj.me_dx;
    associated_me_msg.me_dy = me_obj.me_dy;
    associated_me_msg.me_vx = me_obj.me_vx;
    associated_me_msg.me_lane = me_obj.me_lane;
    associated_me_msg.me_timestamp = me_obj.me_timestamp;
}

// Used to delete potential objects (in temporary array) if havn't seen for a long time
void DataAssociation::delete_potential_objects() {

    for (unsigned i = 0; i < potential_objs.size(); i++){
        if(global_clk - potential_objs[i].timestamp > secondsToDelete){
            // std::cout << "Potential object deleted because too old in temp array" << std::endl;
            potential_objs.erase(potential_objs.begin()+i);
        }
    }
}


bool DataAssociation::objects_match_radar(ObjectState obj, RadarObject& filtered) {
    // filter dx, dy, vx
    if ((abs(filtered.radar_dx - obj.dx) < DX_TOL) && (abs(filtered.radar_dy - obj.dy) < DY_TOL) && (abs(filtered.radar_vx - obj.vx) < VX_TOL)) {
        printf("Object matched with dx of %.2f, dy of %.2f, and vx of %.2f\n", 
            filtered.radar_dx - obj.dx, filtered.radar_dy - obj.dy, filtered.radar_vx - obj.vx);
        return 1;
    }
    return 0;
}

bool DataAssociation::objects_match_me(ObjectState obj, MobileyeObject& filtered) {
    // filter dx, dy, vx
    if ((abs(filtered.me_dx - obj.dx) < DX_TOL) && (abs(filtered.me_dy - obj.dy) < DY_TOL) && (abs(filtered.me_vx - obj.vx) < VX_TOL)) {
        printf("Object matched with dx of %.2f, dy of %.2f, and vx of %.2f\n", 
            filtered.me_dx - obj.dx, filtered.me_dy - obj.dy, filtered.me_vx - obj.vx);
        return 1;
    }
    return 0;
}

void DataAssociation::sensor_radar_data_obj_callback(const common::radar_object_data& recvd_data) {

    global_clk = recvd_data.radar_timestamp;
    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    std::vector<RadarObject> filtered_radar_obj;

    // filter detections
    if(FRONT_RADAR && LEFT_CORNER_RADAR && RIGHT_CORNER_RADAR){
        filtered_radar_obj = filter_radar(recvd_data);
        std::cout << "Filtered radar object size " << filtered_radar_obj.size() << std::endl;
        // std::cout << "Valid Radar diagnostics service call" << std::endl;

         // Initialize service to sensor fusion
        sensor_fusion::env_state_srv srv;
        std::vector<ObjectState> stateVector;

        // Make a service call every time a new message comes in
        if (client.call(srv)){
            // std::cout<<"Radar service called successfully\n";
            for (size_t srv_index = 0; srv_index < srv.response.id.size(); srv_index++) {
                ObjectState someObj(srv.response.id[srv_index], srv.response.dx[srv_index], srv.response.dy[srv_index], srv.response.timestamp[srv_index]);
                stateVector.push_back(someObj); 
            }
        }
        else {
            ROS_ERROR("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?");
        }
        // std::cout << "Size of state vector: " << stateVector.size() << std::endl;

        // Loop through each object in the filtered list
        for (size_t r_index = 0; r_index < filtered_radar_obj.size(); r_index++){
            bool matched = 0;

            // Create radar object
            RadarObject radar_obj = filtered_radar_obj[r_index];
            printf("Radar object index: %d: %f, %f, %f, %f\n", r_index, radar_obj.radar_dx, radar_obj.radar_dy, radar_obj.radar_vx, radar_obj.radar_vy);

            // check if detections match objects in environment state vector
            for (auto obj : stateVector) {
                if (objects_match_radar(obj, radar_obj)){
                    printf("%lu matched, sending now\n", obj.id);
                    associated_radar_msg.obj_id = obj.id;
                    pub_radar_signals(associated_radar_msg, radar_obj);
                    radar_to_kf_pub.publish(associated_radar_msg);
                    matched = 1;
                    break;
                }
            }         

            if (!matched) {
                // Check if detections match objects in temporary array
                for (auto obj_iterator = potential_objs.begin(); obj_iterator != potential_objs.end(); obj_iterator++) {
                    
                    if (objects_match_radar(*obj_iterator, radar_obj)){
                        // if match criterion, increment iterator
                        obj_iterator->dx = radar_obj.radar_dx;
                        obj_iterator->dy = radar_obj.radar_dy;
                        obj_iterator->count++;
                        matched = 1;
                        printf("radar temp match count: %d \n", obj_iterator->count);
                        // Once iterator > threshold, publish
                        if (obj_iterator->count > POTENTIAL_THRESHOLD) {
                            associated_radar_msg.obj_id = next_id++;
                            pub_radar_signals(associated_radar_msg, radar_obj);
                            radar_to_kf_pub.publish(associated_radar_msg);
                            potential_objs.erase(obj_iterator); // erase this object from temporary list since it has been confirmed
                            std::cout << "Publishing new object that passed threshold and removing from potential queue" << std::endl;
                        }
                        break;
                    }
                }
            }

            // Not match env state vector or temporary array 
            if (!matched) {                 
                std::cout << "added radar object to potentials" << std::endl;
                potential_objs.emplace_back(ObjectState(radar_obj.radar_dx, radar_obj.radar_dy));
            }      
        }
    }
    else{
        std::cout<< "Invalid radar service call" << std::endl;
    }
}

void DataAssociation::sensor_me_data_obj_callback(const common::mobileye_object_data& recvd_data) {

    global_clk = recvd_data.me_timestamp;
    std::cout << "Potential objs size: " << potential_objs.size() << std::endl;

    std::vector<MobileyeObject> filtered_me_obj;

    // filter detections
    if(MOBILEYE){
        std::cout << "Mobileye message valid!" << std::endl;
        filtered_me_obj = filter_me(recvd_data);
        // Initalize service
        sensor_fusion::env_state_srv srv;
        std::vector<ObjectState> envState;

        // Call service everytime new me callback message received
        if (client.call(srv)){
            // std::cout<<"Mobileye service called successfully\n";
            for (size_t srv_index = 0; srv_index < srv.response.id.size(); srv_index ++) {
                ObjectState someObj(srv.response.id[srv_index], srv.response.dx[srv_index], srv.response.dy[srv_index], srv.response.timestamp[srv_index]);
                envState.push_back(someObj); 
            }
        } 
        else {
            ROS_ERROR("Failed to call service, but continuing to the already stored potential objects, maybe it matches up there!?");
        }

        std::cout << "Size of env state vector mobileye: " << envState.size() << std::endl;

        // Loop through each object in the filtered list
        for (size_t me_index = 0; me_index < filtered_me_obj.size(); me_index++){
            bool me_matched = 0;

            // create mobileye object
            MobileyeObject me_obj = filtered_me_obj[me_index];
            printf("ME object index: %d: %f, %f\n", me_index, me_obj.me_dx, me_obj.me_dy);
            // if the object we received is already in the envState, send it to kf
            for (auto obj : envState) {
                if (objects_match_me(obj, me_obj)) {
                    printf("%lu matched, sending now\n", obj.id);
                    associated_me_msg.obj_id = obj.id;
                    pub_me_signals(associated_me_msg, me_obj);
                    me_to_kf_pub.publish(associated_me_msg);
                    me_matched = 1;
                    break;
                }
            }     

            // now check if it matches any of the potential objects
            if (!me_matched) {                
                for (auto obj_iterator = potential_objs.begin(); obj_iterator != potential_objs.end(); obj_iterator++) {                    
                    if (objects_match_me(*obj_iterator, me_obj)) {
                        obj_iterator->dx = me_obj.me_dx;
                        obj_iterator->dy = me_obj.me_dy;
                        obj_iterator->count++;

                        // Once iterator > threshold, publish
                        if (obj_iterator->count > POTENTIAL_THRESHOLD) {
                            associated_me_msg.obj_id = next_id++;
                            pub_me_signals(associated_me_msg, me_obj);
                            me_to_kf_pub.publish(associated_me_msg);                    
                            potential_objs.erase(obj_iterator);
                            std::cout << "Publishing ME object passed threshold" << std::endl;
                        }
                        me_matched = 1;
                        break;
                    }
                }
            }

            // Not match env state vector or temporary array
            if (!me_matched) {                 
                std::cout << "added ME object to potentials" << std::endl;
                potential_objs.emplace_back(ObjectState(me_obj.me_dx, me_obj.me_dy));
            }            
        }
    }
    else{
        std::cout << "Mobileye message invalid" << std::endl;
    }

}
bool DataAssociation::sensor_diagnostic_callback_CH2(common::sensor_diagnostic_flag_CH2::Request& req,
                                                    common::sensor_diagnostic_flag_CH2::Response& /*res*/) {
    // service callback for front radar
    FRONT_RADAR = req.front_radar;
    return true;
}

bool DataAssociation::sensor_diagnostic_callback_CH3(common::sensor_diagnostic_flag_CH3::Request& req,
                                                    common::sensor_diagnostic_flag_CH3::Response& /*res*/) {
    // service callback for corner radars
    LEFT_CORNER_RADAR = req.left_corner_radar;
    RIGHT_CORNER_RADAR = req.right_corner_radar;
    return true;
}

bool DataAssociation::sensor_diagnostic_callback_CH4(common::sensor_diagnostic_flag_CH4::Request& req,
                                                    common::sensor_diagnostic_flag_CH4::Response& /*res*/) {
    // service callback for mobileye
    MOBILEYE = req.mobileye;
    return true;
}

common::associated_radar_msg DataAssociation::get_associated_radar_msg(){return associated_radar_msg;}
common::associated_me_msg DataAssociation::get_associated_me_msg(){return associated_me_msg;}

int main(int argc, char** argv){
    ros::init(argc, argv, "data_association");
    ros::NodeHandle data_association_handle;
    DataAssociation data_assc = DataAssociation(&data_association_handle);
    while (ros::ok()) {
        data_assc.delete_potential_objects();
        ros::Rate(10).sleep();
        ros::spinOnce();
    }
    return 0;
}
