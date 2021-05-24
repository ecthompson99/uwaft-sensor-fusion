#include "radar_structs.h"
#include "fusion_structs.h"
#include "radar.h"

Sensor_Fusion_TX::Sensor_Fusion_TX(ros::NodeHandle* node_handle) : node_handle(node_handle){
    sensor_fusion_sub = node_handle->subscribe(TOPIC_SF, MESSAGE_BUFFER_SIZE, &Sensor_Fusion_TX::fusion_callback, this);
    fusion_out = {0};
};

void Sensor_Fusion_TX::fusion_callback(const common::tracked_output_msg& recvd_data) {
    for (size_t lane = 0; lane < 3; lane++) {
        fusion_out.obj_id[lane] = recvd_data.obj_id[lane];
        fusion_out.obj_dx[lane] = recvd_data.obj_dx[lane];
        fusion_out.obj_lane[lane] = recvd_data.obj_lane[lane];
        fusion_out.obj_vx[lane] = recvd_data.obj_vx[lane];
        fusion_out.obj_dy[lane] = recvd_data.obj_dy[lane];
        fusion_out.obj_ax[lane] = recvd_data.obj_ax[lane];
        fusion_out.obj_path[lane] = recvd_data.obj_path[lane];
        fusion_out.obj_vy[lane] = recvd_data.obj_vy[lane];
        fusion_out.obj_rc[lane] = 0;
        fusion_out.obj_timestamp[lane] = recvd_data.obj_timestamp[lane];
        fusion_out.obj_track_num[lane] = recvd_data.obj_track_num[lane];
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_tx_rx_CH3");
    ros::NodeHandle can_tx_rx_CH3_handle;

    Sensor_Fusion_TX sensor_fusion_tx = Sensor_Fusion_TX(&can_tx_rx_CH3_handle);

    ros::Time mem3 = ros::Time::now();

    int obj_rc_trk = 0;

    long int id;
    unsigned int dlc;
    unsigned int flag;
    unsigned long time;
    int case_num = 0;
    int radar_num = 0;           // 1 or 2 = valid
    int frame_num = 0;           // 1 = Frame_A, 2 = Frame_B, 3 = general, other = error
    int obj_num = -1;            // 0 to 31 = valid
    int target_object_num = -1;  // 0 to 5 = valid
    uint8_t can_data[8] = {0};

    int unpack_return = -1;  // 0 is successful, negative error code

    // radar counters, one for each radar
    uint8_t tc_check = 0;
    uint8_t mc_check = 0;

    bool pub_data = false;  // 0 if the node has not receieved a starter bit, otherwise 1

    canInitializeLibrary();
    canHandle hnd;
    canStatus stat;

    hnd = canOpenChannel(2, canOPEN_EXCLUSIVE);  // 1 for ch2 and 2 for ch3
    std::cout << "hnd number" << hnd <<std::endl;
    if (hnd < 0) {
        char msg[64];
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }

    canSetBusParams(hnd, canBITRATE_500K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);
    std::cout << "Set up CAN Parameters, starting to read in loop" << std::endl;
    while (ros::ok()) {
        ros::Time now = ros::Time::now();

        // Sensor fusion TX
        size_t size = 8u;
            
        struct emc_fusion_object_obj_track1_t fusion_obj_trk1;
        struct emc_fusion_object_obj_track2_t fusion_obj_trk2; 
        struct emc_fusion_object_obj_track3_t fusion_obj_trk3;

        // Individual variables for each of the three messages ObjTrk1, ObjTrk2, ObjTrk3
        uint8_t fusion_obj_msg_trk1[9] = {0};
        uint8_t fusion_obj_msg_trk2[9] = {0};
        uint8_t fusion_obj_msg_trk3[9] = {0};

        int trk1 = 0;
        int trk2 = 0;
        int trk3 = 0;

        //ObjTrack1
        sensor_fusion_tx.fusion_out.obj_rc[0] = obj_rc_trk;

        fusion_obj_trk1.obj_trk1_id = emc_fusion_object_obj_track1_obj_trk1_id_encode(sensor_fusion_tx.fusion_out.obj_id[0]);
        fusion_obj_trk1.obj_trk1_lane = emc_fusion_object_obj_track1_obj_trk1_lane_encode(sensor_fusion_tx.fusion_out.obj_lane[0]);
        fusion_obj_trk1.obj_trk1_path = emc_fusion_object_obj_track1_obj_trk1_path_encode(sensor_fusion_tx.fusion_out.obj_path[0]);
        fusion_obj_trk1.obj_trk1_lat_range = emc_fusion_object_obj_track1_obj_trk1_lat_range_encode(sensor_fusion_tx.fusion_out.obj_dy[0]);
        fusion_obj_trk1.obj_trk1_rc = emc_fusion_object_obj_track1_obj_trk1_rc_encode(sensor_fusion_tx.fusion_out.obj_rc[0]);  //ROLLING COUNTER??
        fusion_obj_trk1.obj_trk1_long_range = emc_fusion_object_obj_track1_obj_trk1_long_range_encode(sensor_fusion_tx.fusion_out.obj_dx[0]);
        fusion_obj_trk1.obj_trk1_rel_lat_velocity = emc_fusion_object_obj_track1_obj_trk1_rel_lat_velocity_encode(sensor_fusion_tx.fusion_out.obj_vy[0]);
        fusion_obj_trk1.obj_trk1_rel_long_accel = emc_fusion_object_obj_track1_obj_trk1_rel_long_accel_encode(sensor_fusion_tx.fusion_out.obj_ax[0]);
        fusion_obj_trk1.obj_trk1_rel_long_vel = emc_fusion_object_obj_track1_obj_trk1_rel_long_vel_encode(sensor_fusion_tx.fusion_out.obj_vx[0]);

        struct emc_fusion_object_obj_track1_t *fusion_obj_trk1_struct = &fusion_obj_trk1;
        emc_fusion_object_obj_track1_pack(fusion_obj_msg_trk1, fusion_obj_trk1_struct, size); //adding info to the message to ouputted to CAN bus


        //ObjTrack2
        sensor_fusion_tx.fusion_out.obj_rc[1] = obj_rc_trk;

        fusion_obj_trk2.obj_trk2_id = emc_fusion_object_obj_track2_obj_trk2_id_encode(sensor_fusion_tx.fusion_out.obj_id[1]);
        fusion_obj_trk2.obj_trk2_lane = emc_fusion_object_obj_track2_obj_trk2_lane_encode(sensor_fusion_tx.fusion_out.obj_lane[1]);
        fusion_obj_trk2.obj_trk2_path = emc_fusion_object_obj_track2_obj_trk2_path_encode(sensor_fusion_tx.fusion_out.obj_path[1]);
        fusion_obj_trk2.obj_trk2_lat_range = emc_fusion_object_obj_track2_obj_trk2_lat_range_encode(sensor_fusion_tx.fusion_out.obj_dy[1]);
        fusion_obj_trk2.obj_trk2_rc = emc_fusion_object_obj_track2_obj_trk2_rc_encode(sensor_fusion_tx.fusion_out.obj_rc[1]);  //ROLLING COUNTER??
        fusion_obj_trk2.obj_trk2_long_range = emc_fusion_object_obj_track2_obj_trk2_long_range_encode(sensor_fusion_tx.fusion_out.obj_dx[1]);
        fusion_obj_trk2.obj_trk2_rel_lat_velocity = emc_fusion_object_obj_track2_obj_trk2_rel_lat_velocity_encode(sensor_fusion_tx.fusion_out.obj_vy[1]);
        fusion_obj_trk2.obj_trk2_rel_long_accel = emc_fusion_object_obj_track2_obj_trk2_rel_long_accel_encode(sensor_fusion_tx.fusion_out.obj_ax[1]);
        fusion_obj_trk2.obj_trk2_rel_long_vel = emc_fusion_object_obj_track2_obj_trk2_rel_long_vel_encode(sensor_fusion_tx.fusion_out.obj_vx[1]);     

        struct emc_fusion_object_obj_track2_t *fusion_obj_trk2_struct = &fusion_obj_trk2;
        emc_fusion_object_obj_track2_pack(fusion_obj_msg_trk2, fusion_obj_trk2_struct, size);


        //ObjTrack3
        sensor_fusion_tx.fusion_out.obj_rc[2] = obj_rc_trk;

        fusion_obj_trk3.obj_trk3_id = emc_fusion_object_obj_track3_obj_trk3_id_encode(sensor_fusion_tx.fusion_out.obj_id[2]);
        fusion_obj_trk3.obj_trk3_lane = emc_fusion_object_obj_track3_obj_trk3_lane_encode(sensor_fusion_tx.fusion_out.obj_lane[2]);
        fusion_obj_trk3.obj_trk3_path = emc_fusion_object_obj_track3_obj_trk3_path_encode(sensor_fusion_tx.fusion_out.obj_path[2]);
        fusion_obj_trk3.obj_trk3_lat_range = emc_fusion_object_obj_track3_obj_trk3_lat_range_encode(sensor_fusion_tx.fusion_out.obj_dy[2]);
        fusion_obj_trk3.obj_trk3_rc = emc_fusion_object_obj_track3_obj_trk3_rc_encode(sensor_fusion_tx.fusion_out.obj_rc[2]);  //ROLLING COUNTER??
        fusion_obj_trk3.obj_trk3_long_range = emc_fusion_object_obj_track3_obj_trk3_long_range_encode(sensor_fusion_tx.fusion_out.obj_dx[2]);
        fusion_obj_trk3.obj_trk3_rel_lat_velocity = emc_fusion_object_obj_track3_obj_trk3_rel_lat_velocity_encode(sensor_fusion_tx.fusion_out.obj_vy[2]);
        fusion_obj_trk3.obj_trk3_rel_long_accel = emc_fusion_object_obj_track3_obj_trk3_rel_long_accel_encode(sensor_fusion_tx.fusion_out.obj_ax[2]);
        fusion_obj_trk3.obj_trk3_rel_long_vel = emc_fusion_object_obj_track3_obj_trk3_rel_long_vel_encode(sensor_fusion_tx.fusion_out.obj_vx[2]);  

        struct emc_fusion_object_obj_track3_t *fusion_obj_trk3_struct = &fusion_obj_trk3;
        emc_fusion_object_obj_track3_pack(fusion_obj_msg_trk3, fusion_obj_trk3_struct, size);

        // rolling counter
        if (obj_rc_trk > 3) {
          obj_rc_trk = 0;
        } else {
          obj_rc_trk = obj_rc_trk + 1;
        }

        //difference between time stamp of current time and previous cycle should be more than .1 seconds
        if((now.toSec()-mem3.toSec())>.1){
            
            //writing the three messages to the CAN bus 
            canWrite(hnd, 1089 , fusion_obj_msg_trk1, SIZE_OF_MSG, canOPEN_EXCLUSIVE);
            canWrite(hnd, 1090 , fusion_obj_msg_trk2, SIZE_OF_MSG, canOPEN_EXCLUSIVE);
            canWrite(hnd, 1091 , fusion_obj_msg_trk3, SIZE_OF_MSG, canOPEN_EXCLUSIVE);
            
            mem3 = now;//timestamp of previous cycle 
            
        }

        ros::spinOnce();
        ros::Duration(0.005).sleep();  // the sleep must be less than 5 ms
    }
    canBusOff(hnd);
    canClose(hnd);
  return 0;
}

