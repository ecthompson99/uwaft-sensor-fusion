#include <canlib.h>

#include <stdio.h>
#include <sstream>
#include <vector>

#include "ros/ros.h"

#include "can_tx_rx/ext_log_data2_2_32.c"
#include "can_tx_rx/ext_log_data2_2_32.h"

#include "can_tx_rx/mobileye_object_data_msg.h"

static const uint16_t TX_RX_MESSAGE_BUFFER_SIZE = 1000;
static const uint64_t FRAME_INTERVAL = 50;

void test_function (uint8_t can_data[8], long int & id, int count) {
    
    double me_dx = 3.2;
    double me_dy = 6.4;

    double me_vx = 34.6;
    double me_ax = 12.4;

    uint8_t me_object_id = 1;
    uint8_t me_object_lane = 2;

    if (count == 0) {
        id = 1849;
        ext_log_data2_2_32_obstacle_data_a1_t testA1;
        testA1.obstacle_pos_x = ext_log_data2_2_32_obstacle_data_a1_obstacle_pos_x_encode(me_dx);
        testA1.obstacle_pos_y = ext_log_data2_2_32_obstacle_data_a1_obstacle_pos_y_encode(me_dy);
        testA1.obstacle_vel_x = ext_log_data2_2_32_obstacle_data_a1_obstacle_vel_x_encode(me_vx);
        testA1.obstacle_vel_x = ext_log_data2_2_32_obstacle_data_a1_obstacle_id_encode(me_object_id);
        ext_log_data2_2_32_obstacle_data_a1_pack(can_data, &testA1, 8);
    }
    else if (count == 1) {
        id = 1850;
        ext_log_data2_2_32_obstacle_data_b1_t testB1;
        testB1.obstacle_lane = ext_log_data2_2_32_obstacle_data_b1_obstacle_lane_encode(me_object_lane);
        ext_log_data2_2_32_obstacle_data_b1_pack(can_data, &testB1, 8);
    }
    else if (count == 2) {
        id = 1851;
        ext_log_data2_2_32_obstacle_data_c1_t testC1;
        testC1.object_accel_x = ext_log_data2_2_32_obstacle_data_c1_object_accel_x_encode(me_ax);
        ext_log_data2_2_32_obstacle_data_c1_pack(can_data, &testC1, 8);
    }
    else{
        id = 0;
    }
}

bool time_control (std::string frame, uint8_t obj, bool publish_choice[10], 
    uint64_t obj_time[10], uint64_t &time) {
    if (frame == "A") {
        obj_time[obj] = time;
        publish_choice[obj] = 1;
        return true;
    }
    
    else if (frame == "B") {
        if (time - obj_time[obj] > FRAME_INTERVAL) {
            publish_choice[obj] = 0;
            return false;
        }
        else {
            obj_time[obj] = time;
            return true;
        }
    }
    else if (frame == "C") {
        if (!publish_choice[obj]) {
            return false;
        }
        else if (time - obj_time[obj] > FRAME_INTERVAL) {
            publish_choice[obj] = 0;
            return false;
        }
        else {
            obj_time[obj] = time;
            return true;
        }
    }
    else {}
}

void publish_to_node (int obj_num, can_tx_rx::mobileye_object_data_msg mobileye_obj_data_msg, 
double obj_dx[10], double obj_dy[10], double obj_vx[10], double obj_ax[10], uint8_t obj_id[10], 
uint8_t obj_lane[10], uint64_t obj_time[10], ros::Publisher data_pub) {
    mobileye_obj_data_msg.me_dx = obj_dx[obj_num];
    mobileye_obj_data_msg.me_dy = obj_dy[obj_num];
    mobileye_obj_data_msg.me_vx = obj_vx[obj_num];
    mobileye_obj_data_msg.me_ax = obj_ax[obj_num];
    mobileye_obj_data_msg.me_object_id = obj_id[obj_num];
    mobileye_obj_data_msg.me_object_lane = obj_lane[obj_num];
    mobileye_obj_data_msg.me_timestamp = obj_time[obj_num];

    data_pub.publish(mobileye_obj_data_msg);
    ros::spinOnce();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "can_tx_rx_CH4");
    ros::NodeHandle can_tx_rx_CH4_handle;

    ros::Publisher data_pub = can_tx_rx_CH4_handle.advertise<can_tx_rx::mobileye_object_data_msg>(
    "mobileye_object_data", TX_RX_MESSAGE_BUFFER_SIZE);

    can_tx_rx::mobileye_object_data_msg mobileye_obj_data_msg;

    /*canHandle hnd;
    
    canInitializeLibrary();

    hnd = canOpenChannel(3, canOPEN_EXCLUSIVE);

    if (hnd < 0) {
        char msg[64];
        canGetErrorText((canStatus)hnd, msg, sizeof(msg));
        fprintf(stderr, "canOpenChannel failed (%s)\n", msg);
        exit(1);
    }
    
    canSetBusParams(hnd, canBITRATE_250K, 0, 0, 0, 0, 0);
    canSetBusOutputControl(hnd, canDRIVER_NORMAL);
    canBusOn(hnd);
    */
    long int id;
    unsigned int dlc;
    unsigned int flag;
    uint64_t time;
    uint8_t obj_num = -1; // 0 to 31 = valid
    uint8_t size_of_msg = 8;
    uint8_t can_data[8] = {0};

    int unpack_return;

    double obj_dx[10] = {0};
    double obj_dy[10] = {0};

    double obj_vx[10] = {};
    double obj_ax[10] = {};

    uint8_t obj_id[10] = {0};
    uint8_t obj_lane[10] = {0};

    uint64_t obj_time[10] = {0};
    bool publish_choice[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    
    int TEST_COUNT = 0;

    while (ros::ok()) {
        //canStatus stat = canRead(hnd, &id, &can_data, &dlc, &flag, &time);
        test_function(can_data, id, TEST_COUNT);
        TEST_COUNT++;
        if (/*canOK == stat*/1) {
            switch (id) {
                //A-frames
                case 1849:
                    time_control ("A", 0, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a1_t target_a1_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a1_unpack(&target_a1_obj, can_data, size_of_msg);
                    obj_dx[0] = ext_log_data2_2_32_obstacle_data_a1_obstacle_pos_x_decode(target_a1_obj.obstacle_pos_x);
                    obj_dy[0] = ext_log_data2_2_32_obstacle_data_a1_obstacle_pos_y_decode(target_a1_obj.obstacle_pos_y);
                    obj_vx[0] = ext_log_data2_2_32_obstacle_data_a1_obstacle_vel_x_decode(target_a1_obj.obstacle_vel_x);
                    obj_id[0] = ext_log_data2_2_32_obstacle_data_a1_obstacle_id_decode(target_a1_obj.obstacle_id);
                    break;
                case 1852:
                    time_control ("A", 1, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a2_t target_a2_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a2_unpack(&target_a2_obj, can_data, size_of_msg);
                    obj_dx[1] = ext_log_data2_2_32_obstacle_data_a2_obstacle_pos_x_decode(target_a2_obj.obstacle_pos_x);
                    obj_dy[1] = ext_log_data2_2_32_obstacle_data_a2_obstacle_pos_y_decode(target_a2_obj.obstacle_pos_y);
                    obj_vx[1] = ext_log_data2_2_32_obstacle_data_a2_obstacle_vel_x_decode(target_a2_obj.obstacle_vel_x);
                    obj_id[1] = ext_log_data2_2_32_obstacle_data_a2_obstacle_id_decode(target_a2_obj.obstacle_id);
                    break;
                case 1855:
                    time_control ("A", 2, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a3_t target_a3_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a3_unpack(&target_a3_obj, can_data, size_of_msg);
                    obj_dx[2] = ext_log_data2_2_32_obstacle_data_a3_obstacle_pos_x_decode(target_a3_obj.obstacle_pos_x);
                    obj_dy[2] = ext_log_data2_2_32_obstacle_data_a3_obstacle_pos_y_decode(target_a3_obj.obstacle_pos_y);
                    obj_vx[2] = ext_log_data2_2_32_obstacle_data_a3_obstacle_vel_x_decode(target_a3_obj.obstacle_vel_x);
                    obj_id[2] = ext_log_data2_2_32_obstacle_data_a3_obstacle_id_decode(target_a3_obj.obstacle_id);
                    break;
                case 1858:
                    time_control ("A", 3, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a4_t target_a4_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a4_unpack(&target_a4_obj, can_data, size_of_msg);
                    obj_dx[3] = ext_log_data2_2_32_obstacle_data_a4_obstacle_pos_x_decode(target_a4_obj.obstacle_pos_x);
                    obj_dy[3] = ext_log_data2_2_32_obstacle_data_a4_obstacle_pos_y_decode(target_a4_obj.obstacle_pos_y);
                    obj_vx[3] = ext_log_data2_2_32_obstacle_data_a4_obstacle_vel_x_decode(target_a4_obj.obstacle_vel_x);
                    obj_id[3] = ext_log_data2_2_32_obstacle_data_a4_obstacle_id_decode(target_a4_obj.obstacle_id);
                    break;
                case 1861:
                    time_control ("A", 4, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a5_t target_a5_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a5_unpack(&target_a5_obj, can_data, size_of_msg);
                    obj_dx[4] = ext_log_data2_2_32_obstacle_data_a5_obstacle_pos_x_decode(target_a5_obj.obstacle_pos_x);
                    obj_dy[4] = ext_log_data2_2_32_obstacle_data_a5_obstacle_pos_y_decode(target_a5_obj.obstacle_pos_y);
                    obj_vx[4] = ext_log_data2_2_32_obstacle_data_a5_obstacle_vel_x_decode(target_a5_obj.obstacle_vel_x);
                    obj_id[4] = ext_log_data2_2_32_obstacle_data_a5_obstacle_id_decode(target_a5_obj.obstacle_id);
                    break;
                case 1864:
                    time_control ("A", 5, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a6_t target_a6_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a6_unpack(&target_a6_obj, can_data, size_of_msg);
                    obj_dx[5] = ext_log_data2_2_32_obstacle_data_a6_obstacle_pos_x_decode(target_a6_obj.obstacle_pos_x);
                    obj_dy[5] = ext_log_data2_2_32_obstacle_data_a6_obstacle_pos_y_decode(target_a6_obj.obstacle_pos_y);
                    obj_vx[5] = ext_log_data2_2_32_obstacle_data_a6_obstacle_vel_x_decode(target_a6_obj.obstacle_vel_x);
                    obj_id[5] = ext_log_data2_2_32_obstacle_data_a6_obstacle_id_decode(target_a6_obj.obstacle_id);
                    break;
                case 1867:
                    time_control ("A", 6, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a7_t target_a7_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a7_unpack(&target_a7_obj, can_data, size_of_msg);
                    obj_dx[6] = ext_log_data2_2_32_obstacle_data_a7_obstacle_pos_x_decode(target_a7_obj.obstacle_pos_x);
                    obj_dy[6] = ext_log_data2_2_32_obstacle_data_a7_obstacle_pos_y_decode(target_a7_obj.obstacle_pos_y);
                    obj_vx[6] = ext_log_data2_2_32_obstacle_data_a7_obstacle_vel_x_decode(target_a7_obj.obstacle_vel_x);
                    obj_id[6] = ext_log_data2_2_32_obstacle_data_a7_obstacle_id_decode(target_a7_obj.obstacle_id);
                    break;
                case 1870:
                    time_control ("A", 7, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a8_t target_a8_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a8_unpack(&target_a8_obj, can_data, size_of_msg);
                    obj_dx[7] = ext_log_data2_2_32_obstacle_data_a8_obstacle_pos_x_decode(target_a8_obj.obstacle_pos_x);
                    obj_dy[7] = ext_log_data2_2_32_obstacle_data_a8_obstacle_pos_y_decode(target_a8_obj.obstacle_pos_y);
                    obj_vx[7] = ext_log_data2_2_32_obstacle_data_a8_obstacle_vel_x_decode(target_a8_obj.obstacle_vel_x);
                    obj_id[7] = ext_log_data2_2_32_obstacle_data_a8_obstacle_id_decode(target_a8_obj.obstacle_id);
                    break;
                case 1873:
                    time_control ("A", 8, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a9_t target_a9_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a9_unpack(&target_a9_obj, can_data, size_of_msg);
                    obj_dx[8] = ext_log_data2_2_32_obstacle_data_a9_obstacle_pos_x_decode(target_a9_obj.obstacle_pos_x);
                    obj_dy[8] = ext_log_data2_2_32_obstacle_data_a9_obstacle_pos_y_decode(target_a9_obj.obstacle_pos_y);
                    obj_vx[8] = ext_log_data2_2_32_obstacle_data_a9_obstacle_vel_x_decode(target_a9_obj.obstacle_vel_x);
                    obj_id[8] = ext_log_data2_2_32_obstacle_data_a9_obstacle_id_decode(target_a9_obj.obstacle_id);
                    break;
                case 1876:
                    time_control ("A", 9, publish_choice, obj_time, time);
                    ext_log_data2_2_32_obstacle_data_a10_t target_a10_obj;
                    unpack_return = ext_log_data2_2_32_obstacle_data_a10_unpack(&target_a10_obj, can_data, size_of_msg);
                    obj_dx[9] = ext_log_data2_2_32_obstacle_data_a10_obstacle_pos_x_decode(target_a10_obj.obstacle_pos_x);
                    obj_dy[9] = ext_log_data2_2_32_obstacle_data_a10_obstacle_pos_y_decode(target_a10_obj.obstacle_pos_y);
                    obj_vx[9] = ext_log_data2_2_32_obstacle_data_a10_obstacle_vel_x_decode(target_a10_obj.obstacle_vel_x);
                    obj_id[9] = ext_log_data2_2_32_obstacle_data_a10_obstacle_id_decode(target_a10_obj.obstacle_id);
                    break;
                
                //B-frames
                case 1850:
                    if (time_control ("B", 0, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b1_t target_b1_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b1_unpack(&target_b1_obj, can_data, size_of_msg);
                        obj_lane[0] = ext_log_data2_2_32_obstacle_data_b1_obstacle_lane_decode(target_b1_obj.obstacle_lane);
                    }
                    break;
                case 1853:
                    if (time_control ("B", 1, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b2_t target_b2_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b2_unpack(&target_b2_obj, can_data, size_of_msg);
                        obj_lane[1] = ext_log_data2_2_32_obstacle_data_b2_obstacle_lane_decode(target_b2_obj.obstacle_lane);
                    }
                    break;
                case 1856:
                    if (time_control ("B", 2, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b3_t target_b3_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b3_unpack(&target_b3_obj, can_data, size_of_msg);
                        obj_lane[2] = ext_log_data2_2_32_obstacle_data_b3_obstacle_lane_decode(target_b3_obj.obstacle_lane);
                    }
                    break;
                case 1859:
                    if (time_control ("B", 3, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b4_t target_b4_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b4_unpack(&target_b4_obj, can_data, size_of_msg);
                        obj_lane[3] = ext_log_data2_2_32_obstacle_data_b4_obstacle_lane_decode(target_b4_obj.obstacle_lane);
                    }
                    break;
                case 1862:
                    if (time_control ("B", 4, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b5_t target_b5_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b5_unpack(&target_b5_obj, can_data, size_of_msg);
                        obj_lane[4] = ext_log_data2_2_32_obstacle_data_b5_obstacle_lane_decode(target_b5_obj.obstacle_lane);
                    }
                    break;
                case 1865:
                    if (time_control ("B", 5, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b6_t target_b6_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b6_unpack(&target_b6_obj, can_data, size_of_msg);
                        obj_lane[5] = ext_log_data2_2_32_obstacle_data_b6_obstacle_lane_decode(target_b6_obj.obstacle_lane);
                    }
                    break;
                case 1868:
                    if (time_control ("B", 6, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b7_t target_b7_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b7_unpack(&target_b7_obj, can_data, size_of_msg);
                        obj_lane[6] = ext_log_data2_2_32_obstacle_data_b7_obstacle_lane_decode(target_b7_obj.obstacle_lane);
                    }
                    break;
                case 1871:
                    if (time_control ("B", 7, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b8_t target_b8_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b8_unpack(&target_b8_obj, can_data, size_of_msg);
                        obj_lane[7] = ext_log_data2_2_32_obstacle_data_b8_obstacle_lane_decode(target_b8_obj.obstacle_lane);
                    }
                    break;
                case 1874:
                    if (time_control ("B", 8, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b9_t target_b9_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b9_unpack(&target_b9_obj, can_data, size_of_msg);
                        obj_lane[8] = ext_log_data2_2_32_obstacle_data_b9_obstacle_lane_decode(target_b9_obj.obstacle_lane);
                    }
                    break;
                case 1877:
                    if (time_control ("B", 9, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_b10_t target_b10_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_b10_unpack(&target_b10_obj, can_data, size_of_msg);
                        obj_lane[9] = ext_log_data2_2_32_obstacle_data_b10_obstacle_lane_decode(target_b10_obj.obstacle_lane);
                    }
                    break;
                
                //C-frames with publishing
                case 1851:
                    if (time_control ("C", 0, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c1_t target_c1_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c1_unpack(&target_c1_obj, can_data, size_of_msg);
                        obj_ax[0] = ext_log_data2_2_32_obstacle_data_c1_object_accel_x_decode(target_c1_obj.object_accel_x);
                        publish_to_node(0, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1854:
                    if (time_control ("C", 1, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c2_t target_c2_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c2_unpack(&target_c2_obj, can_data, size_of_msg);
                        obj_ax[1] = ext_log_data2_2_32_obstacle_data_c2_object_accel_x_decode(target_c2_obj.object_accel_x);
                        publish_to_node(1, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1857:
                    if (time_control ("C", 2, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c3_t target_c3_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c3_unpack(&target_c3_obj, can_data, size_of_msg);
                        obj_ax[2] = ext_log_data2_2_32_obstacle_data_c3_object_accel_x_decode(target_c3_obj.object_accel_x);
                        publish_to_node(2, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1860:
                    if (time_control ("C", 3, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c4_t target_c4_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c4_unpack(&target_c4_obj, can_data, size_of_msg);
                        obj_ax[3] = ext_log_data2_2_32_obstacle_data_c4_object_accel_x_decode(target_c4_obj.object_accel_x);
                        publish_to_node(3, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1863:
                    if (time_control ("C", 4, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c5_t target_c5_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c5_unpack(&target_c5_obj, can_data, size_of_msg);
                        obj_ax[4] = ext_log_data2_2_32_obstacle_data_c5_object_accel_x_decode(target_c5_obj.object_accel_x);
                        publish_to_node(4, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1866:
                    if (time_control ("C", 5, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c6_t target_c6_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c6_unpack(&target_c6_obj, can_data, size_of_msg);
                        obj_ax[5] = ext_log_data2_2_32_obstacle_data_c6_object_accel_x_decode(target_c6_obj.object_accel_x);
                        publish_to_node(5, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1869:
                    if (time_control ("C", 6, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c7_t target_c7_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c7_unpack(&target_c7_obj, can_data, size_of_msg);
                        obj_ax[6] = ext_log_data2_2_32_obstacle_data_c7_object_accel_x_decode(target_c7_obj.object_accel_x);
                        publish_to_node(6, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1872:
                    if (time_control ("C", 7, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c8_t target_c8_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c8_unpack(&target_c8_obj, can_data, size_of_msg);
                        obj_ax[7] = ext_log_data2_2_32_obstacle_data_c8_object_accel_x_decode(target_c8_obj.object_accel_x);
                        publish_to_node(7, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1875:
                    if (time_control ("C", 8, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c9_t target_c9_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c9_unpack(&target_c9_obj, can_data, size_of_msg);
                        obj_ax[8] = ext_log_data2_2_32_obstacle_data_c9_object_accel_x_decode(target_c9_obj.object_accel_x);
                        publish_to_node(8, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
                case 1878:
                    if (time_control ("C", 9, publish_choice, obj_time, time)) {
                        ext_log_data2_2_32_obstacle_data_c10_t target_c10_obj;
                        unpack_return = ext_log_data2_2_32_obstacle_data_c10_unpack(&target_c10_obj, can_data, size_of_msg);
                        obj_ax[9] = ext_log_data2_2_32_obstacle_data_c10_object_accel_x_decode(target_c10_obj.object_accel_x);
                        publish_to_node(9, mobileye_obj_data_msg, obj_dx, obj_dy, obj_vx, obj_ax, obj_id, obj_lane, obj_time, data_pub);
                    }
                    break;
            }
        }
    }
    //canBusOff(hnd);
    //canClose(hnd);

    return 0;
}