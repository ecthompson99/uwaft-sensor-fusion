#include "sensor_diag.h" //message and service file declarations

int main(int argc, char **argv){

    ros::init(argc, argv, "mock_can_rx");
    ros::NodeHandle n;

    ros::Publisher diag_pub = n.advertise <common::sensor_diagnostic_data_msg>("sensor_diag_data",10);

    common::sensor_diagnostic_data_msg msg;

    // Set values of signals for unit testing
    msg.channel_number = 2;
    msg.radar_number = 1;

    msg.radar_mess_starter_consist_bit = 0;

    msg.radar_mess_aconsist_bit = 0;
    msg.radar_mess_bconsist_bit = 0;

    msg.radar_mess_ender_cosist_bit = 0;
    msg.radar_tc_counter = 0;
    msg.radar_packet_checksum = 0;

    msg.r_stat_itc_info = 0;
    msg.r_stat_sgu_fail = 0;
    msg.r_stat_hw_fail = 0;
    msg.r_stat_horizontal_misalignment = 0;
    msg.r_stat_absorption_blindness = 0;
    msg.r_stat_distortion_blindness = 0;
    msg.r_stat_mc = 0;
    msg.r_stat_crc = 0;

    msg.headway_valid = 0;
    msg.maintenance = 0;
    msg.failsafe = 0;

    msg.quality = 0;

    diag_pub.publish(msg);
    ros::spin();

}  