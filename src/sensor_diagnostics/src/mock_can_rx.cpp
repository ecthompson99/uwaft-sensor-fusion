#include "sensor_diag.h" //message and service file declarations

int main(int argc, char **argv){
    ros::init(argc, argv, "mock_can_rx");
    ros::NodeHandle n;

    ros::Publisher diag_pub = n.advertise <common::sensor_diagnostic_data_msg>("sensor_diag_data",10);

    uint8_t tc_counter = 1;
    uint8_t mc_counter = 1;

    while(ros::ok()){

        common::sensor_diagnostic_data_msg msg;
        // Set values of signals for unit testing
        msg.channel_number = 2;
        msg.radar_number = 1;

        msg.radar_mess_starter_consist_bit = 1;  // Invalid if not all consist bits are the same

        msg.radar_mess_aconsist_bit = 1;
        msg.radar_mess_bconsist_bit = 1;

        msg.radar_mess_ender_cosist_bit = 1;
        msg.radar_tc_counter = tc_counter;
        msg.radar_packet_checksum = 0;  // Invalid if not 0

        msg.r_stat_itc_info = 0;                 // Invalid if != 0
        msg.r_stat_sgu_fail = 0;                 // Invalid if = 1
        msg.r_stat_hw_fail = 0;                  // Invalid if = 1
        msg.r_stat_horizontal_misalignment = 0;  // Invalid if abs value is greater than 0.0152
        msg.r_stat_absorption_blindness = 0;     // Invalid if value is greater than 0.1
        msg.r_stat_distortion_blindness = 0;     // Invalid if value is greater than 0.1
        msg.r_stat_mc = mc_counter;
        // msg.r_stat_crc = 0;

        msg.headway_valid = 1;  // Invalid if 0
        msg.maintenance = 0;    // Invalid if 1
        msg.failsafe = 0;       // Invalid if 1

        msg.quality = 2;  // Invalid if less than
        std::cout << +msg.channel_number << std::endl;

        diag_pub.publish(msg);

        tc_counter++;
        mc_counter++;
        if (mc_counter == 16) {
          mc_counter = 0;
        }

        ros::Rate(1500).sleep();
        ros::spinOnce();    
    }

}  