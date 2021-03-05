#include <canlib.h>
#include <iostream>
#include <stdio.h>

namespace CAN_Helper {
    void initialize_can(int channel_number, canHandle &hnd, canStatus &stat);
    double signal_in_range(double val, bool cond);
    struct message_constants {
        const int drive_ctrl_msg1 = 1072;
        const int pcm_to_cav2 = 1073;
        const int pcm_to_cav3 = 1074;

        // CH2 and 3
        const int ender1 = 1665;
        const int ender2 = 1667;
        const int starter1 = 1280;
        const int starter2 = 1282;
        const int status1 = 1670;
        const int status2 = 1672;

        const int diag_response1 = 1985; 
        const int diag_response2 = 1958;
        const int diag_request1 = 1879;
        const int diag_request2 = 1957;

        const int target_AB1 = 1604;
        const int target_AB2 = 1659;

        const int radar_AB1 = 1284;
        const int radar_AB2 = 1599;

        // CH4
        const int traffic1 = 1824;
        const int traffic2 = 1830;
        const int obstacle_A1 = 1849;
        const int obstacle_A2 = 1876;
        const int obstacle_B1 = 1850;
        const int obstacle_B2 = 1877;
        const int obstacle_C1 = 1851;
        const int obstacle_C2 = 1878;
        const int left_lane_A = 1894;
        const int left_lane_B = 1895;
        const int right_lane_A = 1896;
        const int right_lane_B = 1897;
        const int diagnostics = 1792;
    };
}
