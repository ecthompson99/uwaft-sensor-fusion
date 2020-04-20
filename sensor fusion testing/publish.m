clc
load('ACCGapTestMed_20191103_155525_CAN.mat');

radar_pub = rospublisher('/radar_topic','sensor_fusion_testing/radar_object_data');
radar_msg = rosmessage(radar_pub);
me_pub = rospublisher('/mobileye_topic','sensor_fusion_testing/mobileye_object_data');
me_msg = rosmessage(me_pub);

