clc
radar_pub = rospublisher('/radar_topic','sensor_fusion_testing/radar_object_data');
radar_msg = rosmessage(radar_pub);
radar_msg.RadarDx = 1.0;
radar_msg.RadarDy = 1.1;
radar_msg.RadarVx = 1.2;
radar_msg.RadarVy = 1.3;
radar_msg.RadarTimestamp = 0.9;
for i=1:10000000
send(radar_pub,radar_msg);
end
