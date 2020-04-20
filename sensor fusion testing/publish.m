rosshutdown
clear
clc
load('radar.mat');

rosinit
radar_pub = rospublisher('/radar_topic','sensor_fusion_testing/radar_object_data');
radar_msg = rosmessage(radar_pub);

timetable_height=height(radar_final);
if mod(radar_final.ID(1),2) == 0
    start_index = 2;
else
    start_index = 1;
end

if mod(radar_final.ID(timetable_height),2) == 0
    end_index = timetable_height;
else
    end_index = timetable_height-1;
end

for i = start_index:2:end_index
    fields_A=fieldnames(radar_final.Signals{i,1});
    fields_B=fieldnames(radar_final.Signals{i+1,1});
    radar_msg.RadarDx = getfield(radar_final.Signals{i,1},fields_A{7})
    radar_msg.RadarDy = getfield(radar_final.Signals{i,1},fields_A{6});
    radar_msg.RadarVx = getfield(radar_final.Signals{i,1},fields_A{5});
    radar_msg.RadarVy = getfield(radar_final.Signals{i+1,1},fields_B{8});
    radar_msg.RadarTimestamp = radar_final.time_in_sec(i);
    send(radar_pub,radar_msg);
end
rosshutdown