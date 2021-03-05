rosshutdown
clear
clc
load('~/kaiROS/sensor fusion testing/radar.mat');

rosinit
radar_pub = rospublisher('/radar_from_matlab','sensor_fusion_testing/radar_object_data_from_matlab');
radar_msg = rosmessage(radar_pub);
input('uhh')
for i = 49:2:height(radar_final)
    % input('uhh');
    fields_A=fieldnames(radar_final.Signals{i,1});
    fields_B=fieldnames(radar_final.Signals{i+1,1});
    radar_msg.RadarDx = getfield(radar_final.Signals{i,1},fields_A{7});
    radar_msg.RadarDy = getfield(radar_final.Signals{i,1},fields_A{6});
    radar_msg.RadarVx = getfield(radar_final.Signals{i,1},fields_A{5});
    radar_msg.RadarVy = getfield(radar_final.Signals{i+1,1},fields_B{8});
    radar_msg.RadarAx = getfield(radar_final.Signals{i,1},fields_A{4});
    radar_msg.RadarTimestamp = radar_final.time_in_sec(i);
    radar_msg.RadarDxSigma = getfield(radar_final.Signals{i+1,1},fields_B{10});
    radar_msg.RadarDySigma = getfield(radar_final.Signals{i+1,1},fields_B{3});
    radar_msg.RadarVxSigma = getfield(radar_final.Signals{i+1,1},fields_B{9});
    radar_msg.RadarAxSigma = getfield(radar_final.Signals{i+1,1},fields_B{11});
    send(radar_pub,radar_msg);
end
rosshutdown