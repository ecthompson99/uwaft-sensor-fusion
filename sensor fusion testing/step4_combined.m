rosshutdown
clear
clc
load('~/kaiROS/sensor fusion testing/mobileye.mat')
load('~/kaiROS/sensor fusion testing/radar.mat')

rosinit
me_pub = rospublisher('/mobileye_from_matlab','sensor_fusion_testing/mobileye_object_data_from_matlab');
me_msg = rosmessage(me_pub);
radar_pub = rospublisher('/radar_from_matlab','sensor_fusion_testing/radar_object_data_from_matlab');
radar_msg = rosmessage(radar_pub);

radar_index = 49;
me_index = 1;
clk = 0;
radar_msg.RadarTimestamp = radar_final.time_in_sec(radar_index);
me_msg.MeTimestamp = me_final.time_in_sec(me_index);

input('uhh');
while true
    % input('uhh');

    while radar_msg.RadarTimestamp > clk + .001 && me_msg.MeTimestamp > clk + .001
        clk = clk + .001;
    end

    if radar_msg.RadarTimestamp < clk + .001
        fields_A=fieldnames(radar_final.Signals{radar_index,1});
        fields_B=fieldnames(radar_final.Signals{radar_index+1,1});
        radar_msg.RadarDx = getfield(radar_final.Signals{radar_index,1},fields_A{7});
        radar_msg.RadarDy = getfield(radar_final.Signals{radar_index,1},fields_A{6});
        radar_msg.RadarVx = getfield(radar_final.Signals{radar_index,1},fields_A{5});
        radar_msg.RadarVy = getfield(radar_final.Signals{radar_index+1,1},fields_B{8});
        radar_msg.RadarAx = getfield(radar_final.Signals{radar_index,1},fields_A{4});
        radar_msg.RadarDxSigma = getfield(radar_final.Signals{radar_index+1,1},fields_B{10});
        radar_msg.RadarDySigma = getfield(radar_final.Signals{radar_index+1,1},fields_B{3});
        radar_msg.RadarVxSigma = getfield(radar_final.Signals{radar_index+1,1},fields_B{9});
        radar_msg.RadarAxSigma = getfield(radar_final.Signals{radar_index+1,1},fields_B{11});
        send(radar_pub,radar_msg);
        radar_index = radar_index + 2;
        radar_msg.RadarTimestamp = radar_final.time_in_sec(radar_index);
    end

    if me_msg.MeTimestamp < clk + .001
        fields_A_me=fieldnames(me_final.Signals{me_index,1});
        me_msg.MeDx = getfield(me_final.Signals{me_index,1},fields_A_me{9});
        me_msg.MeDy = getfield(me_final.Signals{me_index,1},fields_A_me{8});
        me_msg.MeVx = getfield(me_final.Signals{me_index,1},fields_A_me{7});
        send(me_pub,me_msg);
        me_index = me_index + 3;
        me_msg.MeTimestamp = me_final.time_in_sec(me_index);
    end

end
rosshutdown
