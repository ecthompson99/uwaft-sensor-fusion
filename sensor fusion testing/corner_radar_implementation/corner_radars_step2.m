rosshutdown
clear
clc
load('right_radar.mat');
load('left_radar.mat');

rosinit
left_radar_pub = rospublisher('/left_radar_from_matlab','sensor_fusion_testing/left_radar_object_data_from_matlab');
right_radar_pub = rospublisher('/right_radar_from_matlab','sensor_fusion_testing/right_radar_object_data_from_matlab');
left_radar_msg = rosmessage(left_radar_pub);
right_radar_msg = rosmessage(right_radar_pub);

left_index = 1;
right_index = 1;
clk = 0;
pub_rate = 0.001;
left_radar_msg.RadarTimestamp = left_radar.time_in_sec(left_index);
right_radar_msg.RadarTimestamp = right_radar.time_in_sec(right_index);

while true

    while left_radar_msg.RadarTimestamp > clk + pub_rate && right_radar_msg.RadarTimestamp > clk + pub_rate
        clk = clk + pub_rate;
    end

    if left_radar_msg.RadarTimestamp < clk + pub_rate
        fields_A=fieldnames(left_radar.Signals{left_index,1});
        fields_B=fieldnames(left_radar.Signals{left_index+1,1});
        
        left_radar_msg.RadarDx = getfield(left_radar.Signals{left_index,1},fields_A{7});
        left_radar_msg.RadarDy = getfield(left_radar.Signals{left_index,1},fields_A{6});
        left_radar_msg.RadarVx = getfield(left_radar.Signals{left_index,1},fields_A{5});
        left_radar_msg.RadarVy = getfield(left_radar.Signals{left_index+1,1},fields_B{8});
        left_radar_msg.RadarAx = getfield(left_radar.Signals{left_index,1},fields_A{4});
        left_radar_msg.RadarDxSigma = getfield(left_radar.Signals{left_index+1,1},fields_B{10});
        left_radar_msg.RadarDySigma = getfield(left_radar.Signals{left_index+1,1},fields_B{3});
        left_radar_msg.RadarVxSigma = getfield(left_radar.Signals{left_index+1,1},fields_B{9});
        left_radar_msg.RadarAxSigma = getfield(left_radar.Signals{left_index+1,1},fields_B{11});
        send(left_radar_pub,left_radar_msg);
        left_index = left_index + 2;
        left_radar_msg.RadarTimestamp = left_radar.time_in_sec(left_index);
    end

    if right_radar_msg.RadarTimestamp < clk + pub_rate
        fields_A=fieldnames(right_radar.Signals{right_index,1});
        fields_B=fieldnames(right_radar.Signals{right_index+1,1});
        
        right_radar_msg.RadarDx = getfield(right_radar.Signals{right_index,1},fields_A{7});
        right_radar_msg.RadarDy = getfield(right_radar.Signals{right_index,1},fields_A{6});
        right_radar_msg.RadarVx = getfield(right_radar.Signals{right_index,1},fields_A{5});
        right_radar_msg.RadarVy = getfield(right_radar.Signals{right_index+1,1},fields_B{8});
        right_radar_msg.RadarAx = getfield(right_radar.Signals{right_index,1},fields_A{4});
        right_radar_msg.RadarDxSigma = getfield(right_radar.Signals{right_index+1,1},fields_B{10});
        right_radar_msg.RadarDySigma = getfield(right_radar.Signals{right_index+1,1},fields_B{3});
        right_radar_msg.RadarVxSigma = getfield(right_radar.Signals{right_index+1,1},fields_B{9});
        right_radar_msg.RadarAxSigma = getfield(right_radar.Signals{right_index+1,1},fields_B{11});
        send(right_radar_pub,right_radar_msg);
        right_index = right_index + 2;
        right_radar_msg.RadarTimestamp = right_radar.time_in_sec(right_index);
    end

end
rosshutdown




