rosshutdown
clear
clc
load('right_radar.mat');
load('left_radar.mat');

rosinit
left_radar_pub = rospublisher('/left_radar_from_matlab','sensor_fusion_testing/radar_object_data_from_matlab');
right_radar_pub = rospublisher('/right_radar_from_matlab','sensor_fusion_testing/radar_object_data_from_matlab');
left_radar_msg = rosmessage(left_radar_pub);
right_radar_msg = rosmessage(right_radar_pub);

left_index = 1;
right_index = 1;
clk = 0;
pub_rate = 0.001;
left_radar_msg.RadarTimestamp = left_final.left_time_sec(left_index);
right_radar_msg.RadarTimestamp = right_final.right_time_sec(right_index);

while true

    while left_radar_msg.RadarTimestamp > clk + pub_rate && right_radar_msg.RadarTimestamp > clk + pub_rate
        clk = clk + pub_rate;
    end

    if left_radar_msg.RadarTimestamp < clk + pub_rate
        fields_A=fieldnames(left_final.Signals{left_index,1});
        fields_B=fieldnames(left_final.Signals{left_index+1,1});
        
        left_radar_msg.RadarDx = getfield(left_final.Signals{left_index,1},fields_A{7});
        left_radar_msg.RadarDy = getfield(left_final.Signals{left_index,1},fields_A{6});
        left_radar_msg.RadarVx = getfield(left_final.Signals{left_index,1},fields_A{5});
        left_radar_msg.RadarVy = getfield(left_final.Signals{left_index+1,1},fields_B{8});
        left_radar_msg.RadarAx = getfield(left_final.Signals{left_index,1},fields_A{4});
        left_radar_msg.RadarDxSigma = getfield(left_final.Signals{left_index+1,1},fields_B{10});
        left_radar_msg.RadarDySigma = getfield(left_final.Signals{left_index+1,1},fields_B{3});
        left_radar_msg.RadarVxSigma = getfield(left_final.Signals{left_index+1,1},fields_B{9});
        left_radar_msg.RadarAxSigma = getfield(left_final.Signals{left_index+1,1},fields_B{11});
        send(left_radar_pub,left_radar_msg);
        left_index = left_index + 2;
        left_radar_msg.RadarTimestamp = left_final.left_time_sec(left_index);
    end

    if right_radar_msg.RadarTimestamp < clk + pub_rate
        fields_A=fieldnames(right_final.Signals{right_index,1});
        fields_B=fieldnames(right_final.Signals{right_index+1,1});
        
        right_radar_msg.RadarDx = getfield(right_final.Signals{right_index,1},fields_A{7});
        right_radar_msg.RadarDy = getfield(right_final.Signals{right_index,1},fields_A{6});
        right_radar_msg.RadarVx = getfield(right_final.Signals{right_index,1},fields_A{5});
        right_radar_msg.RadarVy = getfield(right_final.Signals{right_index+1,1},fields_B{8});
        right_radar_msg.RadarAx = getfield(right_final.Signals{right_index,1},fields_A{4});
        right_radar_msg.RadarDxSigma = getfield(right_final.Signals{right_index+1,1},fields_B{10});
        right_radar_msg.RadarDySigma = getfield(right_final.Signals{right_index+1,1},fields_B{3});
        right_radar_msg.RadarVxSigma = getfield(right_final.Signals{right_index+1,1},fields_B{9});
        right_radar_msg.RadarAxSigma = getfield(right_final.Signals{right_index+1,1},fields_B{11});
        send(right_radar_pub,right_radar_msg);
        right_index = right_index + 2;
        right_radar_msg.RadarTimestamp = right_final.right_time_sec(right_index);
    end

end
rosshutdown







