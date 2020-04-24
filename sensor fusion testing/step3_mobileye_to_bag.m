rosshutdown
clear
clc
load('~/kaiROS/sensor fusion testing/mobileye.mat');

rosinit
me_pub = rospublisher('/mobileye_from_matlab','sensor_fusion_testing/mobileye_object_data_from_matlab');
me_msg = rosmessage(me_pub);

timetable_height=height(me_final);
if mod(me_final.ID(1)-1848,3) == 1
    start_index = 1;
elseif mod(me_final.ID(1)-1848,3) == 2
    start_index = 3;
else 
    start_index = 2;    
end

if mod(me_final.ID(timetable_height)-1848,3) == 0
    end_index = timetable_height;
elseif mod(me_final.ID(timetable_height)-1848,3) == 1
    end_index = timetable_height-1;
else
    end_index = timetable_height-2;
end

for i = start_index:3:9000
    input('uhh');
    fields_A=fieldnames(me_final.Signals{i,1});
    me_msg.MeDx = getfield(me_final.Signals{i,1},fields_A{9});
    me_msg.MeDy = getfield(me_final.Signals{i,1},fields_A{8});
    me_msg.MeVx = getfield(me_final.Signals{i,1},fields_A{7});
    me_msg.MeTimestamp = me_final.time_in_sec(i);
    send(me_pub,me_msg);
end
rosshutdown