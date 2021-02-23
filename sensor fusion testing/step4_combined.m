%rosshutdown
clear
clc
load('./mat files/mobileye.mat');
load('./mat files/radar.mat');
load('./mat files/vehicle.mat');

rosinit
me_pub = rospublisher('/Mobileye_CAN_Rx','common/mobileye_object_data');
me_msg = rosmessage(me_pub);
radar_pub = rospublisher('/Radar_One_CAN_Rx','common/radar_object_data');
radar_msg = rosmessage(radar_pub);
veh_pub = rospublisher('/Radar_One_CAN_Rx','common/radar_object_data');
veh_msg = rosmessage(veh_pub);

% array index start from 1
veh_index = 1;
radar_index = 1;
radar_obj = 1;
me_index = 1;
me_obj = 1;
clk = 0;
time_interval = 0.01; % publish rate
radar_msg.RadarTimestamp = radar_final.time_in_sec(radar_index);
me_msg.MeTimestamp = me_final.time_in_sec(me_index);


while true

    while radar_msg.RadarTimestamp > clk + time_interval && me_msg.MeTimestamp > clk + time_interval ...
            && veh_final.time_in_sec(veh_index) > clk + time_interval
        clk = clk + time_interval;
    end

    if radar_msg.RadarTimestamp < clk + time_interval
        if radar_obj > 32
            radar_obj = 1;
        end
            
        fields_A=fieldnames(radar_final.Signals{radar_index,1});
        fields_B=fieldnames(radar_final.Signals{radar_index+1,1});
        radar_msg.RadarDx(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{7});
        radar_msg.RadarDy(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{6});
        radar_msg.RadarVx(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{5});
        radar_msg.RadarVy(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{8});
        radar_msg.RadarAx(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{4});
        radar_msg.RadarDxSigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{10});
        radar_msg.RadarDySigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{3});
        radar_msg.RadarVxSigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{9});
        radar_msg.RadarAxSigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{11});
        radar_msg.RadarWExist(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{3});
        radar_msg.RadarWObstacle(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{1});
        radar_msg.RadarFlagValid(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{8});
        radar_msg.RadarWNonObstacle(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{2});
        radar_msg.FlagMeas(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{10});
        radar_msg.FlagHist(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{9});
        radar_msg.DLength(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{4});
        radar_msg.RadarDz(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{6});
        radar_msg.MovingState(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{7});        
        radar_msg.RadarWClass(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{1});
        radar_msg.RadarObjClass(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{5});
        radar_msg.DxRearLoss(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{2});        
        
        send(radar_pub,radar_msg);
        radar_index = radar_index + 2;
        radar_obj = radar_obj + 1;
        
        radar_msg.RadarTimestamp = radar_final.time_in_sec(radar_index);
    end
    
    if veh_final.time_in_sec(veh_index) < clk + time_interval
        % 1000/3600 is conversion from km/h to m/s (match radar velocity
        % units)
        veh_msg.VehVEgo = veh_final.Signals{veh_index,1}.VehSpdAvgDrvn*1000/3600; 
        send(veh_pub,veh_msg);
        veh_index = veh_index + 1;
    end        

    if me_msg.MeTimestamp < clk + time_interval
        if me_obj > 10
            me_obj = 1;
        end
        
        fields_A_me=fieldnames(me_final.Signals{me_index,1});
        fields_B_me=fieldnames(me_final.Signals{me_index+1,1});
        fields_C_me=fieldnames(me_final.Signals{me_index+2,1});

        me_msg.MeDx(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{9});
        me_msg.MeDy(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{8});
        me_msg.MeVx(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{7});
        me_msg.MeAx(me_obj) = me_final.Signals{me_index+2,1}.(fields_C_me{1});
        me_msg.MeType(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{6});
        me_msg.MeStatus(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{5});
        me_msg.MeValid(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{3});
        me_msg.MeCutInCutOut(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{1});
        me_msg.MeAge(me_obj) = me_final.Signals{me_index+1,1}.(fields_B_me{7});
        me_msg.MeLane(me_obj) = me_final.Signals{me_index+1,1}.(fields_B_me{6});
        me_msg.MeCipvFlag(me_obj) = me_final.Signals{me_index+1,1}.(fields_B_me{5});
     
        send(me_pub,me_msg);
        me_index = me_index + 3;
        me_obj = me_obj + 1;
        
        me_msg.MeTimestamp = me_final.time_in_sec(me_index);
    end

end
%rosshutdown
