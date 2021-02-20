rosshutdown
clear
clc
load('./mat files/mobileye.mat');
load('./mat files/radar.mat');

rosinit
me_pub = rospublisher('/Mobileye_CAN_Rx','common/mobileye_object_data');
me_msg = rosmessage(me_pub);
radar_pub = rospublisher('/Radar_One_CAN_Rx','common/radar_object_data');
radar_msg = rosmessage(radar_pub);

radar_index = 0;
radar_obj = 0;
me_index = 0;
me_obj = 0;
clk = 0;
time_interval = 0.01;
radar_msg.radar_timestamp = radar_final.time_in_sec(radar_index);
me_msg.me_timestamp = me_final.time_in_sec(me_index);

while true

    while radar_msg.radar_timestamp > clk + time_interval && me_msg.me_timestamp > clk + time_interval
        clk = clk + time_interval;
    end

    if radar_msg.radar_timestamp < clk + time_interval
        if radar_obj == 31
            radar_obj = 0;
        end
            
        fields_A=fieldnames(radar_final.Signals{radar_index,1});
        fields_B=fieldnames(radar_final.Signals{radar_index+1,1});
        radar_msg.radar_dx(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{7});
        radar_msg.radar_dy(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{6});
        radar_msg.radar_vx(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{5});
        radar_msg.radar_vy(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{8});
        radar_msg.radar_ax(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{4});
        radar_msg.radar_dx_sigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{10});
        radar_msg.radar_dy_sigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{3});
        radar_msg.radar_vx_sigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{9});
        radar_msg.radar_ax_sigma(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{11});
       
        radar_msg.radar_w_exist(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{3});
        radar_msg.radar_w_obstacle(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{1});
        radar_msg.radar_flag_valid(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{8});
        radar_msg.radar_w_non_obstacle(radar_obj) = radar_final.Signals{radar_index,1}.(fields_B{2});
        radar_msg.flag_meas(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{10});
        radar_msg.flag_hist(radar_obj) = radar_final.Signals{radar_index,1}.(fields_A{9});
        radar_msg.d_length(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{4});
        radar_msg.radar_dz(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{6});
        radar_msg.moving_state(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{7});        
        radar_msg.radar_w_class(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{1});
        radar_msg.radar_obj_class(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{5});
        radar_msg.dx_rear_loss(radar_obj) = radar_final.Signals{radar_index+1,1}.(fields_B{2});        
        
        send(radar_pub,radar_msg);
        radar_index = radar_index + 2;
        radar_obj = radar_obj + 1;
        radar_msg.radar_timestamp = radar_final.time_in_sec(radar_index);
    end

    if me_msg.me_timestamp < clk + time_interval
        if me_obj == 9
            me_obj = 0;
        end
        
        fields_A_me=fieldnames(me_final.Signals{me_index,1});
        fields_B_me=fieldnames(me_final.Signals{me_index+1,1});
        fields_C_me=fieldnames(me_final.Signals{me_index+2,1});

        me_msg.me_dx(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{9});
        me_msg.me_dy(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{8});
        me_msg.me_vx(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{7});
        me_msg.me_ax(me_obj) = me_final.Signals{me_index+2,1}.(fields_C_me{1});
        me_msg.me_type(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{6});
        me_msg.me_status(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{5});
        me_msg.me_valid(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{3});
        me_msg.me_cut_in_cut_out(me_obj) = me_final.Signals{me_index,1}.(fields_A_me{1});
        me_msg.me_age(me_obj) = me_final.Signals{me_index+1,1}.(fields_B_me{7});
        me_msg.me_lane(me_obj) = me_final.Signals{me_index+1,1}.(fields_B_me{6});
        me_msg.me_cipv_flag(me_obj) = me_final.Signals{me_index+1,1}.(fields_B_me{5});
     
        send(me_pub,me_msg);
        me_index = me_index + 3;
        me_obj = me_obj + 1;
        me_msg.me_timestamp = me_final.time_in_sec(me_index);
    end

end
rosshutdown
