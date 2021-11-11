%% Script to decode BLF drive cycle data and generate radar, mobileye and vehicle data CSVs
% CSVs used in ROS node "blf_sensor_data_matlab" to generate a bag file
% with the sensor data

clear
clc

% Name of BLF file located in BLFs folder
drive_cycle = "UWAFT_EMC Y3_ScenarioA_15_2021-05-29_17-34-21";

% Front radar is on channel 2
candb_radar = canDatabase("../DBCs/XGU.dbc");
data_radar = blfread(strcat("../BLFs/",drive_cycle,".blf"), 2,...
                "Database", candb_radar, "CANStandardFilter", (1285:1596));
time_in_sec = seconds(data_radar.Time);
radar_final = addvars(data_radar,time_in_sec);

clear time_in_sec;
clear candb_radar;
clear data_radar;
writetimetable(radar_final,strcat("../Excel Files/",drive_cycle,".xlsx"),'Sheet',1);

% Mobileye is on channel 4
candb_me = canDatabase('../DBCs/ext_log_data.dbc');
data_me = blfread(strcat("../BLFs/",drive_cycle,".blf"), 4,...
                "Database", candb_me,"CANStandardFilter", (1849:1851));
time_in_sec = seconds(data_me.Time);
me_final = addvars(data_me,time_in_sec);

clear time_in_sec;
clear candb_me;
clear data_me;
writetimetable(me_final, strcat("../Excel Files/",drive_cycle,".xlsx"),'Sheet',2);

% Vehicle data is on channel 1
candb_veh = canDatabase('../DBCs/EMC_PCM_CAV_Interface_UWAFT.dbc');
data_veh = blfread(strcat("../BLFs/",drive_cycle,".blf"), 1,...
                "Database", candb_veh,"CANStandardFilter", (1072:1074));
time_in_sec = seconds(data_veh.Time);
veh_final = addvars(data_veh,time_in_sec);

clear time_in_sec;
clear candb_veh;
clear data_veh;
writetimetable(veh_final, strcat("../Excel Files/",drive_cycle,".xlsx"),'Sheet',3);

%% Export sensor data to csvs (located in "Excel files" folder) to use in ROS node that generates rosbag

% Index of first object 
%(OBJ00 for radar, ObstacleDataA for mobileye, PcmToCav_1 for vehicle data)
veh_index = 1;
radar_index = 1;
me_index = 1;

for i = radar_index:64:(height(radar_final) - 64)
    time = radar_final.time_in_sec(i);
    radar_entry = zeros(1, 32*21); % 32 objects with 21 data values each
    
    for j = 0:2:62
        obj = j/2;
        fields_A = fieldnames(radar_final.Signals{i+j, 1});
        fields_B = fieldnames(radar_final.Signals{i+j+1, 1});
        
        radar_entry(1, 1 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{7}); % radar_dx
        radar_entry(1, 2 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{6}); % radar_dy
        radar_entry(1, 3 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{5}); % radar_vx
        radar_entry(1, 4 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{8}); % radar_vy
        radar_entry(1, 5 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{4}); % radar_ax
        radar_entry(1, 6 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{10}); % radar_dx_sigma
        radar_entry(1, 7 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{3}); % radar_dy_sigma
        radar_entry(1, 8 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{9}); % radar_vx_sigma
        radar_entry(1, 9 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{11}); % radar_ax_sigma
        radar_entry(1, 10 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{3}); % radar_W_exist
        radar_entry(1, 11 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{1}); % radar_W_obstacle
        radar_entry(1, 12 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{8}); % radar_flag_valid
        radar_entry(1, 13 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{2}); % radar_W_non_obstacle
        radar_entry(1, 14 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{10}); % flag_meas
        radar_entry(1, 15 + 21*obj) = radar_final.Signals{i+j,1}.(fields_A{9}); % flag_history
        radar_entry(1, 16 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{4}); % d_length
        radar_entry(1, 17 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{6}); % radar_dz
Step 5 (Run the rosbag with sensor fusion + ACC) :        radar_entry(1, 18 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{7}); % moving_state        
        radar_entry(1, 19 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{1}); % radar_w_class
        radar_entry(1, 20 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{5}); % radar_obj_class
        radar_entry(1, 21 + 21*obj) = radar_final.Signals{i+j+1,1}.(fields_B{2}); % radar_dx_near_loss
    end
    
    data_output_ = {time, radar_entry};
                
    writecell(data_output_, strcat("../Excel Files/",drive_cycle,"_radar.csv"), "WriteMode", "append");
    
end

for i = me_index:3:(height(me_final) - 3)
    time = me_final.time_in_sec(i);
    me_entry = zeros(1, 1*11); % 1 object(s) with 11 data values each
    
    for j = 0:3:1 % 0:3:27 for 10 objects
        obj = j/2;
        fields_A_me=fieldnames(me_final.Signals{i+j,1});
        fields_B_me=fieldnames(me_final.Signals{i+j+1,1});
        fields_C_me=fieldnames(me_final.Signals{i+j+2,1});
        
        me_entry(1, 1 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{9}); % me_dx
        me_entry(1, 2 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{8}); % me_dy
        me_entry(1, 3 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{7}); % me_vx
        me_entry(1, 4 + 11*obj) = me_final.Signals{i+j+2,1}.(fields_C_me{1}); % me_ax
        me_entry(1, 5 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{6}); % me_type
        me_entry(1, 6 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{5}); % me_status
        me_entry(1, 7 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{3}); % me_valid
        me_entry(1, 8 + 11*obj) = me_final.Signals{i+j,1}.(fields_A_me{1}); % me_cut_in_cut_out
        me_entry(1, 9 + 11*obj) = me_final.Signals{i+j+1,1}.(fields_B_me{7}); % me_age
        me_entry(1, 10 + 11*obj) = me_final.Signals{i+j+1,1}.(fields_B_me{6}); % me_lane
        me_entry(1, 11 + 11*obj) = me_final.Signals{i+j+1,1}.(fields_B_me{5}); % me_cipv_flag
    end
    
    data_output_ = {time, me_entry};
                
    writecell(data_output_, strcat("../Excel Files/",drive_cycle,"_mobileye.csv"), "WriteMode", "append");
    
end

for i = veh_index:3:(height(veh_final) - 3)
    time = veh_final.time_in_sec(i);
    veh_entry = zeros(1, 1*5);
    
    fields_1_veh=fieldnames(veh_final.Signals{i,1});
    fields_2_veh=fieldnames(veh_final.Signals{i+1,1});
    fields_3_veh=fieldnames(veh_final.Signals{i+2,1});
    
    veh_entry(1,1) = veh_final.Signals{i,1}.(fields_1_veh{1}); % vehicle speed     
    veh_entry(1,2) = veh_final.Signals{i,1}.(fields_1_veh{3}); % acc allowed
    veh_entry(1,3) = veh_final.Signals{i,1}.(fields_1_veh{5}); % steering angle
    veh_entry(1,4) = veh_final.Signals{i+1,1}.(fields_2_veh{1}); % set speed
    veh_entry(1,5) = veh_final.Signals{i+1,1}.(fields_2_veh{2}); % acc active
    data_output_ = {time, veh_entry};
                
    writecell(data_output_, strcat("../Excel Files/",drive_cycle,"_vehicle.csv"), "WriteMode", "append");
    
end