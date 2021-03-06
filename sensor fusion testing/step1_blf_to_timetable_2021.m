clear all

drive_cycle = "WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54";

% Front radar is on channel 3
candb_radar = canDatabase("./DBCs/XGU.dbc");
data_radar = blfread(strcat("./BLFs/",drive_cycle,".blf"), 3,...
                "Database", candb_radar, "CANStandardFilter", (1285:1598));
time_in_sec = seconds(data_radar.Time);
radar_final = addvars(data_radar,time_in_sec);

clear time_in_sec;
clear candb_radar;
clear data_radar;
save('./mat files/radar.mat');
writetimetable(radar_final,strcat("./Excel files/",drive_cycle,".xlsx"),'Sheet',1);
clear radar_final;

% Mobileye is on channel 2 
candb_me = canDatabase('./DBCs/ext_log_data - Copy.dbc');
data_me = blfread(strcat("./BLFs/",drive_cycle,".blf"), 2,...
                "Database", candb_me,"CANStandardFilter", (1849:1851));
time_in_sec = seconds(data_me.Time);
me_final = addvars(data_me,time_in_sec);

clear time_in_sec;
clear candb_me;
clear data_me;
save('./mat files/mobileye.mat');
writetimetable(me_final, strcat("./Excel files/",drive_cycle,".xlsx"),'Sheet',2);
clear me_final;

% Vehicle data is on channel 4
candb_veh = canDatabase('./DBCs/GlobalAHS_GM_Confidential.dbc');
data_veh = blfread(strcat("./BLFs/",drive_cycle,".blf"), 4,...
                "Database", candb_veh,"CANStandardFilter", (1001));
time_in_sec = seconds(data_veh.Time);
veh_final = addvars(data_veh,time_in_sec);

clear time_in_sec;
clear candb_veh;
clear data_veh;
save('./mat files/vehicle.mat');
writetimetable(veh_final, strcat("./Excel files/",drive_cycle,".xlsx"),'Sheet',3);
clear veh_final;