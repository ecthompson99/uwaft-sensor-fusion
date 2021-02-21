clear all

% Load DBCs
candb_radar = canDatabase("./DBCs/XGU.dbc");
candb_me = canDatabase('./DBCs/ext_log_data - Copy.dbc');
candb_veh = canDatabase('./DBCs/GlobalAHS_GM_Confidential.dbc');

% Front radar is on channel 3
data_radar = blfread("./BLFs/WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54.blf", 3,...
                "Database", candb_radar, "CANStandardFilter", (1285:1598));
time_in_sec = seconds(data_radar.Time);
radar_final = addvars(data_radar,time_in_sec);
writetimetable(radar_final,'./Excel files/WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54.xlsx','Sheet',1);

clear time_in_sec;
save('./mat files/radar.mat');

% Mobileye is on channel 2 
data_me = blfread("./BLFs/WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54.blf", 2,...
                "Database", candb_me,"CANStandardFilter", (1849:1851));
time_in_sec = seconds(data_me.Time);
me_final = addvars(data_me,time_in_sec);

clear time_in_sec;
save('./mat files/mobileye.mat');
writetimetable(me_final,'./Excel files/WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54.xlsx','Sheet',2);

% Vehicle data is on channel 4
data_veh = blfread("./BLFs/WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54.blf", 4,...
                "Database", candb_veh,"CANStandardFilter", (1001));
time_in_sec = seconds(data_veh.Time);
veh_final = addvars(data_veh,time_in_sec);

clear time_in_sec;
save('./mat files/vehicle.mat');
writetimetable(veh_final,'./Excel files/WVE_Approach_Test_20mph_test3006_2021-02-18_21-39-54.xlsx','Sheet',3);

% clear data
clear candb_radar;
clear data_radar;
clear candb_me;
clear data_me;
clear candb_veh;
clear data_veh;