clear
clc

% import CAN corner radar database
candb_cr = canDatabase("Bosch_XGU_CornerRadar.dbc");

% filter for left radar input and desired signals
left_radar = blfread("bve_testing_2019-11-08_13-25-36.blf",1,...
    "Database", candb_cr, "CANStandardFilter", (1285:1655));

% filter for right radar input and desired signals
right_radar = blfread("bve_testing_2019-11-08_13-25-36.blf",2,...
    "Database", candb_cr, "CANStandardFilter", (1287:1657));

left_time_sec = seconds(left_radar.Time);
right_time_sec = seconds(right_radar.Time);
left_final = addvars(left_radar,left_time_sec);
right_final = addvars(right_radar,right_time_sec);

writetimetable(left_final,'left_corner_radar.xlsx','Sheet',1);
writetimetable(right_final,'right_corner_radar.xlsx','Sheet',1);

save('left_radar.mat');
save('right_radar.mat');