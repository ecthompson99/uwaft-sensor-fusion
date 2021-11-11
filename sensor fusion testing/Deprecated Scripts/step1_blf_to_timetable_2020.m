candb_radar = canDatabase("XGU.dbc");

data_radar = blfread("ACCGapTestMed_20191103_155525_CAN.blf", 3,...
                "Database", candb_radar, "CANStandardFilter", (1285:1658));
time_in_sec = seconds(data_radar.Time);
radar_final = addvars(data_radar,time_in_sec);
writetimetable(radar_final,'testing.xlsx','Sheet',1);

clear candb_radar;
clear data_radar;
clear time_in_sec;
save('radar.mat');
clear radar_final;

candb_me = canDatabase('ExtLogData2_2.32.dbc');
data_me = blfread("ACCGapTestMed_20191103_155525_CAN.blf",6,...
                "Database", candb_me,"CANStandardFilter", (1849:1878));
time_in_sec = seconds(data_me.Time);
me_final = addvars(data_me,time_in_sec);
clear candb_me;
clear data_me;
clear time_in_sec;
save('mobileye.mat');
writetimetable(me_final,'testing.xlsx','Sheet',2);

