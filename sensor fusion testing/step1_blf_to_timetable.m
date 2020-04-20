candb_radar = canDatabase("XGU.dbc");
candb_me = canDatabase('ExtLogData2_2.32.dbc');

data_radar = blfread("ACCGapTestMed_20191103_155525_CAN.blf", 1,...
                "Database", candb_radar, "CANStandardFilter", (1285:1658));
data_me = blfread("ACCGapTestMed_20191103_155525_CAN.blf",6,...
                "Database", candb_me,"CANStandardFilter", (1849:1878));
            
timetable_combined = [data_me; data_radar];
tt_ascending = sortrows(timetable_combined);
time_in_sec = seconds(tt_ascending.Time);

timetable_final = addvars(tt_ascending,time_in_sec);
timetable_length = height(timetable_final);

clear candb_radar;
clear candb_me;
clear data_radar;
clear data_me;
clear timetable_combined;
clear tt_ascending;
clear time_in_sec;

save('ACCGapTestMed_20191103_155525_CAN.mat');
