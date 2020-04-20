candb_radar = canDatabase("XGU.dbc");
candb_me = canDatabase('ExtLogData2_2.32.dbc');

data_radar = blfread("ACCGapTestMed_20191103_155525_CAN.blf", 1,...
                "Database", candb_radar, "CANStandardFilter", (1285:1658));
data_me = blfread("ACCGapTestMed_20191103_155525_CAN.blf",6,...
                "Database", candb_me,"CANStandardFilter", (1849:1878));
            
timetable_combined = [data_me; data_radar];
tt_ascending = sortrows(timetable_combined);
timetable_length = height(tt_ascending);
time_in_sec = seconds(tt_ascending.Time);

timetable_final = addvars(tt_ascending,time_in_sec);

for i=1:timetable_length
    
end