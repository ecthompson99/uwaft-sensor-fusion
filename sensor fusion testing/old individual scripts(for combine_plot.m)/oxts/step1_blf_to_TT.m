format long g

%for oxts
candb_oxts = canDatabase("oxts_v2.dbc");

%first argument: is the blf file
%second argument: is the channel number(column labeled "Chn" in canalyzer
%thrid argument: this Name1 for the dbc file
%fourth argument: this Value1 for Name1 and is the actual dbc file
%fifth argument: Name2 is for message standard ID's
%sixth argument:Value1 
data_oxts = blfread("WVE_Approach_Test_10mph_001_2021-02-18_21-13-44.blf",5,...
                "Database", candb_oxts,"CANStandardFilter", [1537 1543]);
time_in_sec = seconds(data_oxts.Time);
oxts_final1 = addvars(data_oxts,time_in_sec);

writetimetable(oxts_final1,'practise1.xlsx','Sheet',1);

clear candb_oxts;
clear data_oxts;
clear time_in_sec;
save('oxtspractise1.mat');
clear oxts_final1;
