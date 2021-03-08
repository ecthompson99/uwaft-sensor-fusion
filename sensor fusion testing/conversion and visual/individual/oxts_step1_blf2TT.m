%% STEP1_BLF_TO_TT_OXTS

%%This script converts the blf file into a timetable with each signal in a
%struct
clear all;
format long g

candb_oxts = canDatabase("./input files/oxts_v2.dbc");

%first argument: is the blf file
%second argument: is the channel number(column labeled "Chn" in canalyzer
%thrid argument: this Name1 for the dbc file
%fourth argument: this Value1 for Name1 and is the actual dbc file
%fifth argument: Name2 is for message standard ID's
%sixth argument:Value1 
data_oxts = blfread("./input files/WVE_Approach_Test_10mph_test2002_2021-02-18_21-31-15.blf",5,...
                "Database", candb_oxts,"CANStandardFilter", [1537 1543]);
time_in_sec = seconds(data_oxts.Time);
oxts_final = addvars(data_oxts,time_in_sec);

% writetimetable(oxts_final1,'practise1.xlsx','Sheet',1);


save('./mat files/oxts_TT', 'oxts_final');