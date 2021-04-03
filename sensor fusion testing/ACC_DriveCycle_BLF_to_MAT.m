%% Note: before running this script, make sure that empty folders called
% 'BLFs', ,'DBCs', driving_scenarios', and 'mat files' are created in the
% directory where this file is stored.

clear all

% Specify the drivecycle blf in (BLFs) folder
drive_cycle = "uwaft_2020-02-01_12-33-50_314";

% Specify CAN related channels and IDs
CAN_channel = 1;
veh_speed_ID = 1001;
veh_torque_ID = 401;

% Load CAN database
can_db = canDatabase('./DBCs/GlobalAHS_GM_Confidential.dbc');
% Extract necessary CAN messages
blf_speed = blfread(strcat("./BLFs/",drive_cycle,".blf"), CAN_channel,...
                "Database", can_db,"CANStandardFilter", (veh_speed_ID));
% Get time without (sec) as units
time_in_sec = seconds(blf_speed.Time);
% Save to table
blf_speed_final = addvars(blf_speed,time_in_sec);
clear time_in_sec;

blf_torque_request = blfread(strcat("./BLFs/",drive_cycle,".blf"), CAN_channel,...
                "Database", can_db,"CANStandardFilter", (veh_torque_ID));
time_in_sec = seconds(blf_torque_request.Time);
blf_torque_request_final = addvars(blf_torque_request,time_in_sec);
clear time_in_sec;

% Create empty arrays to store extracted CAN signals
speed_extracted = zeros(size(blf_speed_final,1),1);
torque_request_extracted = zeros(size(blf_torque_request_final,1),1);

for i = 1:size(blf_speed_final,1)
    % in km/h
    speed_extracted(i) = blf_speed.Signals{i,1}.VehSpdAvgDrvn;
end

for i = 1:size(blf_torque_request_final,1)
    % Units in Nm
    torque_request_extracted(i) = blf_torque_request.Signals{i,1}.EngActStdyStTorq;
end

% Store time vs signal in new tables and save mat files under 'mat files'
% folder
veh_speed = table(blf_speed_final.time_in_sec, speed_extracted(:,1));
save('./mat files/veh_speed.mat', 'veh_speed');
writetable(veh_speed, strcat("./Excel files/","CAV_veh_speed",".xlsx"),'Sheet',1);

veh_torque_request = table(blf_torque_request_final.time_in_sec, torque_request_extracted(:,1));
%save('./mat files/veh_torque_request.mat', 'veh_torque_request');
%writetable(veh_torque_request, strcat("./Excel files/","CAV_veh_torque_request",".xlsx"),'Sheet',1);

%% Plots the eng torque request
FILE_NAME = './Excel files/CAV_model_torque_request.xlsx';
SHEET = 1;
XL_RANGE = 'A2:B35447';

model_drive_cycle = xlsread(FILE_NAME, SHEET, XL_RANGE);
plot(veh_torque_request.Var1, veh_torque_request.Var2); % in Nm
hold on;
plot(model_drive_cycle(:,1), model_drive_cycle(:,2)); %in Nm

legend('Drive Cycle Output','Model Output');
xlabel('Time (s)') ;
ylabel('Torque Request (Nm)') ;
title('Engine Torque Request for Drive Cycle');
grid on;

