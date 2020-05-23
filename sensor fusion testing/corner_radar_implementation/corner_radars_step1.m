clear
clc

% import CAN corner radar database
candb_cr = canDatabase("Bosch_XGU_CornerRadar.dbc");

% filter for left radar input and desired signals
left_radar = blfread("bve_testing_2019-11-08_13-25-36.blf",1,...
    "Database", candb_cr, "CANStandardFilter", (1285:1655));

% filter for right radar input and desired signals
right_radar = blfread("bve_testing_2019-11-08_13-25-36.blf",2,...
    "Database", candb_cr, "CANStandardFilter", (1285:1655));

left_time_sec = seconds(left_radar.Time);
right_time_sec = seconds(right_radar.Time);
left_final = addvars(left_radar,left_time_sec);
right_final = addvars(right_radar,right_time_sec);


%% Check validity of CAN frames

size_left_final = size(left_final);

for i = 1:size_left_final(1)
    % check A
    if i < size_left_final(1)
        if contains(left_final.Name(i),'_A')
           if contains(left_final.Name(i+1),'_A') && ...
               contains(left_final.Name(i-1),'_B')
                left_final(i,:) = []; % remove row
                size_left_final = size_left_final - 1; % reduce size by 1
           end
        end
    end
end

size_right_final = size(right_final);

for i = 1:size_right_final(1)
    % check A
    if i < size_right_final(1)
        if contains(right_final.Name(i),'_A')
           if contains(right_final.Name(i+1),'_A') && ...
               contains(right_final.Name(i-1),'_B')
                right_final(i,:) = []; % remove row
                size_right_final = size_right_final - 1; % reduce size by 1
           end
        end
    end
end


%% Save data    
writetimetable(left_final,'left_corner_radar.xlsx','Sheet',1);
writetimetable(right_final,'right_corner_radar.xlsx','Sheet',1);

save('left_radar.mat');
save('right_radar.mat');