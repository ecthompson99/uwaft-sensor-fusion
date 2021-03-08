%% This part converts the timetable into readable table with lat,long,heading
%as individual columns
clear all;
load('./mat files/oxts_TT.mat');

pos_ID = 1537; %ID of channel that sends lat and long values
head_ID = 1543; %ID of channel that sends the orientation of vehicle

lengthofvector = size(oxts_final,1);
acclength = 0;

for j = 1:lengthofvector
    if oxts_final.ID(j) == pos_ID
        acclength = acclength +1;
    end
end
    
%initializing three column vectors
longitude = zeros(acclength,1);
latitude = zeros(acclength,1);
head = zeros(acclength,1); %head refers to the direction of the vehicle where
%North is zero degrees
time = zeros(acclength,1);
index_pos = 1;
index_head = 1;

%filling up the the three column vectors with data from oxts timetable
for i = 1:lengthofvector
    if oxts_final.ID(i) == pos_ID
        longitude(index_pos,1) = oxts_final.Signals{i}.PosLon;
        latitude(index_pos,1) = oxts_final.Signals{i}.PosLat;
        time(index_pos,1) = oxts_final.time_in_sec(i);
        index_pos = index_pos + 1;
    elseif oxts_final.ID(i) == head_ID
        head(index_head,1) = oxts_final.Signals{i}.AngleHeading;
        index_head = index_head + 1;      
    end
end
oxts_matrix = [time, latitude, longitude, head]; %adding all the columns together
oxts_table = array2table(oxts_matrix); %converting the matrix to a table

%naming all the columns of the table
oxts_table.Properties.VariableNames(1) = "Time";
oxts_table.Properties.VariableNames(2) = "Latitude";
oxts_table.Properties.VariableNames(3) = "Longitude";
oxts_table.Properties.VariableNames(4) = "Heading";

save('./mat files/oxts_T','oxts_matrix','acclength')
