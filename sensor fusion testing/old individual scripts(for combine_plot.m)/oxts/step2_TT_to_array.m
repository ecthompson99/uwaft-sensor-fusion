load('oxtspractise1.mat'); %oxts_final

pos_ID = 1537; %ID of channel that sends lat and long values
head_ID = 1543; %ID of channel that sends the orientation of vehicle

lengthofvector = size(oxts_final1,1);
acclength = 0;

for j = 1:lengthofvector
    if oxts_final1.ID(j) == pos_ID
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
    if oxts_final1.ID(i) == pos_ID
        longitude(index_pos,1) = oxts_final1.Signals{i}.PosLon;
        latitude(index_pos,1) = oxts_final1.Signals{i}.PosLat;
        time(index_pos,1) = oxts_final1.time_in_sec(i);
        index_pos = index_pos + 1;
    elseif oxts_final1.ID(i) == head_ID
        head(index_head,1) = oxts_final1.Signals{i}.AngleHeading;
        index_head = index_head + 1;      
    end
end

%finding the end time
end_time = time(end);

%setting up a coloumn vector with time increments of 0.01 sec
ideal_time = 0:0.01:end_time;
ideal_time = ideal_time';

%initilizing new coloumn vectors which will store interpolated data
ideal_lat = zeros(length(ideal_time),1);
ideal_long = zeros(length(ideal_time),1);
ideal_head = zeros(length(ideal_time),1);

ideal_lat = interp1(time, latitude, ideal_time);
ideal_long = interp1(time, longitude, ideal_time);
ideal_head = interp1(time, head, ideal_time);

save('oxtsData.mat','ideal_lat','ideal_long','ideal_time','ideal_head');



