%This script takes in a blf file and plots the data in a birdseyePlot all
%in one go.
%This scripts plot objects so that they are relative to the car, meaning
%the forward direction on birdseyePlot is always the direction the car is
%facing.
%It also generates the data required to for range_plot.m for graphs on
%range vs time and range_rate vs time.


%This part converts the blf file into a timetable with each signal in a
%struct
format long g

%for oxts
candb_oxts = canDatabase("oxts_v2.dbc");

%first argument: is the blf file
%second argument: is the channel number(column labeled "Chn" in canalyzer
%thrid argument: this Name1 for the dbc file
%fourth argument: this Value1 for Name1 and is the actual dbc file
%fifth argument: Name2 is for message standard ID's
%sixth argument:Value1 
data_oxts = blfread("WVE_Approach_Test_10mph_test2002_2021-02-18_21-31-15.blf",5,...
                "Database", candb_oxts,"CANStandardFilter", [1537 1543]);
time_in_sec = seconds(data_oxts.Time);
oxts_final1 = addvars(data_oxts,time_in_sec);

writetimetable(oxts_final1,'practise1.xlsx','Sheet',1);

clear candb_oxts;
clear data_oxts;
clear time_in_sec;

%--------------------------------------------------------

%This part converts the timetable into readable individual column vectors
%for each of the values(lat, long, head, time)

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
range = zeros(length(ideal_time),1);

ideal_lat = interp1(time, latitude, ideal_time);
ideal_long = interp1(time, longitude, ideal_time);
ideal_head = interp1(time, head, ideal_time);

clear acclength;
clear end_time; 
clear head;
clear head_ID;
clear i;
clear index_head; 
clear index_pos;
clear j;
clear latitude;
clear lengthofvector;
clear longitude;
clear oxts_final1;
clear pos_ID;
clear time;

%----------------------------------------------------------------
%This part acctually converts the values realative to the vehicle and plots
close all

%% Plot objects in video
%load('simulation_gt.mat'); %ground_truth
%load('sf_output.mat'); %sf_results
set(0,'DefaultFigureVisible','on');

%FOR OXTS
%43.473208
%-80.540375

%for stationary
%43.474323
%-80.538345

%conversion rates are for when located at latitude 43.4741
%using the calculator http://www.csgnetwork.com/degreelenllavcalc.html
lat2m = 111111.9478949894;
lon2m = 80213.90374;

% quad1 = 0:89;
% quad2 = 90:179;
% quad3 = 180:269;
% quad4 = 270:359;
% zero = 0;

%start_lat = input("Enter the latitude of the starting position: ");
%start_long = input("Enter the longitude of the starting position: ");
% y_pos = input("Enter the latitude of the stationary vehicle: ");
% x_pos = input("Enter the longitude of the stationary vehicle: ");

start_lat = ideal_lat(2);
start_long = ideal_long(2);

newVid = VideoWriter('tracked_objects_fr102.avi');
newVid.FrameRate = 10;
newVid.Quality = 10;
open(newVid);
gt_offset = 6;


bep = birdsEyePlot('XLim',[0,110],'YLim',[-10,10]);
%blazerPlotter = detectionPlotter(bep,'MarkerEdgeColor','black','DisplayName','Ground Truth Objects', 'Marker', 'o');
%sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');
oxtsPlotter = detectionPlotter(bep,'MarkerEdgeColor','blue','DisplayName','Stationary Vehicle', 'Marker', 'square');

%dist_y = dist_y*(1/lat2m); %convert distance into lat degrees
%dist_x = dist_x*(1/lon2m); %convert distance into long degrees   
y_pos = 43.47415130; %position of stationary vehicle in lat
x_pos = -80.53892280; %position of stationary vehicle in long



for i = 2:length(ideal_time) % sample plot for now (start:iteration:end)
    
    %OXTS stuff -----------
    
    %distance between ego and stationary vehicle in lat long degrees
    y = y_pos - ideal_lat(i); %ideal_lat is the latitude of the blazer
    x = x_pos - ideal_long(i); %ideal_long is the longitude of the blazer
    
    y = y*lat2m; %convert difference to meters
    x = x*lon2m; %convert difference to meters
    
    angle1 = atand(y/x);
    angle = ideal_head(i);
  
    %determining which quadrent the cars heading angle is in and getting 
    %relative angle
    if ((angle >= 0) & (angle < 90)) | (angle == 360)
        a = (90 - angle) - angle1;
    elseif (angle >= 90) & (angle < 180)
        a = (180 - angle) - (90 - angle1);
    elseif (angle >= 180) & (angle < 270)
        a = (270 - angle) - angle1;
    elseif (angle >= 270) & (angle < 360)
        a = (360 - angle) - (90 - angle1);
    end
    
    h = sqrt(x^2 + y^2);
    range(i,1) = h;
    
    if a < 0
        a = a*(-1);
        x_rel = h*(sind(a));
        x_rel = x_rel*(-1);
    else
        x_rel = h*(sind(a));
    end
    y_rel = h*(cosd(a));
    
    
    %the oxts is somwhere in the middle of the car, these two values offset
    %the location of the blazer on birdseyeview plot to the middle of the
    %bumper
    %x_rel = x_rel - 0.48275; 
    %y_rel = y_rel - 2.8148;
    
    oxts_positions = [y_rel x_rel];
    %-----------------------
   
%     blazer_num_obj = size(ground_truth(i+gt_offset).Objects,1);
%     blazer_positions = zeros(blazer_num_obj);
    %sf_positions = zeros(sf_results(i).Num_Objects,2);

%     for j = 1:blazer_num_obj
%         blazer_positions(j,1:2) = ground_truth(i+gt_offset).Objects(j,:);
%     end
    
    %for j = 1:sf_results(i).Num_Objects
        %sf_positions(j,1:2) = sf_results(i).Objects(1,j).Measurement(:,1);
    %end
    
%   plotDetection(blazerPlotter, blazer_positions);
    %plotDetection(sfPlotter, sf_positions);
    plotDetection(oxtsPlotter, oxts_positions); %for OXTS

    title(gca, char(num2str(ideal_time(i) + " seconds"))); 
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid, frame);
end

save('range.mat','range', 'ideal_time');
close(newVid);









