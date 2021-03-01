clear all
%This script takes in a blf file and plots the data in a birdseyePlot all
%in one go.
%This scripts plot objects so that they are relative to the car, meaning
%the forward direction on birdseyePlot is always the direction the car is
%facing.
%It also generates the data required to for range_plot.m for graphs on
%range vs time and range_rate vs time.
%-----------------------------------------------------------------------
%%

%files to be loaded in are on line 13, 20, 122
%files to be uploaded are on line 234, 343


candb_oxts = canDatabase("./DBCs/oxts_v2.dbc");

%%SF_VISUALIZATION-------------------------------------------------------
%% Load rosbag from SF output

bag = rosbag('./rosbags/test3_20mph.bag');

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'binary_class'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%%Save data
data_size = size(tracked_obj_struct,1);
sf_results = struct('Time',[],'Objects', [], 'Num_Objects', []);

for i = 1:data_size
    % Populate time
    sf_results(i).Time = tracked_obj_struct(i).Timestamp;
    
    % Populate num_obj w/ size of Dx
    sf_results(i).Num_Objects = size(tracked_obj_struct(i).Dx,1);
    
    % Populate objects
    for j = 1:size(tracked_obj_struct(i).Dx,1)
        object = objectDetection(tracked_obj_struct(i).Timestamp,...
        [tracked_obj_struct(i).Dx(j); tracked_obj_struct(i).Dy(j);tracked_obj_struct(i).Vx(j); tracked_obj_struct(i).Vy(j)]);
    
       if size(sf_results(i).Objects,2) == 0
           sf_results(i).Objects = object;  
       else
            sf_results(i).Objects(1,size(sf_results(i).Objects,2)+1) = object; 
       end
    end
end


%------------------------------------------------------------------------
%% INTERPOLATE_SF.M
start_sf = round(sf_results(1).Time,2); %what time sensor fusion starts
end_sf = round(sf_results(size(sf_results,2)).Time,2); %what time sensor fusion ends
ideal_length = (end_sf*100)-(start_sf*100);
size_sf = size(sf_results,2);
max_objects = max([sf_results.Num_Objects]'); %finding the max number of objects

%create time column vector with proper increments
increment = 0.01; 
ideal_time_sf = start_sf:increment:end_sf;
ideal_time_sf = ideal_time_sf';

%creating a sf_results row with all NaN values
ideal_sf_results(1,ideal_length).Time = NaN;
ideal_sf_results(1,ideal_length).Objects = NaN;
ideal_sf_results(1,ideal_length).Num_Objects = NaN;

%setting a range for acceptable value from sf_results
rangeT = 0.005;

for i = 1:(ideal_length)
    ideal_sf_results(i).Time = ideal_time_sf(i);
    maxT = ideal_time_sf(i) + rangeT;
    minT = ideal_time_sf(i) - rangeT;
    
    for j = 1:size_sf
        if (sf_results(j).Time >= minT) & (sf_results(j).Time <= maxT)
            ideal_sf_results(i).Objects = sf_results(j).Objects;
            ideal_sf_results(i).Num_Objects = sf_results(j).Num_Objects;  
        end
    end    
end

clear end_sf;
clear i;
clear ideal_length;
clear ideal_time_sf;
clear increment;
clear j;
% clear max_objects
clear maxT;
clear minT;
clear rangeT;
clear sf_results;
clear size_sf;
% clear start_sf;
%-------------------------------------------------------------------------
%% STEP1_BLF_TO_TT_OXTS

%%This part converts the blf file into a timetable with each signal in a
%struct
format long g


%first argument: is the blf file
%second argument: is the channel number(column labeled "Chn" in canalyzer
%thrid argument: this Name1 for the dbc file
%fourth argument: this Value1 for Name1 and is the actual dbc file
%fifth argument: Name2 is for message standard ID's
%sixth argument:Value1 
data_oxts = blfread("./BLFs/WVE_Approach_Test_10mph_test2002_2021-02-18_21-31-15.blf",5,...
                "Database", candb_oxts,"CANStandardFilter", [1537 1543]);
time_in_sec = seconds(data_oxts.Time);
oxts_final1 = addvars(data_oxts,time_in_sec);

% writetimetable(oxts_final1,'practise1.xlsx','Sheet',1);

clear candb_oxts;
clear data_oxts;
clear time_in_sec;

%%--------------------------------------------------------
%% 

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
increment = 0.01;
ideal_time = 0:increment:end_time;
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
% clear latitude;
clear lengthofvector;
% clear longitude;
clear oxts_final1;
clear pos_ID;
% clear time;

save('./mat files/csv_test2_10mph', 'time','latitude','longitude')
%----------------------------------------------------------------
%This part acctually converts the values realative to the vehicle and plots
close all

%% Plot objects in video
set(0,'DefaultFigureVisible','on');

t = 10; %offset value just for OXTS(determines at what index the data will start plotting)

%for stationary vehicle(drive upto its location and use NavDisplay to get
%these coordinate or Google Maps)
%43.474323
%-80.538345
y_pos = 43.47415130; %position of stationary vehicle in lat
x_pos = -80.53892280; %position of stationary vehicle in long


%conversion rates are for when located at latitude 43.4741
%also can use google maps to get conversion rate
%using the calculator http://www.csgnetwork.com/degreelenllavcalc.html
lat2m = 111111.9478949894;
lon2m = 80213.90374;


% y_pos = input("Enter the latitude of the stationary vehicle: ");
% x_pos = input("Enter the longitude of the stationary vehicle: ");

start_lat = ideal_lat(t);
start_long = ideal_long(t);

newVid = VideoWriter('./video(.avi)/test3_20mph.avi');
newVid.FrameRate = 100;
newVid.Quality = 10;
open(newVid);

bep = birdsEyePlot('XLim',[0,120],'YLim',[-20,20]);
sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');
oxtsPlotter = detectionPlotter(bep,'MarkerEdgeColor','blue','DisplayName','Ground Truth Object', 'Marker', 'square');

k = -1;

%creating array with time,x,y for oxts and sensor fusion for csv file
oxts_csv2 = NaN(length(ideal_time),3);
oxts_csv2(:,1) = ideal_time;
sf_csv = NaN(length(ideal_time),(max_objects*2)+1);
sf_csv(:,1) = ideal_time;

end_sf = size(ideal_sf_results,2); %what index sensor fusion stops

%creating empty NaN arrays for sf to fill in with range and range rate data
sf_range = NaN(length(ideal_time),max_objects);%assuming there are going to be a max of three objects
sf_rangeRate = NaN(length(ideal_time),max_objects);%assuming there are going to be a max of three objects

for i = t:length(ideal_time) %
    
    %this is done so that the old ploted point doesn't stay on birdseyeplot
    plotDetection(sfPlotter, [NaN NaN]);
    plotDetection(oxtsPlotter, [NaN NaN]);
    
    %OXTS stuff ----------------------------------------------------------
   
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
    y_rel = sqrt(y_rel^2);
    y_rel = y_rel + 0.15;
    
    
    %the oxts is somwhere in the middle of the car, these two values offset
    %the location of the blazer on birdseyeview plot to the middle of the
    %bumper
    %x_rel = x_rel - 0.48275; 
    %y_rel = y_rel - 2.8148;
    
    oxts_positions = [y_rel x_rel]; 
    
    %filling in data for csv files
    oxts_csv2(i,2) = x_rel;
    oxts_csv2(i,3) = y_rel;
    %-----------------------

    %SENSOR FUSION STUFF
    if (((i-t)/100) >= start_sf) && (end_sf >= ((i-t)-k)) %accounts for difference in start times for sf and oxts
        if ideal_sf_results((i-t)-k).Num_Objects ~= NaN     %checking if there is something to plot   
            sf_positions = zeros(ideal_sf_results((i-t)-k).Num_Objects,2); %making an empty array for the coordinates of all the objects
            for j = 1:ideal_sf_results((i-t)-k).Num_Objects       %repeat for the amount of objects
                sf_positions(j,1:2) = ideal_sf_results((i-t)-k).Objects(1,j).Measurement(1:2,1); %filling in empty array
                sf_range((i-t),j) = sqrt((sf_positions(j,1))^2 + (sf_positions(j,2))^2); %filling in NaN array with sf range data for each object
                sf_rangeRate((i-t),j) = (sf_range((i-t),j) - sf_range(i-t-1,j))/increment; %calculating the range rate for each object and filling in array
            end
            plotDetection(sfPlotter, sf_positions); %plotting sf objects onto birds eye plot
        end
    else 
        k = k+1; %helps determine when sf data should start plotting
    end


    plotDetection(oxtsPlotter, oxts_positions); %plotting ground truth data onto birds eye plot
    
    title(gca, char(num2str(round(ideal_time(i),1) + " seconds"))); 
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid, frame);
end

%saving data to graph range and rate rate against time
save('./range_plot/range_test3_20mph.mat','range', 'ideal_time', 'ideal_sf_results', 'sf_range','sf_rangeRate', 'max_objects');
save('./mat files/oxts_20mph_csv.mat','oxts_csv2')
close(newVid);









