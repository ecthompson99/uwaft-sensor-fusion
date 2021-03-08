clear all
%This script takes in a blf file and a rosbag file and plots the data in a birdseyePlot all
%in one go.
%This scripts plot objects so that they are relative to the car, meaning
%the forward direction on birdseyePlot is always the direction the car is
%facing.
%It also generates the data required to for range_plot.m for graphs on
%range vs time and range_rate vs time.
%-----------------------------------------------------------------------
%% Load rosbag from SF output

bag = rosbag('./input files/test2_10mph.bag');

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

clear bag;
clear tracked_obj_bag;
clear tracked_obj_struct;
clear data_size;
clear i;
clear object;

%% TURNING IT INTO A TABLE WHERE EACH OBJECT HAS A SEPERATE COLUMN (sf_clean)

sf_results_t = struct2table(sf_results);
length_sf = size(sf_results_t,1); %number of rows in sf_results
max_objects = max([sf_results_t.Num_Objects]'); %finding the max number of objects

sf_array = NaN(length_sf,2);
sf_clean = array2table(sf_array);
clear sf_array;

empty = cell(length_sf,1); %making an empty cell array for objects

sf_clean.Properties.VariableNames(1) = {'Time'}; %naming the first column Time
sf_clean.Properties.VariableNames(2) = {'Num_Objects'}; %naming the second column Num_Object
for i  = 1:max_objects
    nameofcolumn = "Object" + i; %naming the rest of the columns by object number
    sf_clean.(nameofcolumn) = empty; %adding the empty column for each object
    sf_clean.Properties.VariableNames(i+2) = nameofcolumn; %adding the object columns name to the table
end

sf_clean.Time = sf_results_t.Time; %fill in time
sf_clean.Num_Objects = sf_results_t.Num_Objects; %filling in number of objects

for i = 1:length_sf
    for j = 1:sf_clean.Num_Objects(i)
        nameofcolumn = "Object" + j;
        sf_clean.(nameofcolumn){i,1} = sf_results_t.Objects{i,1}(1,j); %filling in all the object columns
    end    
end   

clear sf_results_t;
clear sf_array;
clear empty;
clear nameofcolumn;
clear i;

%% PUTTING THE DATA FOR EACH OBJECT INTO ITS OWN TABLE

% totcol = 6; % number of columns. time,dx,dy,vx,vy,Sensor index (measurement noise added afterwards)
% sf_object_array = NaN(length_sf,totcol); %creating the empty array
% sf_object_array = array2table(sf_object_array); %turning the empty array into a table
% 
% measurement noise has to be created as a cell array (just a column) and
% then added into the table later on as the rest of the table contains
% double values
% MeasurementNoise= cell(length_sf,1);
% 
% naming the columns
% sf_object_array.Properties.VariableNames(1) = "Time";
% sf_object_array.Properties.VariableNames(2) = "dy";
% sf_object_array.Properties.VariableNames(3) = "dx";
% sf_object_array.Properties.VariableNames(4) = "vy";
% sf_object_array.Properties.VariableNames(5) = "vx";
% sf_object_array = addvars(sf_object_array, MeasurementNoise, 'After', 'vx');%adding MeasurementNoise column cell array into the table after the column vx
% sf_object_array.Properties.VariableNames(7) = "SensorIndex";
% 
% Object1------------------------------------------------------------------
% index = 0;
% Object1 = sf_object_array;
% Object1_range = NaN(length_sf);
% for i = 1:length_sf
%     if not(isempty(sf_clean.Object1{i,1}))
%         Object1.Time(i-index) = sf_clean.Object1{i}.Time;
%         Object1.dy(i-index) = sf_clean.Object1{i}.Measurement(1);
%         Object1.dx(i-index) = sf_clean.Object1{i}.Measurement(2);
%         Object1.vy(i-index) = sf_clean.Object1{i}.Measurement(3);
%         Object1.vx(i-index) = sf_clean.Object1{i}.Measurement(4);
%         Object1.MeasurementNoise(i-index) = mat2cell(sf_clean.Object1{i}.MeasurementNoise,4,4);
%         Object1.SensorIndex(i-index)= sf_clean.Object1{i}.SensorIndex;
%         
%         range and range rate stuff
%         
%         
%         lastempty = 0;
%         if i>1
%             if not(isempty(sf_clean.Object1{i-1,1}))
%                 
%             
%     else
%         lastempty = 1;
%         index = index+1;
%     end
% end
% 
% Object2------------------------------------------------------------------
% index = 0;
% Object2 = sf_object_array;
% for i = 1:length_sf
%     if not(isempty(sf_clean.Object2{i,1}))
%         Object2.Time(i-index) = sf_clean.Object2{i}.Time;
%         Object2.dy(i-index) = sf_clean.Object2{i}.Measurement(1);
%         Object2.dx(i-index) = sf_clean.Object2{i}.Measurement(2);
%         Object2.vy(i-index) = sf_clean.Object2{i}.Measurement(3);
%         Object2.vx(i-index) = sf_clean.Object2{i}.Measurement(4);
%         Object2.MeasurementNoise(i-index) = mat2cell(sf_clean.Object2{i}.MeasurementNoise,4,4);
%         Object2.SensorIndex(i-index)= sf_clean.Object2{i}.SensorIndex;
%     else
%         index = index+1;
%     end
% end
% 
% Object3------------------------------------------------------------------
% index = 0;
% Object3 = sf_object_array;
% for i = 1:length_sf
%     if not(isempty(sf_clean.Object3{i,1}))
%         Object3.Time(i-index) = sf_clean.Object3{i}.Time;
%         Object3.dy(i-index) = sf_clean.Object3{i}.Measurement(1);
%         Object3.dx(i-index) = sf_clean.Object3{i}.Measurement(2);
%         Object3.vy(i-index) = sf_clean.Object3{i}.Measurement(3);
%         Object3.vx(i-index) = sf_clean.Object3{i}.Measurement(4);
%         Object3.MeasurementNoise(i-index) = mat2cell(sf_clean.Object3{i}.MeasurementNoise,4,4);
%         Object3.SensorIndex(i-index)= sf_clean.Object3{i}.SensorIndex;
%     else
%         index = index+1;
%     end
% end
% 
% clear totcol;
% clear sf_object_array;
% clear index;
% clear MeaasurementNoise;
% clear lastempty;

%save('./range_plot/range_test3_20mph_raw.mat','range', 'ideal_time', 'ideal_sf_results', 'sf_range','sf_rangeRate', 'max_objects');

%% INTERPOLATE_SF.M---------------------------------------------------------

size_sf = size(sf_clean,1);%number of rows in sf_clean/sf_results
%start_sf = round(sf_clean.Time(1),2); %what time sensor fusion starts
end_sf = round(sf_clean.Time(size_sf),2); %what time sensor fusion ends
ideal_length = (end_sf*100) + 1;
max_objects = max([sf_clean.Num_Objects]'); %finding the max number of objects

%create time column vector with proper increments
increment = 0.01; 
ideal_time_sf = 0:increment:end_sf;
ideal_time_sf = ideal_time_sf';

%creating a sf_clean row with all NaN values
% ideal_sf_results(1,ideal_length).Time = NaN;
% ideal_sf_results(1,ideal_length).Objects = NaN;
% ideal_sf_results(1,ideal_length).Num_Objects = NaN;

%setting a range for acceptable value from sf_clean
rangeT = 0.005;


%creating an empty but incremented properly table
%similar to what was done on lines 57 to 72
sf_array = NaN(ideal_length,2);
sf_clean_interp = array2table(sf_array);
empty = cell(ideal_length,1); %making an empty cell column vector for objects
sf_clean_interp.Properties.VariableNames(1) = {'Time'}; %naming the first column Time
sf_clean_interp.Properties.VariableNames(2) = {'Num_Objects'}; %naming the second column Num_Object
for i  = 1:max_objects
    nameofcolumn = "Object" + i; %naming the rest of the columns by object number
    sf_clean_interp.(nameofcolumn) = empty; %adding the empty column for each object
    sf_clean_interp.Properties.VariableNames(i+2) = nameofcolumn; %adding the object columns name to the table
end
sf_clean_interp.Time = ideal_time_sf; %fill in time
% sf_clean_interp.Num_Objects = sf_results_t.Num_Objects; %filling in number of objects


for i = 1:(ideal_length)
%   ideal_sf_results(i).Time = ideal_time_sf(i);
    maxT = ideal_time_sf(i) + rangeT;
    minT = ideal_time_sf(i) - rangeT;
    
    for j = 1:size_sf
        if (sf_clean.Time(j) >= minT) & (sf_clean.Time(j) <= maxT)
            sf_clean_interp(i,:) = sf_clean(j,:); 
        end
    end    
end

sf_time  = ideal_time_sf;

clear end_sf;
clear i;
clear ideal_length;
clear ideal_time_sf;
clear increment;
clear j;
% clear max_objects
clear maxT;
clear minT;
% clear rangeT;
clear sf_results;
clear size_sf;
% clear start_sf;
%% STEP1_BLF_TO_TT_OXTS

%%This part converts the blf file into a timetable with each signal in a
%struct
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
oxts_final1 = addvars(data_oxts,time_in_sec);

% writetimetable(oxts_final1,'practise1.xlsx','Sheet',1);

clear candb_oxts;
clear data_oxts;
clear time_in_sec;

%% This part converts the timetable into readable table with lat,long,heading

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
oxts_matrix = [time, latitude,longitude, head]; %adding all the columns together
oxts_table = array2table(oxts_matrix); %converting the matrix to a table
%naming all the columns of the table
oxts_table.Properties.VariableNames(1) = "Time";
oxts_table.Properties.VariableNames(2) = "Latitude";
oxts_table.Properties.VariableNames(3) = "Longitude";
oxts_table.Properties.VariableNames(4) = "Heading";

 clear acclength;
% clear end_time; 
% clear head;
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


%% getting dx, dy from OXTS-----------------------------------------------
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

k = -1;

relpos_oxts = NaN(length(time),3);
relpos_oxts(:,1) = time;

for i = t:length(time)
    %distance between ego and stationary vehicle in lat long degrees
    y = y_pos - latitude(i); %latitude is the latitude of the blazer
    x = x_pos - longitude(i); %longitude is the longitude of the blazer
    
    y = y*lat2m; %convert difference to meters
    x = x*lon2m; %convert difference to meters
    
    angle1 = atand(y/x);
    angle = head(i);
    
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
    range(i,1) = h; %getting range for oxts
    
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
    
    relpos_oxts(i,2) = y_rel;
    relpos_oxts(i,3) = x_rel;
    
end

clear k;
clear y;
clear x;
%clear x_pos;
clear x_rel;
%clear y_pos;
clear y_rel;
clear a;
clear h;
clear angle;
clear angle1;
clear t;
save('rangeraw_test2_10mph', 'sf_clean', 'relpos_oxts');

%% interpolating OXTS data(dx,dy)

%finding the end time
end_time = time(end);

%setting up a coloumn vector with time increments of 0.01 sec
increment = 0.01;
ideal_time = 0:increment:end_time;
ideal_time = ideal_time';

%initilizing new coloumn vectors which will store interpolated data
range = zeros(length(ideal_time),1);

ideal_dy = interp1(time, relpos_oxts(:,2), ideal_time);
ideal_dx = interp1(time, relpos_oxts(:,3), ideal_time);
relpos_oxts_interp = [ideal_time, ideal_dy, ideal_dx];

oxts_time = ideal_time;

clear end_time;
clear increment;
clear ideal_time;
clear ideal_dx;
clear ideal_dy;

% save('./mat files/csv_test2_10mph', 'time','latitude','longitude')
close all

%% video stuff------------------------------------------------------------

%finding wether oxts or sensor fusion data is longer
if length(oxts_time) >= length(sf_time)
    max_time = oxts_time;
else
    max_time = sf_time;
end

set(0,'DefaultFigureVisible','on');
newVid = VideoWriter('test2_10mph_raw.avi');
newVid.FrameRate = 100;
newVid.Quality = 10;
open(newVid);
bep = birdsEyePlot('XLim',[0,120],'YLim',[-20,20]);
sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');
oxtsPlotter = detectionPlotter(bep,'MarkerEdgeColor','blue','DisplayName','Ground Truth Object', 'Marker', 'square');
for i = 1:length(max_time) %
    
    %this is done so that the old ploted point doesn't stay on birdseyeplot
    plotDetection(sfPlotter, [NaN NaN]);
    plotDetection(oxtsPlotter, [NaN NaN]);
    
    
    %FOR OXTS
    if i <= length(oxts_time) %plot as long as there is data coming from relpos_oxts_interp
        oxts_positions = [relpos_oxts_interp(i,2) relpos_oxts_interp(i,3)]; 
    end
    
    %SENSOR FUSION STUFF
    if i <= length(sf_time)
        if ~isnan(sf_clean_interp.Num_Objects(i)) %checking if there is something to plot   
            sf_positions = NaN(max_objects,2); %making an empty array for the coordinates of all the objects
            for j = 1:sf_clean_interp.Num_Objects(i)%sf_clean_interp.Num_Objects(i)       %repeat for the amount of objects
                %the first brackets tell you the column and row number, the second bracket is 
                %always going to be {1,1} meaning the first cell and the last bracket is for 
                %which value in 'Measurement'.
                sf_positions(j,1:2) = sf_clean_interp{i,j+2}{1}.Measurement(1:2); 
%                 sf_range((i-t),j) = sqrt((sf_positions(j,1))^2 + (sf_positions(j,2))^2); %filling in NaN array with sf range data for each object
%                 sf_rangeRate((i-t),j) = (sf_range((i-t),j) - sf_range(i-t-1,j))/increment; %calculating the range rate for each object and filling in array
            end
            plotDetection(sfPlotter, sf_positions); %plotting sf objects onto birds eye plot
        end
    end

    plotDetection(oxtsPlotter, oxts_positions); %plotting ground truth data onto birds eye plot
    
    title(gca, char(num2str(round(max_time(i),1) + " seconds"))); 
    grid on;
    frame = getframe(gcf);
    writeVideo(newVid, frame);
end

close(newVid);

