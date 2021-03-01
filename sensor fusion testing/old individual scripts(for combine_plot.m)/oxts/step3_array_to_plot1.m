%This scripts plot objects so that they are relative to the car, meaning
%the forward direction on birdseyePlot is always the direction the car is
%facing.
%It also generates the data required to for range_plot.m for graphs on
%range vs time and range_rate vs time.

load('oxtsData.mat');

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

newVid = VideoWriter('tracked_objects_oxts.avi');
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
