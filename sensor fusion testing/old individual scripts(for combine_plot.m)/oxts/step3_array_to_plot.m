clear var
close all

%WARNING! This script is the old method of plotting OXTS and doesn't plot objects
%relative to the car. Instead in birdseyePlot the forward direction is
%always north.

%% Plot objects in video
load('simulation_gt.mat'); %ground_truth
load('sf_output.mat'); %sf_results
load('oxtsData.mat'); %oxts_final
set(0,'DefaultFigureVisible','on');

%FOR OXTS
%43.473208
%-80.540375
%conversion rates are for when located at latitude 43.4741
%using the calculator http://www.csgnetwork.com/degreelenllavcalc.html
lat2m = 111101.9478949894;
lon2m = 80911.2112851164;

%start_lat = input("Enter the latitude of the starting position: ");
%start_long = input("Enter the longitude of the starting position: ");
dist_y = input("Enter the distance(in the y direction) from starting postion to the stationary vehicle: ");
dist_x = input("Enter the distance(in the x direction) from starting postion to the stationary vehicle: ");


newVid = VideoWriter('tracked_objects_oxts.avi');
newVid.FrameRate = 10;
newVid.Quality = 10;
open(newVid);
gt_offset = 6;


bep = birdsEyePlot('XLim',[-500,500],'YLim',[-500,500]);
blazerPlotter = detectionPlotter(bep,'MarkerEdgeColor','black','DisplayName','Ground Truth Objects', 'Marker', 'o');
sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');
oxtsPlotter = detectionPlotter(bep,'MarkerEdgeColor','blue','DisplayName','Stationary Vehicle', 'Marker', 'square');

dist_y = dist_y*(1/lat2m); %convert distance into lat degrees
dist_x = dist_x*(1/lon2m); %convert distance into long degrees
    
y_pos = dist_y + 43.473208; %start_lat; %position of stationary vehicle in lat
x_pos = dist_x + -80.540375;%start_long; %position of stationary vehicle in long

for i = 7:size(sf_results,2) % sample plot for now (start:iteration:end)
    
    %FOR OXTS
    
    y = y_pos - ideal_lat(i+7500); %ideal_lat is the latitude of the blazer
    x = x_pos - ideal_long(i+7500); %ideal_long is the longitude of the blazer
  
    y = y*lat2m; %convert difference to meters
    x = x*lon2m; %convert difference to meters
    
    %the oxts is somwhere in the middle of the car, these two values offset
    %the location of the blazer on birdseyeview plot to the middle of the
    %bumper
    x = x - 0.48275; 
    y = y - 2.8148;
    
    oxts_positions = [y x];
    %-----------------------
   
    blazer_num_obj = size(ground_truth(i+gt_offset).Objects,1);
    blazer_positions = zeros(blazer_num_obj);
    sf_positions = zeros(sf_results(i).Num_Objects,2);

    for j = 1:blazer_num_obj
        blazer_positions(j,1:2) = ground_truth(i+gt_offset).Objects(j,:);
    end
    
    for j = 1:sf_results(i).Num_Objects
        sf_positions(j,1:2) = sf_results(i).Objects(1,j).Measurement(:,1);
    end
    
    plotDetection(blazerPlotter, blazer_positions);
    plotDetection(sfPlotter, sf_positions);
    plotDetection(oxtsPlotter, oxts_positions); %for OXTS

    title(gca, char(num2str(sf_results(i).Time + " seconds"))); 
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid, frame);
end

close(newVid);