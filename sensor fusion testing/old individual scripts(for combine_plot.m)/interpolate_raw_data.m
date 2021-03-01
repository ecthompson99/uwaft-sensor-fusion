load('./mat files/simulation_gt.mat');

%create time column vector with proper increments
increment = 0.01; 
end_time = radar_final.Time(end);
time_radar = 0:increment:end_time;
time_radar