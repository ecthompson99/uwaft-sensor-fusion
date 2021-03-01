clear all

load('./visualization_overlay/sensor_fusion/sf_output1.mat');

start_sf = round(sf_results1(1).Time,2); %what time sensor fusion starts
end_sf = round(sf_results1(size(sf_results1,2)).Time,2); %what time sensor fusion ends
ideal_length = (end_sf*100)-(start_sf*100);
size_sf = size(sf_results1,2)
max_objects = max([sf_results1.Num_Objects]'); %finding the max number of objects

%create time column vector with proper increments
increment = 0.01; 
ideal_time_sf = start_sf:increment:end_sf;
ideal_time_sf = ideal_time_sf';

%create array of nan values for object coordinates(first column x dist,
%second column y dist etc.
%object_locations = NaN(end_sf*100,max_objects);

%creating new struct with proper increments and NaN gaps


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
        if (sf_results1(j).Time >= minT) & (sf_results1(j).Time <= maxT)
            ideal_sf_results(i).Objects = sf_results1(j).Objects;
            ideal_sf_results(i).Num_Objects = sf_results1(j).Num_Objects;  
        end
    end    
end

ideal_sf_results1 = ideal_sf_results
save('./mat files/ideal_sf_results','ideal_sf_results1');
