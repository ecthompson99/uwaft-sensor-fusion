%% INTERPOLATE_SF.M---------------------------------------------------------
%this script interpolates sensor fusion data (not true interpolation)
%it does this by extraction sensor fusion data only for the increments we
%need. This is done by checking if the time at which sf data was recorded
%falls within 0.0# plus minus 0.005 
%the gaps are filled with a an empty row of values
clear all;

load('./mat files/sf_cleanT.mat');

size_sf = size(sf_clean,1);%number of rows in sf_clean/sf_results
end_sf = round(sf_clean.Time(size_sf),2); %what time sensor fusion ends
ideal_length = (end_sf*100) + 1;
max_objects = max([sf_clean.Num_Objects]'); %finding the max number of objects

%create time column vector with proper increments
increment = 0.01; 
ideal_time_sf = 0:increment:end_sf;
ideal_time_sf = ideal_time_sf';

%setting a range for acceptable value from sf_clean
rangeT = 0.005;

%creating an empty but incremented properly table
%similar to what was done in script step2_sf_TT_to_T.mat
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

%looping through the whole table
for i = 1:(ideal_length)
    %setting the range
    maxT = ideal_time_sf(i) + rangeT;
    minT = ideal_time_sf(i) - rangeT;
    
    %checking if within range, if so, insert the row from sf_clean
    for j = 1:size_sf
        if (sf_clean.Time(j) >= minT) & (sf_clean.Time(j) <= maxT)
            sf_clean_interp(i,:) = sf_clean(j,:); 
        end
    end    
end

sf_time  = ideal_time_sf;

save('./mat files/sf_interp', 'sf_clean_interp', 'sf_time', 'max_objects'); 