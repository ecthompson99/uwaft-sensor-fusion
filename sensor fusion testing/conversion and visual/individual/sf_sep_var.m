% PUTTING THE DATA FOR EACH OBJECT INTO ITS OWN TABLE
%this script basically makes a variable for each object and each object has
%its own table with time, dx,dy,vx,vy,sensor index and measurement noise
%code to generate a variable for each object has to be hardcoded in.

clear all;
load('./mat files/sf_cleanT.mat');

totcol = 6; % number of columns. time,dx,dy,vx,vy,Sensor index (measurement noise added afterwards)
sf_object_array = NaN(length_sf,totcol); %creating the empty array
sf_object_array = array2table(sf_object_array); %turning the empty array into a table

% measurement noise has to be created as a cell array (just a column) and
% then added into the table later on as the rest of the table contains
% double values
MeasurementNoise= cell(length_sf,1);

%naming the columns
sf_object_array.Properties.VariableNames(1) = "Time";
sf_object_array.Properties.VariableNames(2) = "dy";
sf_object_array.Properties.VariableNames(3) = "dx";
sf_object_array.Properties.VariableNames(4) = "vy";
sf_object_array.Properties.VariableNames(5) = "vx";
sf_object_array = addvars(sf_object_array, MeasurementNoise, 'After', 'vx');%adding MeasurementNoise column cell array into the table after the column vx
sf_object_array.Properties.VariableNames(7) = "SensorIndex";


%for each additional object copy/paste the section of code below and
%replace Object1 with the required additonal Object#

%Object1------------------------------------------------------------------
index = 0;
Object1 = sf_object_array;
for i = 1:length_sf
    if not(isempty(sf_clean.Object1{i,1}))
        Object1.Time(i-index) = sf_clean.Object1{i}.Time;
        Object1.dy(i-index) = sf_clean.Object1{i}.Measurement(1);
        Object1.dx(i-index) = sf_clean.Object1{i}.Measurement(2);
        Object1.vy(i-index) = sf_clean.Object1{i}.Measurement(3);
        Object1.vx(i-index) = sf_clean.Object1{i}.Measurement(4);
        Object1.MeasurementNoise(i-index) = mat2cell(sf_clean.Object1{i}.MeasurementNoise,4,4);
        Object1.SensorIndex(i-index)= sf_clean.Object1{i}.SensorIndex;                     
    else
        index = index+1;
    end
end

%Object2------------------------------------------------------------------
index = 0;
Object2 = sf_object_array;
for i = 1:length_sf
    if not(isempty(sf_clean.Object2{i,1}))
        Object2.Time(i-index) = sf_clean.Object2{i}.Time;
        Object2.dy(i-index) = sf_clean.Object2{i}.Measurement(1);
        Object2.dx(i-index) = sf_clean.Object2{i}.Measurement(2);
        Object2.vy(i-index) = sf_clean.Object2{i}.Measurement(3);
        Object2.vx(i-index) = sf_clean.Object2{i}.Measurement(4);
        Object2.MeasurementNoise(i-index) = mat2cell(sf_clean.Object2{i}.MeasurementNoise,4,4);
        Object2.SensorIndex(i-index)= sf_clean.Object2{i}.SensorIndex;
    else
        index = index+1;
    end
end

% %Object3------------------------------------------------------------------
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

clear totcol;
clear sf_object_array;
clear index;
clear MeaasurementNoise;
clear lastempty;
clear MeasurementNoise;
clear i;
clear length_sf;
clear sf_clean;

save('./mat files/ind_obj_T');