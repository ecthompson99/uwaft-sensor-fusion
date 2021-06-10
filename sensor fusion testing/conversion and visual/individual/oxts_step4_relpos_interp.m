%% interpolating OXTS data(dx,dy)
%this script interpolated oxts data using a interpolation function in
%matlab called interp1
clear all;
load('./mat files/oxts_relative');


%finding the end time
end_time = relpos_oxts(length(relpos_oxts),1); %getting the time at which oxts 
%stops recording data

%setting up a coloumn vector with time increments of 0.01 sec
increment = 0.01;
ideal_time = 0:increment:end_time;
ideal_time = ideal_time';

%initilizing new coloumn vectors which will store interpolated data
range = zeros(length(ideal_time),1);

%interpolating data
ideal_dy = interp1(relpos_oxts(:,1), relpos_oxts(:,2), ideal_time);
ideal_dx = interp1(relpos_oxts(:,1), relpos_oxts(:,3), ideal_time);
relpos_oxts_interp = [ideal_time, ideal_dy, ideal_dx];

oxts_time = ideal_time;

save('./mat files/oxts_interp','relpos_oxts_interp','oxts_time');

