%% getting dx, dy from OXTS-----------------------------------------------
%for oxts_matrix the columns are in the following order:
%time, latitude, longitude, heading
%this  script takes in latitude, longitude and heading and converts it into
%relative distance in meters from ego car to target and outputs a matrix 
%with time, dx,dy
clear all;
load('./mat files/oxts_T');

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

relpos_oxts = NaN(acclength,3); %making an empty matrix 
relpos_oxts(:,1) = oxts_matrix(:,1); %adding in time to the first column

for i = t:acclength
    %distance between ego and stationary vehicle in lat long degrees
    y = y_pos - oxts_matrix(i,2); %latitude difference
    x = x_pos - oxts_matrix(i,3); %longitude difference
    
    y = y*lat2m; %convert difference to meters
    x = x*lon2m; %convert difference to meters
    
    angle1 = atand(y/x);
    angle = oxts_matrix(i,4);%getting heading angle
    
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

save('./mat files/range_test3_20mph', 'relpos_oxts','-append');
save('./mat files/oxts_relative','relpos_oxts');




