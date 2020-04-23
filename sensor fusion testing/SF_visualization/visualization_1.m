clear
clc

% Load information from rosbag
[target_A, tracked_A, target_count, target_obj, tracked_count, tracked_obj] = load_rosbag();

%% Create Plot

% targetObjectsName = "Target Objects"; %Legend label for target objects
% targetObjectsMarker = 'o'; %Plot marker for target objects
% 
% trackedObjectsName = "Tracked Objects"; %Legend label for tracked objects
% trackedObjectsMarker = '+'; %Plot marker for tracked objects
% 
% detectionMarkers = [targetObjectsMarker, trackedObjectsMarker];

%markers = {'+','o','*','x','v','d','^','s','>','<'};

% Create bird's eye plot
bep = birdsEyePlot('XLim',[0,90],'YLim',[-35,35]);
targetPlotter = detectionPlotter(bep,'DisplayName','Target Objects', ...
    'MarkerEdgeColor','red','Marker', 'o');
trackedPlotter = detectionPlotter(bep,'DisplayName','Tracked Objects', ...
    'MarkerEdgeColor','blue', 'Marker', '+');

grid on; 

% create empty arrays for vehicles
target_pos = cell(size(target_obj,1),1);
target_vel = cell(size(target_obj,1),1);

tracked_pos = cell(size(tracked_obj,1),1);
tracked_vel = cell(size(tracked_obj,1),1);

% variables used to loop through target_A rows and tracked_A rows
counter = 0; % used to keep track of unique object
num_targets = double(target_count); % permanent array

counter_1 = 0; % used to keep track of unique object
num_tracked = double(tracked_count); % permanent array


for i = 1:size(target_obj,1)% loop through target_id 
    target_pos{i} = [target_A((counter + 1):(counter + num_targets(i)), 2) ...
        target_A((counter + 1):(counter + num_targets(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    target_vel{i} = [target_A((counter + 1):(counter + num_targets(i)), 4) ...
        target_A((counter + 1):(counter + num_targets(i)), 8)]; % take vx and vy in columns 4 and 8 of array
    
    counter = counter + num_targets(i);
end

for i = 1:size(target_obj,1)% loop through target_id 
    plotDetection(targetPlotter, target_pos{i}, target_pos{i});
end


for i = 1:size(tracked_obj,1)% loop through target_id 
    tracked_pos{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 2) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    tracked_vel{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 4) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 8)]; % take vx and vy in columns 4 and 8 of array
    
    counter_1 = counter_1 + num_tracked(i);
    
end

% Plot Detections
for i = 1:size(tracked_obj,1)% loop through target_id 
    plotDetection(trackedPlotter, tracked_pos{i}, tracked_vel{i});
end




