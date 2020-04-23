clear
clc

% Load information from rosbag
[target_A, tracked_A, target_count, target_obj, tracked_count, tracked_obj] = load_rosbag();

%% Create Plot

targetObjectsName = "Target Objects"; %Legend label for target objects
targetObjectsColor = 'red'; %Plot marker for target objects

trackedObjectsName = "Tracked Objects"; %Legend label for tracked objects
trackedObjectColor = 'blue'; %Plot marker for tracked objects

detectionColors = [targetObjectsColor, trackedObjectColor];

timestepSize = 0.1; %Timestep size in seconds.determined by the blf to .mat 
                    %conversion
                    
startSeconds = 0; %The starting time, in seconds, rounded to the nearest 
                  %timestepSize. Values before the first available timestep.
                  
endSeconds = 300000; %The ending time, in seconds, rounded to the nearest 
                  %timestepSize. Values exceeding the available data play
                  %the log to its end.
                  
detectionMarkers = ['+';'o';'*';'x';'v';'d';'^';'s';'>';'<'];

            
% create empty arrays for vehicles
target_pos = cell(size(target_obj,1),1);
target_vel = cell(size(target_obj,1),1);
target_timestamp = cell(size(target_obj,1),1);

tracked_pos = cell(size(tracked_obj,1),1);
tracked_vel = cell(size(tracked_obj,1),1);
tracked_timestamp = cell(size(tracked_obj,1),1);

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
    target_timestamp{i} = [target_A((counter + 1):(counter + num_targets(i)), 10)]; % take timestamp in col 10
    counter = counter + num_targets(i);
end

for i = 1:size(tracked_obj,1)% loop through tracked_id 
    tracked_pos{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 2) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    tracked_vel{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 4) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 8)]; % take vx and vy in columns 4 and 8 of array
    tracked_timestamp{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 10)]; % take timestamp in col 10

    counter_1 = counter_1 + num_tracked(i);
    
end


% create target objects
targetObjects = cell(size(target_obj,1),1);
for i = 1:size(target_obj,1)
    targetObjects{i} = SFObject(target_obj(i), target_pos{i}, detectionMarkers(i, :),...
        detectionColors(1), target_count(i));
end

% create tracked objects
trackedObjects = cell(size(tracked_obj,1),1);
for i = 1:size(tracked_obj,1)
    trackedObjects{i} = SFObject(tracked_obj(i), tracked_pos{i}, detectionMarkers(i, :),...
        detectionColors(2), tracked_count(i));
end

r = BEPlotter(targetObjects, trackedObjects, timestepSize);
replay(r, startSeconds, endSeconds);
disp("Wrote contents of " + logFilename + " to a file.")







