%% Load rosbag
% ADDINS NEEDED: 'Robotics System Toolbox' 
clear
clc

bag = rosbag('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\env_state.bag');

%% Filter info

% Select topics 
sel_targets = select(bag, 'Topic', 'target_obj'); 
sel_tracked = select(bag, 'Topic', 'tracked_obj'); 

% Read msgs into cells
target_struct = cell2mat(readMessages(sel_targets, 'DataFormat', 'struct'));
tracked_struct = cell2mat(readMessages(sel_tracked, 'DataFormat', 'struct'));

% Create Tables, sorted by ID (3rd col)
target_T = sortrows(struct2table(target_struct),3);
tracked_T = sortrows(struct2table(tracked_struct),3);

% Create array: ObjId,ObjDx,ObjLane,ObjVx,ObjDy,ObjAx,ObjPath,ObjVy,ObjTimestamp,ObjTrackNum 
target_A = double(table2array(target_T(:,3:12)));
tracked_A = double(table2array(tracked_T(:,3:12)));

% Find unique objects & num of occurances
[target_obj,ia,target_c] = unique(target_A(:,1),'rows');
[tracked_obj,ib,tracked_c] = unique(tracked_A(:,1),'rows');

target_count = accumarray(target_c,1);
tracked_count = accumarray(tracked_c,1);

% convert to uint8 to array of doubles
target_counts = [double(target_obj), double(target_count)]; % show unique ObjId and num of occurances
tracked_counts = [double(tracked_obj), double(tracked_count)]; % show unique ObjId and num of occurances

%% Driving scenario for target objects

scenario = drivingScenario;

% create empty arrays for vehicles
targets = cell(size(target_obj,1),1);
waypoints = cell(size(target_obj,1),1);
speed_calc = cell(size(target_obj,1),1);

counter = 0; % used to keep track of unique object
num_targets = double(target_count); % permanent array

for i = 1:size(target_obj,1)% loop through target_id 
    targets{i} = vehicle(scenario); % create target vehicle
    % Specify rows in target_A and specify trajectory for unique object
    waypoints{i} = [target_A((counter + 1):(counter + num_targets(i)), 2) ...
        target_A((counter + 1):(counter + num_targets(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    speed_calc{i} = [sqrt(target_A((counter + 1):(counter + num_targets(i)), 4).^2 + ...
        target_A((counter + 1):(counter + num_targets(i)), 8).^2)]; % take vx and vy in columns 4 and 8 of array
    
    counter = counter + num_targets(i);
end

for i = 1:size(target_obj,1)% loop through target_id 
    trajectory(targets{i},waypoints{i},speed_calc{i}(:,1))
end

% add a plot for debug
plot(scenario,'Waypoints','on')

% Start the simulation loop
while advance(scenario)
 %fprintf('The vehicle is located at %.2f m at t=%.0f ms\n', targets{1}.Position(1), s.SimulationTime*1000)
 pause(0.1)
end


%% Driving scenario for tracked objects

% create empty arrays for objects
tracked_pos = cell(size(tracked_obj,1),1);
tracked_vel = cell(size(tracked_obj,1),1);

counter_1 = 0; % used to keep track of unique object
num_tracked = double(tracked_count); % permanent array

% Create bird's eye plot
bep = birdsEyePlot('XLim',[0,90],'YLim',[-35,35]);
objPlotter = detectionPlotter(bep,'DisplayName','Tracked Objects', ...
    'MarkerEdgeColor','red', 'Marker','^');

for i = 1:size(tracked_obj,1)% loop through target_id 
    tracked_pos{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 2) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    tracked_vel{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 4) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 8)]; % take vx and vy in columns 4 and 8 of array
    
    counter_1 = counter_1 + num_tracked(i);
    
end

% Plot Detections
for i = 1:size(tracked_obj,1)% loop through target_id 
    plotDetection(objPlotter, tracked_pos{i}, tracked_vel{i});
end

