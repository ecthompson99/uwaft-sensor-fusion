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
target_A = table2array(target_T(:,3:12));
tracked_A = table2array(tracked_T(:,3:12));

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

% construct a straight road segment 50 m in length.
road(scenario, [0 0 0; 50 0 0]);

counter = 0; % used to keep track of unique object
num_targets = double(target_count); % permanent array

for i = 1:size(target_obj,1)% loop through target_id 
    targets{i} = vehicle(scenario); % create target vehicle
    % Specify rows in target_A and specify trajectory for unique object
    waypoints{i} = [target_A((counter + 1):(counter + num_targets(i)), 2) ...
        target_A((counter + 1):(counter + num_targets(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    speed_calc{i} = [sqrt(double(target_A((counter + 1):(counter + num_targets(i)), 4)).^2 + ...
        double(target_A((counter + 1):(counter + num_targets(i)), 8).^2))]; % take vx and vy in columns 4 and 8 of array
    
    counter = counter + num_targets(i);
end

for i = 1:size(target_obj,1)% loop through target_id 
    trajectory(targets{i},waypoints{i},speed_calc{i}(:,1))
end

% add a plot for debug
plot(s,'Centerline','on','Waypoints','on','RoadCenters','on')

% Start the simulation loop
while advance(s)
 %fprintf('The vehicle is located at %.2f m at t=%.0f ms\n', targets{1}.Position(1), s.SimulationTime*1000)
 pause(0.1)
end


%% Driving scenario for tracked objects




