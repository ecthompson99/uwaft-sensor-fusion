%% Load rosbag
% ADDINS NEEDED: 'Robotics System Toolbox' 

bag = rosbag('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\env_state.bag');
bagInfo = rosbag('info','C:\Users\sophy\OneDrive\UWAFT\Code\env_state.bag');

%% Filter info
% Select topics 
tar_sel = select(bag, 'Topic', 'target_obj'); 
track_sel = select(bag, 'Topic', 'tracked_obj'); 

% Read msgs into cell array
tar_struct = readMessages(tar_sel, 'DataFormat', 'struct');
track_struct = readMessages(track_sel, 'DataFormat', 'struct');

% Transfer cell array into structure array
tar_s_array = cell2mat(tar_struct);
track_s_array = cell2mat(track_struct);

% Create Tables
tar_table = struct2table(tar_s_array);
track_table = struct2table(track_s_array);

% Sort tables by Id (3rd column)
tar_sorted = sortrows(tar_table,3); 
track_sorted = sortrows(track_table,3); 

% Create array: ObjId, ObjDx, ObjLane, ObjVx,ObjDy, ObjAx, ObjPath, ObjVy,ObjTimestamp, ObjTrackNum 
tar_array = table2array(tar_sorted(:,3:12));
track_array = table2array(track_sorted(:,3:12));

% Find unique object nums
targetIds = unique(tar_sorted.ObjId);
trackedIds = unique(track_sorted.ObjId);


 %% Driving scenario
s = drivingScenario;

% create empty array of obj
targets = cell(size(targetIds,1),1);
%tracked = cell(size(targetIds,1),1);

% construct a straight road segment 50 m in length.
road(s, [0 0 0; 50 0 0]);

% create target vehicles
for ta_ids = 1:size(targetIds,1)
    targets{ta_ids} = vehicle(s);
end

% create tracked radar detections
% for tr_ids = 1:size(targetIds,1)
%     tracked{tr_ids} = radarDetectionGenerator('SensorIndex', tr_ids);
% end

% specify for target 1
speed_calc = sqrt(double(tar_array(:, 4)).^2 + double(tar_array(:, 8)).^2); % take vx and vy in columns 6 and 7 of array
waypoints = [tar_array(:, 2) tar_array(:, 5)]; % take dx and dy in columns 2 and 5 of array
trajectory(targets{1},waypoints,speed_calc)

% add a plot for debug
plot(s,'Centerline','on','Waypoints','on','RoadCenters','on')

% Start the simulation loop
while advance(s)
 %fprintf('The vehicle is located at %.2f m at t=%.0f ms\n', targets{1}.Position(1), s.SimulationTime*1000)
 pause(0.1)
end







