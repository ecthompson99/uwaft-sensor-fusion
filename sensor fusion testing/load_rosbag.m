%% Load rosbag
% ADDINS NEEDED: 'Robotics System Toolbox' 

bag = rosbag('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\env_state.bag');

%% Filter info
% Select topics 
sel_targets = select(bag, 'Topic', 'target_obj'); 
sel_tracked = select(bag, 'Topic', 'tracked_obj'); 

% Read msgs into cells
targets = readMessages(sel_targets, 'DataFormat', 'struct');
tracked = readMessages(sel_tracked, 'DataFormat', 'struct');
target_struct = cell2mat(targets);
tracked_struct = cell2mat(tracked);

% Create Tables, sorted by ID (3rd col)
target_T = sortrows(struct2table(target_struct),3);
tracked_T = sortrows(struct2table(tracked_struct),3);

% Create array: ObjId,ObjDx,ObjLane,ObjVx,ObjDy,ObjAx,ObjPath,ObjVy,ObjTimestamp,ObjTrackNum 
target_A = table2array(tar_sorted(:,3:12));
tracked_A = table2array(track_sorted(:,3:12));

% Find unique objects & num of occurances
[target_obj,ia,target_c] = unique(target_A(:,1),'rows');
[tracked_obj,ib,tracked_c] = unique(tracked_A(:,1),'rows');

tracked_count = accumarray(target_c,1);
target_count = accumarray(tracked_c,1);

target_counts = [double(target_obj), double(tracked_count)]; % show unique ObjId and num of occurances
tracked_counts = [double(tracked_obj), double(target_count)];

% 
% % Find unique object nums
 
% A = accumarray(tar_array,1);
% 
% % Split to target objects
% target1 = tar_array(tar_array(:,1)==1,:);
% target2 = tar_array(tar_array(:,1)==2,:);
% target3 = tar_array(tar_array(:,1)==3,:);
% 
% 


 %% Driving scenario
% s = drivingScenario;
% 
% % create empty array of obj
% targets = cell(size(targetIds,1),1);
% %tracked = cell(size(targetIds,1),1);
% 
% % construct a straight road segment 50 m in length.
% road(s, [0 0 0; 50 0 0]);
% 
% % create target vehicles
% for ta_ids = 1:size(targetIds,1)
%     targets{ta_ids} = vehicle(s);
% end
% 
% % create tracked radar detections
% % for tr_ids = 1:size(targetIds,1)
% %     tracked{tr_ids} = radarDetectionGenerator('SensorIndex', tr_ids);
% % end
% 
% % specify for target 1
% speed_calc = sqrt(double(tar_array(:, 4)).^2 + double(tar_array(:, 8)).^2); % take vx and vy in columns 6 and 7 of array
% waypoints = [tar_array(:, 2) tar_array(:, 5)]; % take dx and dy in columns 2 and 5 of array
% trajectory(targets{1},waypoints,speed_calc)
% 
% % add a plot for debug
% plot(s,'Centerline','on','Waypoints','on','RoadCenters','on')
% 
% % Start the simulation loop
% while advance(s)
%  %fprintf('The vehicle is located at %.2f m at t=%.0f ms\n', targets{1}.Position(1), s.SimulationTime*1000)
%  pause(0.1)
% end
% 
% 





