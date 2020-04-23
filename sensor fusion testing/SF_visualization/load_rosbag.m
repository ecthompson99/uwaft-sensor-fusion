function [target_A, tracked_A, target_count, target_obj, tracked_count, tracked_obj] = load_rosbag()

% ADDINS NEEDED: 'Robotics System Toolbox' 
% Load rosbag
bag = rosbag('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\env_state.bag');

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
[target_obj,~,target_c] = unique(target_A(:,1),'rows');
[tracked_obj,~,tracked_c] = unique(tracked_A(:,1),'rows');

target_count = accumarray(target_c,1);
tracked_count = accumarray(tracked_c,1);

% convert to uint8 to array of doubles
%target_counts = [double(target_obj), double(target_count)]; % show unique ObjId and num of occurances
%tracked_counts = [double(tracked_obj), double(tracked_count)]; % show unique ObjId and num of occurances

end
