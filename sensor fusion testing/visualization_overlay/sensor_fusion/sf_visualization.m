clear all
clc
%% Load rosbag from SF output

bag = rosbag('goteam-thresh-5.bag');

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'tracked_obj'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%% Discretize data into bins based on timestep

data_size = size(tracked_obj_struct,1);
time = zeros(data_size,1);

timestep = 0.1; % in seconds
clock = 0.1; % start at timestep

for i = 1:data_size
    time(i) = tracked_obj_struct(i).ObjTimestamp;
end

edges = clock + timestep:timestep:tracked_obj_struct(data_size).ObjTimestamp;
bin_idx = discretize(time,edges); % sort time into bins

%% Refactor data and save to .mat file

sf_results = struct('Time',[],'Objects', [], 'Num_Objects', []);
[unique_bins,~,obj_c] = unique(bin_idx(:,1),'rows');
num_bins = accumarray(obj_c,1); % num_bins is the count total of each unique bin

% Populate time
for i = 1:size(edges,2)
    sf_results(i).Time = edges(i);  
end

% Populate Num_Objects
for j = 1:size(unique_bins,1)
    if ~isnan(unique_bins(j))
        sf_results(unique_bins(j)).Num_Objects = num_bins(j); 
    end
end

for i = 1:data_size
    % Populate object parameters w/ clock, [dx:dy]
    object = objectDetection(tracked_obj_struct(i).ObjTimestamp,[tracked_obj_struct(i).ObjDx;... % time is raw time for reference
        tracked_obj_struct(i).ObjDy]);
    % Populate object array with object
    if ~isnan(bin_idx(i))
        sf_results(bin_idx(i)).Objects(1,size(sf_results(bin_idx(i)).Objects,2)+1) = object; 
    end
end

save('sf_output','sf_results');
