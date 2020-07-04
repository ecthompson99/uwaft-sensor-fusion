clear all
clc
%% Load rosbag from SF output

bag = rosbag('highway_long.bag');

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'binary_class'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%% Refactor data
sf_results = struct('Time',[],'Objects', [], 'Num_Objects', []);
data_size = size(tracked_obj_struct,1);
time = zeros(data_size,1);

for size = 1:data_size
    time(size) = tracked_obj_struct(size).GlobalClk;
end

[unique_bins,~,obj_c] = unique(time(:,1),'rows');
num_obj = accumarray(obj_c,1);

%% Populate mat file
%Populate time, num objects, and object array
for count = 1:data_size
    sf_results(count).Time = time(count);
    sf_results(count).Num_Objects = num_obj(count);
    
    object = objectDetection(tracked_obj_struct(count).GlobalClk,...
        [tracked_obj_struct(count).Dx; tracked_obj_struct(count).Dy]);
    
    if isempty(sf_results(count).Objects())
            sf_results(count).Objects = object;  
    else
            sf_results(count).Objects(1,size(sf_results(count).Objects,1)+1) = object; 
    end
    %sf_results(count).Objects(1,1) = object;  
    
end

save('sf_output','sf_results');


