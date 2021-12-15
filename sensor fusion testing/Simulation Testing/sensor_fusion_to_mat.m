clear all
clc
%% Load rosbag from SF output

bag = rosbag('..\Driving Scenario Designer\bag\scenario3-output.bag');

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'binary_class'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%% Save data
data_size = size(tracked_obj_struct,1);
sf_results = struct('Time',[],'Objects', [], 'Num_Objects', []);

for i = 1:data_size
    % Populate time
    sf_results(i).Time = tracked_obj_struct(i).Timestamp;
    
    % Populate num_obj w/ size of Dx
    sf_results(i).Num_Objects = size(tracked_obj_struct(i).Dx,1);
    
    % Populate objects
    for j = 1:size(tracked_obj_struct(i).Dx,1)
        object = objectDetection(tracked_obj_struct(i).Timestamp,...
        [tracked_obj_struct(i).Dx(j); tracked_obj_struct(i).Dy(j)]);
    
       if size(sf_results(i).Objects,2) == 0
           sf_results(i).Objects = object;  
       else
            sf_results(i).Objects(1,size(sf_results(i).Objects,2)+1) = object; 
       end
    end
end

save('sf_output.mat','sf_results');

