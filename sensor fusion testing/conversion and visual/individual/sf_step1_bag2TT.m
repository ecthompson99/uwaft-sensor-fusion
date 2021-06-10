%% Load rosbag from SF output
clear all;
bag = rosbag('./input files/test3_20mph.bag');

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'binary_class'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%%Save data
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
        [tracked_obj_struct(i).Dx(j); tracked_obj_struct(i).Dy(j);tracked_obj_struct(i).Vx(j); tracked_obj_struct(i).Vy(j)]);
    
       if size(sf_results(i).Objects,2) == 0
           sf_results(i).Objects = object;  
       else
            sf_results(i).Objects(1,size(sf_results(i).Objects,2)+1) = object; 
       end
    end
end


save('./mat files/sf_TT', 'sf_results');