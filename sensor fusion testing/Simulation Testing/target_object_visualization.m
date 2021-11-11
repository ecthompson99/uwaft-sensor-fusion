clear all
clc
%% Script to plot target_obj output against ground truth target data (currently for simulation ground truth)
% Load rosbag from SF output and ground truth

bag = rosbag('../Rosbag Files/sensor_fusion_sim_100m_20mph_target_obj.bag');
load('../Mat Files/stationary_target_approach_100m_20mph_ground_truth.mat'); %ground_truth

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'target_obj'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%% Extract target object data
target_obj_data_size = size(tracked_obj_struct,1);
sf_results = struct('Time',[],'Object', []);

for i = 1:target_obj_data_size
    % Populate time
    sf_results(i).Time = tracked_obj_struct(i).ObjTimestamp;
    
    % Populate object
    sf_results(i).Object = objectDetection(tracked_obj_struct(i).ObjTimestamp,...
        [tracked_obj_struct(i).ObjDx; tracked_obj_struct(i).ObjDy]);
end

% Pad sensor fusion output with missing data to match ground truth
diff = size(ground_truth,2)-target_obj_data_size - 1;
empty_entry = sf_results(1,1);
empty_entry.Object = objectDetection(0, [0;0]);
for i = diff:-1:0
    empty_entry.Time = i * 0.01;
    sf_results = [empty_entry sf_results];
end

Timestamps = [ground_truth(1,:).Time]';
ground_truth_distance = [];
target_object_distance = [];
for i = 1:size(ground_truth,2)
    ground_truth_distance = [ground_truth_distance norm(ground_truth(1,i).Objects)];
    target_object_distance = [target_object_distance norm((sf_results(i).Object.Measurement)')];
end

figure;
plot(Timestamps,ground_truth_distance');
xlabel('Time (seconds)');
ylabel('Displacement (meters)');

hold on

plot(Timestamps,target_object_distance');

hold off
legend('Ground Truth', 'Target Object');