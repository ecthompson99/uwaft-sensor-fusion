clear all
clc
addpath('functions\')
%% Script to plot target_obj output against ground truth target data (currently for simulation ground truth)
% Load rosbag from SF output and ground truth

bag = rosbag('../Driving Scenario Designer/study3/3-5.bag');
load('../Driving Scenario Designer/ground_truth/scenario3_ground_truth.mat'); %ground_truth

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'tracked_obj'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%% Extract target object data
target_obj_data_size = size(tracked_obj_struct,1);
sf_results = struct('Time',[],'Object', []);

for i = 1:target_obj_data_size
    % Populate time
    sf_results(i).Time = tracked_obj_struct(i).ObjTimestamp;
    
    % Populate object
    sf_results(i).Object = objectDetection(tracked_obj_struct(i).ObjTimestamp(1),...
        [tracked_obj_struct(i).ObjDx(1); tracked_obj_struct(i).ObjDy(1)]);
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
num_timestamps = size(Timestamps, 1);
num_objects = size(ground_truth(1).Objects, 1);
ground_truth_distance = zeros(num_timestamps, num_objects);
target_object_distance = [];

for i = 1:num_timestamps
    ground_truth_distance(i, :) = vecnorm(ground_truth(1,i).Objects');
    target_object_distance = [target_object_distance norm((sf_results(i).Object.Measurement)')];
end

figure;
plot(Timestamps, ground_truth_distance');
xlabel('Time (seconds)');
ylabel('Displacement (meters)');

hold on
% ======== My additions =========
[time, obj_dx, obj_dy, obj_vx] = get_tracked_objects(bag, 3);
% ===============================
% plot(Timestamps,target_object_distance');
plot(time, sqrt(obj_dx.^2+obj_dy.^2));
title('Scenario 2: Filtered Object vs. Ground Truth');

hold off
legend('Ground Truth', ...
       'Target Object');