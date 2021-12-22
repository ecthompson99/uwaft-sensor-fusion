clear all
clc
addpath('functions\')
%% Script to plot target_obj output against ground truth target data (currently for simulation ground truth)
% Load rosbag from SF output and ground truth

bag = rosbag('../Driving Scenario Designer/bag/scenario5-output.bag');
load('../Driving Scenario Designer/ground_truth/scenario5_ground_truth.mat'); %ground_truth

% Select tracked obj topic
tracked_obj_bag = select(bag, 'Topic', 'tracked_obj'); 

% Save rosbag as struct
tracked_obj_struct = cell2mat(readMessages(tracked_obj_bag, 'DataFormat', 'struct'));

%% Visualize ground truth vs. sensor fusion output
Timestamps = [ground_truth(1,:).Time]';
num_timestamps = size(Timestamps, 1);
num_objects = size(ground_truth(1).Objects, 1);
ground_truth_distance = zeros(num_timestamps, num_objects);
target_object_distance = [];

for i = 1:num_timestamps
    ground_truth_distance(i, :) = vecnorm(ground_truth(1,i).Objects');
end

figure;
plot(Timestamps, ground_truth_distance', 'DisplayName', 'Ground Truth');
xlabel('Time (seconds)');
ylabel('Displacement (meters)');

hold on
% For a single object use get_filtered_object(bag)
% For multiple vehicles, use get_tracked_objects(bag)
[time, obj_dx, obj_dy, obj_vx] = get_filtered_object(bag);
plot(time, sqrt(obj_dx.^2+obj_dy.^2), 'DisplayName', 'Sensor Fusion Output');
title('Filtered Object vs. Ground Truth');

hold off
legend();