clear all
close all
clc 

%% Assuming in kaiROS\sensor fusion testing

% Load ground truth
file_path = strcat(pwd, '\simulation_gt.mat');
temp = load('../Driving Scenario Designer/ground_truth/scenario2_ground_truth.mat');
ground_truth = temp.ground_truth;

% Load sensor fusion output
% bag_path = strcat(pwd, '\velocity_test_0707.bag');
bag = rosbag('../Driving Scenario Designer/bag/scenario2-output.bag');
temp = select(bag, 'Topic', '/binary_class');
output = readMessages(temp, 'DataFormat', 'struct');

TP = 0;
FP = 0;
FN = 0;
TN = 0;

total = 0;

sf_offset = 6; % Why is this 6?
sf_offset = size(output,1) - size(ground_truth,2);

THRESHOLD = 5;

% Go through every timestep
for i = 1:10:size(output,2)
    gt_ind = i+sf_offset;
    % Loop through ground truth objects
    num_gt = size(ground_truth(gt_ind).Objects,1);
    for a = 1:num_gt

        sf_flags = false(size(output{i}.Dx,1),1);
        gt_flags = false(num_gt,1);

        gt_object = ground_truth(gt_ind).Objects(a,:);

        % Loop through sensor fusion output objects
        for b = 1:size(output{i}.Dx,1) 
            sf_vector = [output{i}.Dx(b,1) output{i,1}.Dy(b,1)];

            % Check distance between vectors
            if norm(sf_vector - gt_object) < THRESHOLD
                % True Positives = number of objects detected by both outputs
                sf_flags(b,1) = true;
                gt_flags(a,1) = true;
                TP = TP + 1;
                total = total + 1;
%                 break;
            end
        end

        % False Negatives = number of objects left in ground truth
        FN = FN + sum(gt_flags == false);
        total = total + sum(gt_flags == false);

        % False Positives = number of objects left in sensor fusion algorithm
        FP = FP + sum(sf_flags == false);
        total = total + sum(sf_flags == false);
    end
end

precision = TP / (TP + FP);
recall = TP / (TP + FN);
accuracy = (TP + TN) / (TP + TN + FP + FN);
f_measure = 2 * ((precision * recall)/(precision + recall));

fprintf('True Positive: %.3f\nTrue Negative: %.3f\nFalse Positive: %.3f\nFalse Negative: %.2f\n', TP, TN, FP, FN)
fprintf('Precision: %.3f\nRecall: %.3f\nAccuracy: %.3f\nF-Measure: %.3f\n', precision, recall, accuracy, f_measure)