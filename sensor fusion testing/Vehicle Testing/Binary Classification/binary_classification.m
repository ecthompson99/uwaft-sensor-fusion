clear all
close all
clc 

%% Assuming in kaiROS\sensor fusion testing

% Load ground truth
load('oxts_interp.mat');
ground_truth = relpos_oxts_interp;

% Load sensor fusion output
load('sf_interp.mat');
output = sf_clean_interp;
output.Num_Objects(isnan(output.Num_Objects)) = 0;%turning NaN values into zeros

%inserting a cloumn for num objects for ground truth after the timme column
%TEMPORARY
num = zeros(size(ground_truth,1),1);
for i = 1:size(ground_truth,1)
    if ~isnan(ground_truth(i,2))
        num(i) = 1;
    end
end    
ground_truth = [ground_truth(:,1) num ground_truth(:,2:end)];

%making sf as long as gt TEMPORARY FIX


TP = 0; %true positive
FP = 0; %false positive
FN = 0; %false negative
TN = 0; %true negative

total = 0;

sf_offset = 6;
% threshold taken from year 1
THRESHOLD = 11;
index = 0;
diff = size(ground_truth,1)-size(output,1);
empty_row = {NaN , 0 , [] , [], []};
for i = 1:diff
    output = [output ; empty_row];
end

% Go through every timestep
for i = 1:max(size(ground_truth,1),size(output,1))
    index = index+1;
    % Loop through ground truth objects
    for a = 1:ground_truth(i,2) %the matrix has two columns(time,num objects) and then two columns for each object(dy,dx)

        sf_flags = false(output.Num_Objects(i),1);%creating a column of zeros according to the number of objects
        gt_flags = false(ground_truth(i,2),1); %same thing ^

        gt_object = [ground_truth(i,(a*2)+1);ground_truth(i,(a*2)+2)]; %getting the specific gt object according to 'a' in column form

        % Loop through sensor fusion output objects
        for b = 1:output.Num_Objects(i) 
            sf_vector = output.("Object" + b){i,1}.Measurement(1:2); %getting the specific sf object according to 'b' in column form

            % Check distance between vectors
            if norm(sf_vector - gt_object) < THRESHOLD
                % True Positives = number of objects detected by both outputs
                sf_flags(b,1) = true; %turn it to true
                gt_flags(a,1) = true; %turn it to true
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