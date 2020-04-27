% Load ground truth
file_path = strcat(pwd, '\KF_clustering_year1\ground_truth.mat')
ground_truth = load(file_path);

% Load sensor fusion output
bag_path = strcat(pwd, '\2020-04-27-00-44-09.bag');
bag = rosbag(bag_path);
temp = select(bag, 'Topic', '/binary_class');
output = readMessages(temp, 'DataFormat', 'struct');

TP = 0
FP = 0
FN = 0
TN = 0

total = 0

% threshold taken from year 1
THRESHOLD = 11;

% remember to change that 9000 (number of frames)
% Go through every timestep
for i = 1:9000

    % Loop through ground truth objects
    for a = 1:size(ground_truth(i))(1) 

        % Loop through sensor fusion output objects
        for b = 1:size(output{i}.Dx)(1) 
            gt_object = ground_truth(i).Objects(1,a);
            sf_vector = [output{i}.Dx(b), output{i}.Dy(b)];

            % Check distance between vectors
            if (norm(sf_vector - gt_object.Measurement) < THRESHOLD
                % Remove matched object from both matrices
                ground_truth(i).Objects(:,a) = [];
                output{i}.Dx(b) = [];

                % True Positives = number of objects detected by both outputs
                TP = TP + 1;
                total = total + 1;
                break;
            end
        end

        % False Negatives = number of objects left in ground truth
        for b = 1:size(output{i}.Dx)(1)
            FN = FN + 1
            total = total + 1
        end

        % False Positives = number of objects left in sensor fusion algorithm
        FP = FP + size(output{1}.Dx)(1);
        total = total + size(output{1}.Dx)(1);
    end
end

precision = TP / (TP + FP);
recall = TP / (TP + FN);
accuracy = (TP + TN) / (TP + TN + FP + FN);
f_measure = 2 * ((precision * recall)/(precision + recall));

fprintf('True Positive: %.3f\nTrue Negative: %.3f\nFalse Positive: %.3f\nFalse Negative: %.2f\n', TP, TN, FP, FN);
fprintf('Precision: %.3f\nRecall: %.3f\nAccuracy: %.3f\nF-Measure: %.3f\n', precision, recall, accuracy, f_measure);