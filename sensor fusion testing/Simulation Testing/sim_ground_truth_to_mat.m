clear
clc

load('../Driving Scenario Designer/sim_output/scenario2.mat');
detections = struct2table(sensor_data_2);

ground_truth = struct('Time', [], 'Objects', []);

for i = 1:height(detections)

    ground_truth(i).Time = detections{i,'Time'};
    actors = detections.ActorPoses{i,1};
    ego = actors(1,1);
    
    for j = 2:size(actors,1)
        ground_truth(i).Objects(j-1,:) = [actors(j,1).Position(1,1) - ego.Position(1,1)  
                                         actors(j,1).Position(1,2) - ego.Position(1,2)];
        
    end

end

save('../Driving Scenario Designer/ground_truth/scenario5_ground_truth_test.mat', 'ground_truth'); 