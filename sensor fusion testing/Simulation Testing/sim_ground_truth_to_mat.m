clear
clc

load('../Driving Scenario Designer/stationary_target_approach_100m_20mph_sensor_data.mat');
detections = struct2table(sensor_data);

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

save('../Mat Files/stationary_target_approach_100m_20mph_ground_truth.mat', 'ground_truth'); 