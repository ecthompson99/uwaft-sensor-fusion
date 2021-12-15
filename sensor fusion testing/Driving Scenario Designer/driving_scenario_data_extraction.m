% load("sim_output\scenario2.mat");

% save("./sim_output/scenario3e.mat", "sensor_data_3e")
%load the exported data from scenario designer (1 x (#time_stamps))
data = struct2table(sensor_data); % name of variable output from Driving Scenario Designer

for i = 1:height(data)
    time = data{i, 'Time'};
    disp(time);
    objects = data.ObjectDetections{i,1};
    
    egoPose = data.ActorPoses{i, 1};
    egoPose = egoPose(1).Velocity;
    egoVelocity = egoPose(1);
    
    r1_count = 1;
    r2_count = 1;
    r3_count = 1;
    cam_count = 1;
    
    r1_dx_ = zeros(1, 32);
    r1_dy_ = zeros(1, 32);
    r1_vx_ = zeros(1, 32);
    r1_vy_ = zeros(1, 32);
    r2_dx_ = zeros(1, 32);
    r2_dy_ = zeros(1, 32);
    r2_vx_ = zeros(1, 32);
    r2_vy_ = zeros(1, 32);
    r3_dx_ = zeros(1, 32);
    r3_dy_ = zeros(1, 32);
    r3_vx_ = zeros(1, 32);
    r3_vy_ = zeros(1, 32);
    cam_dx_ = zeros(1, 10);
    cam_dy_ = zeros(1, 10);
    cam_vx_ = zeros(1, 10);
    
    for j = 1:size(objects,1) 
        
        %Center Radar detections
        if (objects{j,1}.SensorIndex == 1)
            r1_dx_(1,r1_count) = objects{j,1}.Measurement(1,1);
            r1_dy_(1,r1_count) = objects{j,1}.Measurement(2,1);
            r1_vx_(1,r1_count) = objects{j,1}.Measurement(4,1);
            r1_vy_(1,r1_count) = objects{j,1}.Measurement(5,1);
            r1_count = r1_count + 1;
        
        %Right Radar detections
        elseif (objects{j,1}.SensorIndex == 2)
            r2_dx_(1,r2_count) = objects{j,1}.Measurement(1,1);
            r2_dy_(1,r2_count) = objects{j,1}.Measurement(2,1);
            r2_vx_(1,r2_count) = objects{j,1}.Measurement(4,1);
            r2_vy_(1,r2_count) = objects{j,1}.Measurement(5,1);
            r2_count = r2_count + 1;
            
        %Left Radar detections
        elseif (objects{j,1}.SensorIndex == 3)
            r3_dx_(1,r3_count) = objects{j,1}.Measurement(1,1);
            r3_dy_(1,r3_count) = objects{j,1}.Measurement(2,1);
            r3_vx_(1,r3_count) = objects{j,1}.Measurement(4,1);
            r3_vy_(1,r3_count) = objects{j,1}.Measurement(5,1);
            r3_count = r3_count + 1;
            
        %Camera detections
        elseif (objects{j,1}.SensorIndex == 4) 
            cam_dx_(1,cam_count) = objects{j,1}.Measurement(1,1);
            cam_dy_(1,cam_count) = objects{j,1}.Measurement(2,1);
            cam_vx_(1,cam_count) = objects{j,1}.Measurement(4,1);
            cam_count = cam_count + 1;
 
        end
    end
    
    data_output_ = {time, ...
                    r1_dx_, r1_dy_, r1_vx_, r1_vy_, ...
                    r2_dx_, r2_dy_, r2_vx_, r2_vy_, ...
                    r3_dx_, r3_dy_, r3_vx_, r3_vy_, ...
                    cam_dx_, cam_dy_, cam_vx_, ...
                    egoVelocity};
                
    writecell(data_output_, "./csv/scenario2_again.csv", "WriteMode", "append");
   
end