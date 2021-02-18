load("driving_scenarios\stationary_target_approach_fast.mat");

%load the exported data from scenario designer (1 x (#time_stamps))
data = struct2table(sensor_data);

% vx and time cache to determine ax for radars
prev1_vx_ = 0;
prev2_vx_ = 0;
prev3_vx_ = 0;
prev_time_ = 0;

for i = 1:height(data)
    time = data{i, 'Time'};
    objects = data.ObjectDetections{i,1};
    
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
    
%     r1_dx_(:, ~any(r1_dx_, 1)) = [];
%     r1_dy_(:, ~any(r1_dy_, 1)) = [];
%     r1_vx_(:, ~any(r1_vx_, 1)) = [];
%     r1_vy_(:, ~any(r1_vy_, 1)) = [];
%     
%     r2_dx_(:, ~any(r2_dx_, 1)) = [];
%     r2_dy_(:, ~any(r2_dy_, 1)) = [];
%     r2_vx_(:, ~any(r2_vx_, 1)) = [];
%     r2_vy_(:, ~any(r2_vy_, 1)) = [];
%     
%     r3_dx_(:, ~any(r3_dx_, 1)) = [];
%     r3_dy_(:, ~any(r3_dy_, 1)) = [];
%     r3_vx_(:, ~any(r3_vx_, 1)) = [];
%     r3_vy_(:, ~any(r3_vy_, 1)) = [];
%     
%     cam_dx_(:, ~any(cam_dx_, 1)) = [];
%     cam_dy_(:, ~any(cam_dy_, 1)) = [];
%     cam_vx_(:, ~any(cam_vx_, 1)) = [];
    
    data_output_ = {time, ...
                    r1_dx_, r1_dy_, r1_vx_, r1_vy_, ...
                    r2_dx_, r2_dy_, r2_vx_, r2_vy_, ...
                    r3_dx_, r3_dy_, r3_vx_, r3_vy_, ...
                    cam_dx_, cam_dy_, cam_vx_};
                
    writecell(data_output_, "sim_sensor_output_stationary_approach.csv", "WriteMode", "append");
   
end

    