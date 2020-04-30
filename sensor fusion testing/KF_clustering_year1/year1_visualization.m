clear all
close all
clc 

%% Detection Clustering and Tracking
load('blazer_sensor.mat'); %vehicle_final
                    
results = struct('Time',[],'Objects', [], 'Num_Objects', []);
current_step = 1;
timestep = 0.1;
clock = 0; % in seconds
vehicle_size = 4.7;
x_tol = 5;
y_tol = 2;
detections = objectDetection.empty(1,0);
det_count = 1;
buffer = struct('Time',[NaN],'Dx',[NaN],'Dy',[NaN],'Count',[NaN],'Temp',[NaN]);

for i = 1:size(vehicle_final,1)
    % Add detections within 0.1 seconds
    det_time = vehicle_final.time_in_sec(i);
    if det_time > clock+timestep
        % Cluster detections and iterate to next timestep        
        objects = clusterDetectionsY1(detections,vehicle_size);
        
        if size(detections,2) > 1
            % Save data to struct 
            results(current_step).Time = clock + 0.1;
            results(current_step).Objects = objects;
            results(current_step).Num_Objects = size(objects,2);
            current_step = current_step + 1;
        end
        
        clock = clock + 0.1;
        detections = objectDetection.empty(1,0);
        det_count = 1;
    end    
    

    % Find index of dx and dy depending on Vision or Radar signal
    track_num = strtok(reverse(vehicle_final.Name(i)),'_');
    track_num = reverse(track_num);
    if contains(vehicle_final.Name(i),'F_Vision')            
        dy_name = strcat('FwdVsnAzmthTrk',track_num,'Rev');
        dx_name = strcat('FwdVsnRngTrk',track_num,'Rev');

    else
        dy_name = strcat('FLRRTrk',track_num,'Azimuth');
        dx_name = strcat('FLRRTrk',track_num,'Range');

    end

    dx = vehicle_final.Signals{i,1}.(dx_name{1});
    dy = vehicle_final.Signals{i,1}.(dy_name{1});

    % Add to detections if valid object
    if dx > 0 && dx < 250 && abs(dy) < 128 % check valid measurements

        % remove temp detections after 5 seconds
        tracked = false;
        total_obj = size(buffer.Time,2);
        for j = 1:total_obj 
            if det_time-buffer.Time(j) > 5
                buffer.Time(j) = NaN;
                buffer.Dx(j) = NaN;
                buffer.Dy(j) = NaN;
                buffer.Count(j) = NaN;
                buffer.Temp(j) = NaN;
%                 break;

            % check current detection to buffer
            elseif ((buffer.Dx(j) - dx)^2 + (buffer.Dy(j) - dy)^2)^0.5 <vehicle_size ...
                   | ((abs(buffer.Dx(j) - dx) < x_tol) & (abs(buffer.Dy(j) - dy) < y_tol))
                buffer.Count(j) = buffer.Count(j) + 1;
                buffer.Time(j) = det_time;
                buffer.Dx(j) = dx;
                buffer.Dy(j) = dy;
                tracked = true;
                if buffer.Count(j) >= 5 | buffer.Temp(j) == false
                    detections(det_count) = objectDetection(clock,[dx;dy]);
%                     buffer.Time(j) = NaN;
%                     buffer.Dx(j) = NaN;
%                     buffer.Dy(j) = NaN;
                    buffer.Temp(j) = false;
                    det_count = det_count + 1;
                end
            end
        end

        % New object
        if tracked == false
           buffer.Time(total_obj+1) = det_time;
           buffer.Dx(total_obj+1) = dx;
           buffer.Dy(total_obj+1) = dy;
           buffer.Count(total_obj+1) = 1;
           buffer.Temp(total_obj+1) = true;
        end

    end
    
    for k = size(buffer.Time,2):-1:1
        if isnan(buffer.Time(k))
            buffer.Time(k) = [];
            buffer.Dx(k) = [];
            buffer.Dy(k) = [];
            buffer.Count(k) = [];
            buffer.Temp(k) = [];
        end
    end
    
end

save('ground_truth','results');
