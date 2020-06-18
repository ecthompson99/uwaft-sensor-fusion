clear all
close all
clc 

%% Detection Clustering and Tracking
load('mat files\sensors.mat'); 

%combined_sorted = combined_sorted(1:10000,:);

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
dx_thresh = 3.5;

for i = 1:size(combined_sorted,1)
    % Add detections within 0.1 seconds
    det_time = combined_sorted.in_sec(i);
    if det_time > clock+timestep
        % Cluster detections and iterate to next timestep        
        objects = cluster_detections(detections,vehicle_size);
        
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
    

    % Find index of dx and dy depending on Radar (tracked) or Mobileye signal
     track_num = extractBetween(combined_sorted.Name(i),"Obj","_A");
%     track_num = reverse(track_num);
    if contains(combined_sorted.Name(i),'Radar1_Obj') && ...
            contains(combined_sorted.Name(i),'_A')
        dx_name = strcat('Radar1_Obj',track_num,'_dx');
        dy_name = strcat('Radar1_Obj',track_num,'_dy');
        dx = combined_sorted.Signals{i,1}.(dx_name{1}) + dx_thresh;
        dy = combined_sorted.Signals{i,1}.(dy_name{1});
    elseif contains(combined_sorted.Name(i),'ObstacleDataA')
        dx_name = 'ObstaclePosX';
        dy_name = 'ObstaclePosY';
        dx = combined_sorted.Signals{i,1}.(dx_name);
        dy = combined_sorted.Signals{i,1}.(dy_name);
    else
        continue
    end

    % Add to detections if valid object
    if dx > 0 && dx < 100 && abs(dy) < 5 % check valid measurements

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

save('mat files\ground_truth.mat','results');

%% Plot data and record video
set(0,'DefaultFigureVisible','off');

vid_path = strcat(pwd, '\recording\sensors.avi');
newVid = VideoWriter(vid_path);
newVid.FrameRate = 1;
newVid.Quality = 100;
open(newVid);
step_size = 10;
start = 10;
end_loop = size(results,2);

pic_path = strcat(pwd, '\plot\plot');


for i = start:step_size:end_loop % sample plot for now

    bep = birdsEyePlot('XLim',[0,100],'YLim',[-5,10]);
    blazerPlotter = detectionPlotter(bep,'DisplayName','Ground Truth Objects', 'Marker', 'o');
    
    positions = zeros(results(i).Num_Objects,2);
    for j = 1:results(i).Num_Objects
        positions(j,1:2) = results(i).Objects(1,j).Measurement(:,1);
    end
    
    plotDetection(blazerPlotter, positions);
    title(gca, char(num2str(results(i).Time + " seconds"))); % start from 0
    grid on;
    
    % save png images
    file = strcat(pic_path,num2str(i),'.png');
    saveas(gcf,file);
     
    % write video and delete saved png images
    writeVideo(newVid,imread(file));%within the for loop saving one frame at a time
    delete(file);
end

close(newVid);
