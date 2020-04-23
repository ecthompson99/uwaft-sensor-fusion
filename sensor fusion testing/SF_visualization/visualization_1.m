clear
clc

% Load information from rosbag
[target_A, tracked_A, target_count, target_obj, tracked_count, tracked_obj] = load_rosbag();

%% Create Figure

% Make a figure
hFigure = figure('Position', [0, 0, 1200, 640], 'Name', 'Sensor Fusion Output'); % x,y,xlength,ylength
movegui(hFigure, [0 -1]); % Moves the figure to the left and a little down from the top

% Add a panel for a bird's-eye plot
hBEVPlot = uipanel(hFigure, 'Position', [0 0 0.5 1], 'Title', 'Tracked Objects');
hCarPlot = uipanel(hFigure, 'Position', [0.5 0 0.5 1], 'Title', 'Target Objects');

% Create bird's-eye plot for the ego car and sensor coverage
hTracked = axes(hBEVPlot);
hTarget = axes(hCarPlot);

%%  Driving scenario for targets
scenario = drivingScenario;

% create empty arrays for vehicles
targets = cell(size(target_obj,1),1);
waypoints = cell(size(target_obj,1),1);
speed_calc = cell(size(target_obj,1),1);

counter = 0; % used to keep track of unique object
num_targets = double(target_count); % permanent array

for i = 1:size(target_obj,1)% loop through target_id 
    targets{i} = vehicle(scenario); % create target vehicle
    % Specify rows in target_A and specify trajectory for unique object
    waypoints{i} = [target_A((counter + 1):(counter + num_targets(i)), 2) ...
        target_A((counter + 1):(counter + num_targets(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    speed_calc{i} = [sqrt(target_A((counter + 1):(counter + num_targets(i)), 4).^2 + ...
        target_A((counter + 1):(counter + num_targets(i)), 8).^2)]; % take vx and vy in columns 4 and 8 of array
    
    counter = counter + num_targets(i);
end

for i = 1:size(target_obj,1)% loop through target_id 
    trajectory(targets{i},waypoints{i},speed_calc{i}(:,1))
end

% add a plot for debug
plot(scenario,'Parent', hTarget,'Waypoints','on')

hold on

% Start the simulation loop
while advance(scenario)
 %fprintf('The vehicle is located at %.2f m at t=%.0f ms\n', targets{1}.Position(1), s.SimulationTime*1000)
 pause(0.1)
end


%% Driving scenario for tracked objects

% create empty arrays for objects
tracked_pos = cell(size(tracked_obj,1),1);
tracked_vel = cell(size(tracked_obj,1),1);
labels = cell(size(tracked_obj,1),1);

counter_1 = 0; % used to keep track of unique object
num_tracked = double(tracked_count); % permanent array

% Create bird's eye plot
bep = birdsEyePlot('Parent', hTracked,'XLim',[0,90],'YLim',[-35,35]);
objPlotter = detectionPlotter(bep,'DisplayName','Tracked Objects', ...
    'MarkerEdgeColor','red', 'Marker','^');

for i = 1:size(tracked_obj,1)% loop through target_id 
    tracked_pos{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 2) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 5)]; % take dx and dy in columns 2 and 5 of array
    tracked_vel{i} = [tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 4) ...
        tracked_A((counter_1 + 1):(counter_1 + num_tracked(i)), 8)]; % take vx and vy in columns 4 and 8 of array
    
    counter_1 = counter_1 + num_tracked(i);
    
end

% Plot Detections
for i = 1:size(tracked_obj,1)% loop through target_id 
    plotDetection(objPlotter, tracked_pos{i}, tracked_vel{i});
end

