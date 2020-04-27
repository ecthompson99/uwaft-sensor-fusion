clear all
close all
clc 

%[scenario, sensors, egoCar] = generateScenario1();
logFilename = "ACCGapTestLong_20191103_160048_CAN.mat"; %CAN Log .mat File to load

% tracker = multiObjectTracker('FilterInitializationFcn', @initKF, ...
%     'AssignmentThreshold', 27, 'ConfirmationParameters', [4 5], ...
%     'MaxNumSensors', 2); %blazer radar and vision sensors

blazerRadarObjectsName = "Blazer Radar Objects"; %Legend label for obstacles
blazerRadarObjectsMarker = '*';%Plot marker for radar objects

blazerVisionObjectsName = "Blazer Vision Objects"; %Legend label for obstacles
blazerVisionObjectsMarker = '+';%Plot marker for radar objects

detectionMarkers = [blazerRadarObjectsMarker, blazerVisionObjectsMarker];

contrastingColors = [[0 0.4470 0.7410];...
                [0.8500 0.3250 0.0980];[0.9290 0.6940 0.1250];...
                [0.4940 0.1840 0.5560];[0.4660 0.6740 0.1880];...
                [0.3010 0.7450 0.9330];[0.6350 0.0780 0.1840]];

blazerRadarObjects = DetectionGroupY1(logFilename, ...
    blazerRadarObjectsName, "FLRRTrk.*Azimuth", "FLRRTrk.*Range$", ...
    detectionMarkers(1), contrastingColors(1, :));
blazerVisionObjects = DetectionGroupY1(logFilename, ...
    blazerVisionObjectsName, "FwdVsnAzmthTrk1Rev", "FwdVsnRngTrk1Rev", ...
    detectionMarkers(2), contrastingColors(2, :));

%create matrices with row columns, dx/dy, each column is a single timestep
radarObjectPositions = blazerRadarObjects.positions;
visionObjectPositions = blazerVisionObjects.positions;

timestepSize = 0.1; %Timestep size in seconds.determined by the blf to .mat 
                    %conversion
                    
results = struct('Time',[],'Objects', [], 'Num_Objects', []);
current_step = 1;
time = 0;
num_frames = size(radarObjectPositions,2); %Number of timesteps

for i = 1:num_frames
    % Update the tracker at each timestep
    detections = objectDetection.empty(10,0);
    
    % Add vision objects to detections
    measurement = cell2mat(blazerVisionObjects.positions(1,1));
    visionObject = objectDetection(time,measurement);
    detections(1) = visionObject;
    
    % Add radar objects to detections
    radarTime = cell2mat(blazerRadarObjects.positions(1,i));
    for j = 1:size(radarTime,1)
        measurement = radarTime(j,1:2);
        if abs(measurement(1,1))<35 || measurement(1,2)<90
            object = objectDetection(time,measurement);
            detections(j) = object;
        end
    end
    
    % Detection clustering
    vehicleLength = 4.7;
    detectionObjects = clusterDetectionsY1(detections, vehicleLength);
%     confirmedTracks = updateTracks(tracker, detectionObjects, time);
    
    % Save all data into a struct so the values can be checked after the
    % simulation
    results(current_step).Time = time;
    results(current_step).Objects = detectionObjects;
    results(current_step).Num_Objects = size(detectionObjects,2);

    current_step = current_step + 1;
    time = time + timestepSize;
end 


% Create bird's eye plot
bep = birdsEyePlot('XLim',[0,90],'YLim',[-35,35]);

trackedPlotter = detectionPlotter(bep,'DisplayName','Blazer Objects', 'Marker', 'o');

for i = 1:num_frames
    positions = zeros(results(i).Num_Objects,2);
    for j = 1:results(i).Num_Objects
        positions(1,1:2) = results(i).Objects(1,j).Measurement;
    end
    plotDetection(trackedPlotter, positions);
    pause(0.1);
end
