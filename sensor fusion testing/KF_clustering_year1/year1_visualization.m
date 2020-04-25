%[scenario, sensors, egoCar] = generateScenario1();
logFilename = "ACCGapTestLong_20191103_160048_CAN.mat"; %CAN Log .mat File to load

tracker = multiObjectTracker('FilterInitializationFcn', @initKF, ...
    'AssignmentThreshold', 27, 'ConfirmationParameters', [4 5]);

blazerRadarObjectsName = "Blazer Radar Objects"; %Legend label for obstacles
blazerRadarObjectsMarker = '*';%Plot marker for radar objects

blazerVisionObjectsName = "Blazer Vision Objects"; %Legend label for obstacles
blazerVisionObjectsMarker = '+';%Plot marker for radar objects

contrastingColors = [[0 0.4470 0.7410];...
                [0.8500 0.3250 0.0980];[0.9290 0.6940 0.1250];...
                [0.4940 0.1840 0.5560];[0.4660 0.6740 0.1880];...
                [0.3010 0.7450 0.9330];[0.6350 0.0780 0.1840]];

blazerRadar = DetectionGroup(logFilename, blazerRadarObjectsName, ... 
    "FLRRTrk.*Azimuth", "FLRRTrk.*Range$", blazerRadarObjectsMarker ...
    contrastingColors(2, :));
blazerVision = DetectionGroup(logFilename, blazerVisionObjectsName, ...
    "FwdVsnAzmthTrk1Rev", "FwdVsnRngTrk1Rev", blazerVisionObjectsMarker ...
    contrastingColors(3, :));

timestepSize = 0.1; %Timestep size in seconds.determined by the blf to .mat 
                    %conversion
                    
results = struct('Time',[],'Tracks', []);
current_step = 1;

while advance(scenario) && ishghandle(BEP.Parent)
    % Get the scenario time
    time = scenario.SimulationTime;

    % Update the tracker if there are new detections
    if any(isValidTime)
        vehicleLength = 4.7;
        detectionObjects = clusterDetections(detections, vehicleLength);
        confirmedTracks = updateTracks(tracker, detectionObjects, time);

    end
    
    % Save all data into a struct so the values can be checked after the
    % simulation
    results(current_step).Time = time;
    results(current_step).Tracks = confirmedTracks;

    current_step = current_step + 1;
end 

calcBinaryClassification(results, current_step);