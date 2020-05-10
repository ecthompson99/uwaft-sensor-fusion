% This is a helper function for use with
% SimulateRadarSensorsIn3DEnvironmentExample and may be changed or removed
% without notice.

%   Copyright 2019 The MathWorks, Inc.

% Loads vehicle trajectories and creates bus definition used by "From
% Workspace" blocks to drive "Simulation 3D Vehicle with Ground Following"
% blocks in the 3D simulation environment.

% Code used to extract road waypoints from section of curved road 3D
% scenario for entry in Driving Scenario Designer.
if false
    [sceneImage, sceneRef] = helperGetSceneImage('CurvedRoad'); %#ok<UNRCH>
    hUI = figure;
    helperSelectSceneWaypoints(sceneImage, sceneRef)
end

% Initialize trajectories for vehicles.
if ~exist('initTrajectories','var') || initTrajectories
    [ego,lead,follow,aggressive,oncoming1,oncoming2,simDuration] = createScenario();
    initTrajectories = false;
end

% Ensure buses exist for the radar blocks.
model = gcs;
if ~exist('BusSimulation3DRadarTruthSensor1','var') && strcmpi('on',get_param(model,'Shown'))
    open_system([model '/Ego Sensors']);
    open_system(model);
end

function [egoWP,leadWP,followWP,aggressiveWP,oncoming1WP,oncoming2WP,simDuration] = createScenario()

% Create scenario set to sample time used by sensors.
scenario = drivingScenario('SampleTime',0.1);

% Following section of vehicle definitions and trajectories are copied from
% M-script exported from Driving Scenario Designer.

% Add the ego vehicle
egoVehicle = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-294.1 5 0]);
waypoints = [-294.1 5 0;
    -277 -6 0;
    -236.3 -35 0;
    -196.2 -64.9 0;
    -154.9 -93.1 0;
    -111.2 -117.3 0;
    -64.3 -134.5 0;
    -15.1 -142.6 0;
    34.6 -139 0;
    81.8 -122.9 0;
    122.3 -93.9 0;
    156.2 -57.2 0;
    183.5 -15.4 0;
    204.2 30.1 0;
    218.3 78 0;
    225.9 127.4 0;
    227.4 177.4 0;
    222.9 227.1 0;
    213.4 276.2 0;
    199.5 324.2 0;
    182.1 371.1 0];
speed = 25;
trajectory(egoVehicle, waypoints, speed);

% Add the non-ego actors
car1 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-275.9 -6.8 0]);
waypoints = [-275.9 -6.8 0;
    -265.5 -13.9 0;
    -225.2 -43.5 0;
    -184.8 -73 0;
    -143 -100.4 0;
    -98.5 -123.1 0;
    -50.7 -137.7 0;
    -1.1 -143.1 0;
    48.3 -136.1 0;
    93.9 -116.1 0;
    132.5 -84.4 0;
    164.3 -46 0;
    189.9 -3.1 0;
    208.9 43.1 0;
    221.3 91.5 0;
    227 141.2 0;
    227.1 191.1 0;
    221 240.7 0;
    210 289.5 0;
    195.2 337.3 0;
    176.9 383.8 0];
speed = 25;
trajectory(car1, waypoints, speed);

car2 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-307 12.5 0]);
waypoints = [-307 12.9 0;
    -286.3 -0.1 0;
    -259.5 -18 0;
    -219.2 -47.7 0;
    -178.7 -77 0;
    -136.6 -103.9 0;
    -91.4 -125.3 0;
    -43.5 -139.4 0;
    6.2 -142.5 0;
    55.4 -133.8 0;
    100.2 -112.3 0;
    137.7 -79.3 0;
    168.7 -40.2 0;
    193.4 3.3 0;
    211.3 49.9 0;
    222.6 98.6 0;
    227.3 148.3 0;
    226.8 198.3 0;
    220 247.8 0;
    208.3 296.4 0;
    193 344 0;
    174.2 390.3 0];
speed = 25;
trajectory(car2, waypoints, speed);

car3 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [-312.2 20.3 0]);
waypoints = [-312.2 20.7 0;
    -297.1 11.6 0;
    -279.2 -2.77555756156289e-17 0;
    -264.2 -9.7 0;
    -229.4 -34.6 0;
    -199 -57.7 0;
    -161.9 -84 0;
    -144.1 -95.4 0;
    -140 -98.3 0;
    -123.5 -111 0;
    -120 -112.8 0;
    -93.5 -125.1 0;
    -64.3 -134.6 0;
    -50.3 -137.7 0;
    6.4 -142.6 0;
    34.6 -139 0;
    62.8 -130.7 0];
speed = 35;
trajectory(car3, waypoints, speed);

car4 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [173 -19 0]);
waypoints = [135 -64.6 0;
    116.5 -83.7 0;
    73.5 -113.7 0;
    45.5 -124.3 0;
    28.4 -128 0;
    10.3 -130.5 0;
    -17 -130.8 0;
    -41.6 -127.8 0;
    -57.2 -124.4 0;
    -80.6 -117.3 0;
    -106.2 -106.5 0;
    -133.5 -92 0;
    -163 -73.8 0;
    -192.1 -53.2 0];
speed = 25;
trajectory(car4, waypoints, speed);

car5 = vehicle(scenario, ...
    'ClassID', 1, ...
    'Position', [162 -31 0]);
waypoints = [137.9 -66.9 0;
    113 -91.7 0;
    86 -110.9 0;
    53.4 -125.8 0;
    29.6 -132 0;
    4.5 -134 0;
    -19.5 -134 0;
    -42.5 -131.6 0;
    -68.4 -125.1 0;
    -101.6 -113.2 0;
    -119.4 -104.6 0;
    -159.4 -81.4 0;
    -204.5 -49 0;
    -241.2 -22 0;
    -282.3 6.9 0;
    -314.4 26.4 0];
speed = 35;
trajectory(car5, waypoints, speed);

% Generate poses for all of the actors in the scenario.
rec = record(scenario);
simDuration = rec(end).SimulationTime;

% Time-series structs of actor poses for each actor in the scenario.
egoWP = getWayPoints(rec,1);
leadWP = getWayPoints(rec,2);
followWP = getWayPoints(rec,3);
aggressiveWP = getWayPoints(rec,4);
oncoming1WP = getWayPoints(rec,5);
oncoming2WP = getWayPoints(rec,6);

% Create actor pose buses for use with "From Workspace" blocks.
busInfo = Simulink.Bus.createObject(rec(1).ActorPoses(1));
bus = evalin('base',busInfo.busName);
assignin('base','BusActorPose',bus);
evalin('base',['clear ' busInfo.busName]);
end

function wp = getWayPoints(rec,actorIdx)
% Returns waypoints packaged in a time-series struct for use with buses and
% "From Workspace" blocks.
t = [rec.SimulationTime];
flds = fieldnames(rec(1).ActorPoses(1));
wp = struct();
for iFld = 1:numel(flds)
    thisFld = flds{iFld};
    data = reshape(cell2mat(arrayfun(@(r)r.ActorPoses(actorIdx).(thisFld),rec,'UniformOutput',false)),[],numel(t))';
    if size(data,2)>1
        data = permute(reshape(data,size(data,1),1,size(data,2)),[2 3 1]);
    end
    wp.(thisFld) = timeseries(data,t(:));
end
end
