%% Simulate Radar Sensors in 3D Environment
% This example shows how to implement a synthetic data simulation for
% tracking and sensor fusion in Simulink(R) with the Automated Driving
% Toolbox(TM) using the 3D simulation environment. It closely follows the
% <docid:driving_examples#mw_8ae0eb21-efca-4a6d-9a4a-0415a638c554 Sensor
% Fusion Using Synthetic Radar and Vision Data in Simulink> example. 
%%

% Copyright 2019 The MathWorks, Inc.

%% Introduction
% Automated Driving Toolbox provides tools for authoring, simulating, and
% visualizing virtual driving scenarios. With these scenarios, you can
% simulate rare and potentially dangerous events, generate synthetic radar
% detections from the scenarios, and use the synthetic detections to test
% vehicle algorithms. This example covers the entire synthetic data
% workflow in Simulink using the 3D simulation environment.

%% Setup and Overview of Model
% Prior to running this example, the roads, actors, and trajectories in the
% scenario were created using this procedure:
%%
% 
% # Extract the center locations from a portion of the road in the
% <docid:driving_examples#mw_bdf810cf-9986-48fa-9e34-793c2f7d4a2d Curved
% Road> 3D scene, using the techniques introduced in
% <docid:driving_examples#mw_f780d508-aca7-46bb-b440-40ec9e6ff0af Select
% Waypoints for 3D Simulation>.
% # Create a road in the
% <docid:driving_ref#mw_07e6310f-b9c9-4f4c-b2f9-51e31d407766 Driving
% Scenario Designer> that has these extracted locations as its road center
% values.
% # Define multiple moving vehicles on the road that have trajectories
% similar to the ones in the scenario defined in
% <docid:driving_examples#mw_8ae0eb21-efca-4a6d-9a4a-0415a638c554 Sensor
% Fusion Using Synthetic Radar and Vision Data in Simulink>.
% # Export the trajectories from the app and load them into the MATLAB(R)
% workspace by using the |helperCreateVehicleTrajectories| script.
% # Read these trajectories into the Simulink model using
% <docid:simulink_ref#f7-890184 From Workspace> blocks.
%
% The actor poses provided by the From Workspace blocks are used by the
% <docid:driving_ref#mw_32cd8e72-2d69-4c3e-98b0-5b918db383a4 Simulation 3D
% Vehicle with Ground Following> blocks to define the locations of the ego
% vehicle, lead vehicle, and other vehicles at each time step of the
% simulation.

%%
close;
if ~ispc
    error(['3D Simulation is supported only on Microsoft', char(174), ' Windows', char(174), '.']);
end

open_system('SimulateSensorsIn3DEnvironmentModel');

%% Simulating Sensor Detections
% In this example, you simulate an ego vehicle that has six radar sensors
% covering the full 360 degrees field of view. The ego vehicle is equipped
% with a long-range radar on both the front and rear of the vehicle. Each
% side of the vehicle has two short-range radars, each covering 90 degrees.
% One radar on each side covers from the middle of the vehicle to the back.
% The other radar on each side covers from the middle of the vehicle
% forward.
%
% The Ego Sensors subsystem contains the six
% <docid:driving_ref#mw_a657f4e8-85c3-41ea-8d9c-1146805720f0 Simulation 3D
% Probabilistic Radar> blocks that model the previously described radars.
% The outputs of the radar blocks are concatenated using a
% <docid:driving_ref#mw_d736c020-7eef-4c00-9248-9762e9746af5 Detection
% Concatenation> block to create a single stream of detections to be fused
% by the <docid:driving_ref#mw_1cb02a28-0c4b-491e-aa45-55747c487a25
% Multi-Object Tracker> block in the top-level model.

%%
open_system('SimulateSensorsIn3DEnvironmentModel/Ego Sensors')

%%
% The probabilistic radars "see" not only an actor's physical dimensions
% (e.g., length, width, and height) but are also sensitive to an actor's
% _electrical_ size. An actor's electrical size is referred to as its radar
% cross-section (RCS). The RCS patterns for the vehicles in the simulation
% are defined using the
% <docid:driving_ref#mw_875dee8d-fc18-4f3d-8eab-bace559a0d66 Simulation 3D
% Probabilistic Radar Configuration> block.
% 
% <<../SimulateSensorsIn3DEnvironment_ProbRadarConfigDialog.png>>
%

%%
% Use this block to define the RCS patterns for all of the actors in the
% simulation. Any actors that do not have a specified RCS pattern use the
% default RCS value.

%% Tracking and Sensor Fusion
% The detections generated by the ego vehicle's suite of radars are
% preprocessed using a helper Detection Clustering block before they are
% fused using the
% <docid:driving_ref#mw_1cb02a28-0c4b-491e-aa45-55747c487a25 Multi-Object
% Tracker> block. The multi-object tracker is configured with the same
% parameters used in the corresponding Simulink example,
% <docid:driving_examples#mw_8ae0eb21-efca-4a6d-9a4a-0415a638c554 Sensor
% Fusion Using Synthetic Radar and Vision Data in Simulink>. The output
% from the Multi-Object Tracker block is a list of confirmed tracks.

%% Display
% The <docid:driving_ref#mw_59742eb7-dce8-4938-9c2e-44d34c7b8891 Bird's-Eye
% Scope> is a model-level visualization tool in Simulink opened from the
% Simulink toolstrip. After opening the scope, click *Find Signals* to set
% up the signals. Then run the simulation to display the ego actor, radar
% detections, and tracks. The following image shows the scope's display for
% this example.
% 
% <<../SimulateSensorsIn3DEnvironment_BirdsEyeScope.png>>
% 

%%
% When the simulation starts, a few seconds are needed to initialize the 3D
% simulation environment, especially when running it for the first time.
% Once this initialization is complete, the 3D simulation environment opens
% in a separate window. The following image is a snapshot of the 3D
% simulation window corresponding to the snapshot of the Bird's-Eye Scope
% shown in the previous image.
% 
% <<../SimulateSensorsIn3DEnvironment_Sim3DDisplay.png>>

%%
% The simulated vehicles are shown in the 3D simulation window. The
% detections and tracks generated by the 3D simulation appear only in the
% Bird's-Eye Scope. The following animation shows the detections and tracks
% for the simulated scenario.
%
% <<../SimulateSensorsIn3DEnvironment_BirdsEyeScope.gif>>


%% Summary
% In this example, you learned how to extract road centers from a 3D
% scenario for use in the Driving Scenario Designer app. You also learned
% how to export the vehicle trajectories created from the road segments for
% use in the 3D simulation environment in Simulink. You then learned how to
% configure multiple probabilistic radar models in the 3D environment and
% how to fuse the detections from the multiple radars located around the
% ego vehicle's perimeter using a multi-object tracker. The confirmed
% tracks generated by the tracker can then be used for control algorithms
% such as adaptive cruise control (ACC) or forward collision warning (FCW).

%%
close_system('SimulateSensorsIn3DEnvironmentModel');