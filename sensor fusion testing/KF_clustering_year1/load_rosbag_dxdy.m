clear 
clc

%function [tracked_objects, timeStep] = load_rosbag()

% ADDINS NEEDED: 'Robotics System Toolbox' 
% Load rosbag
bag = rosbag('C:\Users\Jillian\Documents\UWAFT\kaiROS\sensor fusion testing\KF_clustering_year1\2020-04-25-19-50-08.bag');

% Select topics 
bag_sel = select(bag, 'Topic', 'binary_class'); 

% Read msgs into cells
tracked_struct = cell2mat(readMessages(bag_sel, 'DataFormat', 'struct'));

% Create tables
tracked_T = struct2table(tracked_struct);

% Create array: ObjId,ObjDx,ObjLane,ObjVx,ObjDy,ObjAx,ObjPath,ObjVy,ObjTimestamp 
tracked_A_time = double(table2array(tracked_time(:,3:11)));




%end
