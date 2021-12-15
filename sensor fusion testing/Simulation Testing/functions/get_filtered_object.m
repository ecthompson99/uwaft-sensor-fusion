function [time, dx, dy, vx] = get_filtered_object(bag)
% Tracked object topic tracks the 3 (by default) most prominent objects
% based on sensor fusion results. Typically, these objects will correspond 
% to the center, right, and left lane objects. 
% 
% Arguments
%   bag (rosbag file): rosbag with ROS messages published by a topic
%   num_objs (int): Value between 0 and 3. Number of objects to return.
%
% Returns
%   time (1 x N vector: Timestamps of object detection.
%   dx (num_objs x N vector): Longitudinal distance to objects (x-axis.
%   dy (num_objs x N vector): Latitude distance to objects (y-axis).
%   vx (num_objs x N vector): Speed of the target objects along x-axis.

target_topic = select(bag, 'Topic', '/filtered_obj');
target_struct = readMessages(target_topic, 'DataFormat', 'struct');
time = cellfun(@(m) m.ObjTimestamp, target_struct);
dx = cellfun(@(m) m.ObjDx, target_struct);
dy = cellfun(@(m) m.ObjDy, target_struct);
vx = cellfun(@(m) m.ObjVx, target_struct);

sorted = sortrows([time, dx, dy, vx]);

time = sorted(:, 1);
dx = sorted(:, 2);
dy = sorted(:, 3);
vx = sorted(:, 4);

end

