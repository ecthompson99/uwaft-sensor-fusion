function [time, dx, dy, vx] = get_tracked_objects(bag, num_objs)
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

tracked_topic = select(bag, 'Topic', '/tracked_obj');
tracked_struct = readMessages(tracked_topic, 'DataFormat', 'struct');

num_timestamps = size(tracked_struct, 1);
dx = zeros(num_timestamps, 3);
dy = zeros(num_timestamps, 3);
vx = zeros(num_timestamps, 3);

for c = 1:3
    time = cellfun(@(m) m.ObjTimestamp(c), tracked_struct);
    dx(:, c) = cellfun(@(m) m.ObjDx(c), tracked_struct);
    dy(:, c) = cellfun(@(m) m.ObjDy(c), tracked_struct);
    vx(:, c) = cellfun(@(m) m.ObjVx(c), tracked_struct);
end

sorted = sortrows([time, dx, dy, vx]);

offset = 0;
if num_objs <= 3 && num_objs >= 0
    offset = 3-num_objs;
end

time = sorted(:, 1);
dx = sorted(:, 2:4-offset);
dy = sorted(:, 5:7-offset);
vx = sorted(:, 8:10-offset);

end

