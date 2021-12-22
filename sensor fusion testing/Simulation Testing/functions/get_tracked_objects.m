function [time, dx, dy, vx] = get_tracked_objects(bag)
% Tracked object topic tracks the 3 most prominent objects
% based on sensor fusion results. Typically, these objects will correspond 
% to the center, right, and left lane objects. 
% 
% Arguments
%   bag (rosbag file): rosbag with ROS messages published by a topic
%
% Returns
%   time (1 x N) vector: Timestamps of object detection.
%   dx (3 x N vector): Longitudinal distance to objects (x-axis).
%   dy (3 x N vector): Latitude distance to objects (y-axis).
%   vx (3 x N vector): Speed of the target objects along x-axis.

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

time = sorted(:, 1);
dx = sorted(:, 2:4);
dy = sorted(:, 5:7);
vx = sorted(:, 8:10);

end

