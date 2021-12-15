function [time, dx, dy, vx] = get_target_object(bag)
% Tracked object topic tracks the three most prominent objects, in the
% center, right, and left lanes. This function will output the dx, dy,
% time, and vx values

target_topic = select(bag, 'Topic', '/target_obj');
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

