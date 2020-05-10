<<<<<<< HEAD
clear 
clc

%function [tracked_objects, timeStep] = load_rosbag()

% ADDINS NEEDED: 'Robotics System Toolbox' 
% Load rosbag
bag = rosbag('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\env_state.bag');

% Select topics 
bag_sel = select(bag, 'Topic', 'tracked_obj'); 

% Read msgs into cells
tracked_struct = cell2mat(readMessages(bag_sel, 'DataFormat', 'struct'));

% Create tables
tracked_T = struct2table(tracked_struct);

% Sort tables by timestamp (11th col)
tracked_time = sortrows(tracked_T,11);

% Create array: ObjId,ObjDx,ObjLane,ObjVx,ObjDy,ObjAx,ObjPath,ObjVy,ObjTimestamp 
tracked_A_time = double(table2array(tracked_time(:,3:11)));

% round timestamps to nearest 3 decimal, col 9
tracked_A_time(:,9) = round(tracked_A_time(:,9),3);

% Find unique timestamps & num of occurances (col 9)
[unique_time,~,time_c] = unique(tracked_A_time(:,9),'rows');
num_unique = accumarray(time_c,1);

startTime = 0; % in seconds
endTime = 50; % in seconds
timeStep = 1; % in seconds

% create cell array, each column is time stamp and positions are listed in
% rows
tracked_objects = cell(2,size(unique_time,1));

counter = 0;
while startTime < endTime
    
  %check if timestamp is equal to startTime
   for index = 1:size(unique_time,1)
       if unique_time(index) == startTime
%          First row of tracked_objects shows how many objects at each time
%          stamp
           tracked_objects{1,index} = num_unique(index);
           
%          Second row of tracked_objects show dx, dy of each object at a time stamp
           dx = tracked_A_time((counter + 1):(counter + num_unique(index)), 2); % dx is column 2
           dy = tracked_A_time((counter + 1):(counter + num_unique(index)), 5); % dx is column 5
           tracked_objects{2,index} = [dx,dy];
           
           counter = counter + num_unique(index);
       end
       
   end
   
   startTime = startTime + timeStep;
end



%end
=======
function [A, t_sort] = load_rosbag()

% ADDINS NEEDED: 'Robotics System Toolbox' 
% Load rosbag
bag = rosbag('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\goteam-thresh-5.bag');
%bag_info = rosbag('info','C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\goteam-range-150-10.bag');

% Select topics 
bag_sel1 = select(bag, 'Topic', 'tracked_obj'); 
%bag_sel2 = select(bag, 'Topic', 'target_obj'); 


% %Read msgs into cells
 struct1 = cell2mat(readMessages(bag_sel1, 'DataFormat', 'struct'));
 %struct2 = cell2mat(readMessages(bag_sel2, 'DataFormat', 'struct'));

% 
% % Create tables
 table1 = struct2table(struct1);
 %table2 = struct2table(struct2);

% 
% % Write to excel
%writetable(table1,'C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\goteam-thresh-5.xlsx');
%writetable(table2,'C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\target.xlsx');


% Sort in matlab
 t_col= table1(:,{'ObjId','ObjTimestamp', 'ObjDx', 'ObjDy'});
 t_unique = unique(t_col,'rows');
 t_sort = sortrows(t_unique(:,:),[1,2]);
 
 A = table2array(t_sort);
 
end
>>>>>>> 73414a2c68357ee2f4eb69809d6238083d0d3f8c
