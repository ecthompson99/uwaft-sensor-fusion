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
