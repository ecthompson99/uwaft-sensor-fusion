clear
clc

%% load csv file
% T = readtable('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\goteam-thresh-5.csv');
% A = table2array(T);

[A, T] = load_rosbag();

start_time = 0;
end_time = round(max(T{:,2})); % max end time of entire data set
time_step = 1; % polling frequency
master_time = 1; % used to create rows in master_table

% fill first column of master_table with timestamps
while start_time <= end_time
    master_table(master_time,:) = start_time;
    
    start_time = start_time + time_step;
    master_time = master_time + 1;
end

[unique_obj,~,obj_c] = unique(A(:,1),'rows');
num_obj = accumarray(obj_c,1);

object_info = [unique_obj, num_obj]; % unique objects vs object count


%% Master table

obj_row = 0;
row_proc = 0;

end_obj = zeros(size(num_obj,1),1); % keeps track of end time of object


for obj_index = 1:size(num_obj,1) % loop through rows
%     copy = table('Size',[num_obj(obj_index)+obj_row 4],'VariableTypes',...
%         ["double","double","double","double"]);
    copy = T(obj_row + 1:(num_obj(obj_index)+obj_row),:); % make a copy for all columns of 1 object into array
    
    end_obj(obj_index) = max(copy.ObjTimestamp); % set the end index to the max timestamp of an object
    
    start_time = 0;
    tol_s = 0;
    tol_e = time_step;
    row_index = 1;
    
    while start_time <= end_time    % loop time
        % make a copy for times within bin range (time_step range)
        copy_a = copy((copy.ObjTimestamp > tol_s & copy.ObjTimestamp < tol_e),:); 

        x_total = 0;
        y_total = 0;
        count = 0;

        for i = 1:size(copy_a)
            x_total = x_total + table2array(copy_a(i,3)); % dx total
            y_total = y_total + table2array(copy_a(i,4)); % dy total
            count = count + 1;
        end

        if count > 0
            x_ave = x_total/count; % x average calculation for values in bin
            y_ave = y_total/count; % y average calculation for values in bin

            row_index = cast(tol_e*(1/time_step),'int32');
            
            master_table(row_index , 2 + 2*(obj_index-1)) = x_ave;  % fill master table with average dx and dy
            master_table(row_index , 3 + 2*(obj_index-1)) = y_ave;
        end

        %increment counting variables
        start_time = start_time + time_step;
        tol_s = tol_s + time_step;
        tol_e = tol_e + time_step;
    end
    row_proc = num_obj(obj_index);
    obj_row = row_proc + obj_row;
end

non_0_coord = zeros(size(num_obj,1)*2,1); % used to make a copy of a row in the master table (1 + 2*num_obj columns)

for row_idx = 1:size(master_table,1) % loop through rows
    if row_idx == 1 % make direct copy if first row
        cum_master(row_idx,:) = master_table(row_idx,:); 
    else
        % copy time (first column of master_table)
        cum_master(row_idx,1) = master_table(row_idx,1); 
        
        % loop through columns
        for col_idx = 2:(size(num_obj,1)*2+1) % +1 needed to consider last column
            if mod(col_idx,2) == 0 % ifcolumn is even
                if master_table(row_idx,1) <= end_obj(col_idx/2,1) % half of col = obj idx
                    if master_table(row_idx,col_idx) ~= 0 % if cell is not 0, fill non_0_coord with most recent row
                        non_0_coord(col_idx-1) = master_table(row_idx,col_idx);
                        cum_master(row_idx,col_idx) = master_table(row_idx,col_idx);
                    else % if cell is 0, fill with non_0_coord value
                        cum_master(row_idx,col_idx) = non_0_coord(col_idx-1);
                    end 
                end
            else % odd row
                if master_table(row_idx,1) <= end_obj((col_idx-1)/2,1) % -1/half of col = obj idx
                    if master_table(row_idx,col_idx) ~= 0 % if cell is not 0, fill non_0_coord with most recent row
                        non_0_coord(col_idx-1) = master_table(row_idx,col_idx);
                        cum_master(row_idx,col_idx) = master_table(row_idx,col_idx);
                    else % if cell is 0, fill with non_0_coord value
                        cum_master(row_idx,col_idx) = non_0_coord(col_idx-1);
                    end 
                end
            end
        end        
    end
end

%% Plot objects in video

set(0,'DefaultFigureVisible','off'); % set visability of automatic figures off

newVid = VideoWriter('C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\recordings\test1.avi'); % New video
newVid.FrameRate = 1/time_step; % set framerate relative to time_step
newVid.Quality = 100;
open(newVid);

path = 'C:\Users\sophy\Documents\GitHub\kaiROS\sensor fusion testing\SF_visualization\plots\plot';

x = 0; % temp x
y = 0; % temp y

% [M,I] = max(A); 
% [M_,I_] = min(A);

% get limits of plotting bounds
% maxX = M(3)+2;
% minX = M_(3)-2;
% maxY = M(4)+2;
% minY = M_(4)-2;

% end at target_time + 1 (row)
for i = 1:size(master_table,1)
    bep = birdsEyePlot('XLim',[0,100],'YLim',[-5,10]); % create BEP
    radarPlotter = detectionPlotter(bep,'DisplayName','Tracked Objects',... %settings for plotter
       'Marker', '+');
    
    % create coordinates array
    coordinates = zeros(10,2);
    
    for j = 1:size(num_obj,1)
        x = cum_master(i, 2*j); % fill x with even columns
        y = cum_master(i, 2*j + 1); % fill y with odd columns
        coordinates(j,1) = x; % set coordinates with x and y
        coordinates(j,2) = y;
        labels{j} = num2str(unique_obj(j)); % set labels cell array with objId
    end
    
    % fill position array with coordinates
    for k = 1:size(unique_obj)
        if coordinates(k,1) ==0 && coordinates(k,2) == 0
            positions(k,:) = [NaN,NaN]; % don't fill positions if coordinates = (0,0)
        else
            positions(k,:) = [coordinates(k,1), coordinates(k,2)];
        end
    end      
      
    % plot detections with title
    plotDetection(radarPlotter, positions, labels); 
    
%     Road markings
    axh = gca; % use current axes
    color = 'k'; % black, or [0 0 0]
    linestyle = '-.'; % dotted
    line(get(axh,'XLim'), [-1.65 -1.65], 'Color', color, 'LineStyle', linestyle);
    line(get(axh,'XLim'), [1.65 1.65], 'Color', color, 'LineStyle', linestyle);

    title(gca, char(num2str(i*time_step) + " seconds")); % start from first time step
    grid on;
     
    % save png images
    file = strcat(path,num2str(i),'.png');
    saveas(gcf,file);
     
    % write video and delete saved png images
    writeVideo(newVid,imread(file));%within the for loop saving one frame at a time
    delete(file);
end


close(newVid);




