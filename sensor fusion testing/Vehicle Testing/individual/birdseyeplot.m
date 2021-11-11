%% video stuff------------------------------------------------------------
%this script takes in properly incremented data from sensor fusion and oxts
%(has to be incremented so that the data is syncronized when played
%together) and plots on a graph and records a video.

close all
clear all;
load('./mat files/sf_interp.mat');
load('./mat files/oxts_interp.mat');

%finding wether oxts or sensor fusion data is longer
if length(oxts_time) >= length(sf_time)
    max_time = oxts_time;
else
    max_time = sf_time;
end

set(0,'DefaultFigureVisible','on');
newVid = VideoWriter('test2_10mph.avi'); %creating the video
newVid.FrameRate = 100; %setting the frame rate
newVid.Quality = 10; %setting video quality
open(newVid);
bep = birdsEyePlot('XLim',[0,120],'YLim',[-20,20]); %making the graph to plot on
%setting up information about sf and oxts objects for colour, name, label
sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');
oxtsPlotter = detectionPlotter(bep,'MarkerEdgeColor','blue','DisplayName','Ground Truth Object', 'Marker', 'square');
for i = 1:length(max_time) 
    
    %this is done so that the old plotted point doesn't stay on birdseyeplot
    plotDetection(sfPlotter, [NaN NaN]);
    plotDetection(oxtsPlotter, [NaN NaN]);
    
    
    %FOR OXTS
    if i <= length(oxts_time) %plot as long as there is data coming from relpos_oxts_interp
        oxts_positions = [relpos_oxts_interp(i,2) relpos_oxts_interp(i,3)]; 
    end
    
    %SENSOR FUSION STUFF
    if i <= length(sf_time)
        if ~isnan(sf_clean_interp.Num_Objects(i)) %checking if there is something to plot   
            sf_positions = NaN(max_objects,2); %making an empty array for the coordinates of all the objects
            for j = 1:sf_clean_interp.Num_Objects(i)%sf_clean_interp.Num_Objects(i)       %repeat for the amount of objects
                %the first brackets tell you the column and row number, the second bracket is 
                %always going to be {1,1} meaning the first cell and the last bracket is for 
                %which value in 'Measurement'.
                sf_positions(j,1:2) = sf_clean_interp{i,j+2}{1}.Measurement(1:2); 
%                 sf_range((i-t),j) = sqrt((sf_positions(j,1))^2 + (sf_positions(j,2))^2); %filling in NaN array with sf range data for each object
%                 sf_rangeRate((i-t),j) = (sf_range((i-t),j) - sf_range(i-t-1,j))/increment; %calculating the range rate for each object and filling in array
            end
            plotDetection(sfPlotter, sf_positions); %plotting sf objects onto birds eye plot
        end
    end

    plotDetection(oxtsPlotter, oxts_positions); %plotting ground truth data onto birds eye plot
    
    %setting the title and the clock in the title
    title(gca, char(num2str(round(max_time(i),1) + " seconds"))); 
    grid on;
    frame = getframe(gcf);
    writeVideo(newVid, frame);
end

close(newVid);