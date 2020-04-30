%% Plot objects in video
load('ground_truth_test10.mat');
set(0,'DefaultFigureVisible','off');

vid_path = strcat(pwd, '\KF_clustering_year1\blazer_sensors_test10.avi');
newVid = VideoWriter(vid_path);
newVid.FrameRate = 1;
newVid.Quality = 100;
open(newVid);

pic_path = strcat(pwd, '\KF_clustering_year1\plot');


for i = 1:10:size(results,2) % sample plot for now

    bep = birdsEyePlot('XLim',[0,90],'YLim',[-35,35]);
    blazerPlotter = detectionPlotter(bep,'DisplayName','Blazer Objects', 'Marker', 'o');
    
    positions = zeros(results(i).Num_Objects,2);
    for j = 1:results(i).Num_Objects
        positions(j,1:2) = results(i).Objects(1,j).Measurement(:,1);
    end
    
    plotDetection(blazerPlotter, positions);
    title(gca, char(num2str(results(i).Time + " seconds"))); % start from 0
    grid on;
    
    % save png images
    file = strcat(pic_path,num2str(i),'.png');
    saveas(gcf,file);
     
    % write video and delete saved png images
    writeVideo(newVid,imread(file));%within the for loop saving one frame at a time
    delete(file);
end

close(newVid);