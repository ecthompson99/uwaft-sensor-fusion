%% Plot objects in video
load('FTR_ground_truth.mat');
set(0,'DefaultFigureVisible','off');

newVid = VideoWriter('tracked_objects.avi');
newVid.FrameRate = 1;
newVid.Quality = 100;
open(newVid);

for i = 10:10:100%size(results,2) % sample plot for now (start:iteration:end)

    bep = birdsEyePlot('XLim',[0,100],'YLim',[-5,10]);
    blazerPlotter = detectionPlotter(bep,'DisplayName','Blazer Objects', 'Marker', 'o');
    
    positions = zeros(results(i).Num_Objects,2);
    for j = 1:results(i).Num_Objects
        positions(j,1:2) = results(i).Objects(1,j).Measurement(:,1);
    end
    
    plotDetection(blazerPlotter, positions);
    title(gca, char(num2str(results(i).Time + " seconds"))); % start from 0
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid,frame);
end

close(newVid);