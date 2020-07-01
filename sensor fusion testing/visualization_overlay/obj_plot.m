%% Plot objects in video
load('ground_truth/FTR_ground_truth.mat');
load('sensor_fusion/sf_output.mat');
set(0,'DefaultFigureVisible','off');

newVid = VideoWriter('tracked_objects.avi');
newVid.FrameRate = 1;
newVid.Quality = 100;
open(newVid);

for i = 10:10:3090%size(results,2) % sample plot for now (start:iteration:end)

    bep = birdsEyePlot('XLim',[0,100],'YLim',[-5,10]);
    blazerPlotter = detectionPlotter(bep,'MarkerEdgeColor','black','DisplayName','Ground Truth Objects', 'Marker', 'o');
    sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');

    blazer_positions = zeros(results(i).Num_Objects,2);
    sf_positions = zeros(sf_results(i).Num_Objects,2);

    for j = 1:results(i).Num_Objects
        blazer_positions(j,1:2) = results(i).Objects(1,j).Measurement(:,1);
    end
    
    for j = 1:sf_results(i).Num_Objects
        sf_positions(j,1:2) = sf_results(i).Objects(1,j).Measurement(:,1);
    end
    
    plotDetection(blazerPlotter, blazer_positions);
    plotDetection(sfPlotter, sf_positions);

    title(gca, char(num2str(results(i).Time + " seconds"))); 
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid,frame);
end

close(newVid);