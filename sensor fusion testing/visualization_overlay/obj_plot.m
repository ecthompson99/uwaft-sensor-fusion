%% Plot objects in video
%load('simulation_gt.mat'); %ground_truth
load('sf_output.mat'); %sf_results
set(0,'DefaultFigureVisible','off');

newVid = VideoWriter('drive_20mph_test3.avi');
newVid.FrameRate = 50;
newVid.Quality = 100;
open(newVid);

% gt_offset = 6;

for i = 1:100:size(sf_results,2) % sample plot for now (start:iteration:end)

    bep = birdsEyePlot('XLim',[0,150],'YLim',[-15,15]);
%     blazerPlotter = detectionPlotter(bep,'MarkerEdgeColor','black','DisplayName','Ground Truth Objects', 'Marker', 'o');
    sfPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Sensor Fusion Objects', 'Marker', '+');

%     blazer_num_obj = size(ground_truth(i+gt_offset).Objects,1);
%     blazer_positions = zeros(blazer_num_obj);
    sf_positions = zeros(sf_results(i).Num_Objects,2);

%     for j = 1:blazer_num_obj
%         blazer_positions(j,1:2) = ground_truth(i+gt_offset).Objects(j,:);
%     end
    
    for j = 1:sf_results(i).Num_Objects
        sf_positions(j,1:2) = sf_results(i).Objects(1,j).Measurement(:,1);
    end
    
%     plotDetection(blazerPlotter, blazer_positions);
    plotDetection(sfPlotter, sf_positions);

    title(gca, char(num2str(sf_results(i).Time + " seconds"))); 
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid,frame);
end

close(newVid);