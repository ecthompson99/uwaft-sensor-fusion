load('mobileye2.mat');

set(0,'DefaultFigureVisible','off');

newVid = VideoWriter('me_lane2.avi');
newVid.FrameRate = 10;
newVid.Quality = 100;
open(newVid);

plot_time = 0;
ego_positions = [];
adj_positions = [];
inv_positions = [];

for i = 1:3:height(me_final)
    bep = birdsEyePlot('XLim',[0,150],'YLim',[-15,15]);
    egoPlotter = detectionPlotter(bep,'MarkerEdgeColor','black','DisplayName','Ego Lane', 'Marker', 'o');
    adjPlotter = detectionPlotter(bep,'MarkerEdgeColor','red','DisplayName','Adjacent Lane', 'Marker', 'o');
    sensorPlotter = detectionPlotter(bep,'MarkerEdgeColor','blue','DisplayName','Invalid','Marker','+');
    
    fields_A=fieldnames(me_final.Signals{i,1});
    fields_B=fieldnames(me_final.Signals{i+1,1});
    me_dx = getfield(me_final.Signals{i,1},fields_A{9});
    me_dy = getfield(me_final.Signals{i,1},fields_A{8});
    me_lane = getfield(me_final.Signals{i+1,1},fields_B{6});
    
    if me_lane == 1
       ego_positions = [ego_positions; me_dx me_dy];
    elseif me_lane == 2
       adj_positions = [adj_positions; me_dx me_dy];
    else
       inv_positions = [inv_positions; me_dx me_dy];
    end
    
    if me_final.time_in_sec(i) > plot_time+0.1
        if ~isempty(ego_positions)
            plotDetection(egoPlotter, ego_positions);
        end
        
        if ~isempty(adj_positions)
            plotDetection(adjPlotter, adj_positions);
        end
        
        if ~isempty(inv_positions)
            plotDetection(sensorPlotter, inv_positions);
        end
                
        plot_time = plot_time + 0.1;
        
        ego_positions = [];
        adj_positions = [];
        inv_positions = [];

    end

    title(gca, plot_time);
    grid on;
    
    frame = getframe(gcf);
    writeVideo(newVid,frame);
end

close(newVid);