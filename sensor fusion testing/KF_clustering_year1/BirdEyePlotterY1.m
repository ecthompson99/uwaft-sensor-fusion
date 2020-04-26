classdef BirdEyePlotterY1 %Convenience class for plotting Mobileye obstacles
    % and radar targets from CAN Logs
    properties (SetAccess = private)
        detectionGroups %Vector of DetectionGroups. Used for radarTargets
        carOutline %A single outlineGroup to represent the car
        
        xlims %The limits of the x-axis
        ylims %The limits of the y-axis
        birdPlot %The BirdsEyePlot object
        timestepSize %The size of a timestep in seconds
        numTimesteps %The number of timesteps for which there is data
    end
    methods
        %%
        % Constructor
        function obj = BirdsEyePlotter (detectionGroups,...
                timestepSize)
            
            validateattributes(detectionGroups, {'DetectionGroup'},...
                {'vector'}, "BirdsEyePlotter", "detectionGroups");
            validateattributes(timestepSize, {'double'}, {'scalar'},...
                "BirdsEyePlotter", "timestepSize");
            
            %Set number of timesteps to the length of the longest position
            %array
            obj.timestepSize = timestepSize;
            obj.detectionGroups = detectionGroups;
            
            if ~isempty(obj.detectionGroups)
                maxTimesteps = -inf;
                for i = 1:numel(obj.detectionGroups)
                    if numTimesteps(obj.detectionGroups(i)) > maxTimesteps
                        maxTimesteps = numTimesteps(obj.detectionGroups(i));
                    end
                end
                obj.numTimesteps = maxTimesteps;
            else
                obj.numTimesteps = 0;
            end
            
            %Initialize the car outline
            carwidth = convlength(72, 'in', 'm');
            carlength = convlength(184, 'in', 'm');
            obj.carOutline = OutlineGroup("Our Car", [ -carlength - 0.5,0], ...
                0, carlength, carwidth,...
                [0 0 0]);
            
            %Set x and y limits to the maximum and minimum x and y
            %coordinates in the position data across all plotted objects
            detectLims = zeros(numel(obj.detectionGroups) +...
                numel(obj.carOutline), 4);
            for i = 1:numel(obj.detectionGroups)
                [detectLims(i, 1:2), detectLims(i, 3:4)] = bounds(obj.detectionGroups(i));
            end
            endIdx = numel(obj.detectionGroups);
            detectLims(end, 1:2) = obj.carOutline.positions(1) * [1,1] +...
                obj.carOutline.lengths/2 * [-1, 1];
            detectLims(end, 3:4) = obj.carOutline.positions(2) * [1,1] +...
                obj.carOutline.widths/2 * [-1, 1];
            
            obj.xlims = [min(detectLims(:, 1)), max(detectLims(:, 2))] + [-10, 10];
            obj.ylims = [min(detectLims(:, 3)), max(detectLims(:, 4))] + [-10, 10];
        end
        %%
        % Set the limits of the axes explicitly
        function obj = setLimits (obj, maxx, maxy)
            obj.xlims = [-10, maxx];
            obj.ylims = [-maxy, maxy];
        end
        %%
        % Initialize contents of BirdsEyePlot 
        function obj = initializeBirdsEyePlot(obj)
            obj.birdPlot = birdsEyePlot('XLim', obj.xlims, 'YLim', obj.ylims);
            obj.carOutline = addToBirdsEyePlot(obj.carOutline, obj.birdPlot);
            
            for i = 1:numel(obj.detectionGroups)
                obj.detectionGroups(i) =...
                    addToBirdsEyePlot(obj.detectionGroups(i), obj.birdPlot);
            end
        end
        %%
        % Plot the object positions at a specific timestep
        function obj = updateBirdsEyePlot(obj, timestepNum)
            hold on;
            plotGroup(obj.carOutline);
            
            for i = 1:numel(obj.detectionGroups)
                plotGroup(obj.detectionGroups(i), timestepNum);
            end
            hold off;
        end
        %%
        % Turns a seconds mark into the closest index in the arrays that
        % are indexed by timestep
        function val = roundToTimestepIdx(obj, val)
            if(val <= 0)
                val = 1;
                return;
            end
            val = cast(round(val / obj.timestepSize), 'uint64') + 1;
            if (val > obj.numTimesteps)
                val = obj.numTimesteps;
            end
        end
        %%
        % Replays the stored data between startSeconds and endSeconds
        function obj = replay(obj, startSeconds, endSeconds)
            validateattributes(startSeconds, {'double'}, {'scalar'},...
                "BirdsEyePlotter", "startSeconds");
            validateattributes(endSeconds, {'double'}, {'scalar'},...
                "BirdsEyePlotter", "endSeconds");
            
            %Create a new file with a unique name
            dateTime = clock;
            dateString = "";
            for i = 1:numel(dateTime)
                dateString =  dateString + "_" + num2str(round(dateTime(i)));
            end
            
            vidWriter = VideoWriter(char('recordings/RadarPlotRecording' + dateString + '.mp4'), 'MPEG-4');
            vidWriter.FrameRate = 1/obj.timestepSize;
            vidWriter.Quality = 50;
            open(vidWriter);
            
            % Set initialize the plot
            obj = initializeBirdsEyePlot(obj);
            fig = gcf;
            ax = gca;
            grid on;
            
            startTimestep = roundToTimestepIdx(obj, startSeconds);
            endTimestep = roundToTimestepIdx(obj, endSeconds);
            
            COUNT_FRAMES = 0;
            % Plot the data
            for i = startTimestep:endTimestep
                
                if(any(isnan(obj.birdPlot.XLimits)))
                    close(vidWriter);
                    close(fig);
                    return;
                end
                obj = updateBirdsEyePlot(obj, i);
                
                title(ax, char(num2str(i*obj.timestepSize) + " seconds"));
                
                drawnow;
                writeVideo(vidWriter, getframe(gcf));
                COUNT_FRAMES = COUNT_FRAMES + 1;
            end
            close(vidWriter);
            close(fig);
            disp("NUM_FRAMES: " + COUNT_FRAMES)
        end
    end
end