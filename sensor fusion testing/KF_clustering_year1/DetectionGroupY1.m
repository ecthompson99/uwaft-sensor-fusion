classdef DetectionGroupY1 %Convenience class for the detectionPlotter object 
                        %and plotDetection function for Mobileye radar
                        %objects
    properties (SetAccess = private)
        name %Legend label on the plot
        positions %Cell array of [nx2] coordinates (col 1 is x, col 2 is y)
        
        plotter %The detectionPlotter object
        marker %The plot marker character
        color %[1x3] numeric array representing the plot color
        birdPlot %The BirdsEyePlotter object. Can identify objects that 
                 %have not been plotted.
    end
    methods
        %%
        %Constructor
        function obj = DetectionGroupY1(logFilename, name,  xRegex,...
                yRegex, marker, color)
            validateattributes(logFilename, {'string'}, {'scalar'},...
                "DetectionGroupY1", "logFilename");
            validateattributes(name, {'string'}, {'scalar'},...
                "DetectionGroupY1", "name");
            validateattributes(xRegex, {'string'}, {'scalar'},...
                "DetectionGroupY1", "xRegex");
            validateattributes(yRegex, {'string'}, {'scalar'},...
                "DetectionGroupY1", "yRegex");
            validateattributes(marker, {'char'}, {'scalar'},...
                "DetectionGroupY1", "marker");
            validateattributes(color, {'double'}, {'size', [1,3]},...
                "DetectionGroupY1", "color");
            
            obj.name = name;
            obj.marker = marker;
            obj.color = color;
            obj.plotter = struct.empty(1, 0);
            
            %Get all x and y coordinate arrays with a regex
            xVarDumpStruct = load(logFilename, '-regexp', xRegex);
            yVarDumpStruct = load(logFilename, '-regexp', yRegex);
            
            %Get the names of the objects being tracked
            xCoordObjNames = fieldnames(xVarDumpStruct);
            yCoordObjNames = fieldnames(yVarDumpStruct);
            
            %If the number of objects' x-coords is not equal to the number
            %of y-coords, return an error.
            if(numel(xCoordObjNames) ~= numel(yCoordObjNames))
                disp("Error: xRegex retrieved a different number of" +...
                    "arrays than yRegex");
                obj = DetectionGroupY1.empty(0,1);
                return;
            end
            
            %Get the number of timesteps and detected objects and
            %preallocate position cell array
            numTimesteps = size(xVarDumpStruct.(xCoordObjNames{1}), 1);
            numDetections = numel(xCoordObjNames);
            obj.positions = cell(1, numTimesteps);
            
            %Populate positions cell array
            for j = 1:numTimesteps
                holder = zeros(numDetections, 2);
                for i = 1:numDetections
                    if j <= size(xVarDumpStruct.(xCoordObjNames{i}), 1)
                        holder(i, 1) =...
                            xVarDumpStruct.(xCoordObjNames{i})(j);
                        
                        holder(i, 2) =...
                            yVarDumpStruct.(yCoordObjNames{i})(j);
                            
                        if (obj.name == "Blazer Radar Objects")|| ...
                           (obj.name == "Blazer Vision Objects")
                            %x
                            holder(i, 1) =...
                                yVarDumpStruct.(yCoordObjNames{i})(j) *...
                                    cos(pi*xVarDumpStruct.(xCoordObjNames{i})(j)/180);
                            %y
                            holder(i, 2) =...
                                yVarDumpStruct.(yCoordObjNames{i})(j) *...
                                    sin(pi*xVarDumpStruct.(xCoordObjNames{i})(j)/180);
                            
                            if (yVarDumpStruct.(yCoordObjNames{i})(j) > 255)
                                holder(i, 1) = 0;
                                holder(i, 2) = 0;
                            end
                        else
                            % do nothing
                        end
                    end
                end
                
                %Remove all entries with (0,0) coordinates
                nonZeroDetections =...
                    bitor(logical(holder(:, 1)), logical(holder(:, 2)));
                obj.positions{j} = holder(nonZeroDetections, [true, true]);
            end
        end
        %%
        %Return the maximum and minimum x and y coordinates
        function [xbounds, ybounds] = bounds(obj)
            maxx = -inf;
            maxy = -inf;
            minx = inf;
            miny = inf;
            for j = 1:numel(obj.positions)
                if(~isempty(obj.positions{j}))
                    currMinX = min(obj.positions{j}(:, 1));
                    if (currMinX < minx)
                        minx = currMinX;
                    end
                    currMaxX = max(obj.positions{j}(:, 1));
                    if (currMaxX > maxx)
                        maxx = currMaxX;
                    end
                    currMinY = min(obj.positions{j}(:, 2));
                    if (currMinY < miny)
                        miny = currMinY;
                    end
                    currMaxY = min(obj.positions{j}(:, 2));
                    if (currMaxY > maxy)
                        maxy = currMaxY;
                    end
                end
            end
            xbounds = [minx, maxx];
            ybounds = [miny, maxy];
        end
        %%
        %Initializes the plotter object
        function obj = addToBirdsEyePlot(obj, birdEyePlotObj)
            if(~isempty(obj) &&...
                    (isempty(obj.birdPlot) || isempty(obj.plotter) ||...
                    obj.birdPlot ~= birdEyePlotObj))
                
                obj.birdPlot = birdEyePlotObj;
                
                obj.plotter = detectionPlotter(obj.birdPlot,...
                    'DisplayName', obj.name,...
                    'Marker', obj.marker,...
                    'MarkerEdgeColor', obj.color);
                
            else
                disp("Cannot add empty object to birdsEyePlot");
            end
        end
        %%
        %Plots the positions at timestepNum
        function plotGroup(obj, timestepNum)
            if(timestepNum < numel(obj.positions) &&...
                    ~isempty(obj.positions{timestepNum}))
                plotDetection(obj.plotter, obj.positions{timestepNum});
            end
        end
        %%
        %Returns the number of timesteps in the positions array
        function numTimesteps = numTimesteps(obj)
            numTimesteps = numel(obj.positions);
        end
    end
end