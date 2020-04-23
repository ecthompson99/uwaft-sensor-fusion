classdef SFObject
   
    properties (SetAccess = private)
        m_name %Legend label on the plot
        m_positions %Cell array of [nx2] coordinates (col 1 is x, col 2 is y)
        m_marker %The plot marker character
        m_color %[1x3] numeric array representing the plot color
        m_plotter %The detectionPlotter object
        m_birdPlot
        m_numTimesteps
    end
    methods
        %%
        %Constructor
        function obj = SFObject(name, positions, marker, color, timesteps)
            obj.m_name = name;
            obj.m_marker = marker;
            obj.m_color = color;
            obj.m_positions = positions;
            obj.m_plotter = struct.empty(1, 0);
            obj.m_numTimesteps = timesteps;
        end
        
   
        %%
        %Initializes the plotter object
        function obj = addToBirdsEyePlot(obj, birdEyePlotObj)
            obj.m_birdPlot = birdEyePlotObj;

            obj.m_plotter = detectionPlotter(obj.m_birdPlot,...
                'DisplayName', obj.m_name,...
                'Marker', obj.m_marker,...
                'MarkerEdgeColor', obj.m_color);
        end
        %%
        %Plots the positions at timestepNum
        function plotGroup(obj, timestepNum)
            if(timestepNum < numel(obj.m_positions) &&...
                    ~isempty(obj.m_positions{timestepNum}))
                plotDetection(obj.m_plotter, obj.m_positions{timestepNum});
            end
        end
        %%
        %Returns the number of timesteps in the positions array
        function numTimesteps = numTimesteps(obj)
            numTimesteps = numel(obj.m_positions);
        end
     end
end