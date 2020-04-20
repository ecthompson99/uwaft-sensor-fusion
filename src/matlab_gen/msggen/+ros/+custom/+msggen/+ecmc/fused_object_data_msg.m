classdef fused_object_data_msg < ros.Message
    %fused_object_data_msg MATLAB implementation of ecmc/fused_object_data_msg
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'ecmc/fused_object_data_msg' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '28da8f1984a932b8675bb4007d12febd' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        StdMsgsHeaderClass = ros.msg.internal.MessageFactory.getClassForType('std_msgs/Header') % Dispatch to MATLAB class for message type std_msgs/Header
    end
    
    properties (Dependent)
        Header
        AccelX
        VelX
        PosX
        PosY
    end
    
    properties (Access = protected)
        Cache = struct('Header', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'AccelX', 'Header', 'PosX', 'PosY', 'VelX'} % List of non-constant message properties
        ROSPropertyList = {'accel_x', 'header', 'pos_x', 'pos_y', 'vel_x'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = fused_object_data_msg(msg)
            %fused_object_data_msg Construct the message object fused_object_data_msg
            import com.mathworks.toolbox.ros.message.MessageInfo;
            
            % Support default constructor
            if nargin == 0
                obj.JavaMessage = obj.createNewJavaMessage;
                return;
            end
            
            % Construct appropriate empty array
            if isempty(msg)
                obj = obj.empty(0,1);
                return;
            end
            
            % Make scalar construction fast
            if isscalar(msg)
                % Check for correct input class
                if ~MessageInfo.compareTypes(msg(1), obj.MessageType)
                    error(message('ros:mlros:message:NoTypeMatch', obj.MessageType, ...
                        char(MessageInfo.getType(msg(1))) ));
                end
                obj.JavaMessage = msg(1);
                return;
            end
            
            % Check that this is a vector of scalar messages. Since this
            % is an object array, use arrayfun to verify.
            if ~all(arrayfun(@isscalar, msg))
                error(message('ros:mlros:message:MessageArraySizeError'));
            end
            
            % Check that all messages in the array have the correct type
            if ~all(arrayfun(@(x) MessageInfo.compareTypes(x, obj.MessageType), msg))
                error(message('ros:mlros:message:NoTypeMatchArray', obj.MessageType));
            end
            
            % Construct array of objects if necessary
            objType = class(obj);
            for i = 1:length(msg)
                obj(i,1) = feval(objType, msg(i)); %#ok<AGROW>
            end
        end
        
        function header = get.Header(obj)
            %get.Header Get the value for property Header
            if isempty(obj.Cache.Header)
                obj.Cache.Header = feval(obj.StdMsgsHeaderClass, obj.JavaMessage.getHeader);
            end
            header = obj.Cache.Header;
        end
        
        function set.Header(obj, header)
            %set.Header Set the value for property Header
            validateattributes(header, {obj.StdMsgsHeaderClass}, {'nonempty', 'scalar'}, 'fused_object_data_msg', 'Header');
            
            obj.JavaMessage.setHeader(header.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Header)
                obj.Cache.Header.setJavaObject(header.getJavaObject);
            end
        end
        
        function accelx = get.AccelX(obj)
            %get.AccelX Get the value for property AccelX
            accelx = double(obj.JavaMessage.getAccelX);
        end
        
        function set.AccelX(obj, accelx)
            %set.AccelX Set the value for property AccelX
            validateattributes(accelx, {'numeric'}, {'nonempty', 'scalar'}, 'fused_object_data_msg', 'AccelX');
            
            obj.JavaMessage.setAccelX(accelx);
        end
        
        function velx = get.VelX(obj)
            %get.VelX Get the value for property VelX
            velx = double(obj.JavaMessage.getVelX);
        end
        
        function set.VelX(obj, velx)
            %set.VelX Set the value for property VelX
            validateattributes(velx, {'numeric'}, {'nonempty', 'scalar'}, 'fused_object_data_msg', 'VelX');
            
            obj.JavaMessage.setVelX(velx);
        end
        
        function posx = get.PosX(obj)
            %get.PosX Get the value for property PosX
            posx = double(obj.JavaMessage.getPosX);
        end
        
        function set.PosX(obj, posx)
            %set.PosX Set the value for property PosX
            validateattributes(posx, {'numeric'}, {'nonempty', 'scalar'}, 'fused_object_data_msg', 'PosX');
            
            obj.JavaMessage.setPosX(posx);
        end
        
        function posy = get.PosY(obj)
            %get.PosY Get the value for property PosY
            posy = double(obj.JavaMessage.getPosY);
        end
        
        function set.PosY(obj, posy)
            %set.PosY Set the value for property PosY
            validateattributes(posy, {'numeric'}, {'nonempty', 'scalar'}, 'fused_object_data_msg', 'PosY');
            
            obj.JavaMessage.setPosY(posy);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Header = [];
        end
        
        function cpObj = copyElement(obj)
            %copyElement Implements deep copy behavior for message
            
            % Call default copy method for shallow copy
            cpObj = copyElement@ros.Message(obj);
            
            % Clear any existing cached properties
            cpObj.resetCache;
            
            % Create a new Java message object
            cpObj.JavaMessage = obj.createNewJavaMessage;
            
            % Iterate over all primitive properties
            cpObj.AccelX = obj.AccelX;
            cpObj.VelX = obj.VelX;
            cpObj.PosX = obj.PosX;
            cpObj.PosY = obj.PosY;
            
            % Recursively copy compound properties
            cpObj.Header = copy(obj.Header);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.AccelX = strObj.AccelX;
            obj.VelX = strObj.VelX;
            obj.PosX = strObj.PosX;
            obj.PosY = strObj.PosY;
            obj.Header = feval([obj.StdMsgsHeaderClass '.loadobj'], strObj.Header);
        end
    end
    
    methods (Access = ?ros.Message)
        function strObj = saveobj(obj)
            %saveobj Implements saving of message to MAT file
            
            % Return an empty element if object array is empty
            if isempty(obj)
                strObj = struct.empty;
                return
            end
            
            strObj.AccelX = obj.AccelX;
            strObj.VelX = obj.VelX;
            strObj.PosX = obj.PosX;
            strObj.PosY = obj.PosY;
            strObj.Header = saveobj(obj.Header);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.ecmc.fused_object_data_msg.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.ecmc.fused_object_data_msg;
            obj.reload(strObj);
        end
    end
end
