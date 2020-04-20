classdef associated_radar_msg < ros.Message
    %associated_radar_msg MATLAB implementation of sensor_fusion/associated_radar_msg
    %   This class was automatically generated by
    %   ros.msg.internal.gen.MessageClassGenerator.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    %#ok<*INUSD>
    
    properties (Constant)
        MessageType = 'sensor_fusion/associated_radar_msg' % The ROS message type
    end
    
    properties (Constant, Hidden)
        MD5Checksum = '5a5beb016896672b91532fa12b63e696' % The MD5 Checksum of the message definition
    end
    
    properties (Access = protected)
        JavaMessage % The Java message object
    end
    
    properties (Constant, Access = protected)
        SensorFusionRadarObjectDataClass = ros.msg.internal.MessageFactory.getClassForType('sensor_fusion/radar_object_data') % Dispatch to MATLAB class for message type sensor_fusion/radar_object_data
    end
    
    properties (Dependent)
        Obj
        ObjId
    end
    
    properties (Access = protected)
        Cache = struct('Obj', []) % The cache for fast data access
    end
    
    properties (Constant, Hidden)
        PropertyList = {'Obj', 'ObjId'} % List of non-constant message properties
        ROSPropertyList = {'obj', 'obj_id'} % List of non-constant ROS message properties
    end
    
    methods
        function obj = associated_radar_msg(msg)
            %associated_radar_msg Construct the message object associated_radar_msg
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
        
        function obj_ = get.Obj(obj)
            %get.Obj Get the value for property Obj
            if isempty(obj.Cache.Obj)
                obj.Cache.Obj = feval(obj.SensorFusionRadarObjectDataClass, obj.JavaMessage.getObj);
            end
            obj_ = obj.Cache.Obj;
        end
        
        function set.Obj(obj, obj_)
            %set.Obj Set the value for property Obj
            validateattributes(obj_, {obj.SensorFusionRadarObjectDataClass}, {'nonempty', 'scalar'}, 'associated_radar_msg', 'Obj');
            
            obj.JavaMessage.setObj(obj_.getJavaObject);
            
            % Update cache if necessary
            if ~isempty(obj.Cache.Obj)
                obj.Cache.Obj.setJavaObject(obj_.getJavaObject);
            end
        end
        
        function objid = get.ObjId(obj)
            %get.ObjId Get the value for property ObjId
            objid = typecast(int64(obj.JavaMessage.getObjId), 'uint64');
        end
        
        function set.ObjId(obj, objid)
            %set.ObjId Set the value for property ObjId
            validateattributes(objid, {'numeric'}, {'nonempty', 'scalar'}, 'associated_radar_msg', 'ObjId');
            
            obj.JavaMessage.setObjId(objid);
        end
    end
    
    methods (Access = protected)
        function resetCache(obj)
            %resetCache Resets any cached properties
            obj.Cache.Obj = [];
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
            cpObj.ObjId = obj.ObjId;
            
            % Recursively copy compound properties
            cpObj.Obj = copy(obj.Obj);
        end
        
        function reload(obj, strObj)
            %reload Called by loadobj to assign properties
            obj.ObjId = strObj.ObjId;
            obj.Obj = feval([obj.SensorFusionRadarObjectDataClass '.loadobj'], strObj.Obj);
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
            
            strObj.ObjId = obj.ObjId;
            strObj.Obj = saveobj(obj.Obj);
        end
    end
    
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
            %loadobj Implements loading of message from MAT file
            
            % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.custom.msggen.sensor_fusion.associated_radar_msg.empty(0,1);
                return
            end
            
            % Create an empty message object
            obj = ros.custom.msggen.sensor_fusion.associated_radar_msg;
            obj.reload(strObj);
        end
    end
end
