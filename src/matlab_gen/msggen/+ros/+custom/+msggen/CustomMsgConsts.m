classdef CustomMsgConsts
    %CustomMsgConsts This class stores all message types
    %   The message types are constant properties, which in turn resolve
    %   to the strings of the actual types.
    
    %   Copyright 2014-2020 The MathWorks, Inc.
    
    properties (Constant)
        can_tx_rx_can_comms_data_msg = 'can_tx_rx/can_comms_data_msg'
        can_tx_rx_drive_ctrl_input_msg = 'can_tx_rx/drive_ctrl_input_msg'
        can_tx_rx_raw_sensor_object_data_msg = 'can_tx_rx/raw_sensor_object_data_msg'
        can_tx_rx_sensor_diagnostic_data_msg = 'can_tx_rx/sensor_diagnostic_data_msg'
        ecmc_SensorDiagnosticFlagMsg = 'ecmc/SensorDiagnosticFlagMsg'
        ecmc_can_comms_data_msg = 'ecmc/can_comms_data_msg'
        ecmc_drive_control_input_msg = 'ecmc/drive_control_input_msg'
        ecmc_fused_object_data_msg = 'ecmc/fused_object_data_msg'
        ecmc_raw_sensor_object_data_msg = 'ecmc/raw_sensor_object_data_msg'
        ecmc_sensor_diagnostic_flag_msg = 'ecmc/sensor_diagnostic_flag_msg'
        ecmc_sudo_driver_input_msg = 'ecmc/sudo_driver_input_msg'
        kalman_filter_filtered_object_msg = 'kalman_filter/filtered_object_msg'
        kalman_filter_object_deletion_msg = 'kalman_filter/object_deletion_msg'
        master_task_can_comms_data_msg = 'master_task/can_comms_data_msg'
        master_task_drive_ctrl_input_msg = 'master_task/drive_ctrl_input_msg'
        master_task_sensor_diagnostic_flag_msg = 'master_task/sensor_diagnostic_flag_msg'
        master_task_sudo_driver_input_msg = 'master_task/sudo_driver_input_msg'
        sensor_diagnostic_check_sensor_diagnostic_data_msg = 'sensor_diagnostic_check/sensor_diagnostic_data_msg'
        sensor_diagnostic_check_sensor_diagnostic_flag_msg = 'sensor_diagnostic_check/sensor_diagnostic_flag_msg'
        sensor_fusion_testing_mobileye_object_data_from_matlab = 'sensor_fusion_testing/mobileye_object_data_from_matlab'
        sensor_fusion_testing_radar_object_data_from_matlab = 'sensor_fusion_testing/radar_object_data_from_matlab'
    end
    
    methods (Static, Hidden)
        function messageList = getMessageList
            %getMessageList Generate a cell array with all message types.
            %   The list will be sorted alphabetically.
            
            persistent msgList
            if isempty(msgList)
                msgList = cell(21, 1);
                msgList{1} = 'can_tx_rx/can_comms_data_msg';
                msgList{2} = 'can_tx_rx/drive_ctrl_input_msg';
                msgList{3} = 'can_tx_rx/raw_sensor_object_data_msg';
                msgList{4} = 'can_tx_rx/sensor_diagnostic_data_msg';
                msgList{5} = 'ecmc/SensorDiagnosticFlagMsg';
                msgList{6} = 'ecmc/can_comms_data_msg';
                msgList{7} = 'ecmc/drive_control_input_msg';
                msgList{8} = 'ecmc/fused_object_data_msg';
                msgList{9} = 'ecmc/raw_sensor_object_data_msg';
                msgList{10} = 'ecmc/sensor_diagnostic_flag_msg';
                msgList{11} = 'ecmc/sudo_driver_input_msg';
                msgList{12} = 'kalman_filter/filtered_object_msg';
                msgList{13} = 'kalman_filter/object_deletion_msg';
                msgList{14} = 'master_task/can_comms_data_msg';
                msgList{15} = 'master_task/drive_ctrl_input_msg';
                msgList{16} = 'master_task/sensor_diagnostic_flag_msg';
                msgList{17} = 'master_task/sudo_driver_input_msg';
                msgList{18} = 'sensor_diagnostic_check/sensor_diagnostic_data_msg';
                msgList{19} = 'sensor_diagnostic_check/sensor_diagnostic_flag_msg';
                msgList{20} = 'sensor_fusion_testing/mobileye_object_data_from_matlab';
                msgList{21} = 'sensor_fusion_testing/radar_object_data_from_matlab';
            end
            
            messageList = msgList;
        end
        
        function serviceList = getServiceList
            %getServiceList Generate a cell array with all service types.
            %   The list will be sorted alphabetically.
            
            persistent svcList
            if isempty(svcList)
                svcList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            serviceList = svcList;
        end
        
        function actionList = getActionList
            %getActionList Generate a cell array with all action types.
            %   The list will be sorted alphabetically.
            
            persistent actList
            if isempty(actList)
                actList = cell(0, 1);
            end
            
            % The message list was already sorted, so don't need to sort
            % again.
            actionList = actList;
        end
    end
end
