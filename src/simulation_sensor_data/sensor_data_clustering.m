load("D:\Co_op\UWAFT\3D_Simulation_Environment\highway_sensor_data.mat");
data = struct2table(sensor_data);

EGO_VEHICLE_LENGTH = 4.848;
EGO_VEHICLE_WIDTH = 1.842;


data_output_ = zeros(3,3);

prev1_vx_ = 0;
prev2_vx_ = 0;
prev3_vx_ = 0;
prev_time_ = 0;

for i = 1:height(data)
    time = data{i, 'Time'};
    objects = data.ObjectDetections{i,1};
    
    ave1_dx_ = 0;
    ave1_dy_ = 0;
    ave1_vx_ = 0;
    ave1_vy_ = 0;
    ave2_dx_ = 0;
    ave2_dy_ = 0;
    ave2_vx_ = 0;
    ave2_vy_ = 0;
    ave3_dx_ = 0;
    ave3_dy_ = 0;
    ave3_vx_ = 0;
    ave3_vy_ = 0;
    ave4_dx_ = 0;
    ave4_dy_ = 0;
    ave4_vx_ = 0;
    
    preSensor = 0;
    
    for j = 1:size(objects,1) 
        
        %Radar1 detections
        if (objects{j,1}.SensorIndex == 1)
            if (preSensor ~= 1)
                ave1_dx_ = objects{j,1}.Measurement(1,1);
                ave1_dy_ = objects{j,1}.Measurement(2,1);
                ave1_vx_ = objects{j,1}.Measurement(4,1);
                ave1_vy_ = objects{j,1}.Measurement(5,1);
                preSensor = 1;
            else
                if (abs(objects{j,1}.Measurement(1,1) - ave1_dx_) <= EGO_VEHICLE_LENGTH && abs(objects{j,1}.Measurement(2,1) - ave1_dy_) <= EGO_VEHICLE_WIDTH)
                    ave1_dx_ = (ave1_dx_ + objects{j,1}.Measurement(1,1))/(2.0);
                    ave1_dy_ = (ave1_dy_ + objects{j,1}.Measurement(2,1))/(2.0);
                    ave1_vx_ = (ave1_vx_ + objects{j,1}.Measurement(4,1))/(2.0);
                    ave1_vy_ = (ave1_vy_ + objects{j,1}.Measurement(5,1))/(2.0);
                end
            end
        
        %Radar2 detections
        elseif (objects{j,1}.SensorIndex == 2)
            if (preSensor ~= 2)
                ave2_dx_ = objects{j,1}.Measurement(1,1);
                ave2_dy_ = objects{j,1}.Measurement(2,1);
                ave2_vx_ = objects{j,1}.Measurement(4,1);
                ave2_vy_ = objects{j,1}.Measurement(5,1);
                preSensor = 2;
            else
                if (abs(objects{j,1}.Measurement(1,1) - ave2_dx_) <= EGO_VEHICLE_LENGTH && abs(objects{j,1}.Measurement(2,1) - ave2_dy_) <= EGO_VEHICLE_WIDTH)
                    ave2_dx_ = (ave1_dx_ + objects{j,1}.Measurement(1,1))/(2.0);
                    ave2_dy_ = (ave1_dy_ + objects{j,1}.Measurement(2,1))/(2.0);
                    ave2_vx_ = (ave1_vx_ + objects{j,1}.Measurement(4,1))/(2.0);
                    ave2_vy_ = (ave1_vy_ + objects{j,1}.Measurement(5,1))/(2.0);
                end
            end
            
        %Radar3 detections
        elseif (objects{j,1}.SensorIndex == 3)
            if (preSensor ~= 3)
                ave3_dx_ = objects{j,1}.Measurement(1,1);
                ave3_dy_ = objects{j,1}.Measurement(2,1);
                ave3_vx_ = objects{j,1}.Measurement(4,1);
                ave3_vy_ = objects{j,1}.Measurement(5,1);
                preSensor = 3;
            else
                if (abs(objects{j,1}.Measurement(1,1) - ave3_dx_) <= EGO_VEHICLE_LENGTH && abs(objects{j,1}.Measurement(2,1) - ave3_dy_) <= EGO_VEHICLE_WIDTH)
                    ave3_dx_ = (ave3_dx_ + objects{j,1}.Measurement(1,1))/(2.0);
                    ave3_dy_ = (ave3_dy_ + objects{j,1}.Measurement(2,1))/(2.0);
                    ave3_vx_ = (ave3_vx_ + objects{j,1}.Measurement(4,1))/(2.0);
                    ave3_vy_ = (ave3_vy_ + objects{j,1}.Measurement(5,1))/(2.0);
                end
            end
            
        %Camera detections
        elseif (objects{j,1}.SensorIndex == 4) 
            if (preSensor ~= 4)
                ave4_dx_ = objects{j,1}.Measurement(1,1);
                ave4_dy_ = objects{j,1}.Measurement(2,1);
                ave4_vx_ = objects{j,1}.Measurement(4,1);
                preSensor = 4;
            else
                if (abs(objects{j,1}.Measurement(1,1) - ave4_dx_) <= EGO_VEHICLE_LENGTH && abs(objects{j,1}.Measurement(2,1) - ave4_dy_) <= EGO_VEHICLE_WIDTH)
                    ave4_dx_ = (ave4_dx_ + objects{j,1}.Measurement(1,1))/(2.0);
                    ave4_dy_ = (ave4_dy_ + objects{j,1}.Measurement(2,1))/(2.0);
                    ave4_vx_ = (ave4_vx_ + objects{j,1}.Measurement(4,1))/(2.0);
                end
            end
        end
    end
    
    data_output_(i,1) = time;
    data_output_(i,2) = ave1_dx_;
    data_output_(i,3) = ave1_dy_;
    data_output_(i,4) = ave1_vx_;
    data_output_(i,5) = ave1_vy_;
    if (time ~= 0)
        data_output_(i,6) = (ave1_vx_ - prev1_vx_)/(time - prev_time_); %radar1 Ax
    else
        data_output_(i,6) = 0;
    end
    
    data_output_(i,7) = ave2_dx_;
    data_output_(i,8) = ave2_dy_;
    data_output_(i,9) = ave2_vx_;
    data_output_(i,10) = ave2_vy_;
    if (time ~= 0)
        data_output_(i,11) = (ave2_vx_ - prev2_vx_)/(time - prev_time_); %radar2 Ax
    else
        data_output_(i,11) = 0;
    end
    
    data_output_(i,12) = ave3_dx_;
    data_output_(i,13) = ave3_dy_;
    data_output_(i,14) = ave3_vx_;
    data_output_(i,15) = ave3_vy_;
    if (time ~= 0)
        data_output_(i,16) = (ave3_vx_ - prev3_vx_)/(time - prev_time_); %radar3 Ax
    else
        data_output_(i,16) = 0;   
    end
    
    data_output_(i,17) = ave4_dx_;
    data_output_(i,18) = ave4_dy_;
    data_output_(i,19) = ave4_vx_;
    
    
    prev1_vx_ = ave1_vx_;
    prev2_vx_ = ave2_vx_;
    prev3_vx_ = ave3_vx_;
    prev_time_ = time;
end

writematrix(data_output_, "sim_sensor_output_matlab.csv");

    