%{
Note, for time some topics have 'Timestamp' message that can be used 
similar to accessing other message parameters such as dx, dy etc. However, 
for consistency between topics with and without timestamps, the internal
ROS time is used. This requires the use of MATLAB 2018 and the addition
of custom ROS messages to the path. 
See instructions here: https://wiki.uwaterloo.ca/display/UWAFT/MATLAB+to+ROS%3A+A+Guide
Video guide here: Z:\CAVs\CAVs Team run Workshops\MATLAB 2 ROS @ 3:00 min
%}

%% Setup
clc
clear
i = 0;

bag = rosbag('/home/jaiprajapati/kaiROS/2021-10-24-01-04-50.bag');
bag.AvailableTopics;

timeRange = [0 80];

% Time series in Unix epoch time. For simplicity sake, only startTime is
% subtracted so 0 can be used as starting reference
any_topic = select(bag, 'Topic', '/drive_ctrl_input');
any_ts = timeseries(any_topic);
any_t = any_ts.Time;
startTime = any_t(1);

%% Drive Control
driveCtrl_topic = select(bag, 'Topic', '/drive_ctrl_input');
driveCtrl_struct = readMessages(driveCtrl_topic, 'DataFormat', 'struct');
ego_Speed = cellfun(@(m) m.VehSpd, driveCtrl_struct);
ego_Str = cellfun(@(m) m.StrAng, driveCtrl_struct);
ego_SetSpeed = cellfun(@(m) m.AccSpeedSetPoint, driveCtrl_struct);
ego_Gap = cellfun(@(m) m.AccGapLevel, driveCtrl_struct);
ego_AccActive = cellfun(@(m) m.AccActivation, driveCtrl_struct);

driveCtrl_ts = timeseries(driveCtrl_topic);
driveCtrl_t = driveCtrl_ts.Time - startTime; 

i=i+1;
figure(i);
plot(driveCtrl_t, ego_Speed);
hold on;
plot(driveCtrl_t, ego_Str);
hold off;
legend('Speed','Steering Angle')
title('Drive Control Inputs')
xlim(timeRange);

i=i+1;
figure(i);
plot(driveCtrl_t, ego_SetSpeed);
hold on;
plot(driveCtrl_t, ego_Gap);
plot(driveCtrl_t, ego_AccActive);
hold off;
legend('Set Speed','Gap','Acc Active')
title('Drive Control Inputs pt. 2')
xlim(timeRange);
%% Mobileye
me_topic = select(bag, 'Topic', '/Mobileye_CAN_Rx');
me_struct = readMessages(me_topic, 'DataFormat', 'struct');
me_dx = cellfun(@(m) m.MeDx(1), me_struct);
me_dy = cellfun(@(m) m.MeDy(1), me_struct);
me_vx = cellfun(@(m) m.MeVx(1), me_struct);
me_timestamp = cellfun(@(m) m.MeTimestamp, me_struct);

me_ts = timeseries(me_topic);
me_t = me_ts.Time - startTime;

i=i+1;
figure(i);
scatter(me_t, me_dx);
hold on;
scatter(me_t, me_dy);
yline(1.0,'r'); % MIN_DX
yline(130.0,'r'); % MAX_DX
yline(12.0,'g'); % MAX_DY
hold off
legend('me_dx','me_dy')
title('ME Object time vs dx/dy')

i=i+1;
figure(i);
scatter(me_t, me_vx);
title('ME Object time vs vx')

% Currently commented out
% me_status = cellfun(@(m) m.MeStatus(1), me_struct);
% me_valid = cellfun(@(m) m.MeValid(1), me_struct);
% 
% i=i+1;
% figure(i);
% subplot(1,2,1);
% scatter(me_t, me_status); % 1 or 5 is invalid
% title('Status Flag')
% yline(1,'r');
% yline(5,'r');
% subplot(1,2,2);
% scatter(me_t, me_valid); % 0 is invalid
% title('Valid Flag')
% yline(0,'r');
% sgtitle('ME Object time vs status/valid')

%% Front Radar Pt. 1
fr_topic = select(bag, 'Topic', '/Front_Radar_CAN_Rx');
fr_struct = readMessages(fr_topic, 'DataFormat', 'struct');
fr_timestamp = cellfun(@(m) m.RadarTimestamp, fr_struct); 

fr_ts = timeseries(fr_topic);
fr_t = fr_ts.Time - startTime;

i=i+1;
figure(i);
for c = 1:32
    frObjDx = cellfun(@(m) m.RadarDx(c), fr_struct);
    h(c)=subplot(4,8,c);
    scatter(fr_t, frObjDx);
end
linkaxes(h);
xlim(timeRange);
sgtitle('Front Radar dx for 32 Objects')

selectedObj = 21;

i=i+1;
figure(i);
selected_frObjDx = cellfun(@(m) m.RadarDx(selectedObj), fr_struct);
selected_frObjDy = cellfun(@(m) m.RadarDy(selectedObj), fr_struct);
selected_frObjDz = cellfun(@(m) m.RadarDz(selectedObj), fr_struct);
scatter(fr_t, selected_frObjDx, 'red');
hold on
scatter(fr_t, selected_frObjDy,'green');
% scatter(fr_t, selected_frObjDz,'blue');
% FR dx. dy. dz filter limits
yline(1.0,'r'); % MIN_DX
yline(130.0,'r'); % MAX_DX
yline(12.0,'g'); % MAX_DY
% yline(3.0,'b'); % MAX_DZ
% yline(-3.0,'b'); % MIN_DZ
hold off
% title("Radar dx, dy, dz for Object #" + selectedObj)
% legend('DX', 'DY', 'DZ')
title("Radar dx, dy for Object #" + selectedObj)
legend('DX', 'DY')
xlim(timeRange);

i=i+1;
figure(i);
selected_frObjVx = cellfun(@(m) m.RadarVx(selectedObj), fr_struct);
scatter(fr_t, selected_frObjVx);
title("Radar vx Object #" + selectedObj)
xlim(timeRange);
%% Front Radar Pt. 2
% FR flag/filter plots
i=i+1;
figure(i);
selected_frObjMoving = cellfun(@(m) m.MovingState(selectedObj), fr_struct); % 3 is invalid
selected_frObjWExist = cellfun(@(m) m.RadarWExist(selectedObj), fr_struct); % <95 is invalid
selected_frObjValid = cellfun(@(m) m.RadarFlagValid(selectedObj), fr_struct); % 0 is invalid
% History and measured flag unused due to inaccuracy

subplot(1,3,1);
scatter(fr_t, selected_frObjMoving);
xlim(timeRange);
hold on
yline(3,'r'); % Moving state = 3 is filtered out
title('Moving State')
subplot(1,3,2);
scatter(fr_t, selected_frObjWExist);
xlim(timeRange);
yline(0.95,'r'); % Probability < 95 is filtered out 
title('Exist Probability')
subplot(1,3,3);
scatter(fr_t, selected_frObjValid);
xlim(timeRange);
yline(0,'r'); % Valid flag = 0 is filtered out 
title('Valid')
hold off
sgtitle("Radar flags for Object #" + selectedObj)
xlim(timeRange);

i=i+1;
figure(i);
selected_frVehEgo = cellfun(@(m) m.VehVEgo, fr_struct);
selected_frRadarVX = cellfun(@(m) m.RadarVx(selectedObj), fr_struct);
stationaryFlag = selected_frVehEgo + abs(selected_frRadarVX);
scatter(fr_t, stationaryFlag);
hold on
% scatter(fr_t, selected_frVehEgo);
% scatter(fr_t, abs(selected_frRadarVX));
yline(0.5,'r'); % ego + radar velocity < threshold (0.5 m/s) is filtered out
title("Radar Stationary Flag for Object #" + selectedObj)
xlim(timeRange);

%% Filtered Object
filt_topic = select(bag, 'Topic', '/filtered_obj');
filt_struct = readMessages(filt_topic, 'DataFormat', 'struct');
filt_dx = cellfun(@(m) m.ObjDx(1), filt_struct);
filt_dy = cellfun(@(m) m.ObjDy(1), filt_struct);
filt_vx = cellfun(@(m) m.ObjVx(1), filt_struct);
% filt_timestamp = cellfun(@(m) m.ObjTimestamp, filt_struct);

filt_ts = timeseries(filt_topic);
filt_t = filt_ts.Time - startTime;

i=i+1;
figure(i);
scatter(filt_t, filt_dx);
hold on;
scatter(filt_t, filt_dy);
hold off
legend('filt_dx','filt_dy')
title('filt Object time vs dx/dy')

i=i+1;
figure(i);
scatter(filt_t, filt_vx);
title('filt Object time vs vx')


%% All Tracked Objects

all_tracked_topic = select(bag, 'Topic', '/all_tracked_obj');
all_struct = readMessages(all_tracked_topic, 'DataFormat', 'struct');
all_dx = cellfun(@(m) m.ObjDx, all_struct);
all_id = cellfun(@(m) m.ObjId, all_struct);
all_time = cellfun(@(m) m.ObjTimestamp, all_struct);

all_ts = timeseries(all_tracked_topic);
all_t = all_ts.Time - startTime;

i = i+1;
figure(i);
subplot(1,2,1);
scatter(all_t, all_id);
title('Time vs ID');
subplot(1,2,2);
scatter(all_t, all_dx);
title('Time vs Dx');
ylim([0, 120]);
sgtitle('ALL Tracked Objects')

%% Tracked Lane Objects Output
tracked_topic = select(bag, 'Topic', '/tracked_obj');
tracked_struct = readMessages(tracked_topic, 'DataFormat', 'struct');

tracked_ts = timeseries(tracked_topic);
tracked_t = tracked_ts.Time - startTime;

i=i+1;
figure(i);
for c = 1:3
    trackedObjDx = cellfun(@(m) m.ObjDx(c), tracked_struct);
    subplot(1,3,c);
    scatter(tracked_t, trackedObjDx);
end
sgtitle('Tracked Obj Dx') 

i=i+1;
figure(i);
for c = 1:3
    trackedObjId = cellfun(@(m) m.ObjId(c), tracked_struct);
    subplot(1,3,c);
    scatter(tracked_t, trackedObjId);
end
sgtitle('Tracked Obj Id') 

%% Target Output
target_topic = select(bag, 'Topic', '/target_output');
target_struct = readMessages(target_topic, 'DataFormat', 'struct');
obj_dx = cellfun(@(m) m.ObjDx, target_struct);
obj_dy = cellfun(@(m) m.ObjDy, target_struct);
obj_vx = cellfun(@(m) m.ObjVx, target_struct);

target_ts = timeseries(target_topic);
target_t = target_ts.Time - startTime;

i=i+1;
figure(i);
scatter(target_t, obj_dx);
hold on;
scatter(target_t, obj_dy);
hold off
legend('obj_dx','obj_dy')
ylim([0, 120]);
title('Target Object time vs dx/dy')

i=i+1;
figure(i);
scatter(target_t, obj_vx);
legend('obj_vx')
title('Target Object Vx')

%% Acceleration

acc_topic = select(bag, 'Topic', '/acc_output_msg');
acc_struct = readMessages(acc_topic, 'DataFormat', 'struct');
accel = cellfun(@(m) m.AccAccel, acc_struct);
fault = cellfun(@(m) m.AccFault, acc_struct);

acc_ts = timeseries(acc_topic);
acc_t = acc_ts.Time - startTime;

i=i+1;
figure(i);
plot(acc_t, accel);
title('Acceleration')

i=i+1;
figure(i);
plot(acc_t, fault);
title('Acceleration Fault')

% can_comms_topic = select(bag, 'Topic', '/can_comms_data');
% can_comms_struct = readMessages(can_comms_topic, 'DataFormat', 'struct');
% can_accel = cellfun(@(m) m.LongAccel, can_comms_struct);
% 
% can_comms_ts = timeseries(can_comms_topic);
% can_comms_t = can_comms_ts.Time - startTime;
% 
% i=i+1;
% figure(i);
% plot(can_comms_t, can_accel);
% title('CAN Acceleration')

%% Difference between ME and FR readouts


