%% Setup
clc
clear
i = 0;

bag = rosbag('S:\Engineering\UWAFT\kaiROS\rosbags\ScenD_inputs_fixed_played.bag');
bag.AvailableTopics;


%% Drive Control
driveCtrl_topic = select(bag, 'Topic', '/drive_ctrl_input');
driveCtrl_struct = readMessages(driveCtrl_topic, 'DataFormat', 'struct');
ego_Speed = cellfun(@(m) m.VehSpd, driveCtrl_struct);
ego_Str = cellfun(@(m) m.StrAng, driveCtrl_struct);
ego_SetSpeed = cellfun(@(m) m.AccSpeedSetPoint, driveCtrl_struct);
ego_Gap = cellfun(@(m) m.AccGapLevel, driveCtrl_struct);
ego_AccActive = cellfun(@(m) m.AccActivation, driveCtrl_struct);

i=i+1;
figure(i);
plot(ego_Speed);
hold on;
plot(ego_Str);
hold off;
legend('Speed','Steering Angle')
title('Drive Control Inputs')

i=i+1;
figure(i);
plot(ego_SetSpeed);
hold on;
plot(ego_Gap);
plot(ego_AccActive);
hold off;
legend('Set Speed','Gap','Acc Active')
title('Drive Control Inputs pt. 2')

%% Mobileye
me_topic = select(bag, 'Topic', '/Mobileye_CAN_Rx');
me_struct = readMessages(me_topic, 'DataFormat', 'struct');
me_dx = cellfun(@(m) m.MeDx(1), me_struct);
me_dy = cellfun(@(m) m.MeDy(1), me_struct);
me_timestamp = cellfun(@(m) m.MeTimestamp, me_struct);

i=i+1;
figure(i);
scatter(me_timestamp, me_dx);
hold on;
scatter(me_timestamp, me_dy);
hold off
legend('me_dx','me_dy')
title('ME Object time vs dx/dy')

i=i+1;
figure(i);
me_ratio = me_dy ./ me_dx;
scatter(me_timestamp, me_ratio);
title('ME ratio (dy / dx)')
avg_ratio = mean(me_ratio, 'omitnan');

% i=i+1;
% figure(i);
% hold on;
% for c = 1:10
%     meObjDx = cellfun(@(m) m.MeDx(c), me_struct);
%     scatter(me_timestamp, meObjDx);
% end
% hold off;
% title('10 ME Objects time vs dx')

me_status = cellfun(@(m) m.MeStatus(1), me_struct);
me_valid = cellfun(@(m) m.MeValid(1), me_struct);

i=i+1;
figure(i);
scatter(me_timestamp, me_status); % 1 or 5 is invalid
hold on;
scatter(me_timestamp, me_valid); % 0 is invalid
hold off
legend('status','valid')
title('ME Object time vs status/valid')

me_lane = cellfun(@(m) m.MeLane(1), me_struct);
i=i+1;
figure(i);
scatter(me_timestamp, me_lane); % 1 or 5 is invalid
title('ME Lane Signal')

%% Front Radar
fr_topic = select(bag, 'Topic', '/Front_Radar_CAN_Rx');
fr_struct = readMessages(fr_topic, 'DataFormat', 'struct');
fr_timestamp = cellfun(@(m) m.RadarTimestamp, fr_struct); 

i=i+1;
figure(i);
for c = 1:32
    frObjDx = cellfun(@(m) m.RadarDx(c), fr_struct);
    subplot(4,8,c);
    scatter(fr_timestamp, frObjDx);
end
sgtitle('Front Radar dx for 32 Objects') 
i = i+1;
figure(i);
for c = 1:32
    frObjDz = cellfun(@(m) m.RadarDz(c), fr_struct);
    subplot(4,8,c);
    scatter(fr_timestamp, frObjDz);
end    
sgtitle('Front Radar dz for 32 Objects')
i = i+1;
figure(i);
for c = 1:32
    frObjDy = cellfun(@(m) m.RadarDy(c), fr_struct);
    subplot(4,8,c);
    scatter(fr_timestamp, frObjDy);
end    
sgtitle('Front Radar dy for 32 Objects') 

selectedObj = 20;

i=i+1;
figure(i);
selected_frObjDx = cellfun(@(m) m.RadarDx(selectedObj), fr_struct);
selected_frObjDy = cellfun(@(m) m.RadarDy(selectedObj), fr_struct);
selected_frObjDz = cellfun(@(m) m.RadarDz(selectedObj), fr_struct);
scatter(fr_timestamp, selected_frObjDx);
hold on
scatter(fr_timestamp, selected_frObjDy);
scatter(fr_timestamp, selected_frObjDz);
hold off
title("Radar dx, dy, dz for Object #" + selectedObj)
legend('DX', 'DY', 'DZ')

i=i+1;
figure(i);
selected_frObjMoving = cellfun(@(m) m.MovingState(selectedObj), fr_struct); % 3 is invalid
selected_frObjWExist = cellfun(@(m) m.RadarWExist(selectedObj), fr_struct); % <95 is invalid
selected_frObjValid = cellfun(@(m) m.RadarFlagValid(selectedObj), fr_struct); % 0 is invalid
% % selected_frObjMeas = cellfun(@(m) m.FlagMeas(selectedObj), fr_struct); % 0 here && 1 for history flag is invalid (not currently used)
% % selected_frObjHist = cellfun(@(m) m.FlagHist(selectedObj), fr_struct);
selected_frObjDz = cellfun(@(m) m.RadarDz(selectedObj), fr_struct); % < threshold is invalid

scatter(fr_timestamp, selected_frObjMoving);
hold on
scatter(fr_timestamp, selected_frObjWExist);
scatter(fr_timestamp, selected_frObjValid);
% % scatter(fr_timestamp, selected_frObjMeas);
% % scatter(fr_timestamp, selected_frObjHist);
scatter(fr_timestamp, selected_frObjDz);
hold off
title("Radar flags for Object #" + selectedObj)
legend('Moving', 'Exist', 'Valid', 'Dz')

i=i+1;
figure(i);
selected_frVehEgo = cellfun(@(m) m.VehVEgo, fr_struct);
selected_frRadarVX = cellfun(@(m) m.RadarVx(selectedObj), fr_struct);
stationaryFlag = selected_frVehEgo + abs(selected_frRadarVX); % < threshold (1m/s) is invalid
scatter(fr_timestamp, stationaryFlag);
hold on
scatter(fr_timestamp, selected_frVehEgo);
scatter(fr_timestamp, abs(selected_frRadarVX));
title("Radar Stationary Flag for Object #" + selectedObj)
legend('Flag', 'VehEgo', 'RadarVx')

%% Filtered Object
filt_topic = select(bag, 'Topic', '/filtered_obj');
filt_struct = readMessages(filt_topic, 'DataFormat', 'struct');
filt_dx = cellfun(@(m) m.ObjDx(1), filt_struct);
filt_dy = cellfun(@(m) m.ObjDy(1), filt_struct);
filt_timestamp = cellfun(@(m) m.ObjTimestamp, filt_struct);

i=i+1;
figure(i);
scatter(filt_timestamp, filt_dx);
hold on;
scatter(filt_timestamp, filt_dy);
hold off
legend('filt_dx','filt_dy')
title('filt Object time vs dx/dy')

%% All Tracked Objects

all_tracked_topic = select(bag, 'Topic', '/all_tracked_obj');
all_struct = readMessages(all_tracked_topic, 'DataFormat', 'struct');
all_dx = cellfun(@(m) m.ObjDx, all_struct);
all_id = cellfun(@(m) m.ObjId, all_struct);
all_time = cellfun(@(m) m.ObjTimestamp, all_struct);

i = i+1;
figure(i);
subplot(1,2,1);
scatter(all_time, all_id);
title('Time vs ID');
subplot(1,2,2);
scatter(all_time, all_dx);
title('Time vs Dx');
ylim([0, 120]);
sgtitle('ALL Tracked Objects')

%% Tracked Lane Objects Output
tracked_topic = select(bag, 'Topic', '/tracked_obj');
tracked_struct = readMessages(tracked_topic, 'DataFormat', 'struct');


i=i+1;
figure(i);
for c = 1:3
    tracked_timestamp = cellfun(@(m) m.ObjTimestamp(c), tracked_struct);
    trackedObjDx = cellfun(@(m) m.ObjDx(c), tracked_struct);
    subplot(1,3,c);
    scatter(tracked_timestamp, trackedObjDx);
end
sgtitle('Tracked Obj Dx') 

i=i+1;
figure(i);
for c = 1:3
    trackedObjId = cellfun(@(m) m.ObjId(c), tracked_struct);
    subplot(1,3,c);
    scatter(tracked_timestamp, trackedObjId);
end
sgtitle('Tracked Obj Id') 

%% Target Output
target_topic = select(bag, 'Topic', '/target_output');
target_struct = readMessages(target_topic, 'DataFormat', 'struct');
obj_dx = cellfun(@(m) m.ObjDx, target_struct);
obj_dy = cellfun(@(m) m.ObjDy, target_struct);
obj_vx = cellfun(@(m) m.ObjVx, target_struct);
time = cellfun(@(m) m.ObjTimestamp, target_struct);

i=i+1;
figure(i);
scatter(time, obj_dx);
hold on;
scatter(time, obj_dy);
hold off
legend('obj_dx','obj_dy')
ylim([0, 120]);
title('Target Object time vs dx/dy')

i=i+1;
figure(i);
scatter(time, obj_vx);
legend('obj_vx')
title('Target Object Vx')

%% Acceleration

acc_topic = select(bag, 'Topic', '/acc_output_msg');
acc_struct = readMessages(acc_topic, 'DataFormat', 'struct');
accel = cellfun(@(m) m.AccAccel, acc_struct);
fault = cellfun(@(m) m.AccFault, acc_struct);

i=i+1;
figure(i);
plot(accel);
title('Acceleration')

i=i+1;
figure(i);
plot(fault);
title('Acceleration Fault')

can_comms_topic = select(bag, 'Topic', '/can_comms_data');
can_comms_struct = readMessages(can_comms_topic, 'DataFormat', 'struct');
can_accel = cellfun(@(m) m.LongAccel, can_comms_struct);

i=i+1;
figure(i);
plot(can_accel);
title('CAN Acceleration')