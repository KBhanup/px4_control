function modelID_data = loadFlightData(bag_file, rc_data, save_to_csv)
%% Load and read data from bag file
flight_data = rosbag(bag_file);

% RC inputs
rc_mean = 0.5 * (rc_data.rc_max + rc_data.rc_min);
rc_half_range = 0.5 * (rc_data.rc_max - rc_data.rc_min);

% Attitude/Thrust commands
bag_select = select(flight_data, 'Topic', '/mavros/rc/in');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');

cmd_data = table();
cmd_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);
start_time = cmd_data.time(1);
cmd_data.time = cmd_data.time - start_time;
cmd_data.thrust = rc_data.rc_rev(rc_data.rc_map(1)) *...
    (cellfun(@(m) double(m.Channels(rc_data.rc_map(1))), bag_struct) - rc_data.rc_min(rc_data.rc_map(1))) / (2 * rc_half_range(rc_data.rc_map(1)));
cmd_data.roll = rc_data.rc_rev(rc_data.rc_map(2)) * rc_data.rp_max *...
    (cellfun(@(m) double(m.Channels(rc_data.rc_map(2))), bag_struct) - rc_mean(rc_data.rc_map(2))) / rc_half_range(rc_data.rc_map(2));
cmd_data.pitch = rc_data.rc_rev(rc_data.rc_map(3)) * rc_data.rp_max *...
    (cellfun(@(m) double(m.Channels(rc_data.rc_map(3))), bag_struct) - rc_mean(rc_data.rc_map(3))) / rc_half_range(rc_data.rc_map(3));
cmd_data.yaw_rate = rc_data.rc_rev(rc_data.rc_map(4)) * rc_data.yr_max *...
    (cellfun(@(m) double(m.Channels(rc_data.rc_map(4))), bag_struct) - rc_mean(rc_data.rc_map(4))) / rc_half_range(rc_data.rc_map(4));

% %Unique Data
cmd_data.time = round(cmd_data.time,4);
[C, ia, ic] = unique(cmd_data(:,1),'rows');
cmd_data = cmd_data(ia,:);

% Odometry
bag_select = select(flight_data, 'Topic', '/mavros/local_position/odom');
bag_struct = readMessages(bag_select, 'DataFormat', 'struct');

odom_data = table();
odom_data.time = ...
    cellfun(@(m) double(m.Header.Stamp.Sec), bag_struct) + ...
    cellfun(@(m) double(m.Header.Stamp.Nsec)*1e-9, bag_struct);
odom_data.time = odom_data.time - start_time;
odom_data.x = cellfun(@(m) double(m.Pose.Pose.Position.X), bag_struct);
odom_data.y = cellfun(@(m) double(m.Pose.Pose.Position.Y), bag_struct);
odom_data.z = cellfun(@(m) double(m.Pose.Pose.Position.Z), bag_struct);

odom_data.qw = cellfun(@(m) double(m.Pose.Pose.Orientation.W), bag_struct);
odom_data.qx = cellfun(@(m) double(m.Pose.Pose.Orientation.X), bag_struct);
odom_data.qy = cellfun(@(m) double(m.Pose.Pose.Orientation.Y), bag_struct);
odom_data.qz = cellfun(@(m) double(m.Pose.Pose.Orientation.Z), bag_struct);

odom_data.xdot = cellfun(@(m) double(m.Twist.Twist.Linear.X), bag_struct);
odom_data.ydot = cellfun(@(m) double(m.Twist.Twist.Linear.Y), bag_struct);
odom_data.zdot = cellfun(@(m) double(m.Twist.Twist.Linear.Z), bag_struct);
%Unique Data
odom_data.time = round(odom_data.time,4);
[Q, ip, ir] = unique(odom_data(:,1),'rows');
odom_data = odom_data(ip,:);
%% Process Data
%cmd_data.time = cmd_data.time - start_time;



% Get Euler Angles from Quaternion
% Get velocities on world frame
for i = 1 : length(odom_data.time)
    pdot_B = [0, odom_data.xdot(i), odom_data.ydot(i), odom_data.zdot(i)];
    q = [odom_data.qw(i), odom_data.qx(i), odom_data.qy(i), odom_data.qz(i)];
    
    pdot_W = quatmultiply(q, quatmultiply(pdot_B, quatinv(q)));
    
    odom_data.xdot_W(i) = pdot_W(2);
    odom_data.ydot_W(i) = pdot_W(3);
    odom_data.zdot_W(i) = pdot_W(4);

    ypr = quat2eul(q);
    odom_data.yaw(i) = ypr(1);
    odom_data.pitch(i) = ypr(2);
    odom_data.roll(i) = ypr(3);
end

% Get everything on CMD time
modelID_data = table();
modelID_data.time = cmd_data.time;

modelID_data.x = interp1(odom_data.time, odom_data.x, modelID_data.time, 'spline');
modelID_data.y = interp1(odom_data.time, odom_data.y, modelID_data.time, 'spline');
modelID_data.z = interp1(odom_data.time, odom_data.z, modelID_data.time, 'spline');

modelID_data.xdot = interp1(odom_data.time, odom_data.xdot_W, modelID_data.time, 'spline');
modelID_data.ydot = interp1(odom_data.time, odom_data.ydot_W, modelID_data.time, 'spline');
modelID_data.zdot = interp1(odom_data.time, odom_data.zdot_W, modelID_data.time, 'spline');

modelID_data.qw = interp1(odom_data.time, odom_data.qw, modelID_data.time, 'spline');
modelID_data.qx = interp1(odom_data.time, odom_data.qx, modelID_data.time, 'spline');
modelID_data.qy = interp1(odom_data.time, odom_data.qy, modelID_data.time, 'spline');
modelID_data.qz = interp1(odom_data.time, odom_data.qz, modelID_data.time, 'spline');

modelID_data.yaw = interp1(odom_data.time, odom_data.yaw, modelID_data.time, 'spline');
modelID_data.pitch = interp1(odom_data.time, odom_data.pitch, modelID_data.time, 'spline');
modelID_data.roll = interp1(odom_data.time, odom_data.roll, modelID_data.time, 'spline');

modelID_data.Tcmd = cmd_data.thrust;
modelID_data.Ycmd = cmd_data.yaw_rate;
modelID_data.Pcmd = cmd_data.pitch;
modelID_data.Rcmd = cmd_data.roll;

%% Save to csv
if save_to_csv
    data_filename = [bag_file(1:end-3), 'csv'];
    writetable(modelID_data, data_filename);
end

end