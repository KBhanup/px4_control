%% Clean start
clear; close all; clc;

%% User defined data
bag_file = 'flight_logs_nmpc_flt_2022-08-09-11-24-01_nosensor.bag';
plot_results = true;
save_model_cfg = true;

% RC data
% % Aurelia X6
% rc_data.rc_map = [3, 1, 2, 4];               % [RC_MAP_THROTTLE, RC_MAP_ROLL, RC_MAP_PITCH, RC_MAP_YAW]
% rc_data.rc_max = [2005, 2005, 2004, 2005];   % [RC1_MAX, RC2_MAX, RC3_MAX, RC4_MAX]
% rc_data.rc_min = [982, 982, 985, 982];       % [RC1_MIN, RC2_MIN, RC3_MIN, RC4_MIN]
% rc_data.rc_rev = [1, -1, 1, 1];              % [RC1_REV, RC2_REV, RC3_REV, RC4_REV]
% rc_data.rp_max = deg2rad(35);                % MPC_MAN_TILT_MAX
% rc_data.yr_max = deg2rad(213.2);             % MPC_MAN_Y_MAX


% F550
rc_data.rc_map = [1, 2, 3, 4];               % [RC_MAP_THROTTLE, RC_MAP_ROLL, RC_MAP_PITCH, RC_MAP_YAW]
rc_data.rc_max = [2006, 2006, 2006, 2006];   % [RC1_MAX, RC2_MAX, RC3_MAX, RC4_MAX]
rc_data.rc_min = [982, 982, 982, 982];       % [RC1_MIN, RC2_MIN, RC3_MIN, RC4_MIN]
rc_data.rc_rev = [1, 1, 1, 1];               % [RC1_REV, RC2_REV, RC3_REV, RC4_REV]
rc_data.rp_max = deg2rad(35);                % MPC_MAN_TILT_MAX
rc_data.yr_max = deg2rad(150);                % MPC_MAN_Y_MAX

%% Load flight data
modelID_data = loadFlightData(bag_file, rc_data, false);
%modelID_data = loadFlightDataSim(bag_file, false);

%% Get model
model = getMRModel(modelID_data);

%% Plot results
if plot_results
    plot_model(modelID_data, model)
end

%% Save cfg file
if save_model_cfg
    model_filename = [bag_file(1:end-4), '_parameters.yaml'];
    save_model(model, model_filename);
end