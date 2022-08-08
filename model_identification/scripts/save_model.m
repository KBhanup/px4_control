function save_model(model, model_filename)
fileID = fopen(model_filename, 'w');

fprintf(fileID, '# Attitude Dynamics\n');
fprintf(fileID, 't_roll: %.5f\n', model.roll_time_c);
fprintf(fileID, 'k_roll: %.5f\n', model.roll_gain);
fprintf(fileID, 't_pitch: %.5f\n', model.pitch_time_c);
fprintf(fileID, 'k_pitch: %.5f\n', model.pitch_gain);
fprintf(fileID, '\n');

fprintf(fileID, '# Damping coefficients\n');
fprintf(fileID, 'damping_coef: [%.5f, %.5f, %.5f]\n', model.damping_coeff(1), model.damping_coeff(2), model.damping_coeff(3));
fprintf(fileID, '\n');

fprintf(fileID, '# Thrust coefficient\n');
fprintf(fileID, 'k_thrust: %.5f\n', model.thrust_coeff);
fprintf(fileID, '\n');

fprintf(fileID, '# Gravity on world frame\n');
fprintf(fileID, 'gravity: -9.8066\n');
fprintf(fileID, '\n');

fprintf(fileID, '# NMPC cost function weights\n');
fprintf(fileID, '# pos_w, vel_x -> [x, y, z]\n');
fprintf(fileID, '# att_w -> [roll, pitch, yaw]\n');
fprintf(fileID, 'pos_w: [20, 20, 20]\n');
fprintf(fileID, 'vel_w: [5, 5, 5]\n');
fprintf(fileID, 'att_w: [1, 1, 1]\n');
fprintf(fileID, '\n');

fprintf(fileID, 'yaw_rate_cmd_w: 100\n');
fprintf(fileID, 'pitch_cmd_w: 250\n');
fprintf(fileID, 'roll_cmd_w: 250\n');
fprintf(fileID, 'thrust_cmd_w: 500\n');
fprintf(fileID, '\n');

fprintf(fileID, '# NMPC input constraints\n');
fprintf(fileID, '# bound -> [yaw_cmd, pitch_cmd, roll_cmd, thrust]\n');
fprintf(fileID, 'lbu: [-0.5, -0.2, -0.2, 0.1]\n');
fprintf(fileID, 'ubu: [ 0.5,  0.2,  0.2, 1.0]\n');
fprintf(fileID, '\n');

fprintf(fileID, '# Backup velocity controller weights\n');
fprintf(fileID, '# gain -> [position error gain, velocity error gain]\n');
fprintf(fileID, '# PID -> [kP, kI, kD]\n');
fprintf(fileID, 'x_gain: [0.0, 0.0]\n');
fprintf(fileID, 'y_gain: [0.0, 0.0]\n');
fprintf(fileID, 'z_gain: [0.0, 0.0]\n');
fprintf(fileID, 'o_pid: [0.0, 0.0, 0.0]\n');
fprintf(fileID, '\n');

fprintf(fileID, '# RC Switches\n');
fprintf(fileID, 'auto_channel: \n');
fprintf(fileID, 'auto_on: \n');
fprintf(fileID, 'auto_off: \n');
fprintf(fileID, '\n');

fprintf(fileID, 'offboard_channel: \n');
fprintf(fileID, 'offboard_on: \n');
fprintf(fileID, 'offboard_off: \n');

fclose(fileID);
end