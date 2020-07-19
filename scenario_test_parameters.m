% IMU Noise Parameters
velocity_random_walk_m_sec_sqr_hr = [0 5e-4 1e-3 5e-3 1e-2 5e-2 1e-1];
angle_random_walk_deg_sqr_hr = [0 1e-2 5e-2 1e-1 5e-1 1e0];

accelerometer_variances_dt = velocity_random_walk_m_sec_sqr_hr * (1/60*1/10);
gyro_variances_dt = angle_random_walk_deg_sqr_hr * (1/60*1/10);

accelerometer_bias_m_per_sec2 = [0.1 0.05 -0.1]; % X Y Z
gyro_drift_rad_per_sec = [0 0 0]*pi/180; % yaw pitch roll

% DTM Noise Parameters
dtm_errs_m = [0 1e0 5e0 1e1 1.5e1 2e1];

% LIDAR Noise Parameters
lidar_errs = [0 1e0 5e0 1e1 1.5e1 2e1];

% Initial Errors
ini_pos_err_m = [-5 8 0];
ini_vel_err_m_sec = [0.01 0.02 -0.01];
ini_att_err_rad = [0 0 0]*pi/180;