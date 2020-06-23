function [success, steps] = UnscentedKalmanNavigator( in_mnav, in_mimu, in_mlidar, ...
    in_meta, out_res, out_err, out_prv, DTM, ...
    process_noise, measurement_noise, kalman_alpha, kalman_P, ...
    accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2, ini_pos_err_m, ...
    ini_vel_err_m_sec, ini_att_err_rad, ...
    sim_len, show_only)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin < 18
    sim_len = 0;
end
if nargin < 19
    show_only = 0;
end

F_META = fopen(in_meta, 'rb');
freq_Hz = fread(F_META, 1, 'double');
dt = 1/freq_Hz;
cellsize = fread(F_META, 1, 'double');
n_rays = fread(F_META, 1, 'double');
ray_angles = fread(F_META, n_rays, 'double');
nav_len = fread(F_META, 1, 'double');
fclose(F_META);

imu_bias = [accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2];

if ~size(nav_len,1)
    nav_len = sim_len;
end
nav_len = min(nav_len, sim_len);

o_err = zeros(11, nav_len);
o_res = zeros(10, nav_len);
o_prv = zeros(30, nav_len);

dt_lidar = dt/10;

success = true;
if show_only == 0 || show_only == 2

F_IMU=fopen(in_mimu,'rb');
F_LIDAR=fopen(in_mlidar,'rb');
F_TRU=fopen(in_mnav,'rb');
F_RES=fopen(out_res,'wb');
F_ERR=fopen(out_err,'wb');
F_PRV=fopen(out_prv,'wb');

%Use constant scale for all observations
[Rn, Re, g, sL, cL, WIE_E]=geoparam_v000([0 0 0]');


% Ignore first record - junk
fread(F_IMU,7,'double');
fread(F_TRU, 10, 'double');
fread(F_LIDAR,5,'double');

% Read navigation, IMU, and LIDAR data
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');
lidar_data=fread(F_LIDAR,5,'double');


% Assume we know true initial position, velocity, and orientation
pos = true_val(2:4) + ini_pos_err_m(:);
vel_n = true_val(5:7) + ini_vel_err_m_sec(:);
att = true_val(8:10) + ini_att_err_rad(:);
Cbn = euler2dcm_v000(att);

acc_bias = [0 0 0]';
gyro_drift = [0 0 0]';


% Kalman filter initialization
x = zeros(15,1);

f = @(x,Phi)(Phi*x);
df = @(x,Phi)(Phi);
h = @(x, pos, Cbn, ray)(kalman_ray_trace(x, pos, Cbn, ray, DTM, cellsize));

kalman = unscentedKalmanFilter(f,h,x);

kalman.Alpha = kalman_alpha;
kalman.ProcessNoise = process_noise * dt;
kalman.MeasurementNoise = measurement_noise;
kalman.StateCovariance = kalman_P;


while (~feof(F_IMU))
    pr_count=imu_data(1);
    imu=imu_data(2:7)+imu_bias(:)-[acc_bias;gyro_drift];
    lidar=lidar_data(2);
    ray=lidar_data(3:5);
    
    steps = pr_count-1;

    if mod(pr_count, 100) == 0 
        fprintf('%d\n', pr_count);
    end
    
    % Calculate position error
    pos_err=pos-true_val(2:4);

    % Calculate attitude error
    att_err = dcm2euler_v000(euler2dcm_v000(true_val(8:10))*Cbn');

    % Calculate LIDAR error
    lidar_projected = CalcRayDistances(pos, Cbn, ray, DTM, cellsize)';
    lidar_err = lidar_projected-lidar;
    lidar_err_mean = mean(lidar_err(~isnan(lidar_err)));
    lidar_err_num_valid = sum(~isnan(lidar_err));

    % Write recovered results and errors
    o_err(:, pr_count) = [pr_count;pos_err;att_err;lidar_err_mean;lidar_err_num_valid;-1;-1];
    o_res(:, pr_count) = [pr_count;pos; att; Cbn'*vel_n];

    % Write private data
    o_prv(:, pr_count) = [x(1:9); acc_bias(:); gyro_drift(:); diag(kalman.StateCovariance)];

    
    % IMU step
    [Cbn, vel_n, pos] = ...
        strapdown_pln_dcm_v000(Cbn, vel_n, pos, imu(1:3), imu(4:6), g, dt, 0);
    
    
    Phi = eye(length(x));
    Phi(1:3,4:6) = eye(3)*dt;
    Phi(4:6,7:9) = skew(Cbn*imu(1:3))*dt;
    Phi(4:6,10:12) = Cbn*dt;
    Phi(7:9,13:15) = -Cbn*dt;
    
    P0 = kalman.StateCovariance;
    
    % Linear update - can use EKF for speed
    x1 = Phi*kalman.State;
    P1 = Phi*P0*Phi' + kalman.ProcessNoise;
    
    kalman.State = x1;
    kalman.StateCovariance = P1;
    
    % Effective LIDAR rate is dt/dt_lidar of IMU rate.    
    if(mod(pr_count, dt/dt_lidar) == 0)
        % LIDAR available
        x = correct(kalman, lidar, pos, Cbn, ray);
        kalman.State = zeros(size(x));
    
        % Correct the navigation solution and reset the error state
        pos = pos-x(1:3);
        vel_n = vel_n-x(4:6);
        Cbn = euler2dcm_v000(x(7:9))*Cbn;
        att = dcm2euler_v000(Cbn);
        acc_bias = acc_bias+x(10:12);
        gyro_drift = gyro_drift+x(13:15);
    end
    
    if(mod(pr_count, 1000) == 0)
        % Cancel any numerical errors to make sure covariance stays positive-semidefinite
        kalman.StateCovariance = triu(P1)+triu(P1)'-diag(diag(P1));
    end
    
    if any(abs(pos_err)>30)
        success = false;
        break;
    end

    if sim_len == 1
        success = true;
        break;
    elseif sim_len > 0
        sim_len = sim_len - 1;
    end
    
    % Read next records
    imu_data=fread(F_IMU,7,'double');
    true_val = fread(F_TRU, 10, 'double');
    lidar_data=fread(F_LIDAR,5,'double');
end

% Write result matrices
fwrite(F_ERR,o_err(:,1:1+steps),'double');
fwrite(F_RES,o_res(:,1:1+steps),'double');
fwrite(F_PRV,o_prv(:,1:1+steps),'double');


fclose(F_IMU);
fclose(F_LIDAR);
fclose(F_TRU);
fclose(F_RES);
fclose(F_ERR);
fclose(F_PRV);
if show_only == 0
    return;
end
end
%% Show results
err_plot_nav(out_err,out_res,in_mnav,in_meta,DTM,cellsize,success);
kalman_plot(out_prv, in_meta);
end

%% Supporting functions
function [rho] = kalman_ray_trace(x, pos, Cbn, ray, DTM, cellsize)
    P = pos - x(1:3);
    R = euler2dcm_v000(x(7:9)) * Cbn;
    
    rho = CalcRayDistances(P, R, ray, DTM, cellsize);
end
