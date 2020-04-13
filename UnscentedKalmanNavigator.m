function [success, steps] = UnscentedKalmanNavigator( in_mnav, in_mimu, in_mlidar, ...
    in_meta, out_res, out_err, out_prv, DTM, ...
    process_noise, measurement_noise, kalman_alpha, sim_len, show_only)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin < 12
    sim_len = 0;
end
if nargin < 13
    show_only = 0;
end

F_META = fopen(in_meta, 'rb');
freq_Hz = fread(F_META, 1, 'double');
dt = 1/freq_Hz;
cellsize = fread(F_META, 1, 'double');
n_rays = fread(F_META, 1, 'double');
ray_angles = fread(F_META, n_rays, 'double');
fclose(F_META);

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


%%read the input data from the files
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');
lidar_data=fread(F_LIDAR,2,'double');

% skip first record (wierd bug - record is not correct)
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');
lidar_data=fread(F_LIDAR,2,'double');


% Assume we know initial position, velocity, and orientation (read it from
% true_val)
pos = true_val(2:4);
vel_n = true_val(5:7);
att = true_val(8:10);
Cbn = euler2dcm_v000(att);
lidar = lidar_data(2:end);

bias_acc = [0 0 0]';
bias_gyr = [0 0 0]';
imu_tot = zeros(length(imu_data)-1,dt/dt_lidar);
imu_tot(3,:) = -g;

% LIDAR  model
rays = GenerateRays(ray_angles);


% Kalman filter initialization
x = [pos; vel_n; att; bias_acc; bias_gyr];

f = @(x,u)(ins_nav(x, u, g, dt));
h = @(x,pr_count)(kalman_ray_trace(x, rays, DTM, cellsize,pr_count));

kalman = unscentedKalmanFilter(f,h,x);

kalman.Alpha = kalman_alpha;
kalman.ProcessNoise = process_noise * dt_lidar;
kalman.MeasurementNoise = measurement_noise;

kalman.StateCovariance = ...
    diag([100 100 100                                               ... % pos
          1 1 1                                                     ... % vel
          (pi/180)^2 (pi/180)^2 (pi/180)^2                          ... % att
          1e-4 1e-4 1e-4                                            ... % bias-acc
          (10*pi/180/3600)^2 (10*pi/180/3600)^2 (10*pi/180/3600)^2  ... % bias-gyr
          ]);

while (~feof(F_IMU))
    pr_count=imu_data(1);
    imu=imu_data(2:7);
    lidar=lidar_data(2:end);
    
    steps = pr_count-1;

    if mod(pr_count, 100) == 0 
        fprintf('%d\n', pr_count);
    end
    
    % Calculate position error
    pos_err=pos-true_val(2:4);

    % Calculate attitude error
    att_err = mod(att-true_val(8:10)+pi,2*pi)-pi;

    % Calculate LIDAR error
    lidar_projected = CalcRayDistances(pos, [0 1 0; 1 0 0; 0 0 -1] * Cbn, ...
        rays(:,1+mod(pr_count,size(rays,2))), DTM, cellsize)';
    lidar_err = lidar_projected-lidar;
    lidar_err_mean = mean(lidar_err(~isnan(lidar_err)));
    lidar_err_num_valid = sum(~isnan(lidar_err));

    % Write recovered results and errors
    fwrite(F_ERR,[pr_count;pos_err;att_err;lidar_err_mean;lidar_err_num_valid;-1;-1],'double');
    fwrite(F_RES,[pr_count;pos; att; Cbn'*vel_n],'double');

    % Write private data
    fwrite(F_PRV,[x(:); diag(kalman.StateCovariance)],'double');
    
    
    % Effective LIDAR rate is dt/dt_lidar of IMU rate.
    % IMU = 100Hz, LIDAR = 10Hz => m = 10
    % If LIDAR is available, evaluate UKF.
    m = mod(pr_count, dt/dt_lidar);
    
    try
        if(m==0)
            % LIDAR available
            predict(kalman, imu_tot);
            x = correct(kalman, lidar, pr_count);

            pos = x(1:3)';
            vel_n = x(4:6)';
            att = x(7:9)';
            Cbn = euler2dcm_v000(att);
            bias_acc = x(10:12)';
            bias_gyr = x(12:15)';
        end
    catch
        % Skip correction
    end
    
    imu_tot(:, 1 + m) = imu;
    
    % IMU step
    [Cbn, vel_n, pos] = ...
        strapdown_pln_dcm_v000(Cbn, vel_n, pos, ...
                               imu(1:3) - bias_acc, ...
                               imu(4:6) - bias_gyr, ...
                               g, dt, 0);
   att = dcm2euler_v000(Cbn);

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
    lidar_data=fread(F_LIDAR,2,'double');
end

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
err_plot_nav(out_err,out_res,in_mnav,DTM,cellsize,success);
kalman_plot(out_prv);
end

%%

function [xf] = ins_nav(x, imu_tot, g, dt)
    pos = x(1:3)';
    vel_n = x(4:6)';
    Cbn = euler2dcm_v000(x(7:9)');
    bias_acc = x(10:12)';
    bias_gyr = x(12:15)';
    
    for j=1:size(imu_tot, 2)
        [Cbn, vel_n, pos] = ...
            strapdown_pln_dcm_v000(Cbn, vel_n, pos, ...
                                   imu_tot(1:3, j) - bias_acc, ...
                                   imu_tot(4:6, j) - bias_gyr, ...
                                   g, dt, 0);
    end

    att = dcm2euler_v000(Cbn);
    xf = [pos vel_n att bias_acc bias_gyr]';
end

function [rho] = kalman_ray_trace(x, rays, DTM, cellsize, pr_count)
    P = x(1:3);
    R = [0 1 0; 1 0 0; 0 0 -1] * euler2dcm_v000(x(7:9));
    ray = rays(:,1+mod(pr_count,size(rays,2)));
    
    rho = CalcRayDistances(P, R, ray, DTM, cellsize);
end
