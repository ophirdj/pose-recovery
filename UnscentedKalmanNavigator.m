function success = UnscentedKalmanNavigator( in_mnav, in_mimu, in_mlidar, ...
    in_meta, out_res, out_err, out_prv, window_size, DTM, sim_len, show_only )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin < 10
    sim_len = 0;
end
if nargin < 11
    show_only = 0;
end

F_META = fopen(in_meta, 'rb');
freq_Hz = fread(F_META, 1, 'double');
dt = 1/freq_Hz;
n_rays = fread(F_META, 1, 'double');
span_angle = fread(F_META, 1, 'double');
cellsize = fread(F_META, 1, 'double');
fclose(F_META);

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
lidar_data=fread(F_LIDAR,1+n_rays,'double');

% skip first record (wierd bug - record is not correct)
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');
lidar_data=fread(F_LIDAR,1+n_rays,'double');


% Assume we know initial position, velocity, and orientation (read it from
% true_val)
pos_init = true_val(2:4);
pos = pos_init;
vel_n = true_val(5:7);
att = true_val(8:10);
Cbn = euler2dcm_v000(att);
lidar = lidar_data(2:end);

% LIDAR  model
rays = GenerateRays(span_angle / ((n_rays-1)/2), ((n_rays-1)/2));


% Kalman filter initialization

% x = [position velocity attitude linear_bias linear_drift]
x = [0 0 0 vel_n' att' 0 0 0 0 0 0]';

f = @(x,u)(ins_nav(x, u, g, dt, pos_init));
h = @(x)(kalman_ray_trace(x, rays, DTM, cellsize, pos_init));

kalman = unscentedKalmanFilter(f,h,x);

kalman.Alpha = 0.005;

while (~feof(F_IMU))
    pr_count=imu_data(1);
    imu=imu_data(2:7);
    lidar=lidar_data(2:end);

    fprintf('%d\n', pr_count);
        
    predict(kalman, imu);
    x = correct(kalman, lidar);

    pos = x(1:3)+pos_init;
    vel = x(4:6);
    att = x(7:9);
    Cbn = euler2dcm_v000(att);


    % Calculate position error
    pos_err=pos-true_val(2:4);

    % Calculate attitude error
    att_err = att-true_val(8:10);

    % Calculate LIDAR error
    lidar_err = CalcRayDistances(pos, Cbn * diag([1 1 -1]), rays, DTM, cellsize)'-lidar;
    lidar_err_mean = mean(lidar_err(~isnan(lidar_err)));
    lidar_err_num_valid = sum(~isnan(lidar_err));

    % Write recovered results and errors
    fwrite(F_ERR,[pr_count;pos_err;att_err;lidar_err_mean;lidar_err_num_valid;-1;-1],'double');
    fwrite(F_RES,[pr_count;pos; att; Cbn'*vel_n],'double');

    % Write private data
    fwrite(F_PRV,[x(:); diag(kalman.StateCovariance)],'double');

    if any(abs(pos_err)>50)
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
    lidar_data=fread(F_LIDAR,1+n_rays,'double');
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
end

%%

function [xf] = ins_nav(x, imu, g, dt, pos_init)
    [Cbn, vel_n, pos]=strapdown_pln_dcm_v000(euler2dcm_v000(x(7:9)), x(4:6), x(1:3)+pos_init, imu(1:3) + x(10:12) + x(13:15)*dt, imu(4:6), g, dt, 0);
     xf = zeros(size(x));
     
    xf(1:3) = pos-pos_init; %position
    xf(4:6) = vel_n; %velocity
    xf(7:9) = dcm2euler_v000(Cbn); %attitude
    xf(10:12) = x(10:12); %linear_bias
    xf(13:15) = x(13:15); %linear_drift
end

function [rho] = kalman_ray_trace(x, rays, DTM, cellsize, pos_init)
    P = x(1:3) + pos_init;
    R = euler2dcm_v000(x(7:9)) * diag([1 1 -1]);
    dP = x(10:12);
    dXi = x(13:15);
    
    [rho_c, P_L, R_dot_lambda] = CalcRayDistances(P, R, rays, DTM, cellsize);
    
    N = GetSurfaceNormal(P_L(1,:), P_L(2,:), DTM, cellsize);
    
    rho = zeros(size(rho_c));
    
    for n = 1:length(rho)
        rho(n) = rho_c(n) + N(:,n)' / (N(:,n)' * R_dot_lambda(:,n)) * (dP - rho_c(n) * Wedge(R_dot_lambda) * dXi);
    end
    
%     rho = rho_c + (N ./ repmat(dot(N,R_dot_lambda), [3 1])) * (repmat(dP, [1 size(rays, 2)]) - rho_c * Wedge(R_dot_lambda) * dXi);
end
