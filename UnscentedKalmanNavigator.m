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

% LIDAR  model
rays = GenerateRays(ray_angles);


% Kalman filter initialization

% x = [position velocity attitude linear_bias angular_bias]
x = [pos' vel_n' att' 0 0 0 0 0 0]';

f = @(x,u)(ins_nav(x, u, g, dt));
h = @(x,pr_count)(kalman_ray_trace(x, rays, DTM, cellsize,pr_count));

kalman = unscentedKalmanFilter(f,h,x);

kalman.Alpha = kalman_alpha;

kalman.ProcessNoise = process_noise;
kalman.MeasurementNoise = measurement_noise;

kalman.StateCovariance = diag([3.7e-3 3.7e-3 3.7e-3 ... % pos
                               1.4e-4 1.4e-4 1.4e-4 ... % vel
                               3e-7 3e-7 3e-7       ... % att
                               2e-7 2e-7 2e-7       ... % l-bias
                               5e-8 5e-8 5e-8       ... % a-bias
                               ]);

while (~feof(F_IMU))
    pr_count=imu_data(1);
    imu=imu_data(2:7);
    lidar=lidar_data(2:end);
    
    steps = pr_count-1;

    fprintf('%d\n', pr_count);
    
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
    
    try
        % Effective LIDAR rate is 1/2 of IMU rate.
        % IMU = 100Hz => LIDAR = 50Hz;
        if(mod(pr_count, 2)==0)
            correct(kalman, lidar, pr_count);
        else
%             correct(kalman, lidar_projected, pr_count);
        end
    catch
        % Skip correction
    end
        
    data.IMU = imu;
    data.tru = true_val;
    
    x = predict(kalman, data);

    pos = x(1:3);
    vel_n = x(4:6);
    att = x(7:9);
    Cbn = euler2dcm_v000(att);

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

function [xf] = ins_nav(x, data, g, dt)
    imu = data.IMU;
    tru = data.tru;
    
    [Cbn, vel_n, pos]=strapdown_pln_dcm_v000(euler2dcm_v000(x(7:9)), x(4:6), x(1:3), imu(1:3) - x(10:12), imu(4:6) - x(13:15), g, dt, 0);
    xf = zeros(size(x));
     
    xf(1:3) = pos; %position
    xf(4:6) = vel_n; %velocity
    xf(7:9) = dcm2euler_v000(Cbn); %attitude
    xf(10:12) = x(10:12); %linear_bias
    xf(13:15) = x(13:15); %angular_bias

%     xf(1:3) = tru(2:4); %position
%     xf(4:6) = tru(5:7); %velocity
%     xf(7:9) = tru(8:10); %attitude
%     xf(10:12) = x(10:12); %linear_bias
%     xf(13:15) = x(13:15); %angular_bias
end
function [rho] = kalman_ray_trace(x, rays, DTM, cellsize, pr_count)
    P = x(1:3);
    R = [0 1 0; 1 0 0; 0 0 -1] * euler2dcm_v000(x(7:9));
    
    rho = CalcRayDistances(P, R, rays(:,1+mod(pr_count,size(rays,2))), DTM, cellsize);
end
