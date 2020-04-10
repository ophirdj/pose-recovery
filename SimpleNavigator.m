function success = SimpleNavigator( in_mnav, in_mimu, in_mlidar, ...
    in_meta, out_res, out_err, DTM, sim_len, show_only )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin < 8
    sim_len = 0;
end
if nargin < 9
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


while (~feof(F_IMU))
    pr_count=imu_data(1);
    imu=imu_data(2:7);
    lidar=lidar_data(2:end);

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
    
    [Cbn, vel_n, pos]=strapdown_pln_dcm_v000(Cbn, vel_n, pos, imu(1:3), imu(4:6), g, dt, 0);

    att = dcm2euler_v000(Cbn);

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
    lidar_data=fread(F_LIDAR,2,'double');
end

fclose(F_IMU);
fclose(F_LIDAR);
fclose(F_TRU);
fclose(F_RES);
fclose(F_ERR);
if show_only == 0
    return;
end
end
%% Show results
err_plot_nav(out_err,out_res,in_mnav,DTM,cellsize,success);
end
