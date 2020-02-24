function success = ImuLidarNavigator( in_mnav, in_mimu, in_mlidar, ...
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
pos = true_val(2:4);
vel_n = true_val(5:7);
att = true_val(8:10);
Cbn = euler2dcm_v000(att);
lidar = lidar_data(2:end);

% LIDAR  model
rays = GenerateRays(span_angle / ((n_rays-1)/2), ((n_rays-1)/2));

pos_window = repmat(pos, [1 window_size]);
Cbn_window = repmat(Cbn, [1 1 window_size]);
lidar_window = repmat(lidar, [1 window_size]);
delta_pos_window = repmat([0 0 0]', [1 window_size-1]);
delta_att_window = repmat([0 0 0]', [1 window_size-1]);
att_last = att;

kalmanFilterImu = configureKalmanFilter('ConstantAcceleration',[pos; att; vel_n], [1 1 1]*1e-6, [1, 1, 1]*1e-1, 1e0);

while (~feof(F_IMU))
        pr_count=imu_data(1);
        imu=imu_data(2:7);
        lidar=lidar_data(2:end);
        
        fprintf('%d\n', pr_count);
        
        if pr_count < window_size
            pos = true_val(2:4);
            att = true_val(8:10);
            Cbn = euler2dcm_v000(att);
            vel_n = true_val(5:7);

            pos_window = [pos_window(:,2:end), pos];
            Cbn_window = cat(3, Cbn_window(:,:,2:end), Cbn);
            lidar_window = [lidar_window(:,2:end), lidar];
            delta_pos_window = [delta_pos_window(:,2:end), pos - pos_window(:,end-1)];
            delta_att_window = [delta_att_window(:,2:end), att - att_last];
            att_last = att;
            
            predict(kalmanFilterImu);
            v = correct(kalmanFilterImu, [pos; att; vel_n]);
            pos = v(1:3)';
            att = v(4:6)';
            vel_n = v(7:9)';
        else
            [Cbn, vel_n, pos]=strapdown_pln_dcm_v000(Cbn, vel_n, pos, imu(1:3), imu(4:6), g, dt, 0);
            att = dcm2euler_v000(Cbn);

            predict(kalmanFilterImu);
            v = correct(kalmanFilterImu, [pos; att; vel_n]);
            pos = v(1:3)';
            att = v(4:6)';
            vel_n = v(7:9)';

            pos_window = [pos_window(:,2:end), pos];
            Cbn_window = cat(3, Cbn_window(:,:,2:end), Cbn);
            lidar_window = [lidar_window(:,2:end), lidar];
            delta_pos_window = [delta_pos_window(:,2:end), pos - pos_window(:,end-1)];
            delta_att_window = [delta_att_window(:,2:end), att - att_last];

            [pos_window2, Cbn_window2] = strapdown_lidar(pos_window, Cbn_window, ...
                delta_pos_window, delta_att_window, ...
                rays, lidar_window, DTM, cellsize, vel_n .* dt);
            
            pos = pos_window2(:,end);
            Cbn = Cbn_window2(:,:,end);
            att = dcm2euler_v000(Cbn);
            
            pos_window(:,end) = pos;
            Cbn_window(:,:,end) = Cbn;
            
            att_last = att;
            
            correct(kalmanFilterImu, [pos; att; vel_n]);
        end
        
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
err_plot_nav(out_err,out_res,in_mnav,DTM,cellsize,success)
end

