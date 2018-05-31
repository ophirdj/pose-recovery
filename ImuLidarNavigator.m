function [ ] = ImuLidarNavigator( in_mnav, in_mimu, in_mlidar, ...
    out_res, out_err, window_size, pos_g, dt, DTM, cellsize )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here

F_IMU=fopen(in_mimu,'rb');
F_LIDAR=fopen(in_mlidar,'rb');
F_TRU=fopen(in_mnav,'rb');
F_RES=fopen(out_res,'wb');
F_ERR=fopen(out_err,'wb');

%Use constant scale for all observations
[Rn, Re, g, sL, cL, WIE_E]=geoparam_v000(pos_g(:));


%%read the input data from the files
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');

lidar_data=fread(F_LIDAR,3,'double');
n_rays = lidar_data(2);
span_angle = lidar_data(3);
lidar_data=[lidar_data; fread(F_LIDAR,n_rays,'double')];

% skip first record (wierd bug - record is not correct)
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');
lidar_data=fread(F_LIDAR,3+n_rays,'double');


% Assume we know initial position, velocity, and orientation (read it from
% true_val)
pos = true_val(2:4);
vel_n = true_val(5:7);
Cbn = euler2dcm_v000(true_val(8:10));
lidar = lidar_data(4:end);

% LIDAR  model
rays = GenerateRays(span_angle / ((n_rays-1)/2), ((n_rays-1)/2));

delta_pos_window = repmat([0 0 0]', [1 window_size-1]);
pos_window = repmat(pos, [1 window_size]);
Cbn_window = repmat(Cbn, [1 1 window_size]);
lidar_window = repmat(lidar, [1 window_size]);


n_generated_points = 1;
n_iterations = 10;
lambda = 3.6;

imu_pos = pos;

while (~feof(F_IMU))
        pr_coun=imu_data(1);
        imu=imu_data(2:7);
        lidar=lidar_data(4:end);
        
        [Cbn, vel_n, pos]=strapdown_pln_dcm_v000(Cbn, vel_n, pos, imu(1:3), imu(4:6), g, dt, 0);
        
        delta_pos_window = [delta_pos_window(:,2:end), pos-imu_pos];
        pos_window = [pos_window(:,2:end) pos];
        Cbn_window = cat(3, Cbn_window(:,:,2:end), Cbn);
        lidar_window = [lidar_window(:,2:end) lidar];
        
        imu_pos = pos;
        
        [pos, Cbn, errors, Pps, Rrs] = ...
        FindPoseMulti(pos_window, Cbn_window, ...
        delta_pos_window, ...
        rays, lidar_window', DTM, cellsize, ...
        0, 0, 1, n_iterations, lambda);
        
        % Calculate position error
        pos_err=pos-true_val(2:4);
        
        % Calculate attitude error
        true_Cbn = euler2dcm_v000(true_val(8:10));
        att_err = dcm2euler_v000(Cbn' * true_Cbn);
        
        % Write recovered results and errors
        fwrite(F_ERR,[pr_coun;pos_err;att_err;-1;-1;-1;-1],'double');
        fwrite(F_RES,[pr_coun;pos; dcm2euler_v000(Cbn); Cbn'*vel_n],'double');
        
        % Read next records
        imu_data=fread(F_IMU,7,'double');
        true_val = fread(F_TRU, 10, 'double');
        lidar_data=fread(F_LIDAR,3+n_rays,'double');
end

fclose(F_IMU);
fclose(F_LIDAR);
fclose(F_TRU);
fclose(F_RES);
fclose(F_ERR);

%% Show results

err=readbin_v000([PATH 'err.bin'],11);
res=readbin_v000([PATH 'res.bin'],10);
tru=readbin_v000([PATH 'mnav.bin'],10);

% return;
figure;
plot(res(2,:), res(3,:), 'b');
hold on;
% figure;
plot(tru(2,:),tru(3,:),'r');
grid;
pbaspect([1 1 1]);
daspect([1 1 1]);
legend('Recovered', 'Original');

figure;
plot(err(2,:));
title('Err x');
hold on;
grid;

figure;
plot(err(3,:));
title('Err y');
hold on;
grid;

figure;
plot(err(4,:));
title('Err z');
hold on;
grid;

figure;
plot(err(5,:));
title('Err yaw');
hold on;
grid;

figure;
plot(err(6,:));
title('Err pitch');
hold on;
grid;

figure;
plot(err(7,:));
title('Err roll');
hold on;
grid;

end

