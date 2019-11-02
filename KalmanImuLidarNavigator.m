function success = KalmanImuLidarNavigator( in_mnav, in_mimu, in_mlidar, ...
    in_meta, out_res, out_err, window_size, DTM, sim_len, show_only )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin < 9
    sim_len = 0;
end
if nargin < 10
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


% Kalman filter initialization

% x = [position velocity attitude angular_velocity linear_bias
% linear_drift]
x = [pos' vel_n' att' 0 0 0 0 0 0 0 0 0]';

% initial state covraiance
P = 1e-6*eye(length(x));
% covariance of process
Q=0.1^2*eye(length(x));
% covariance of measurement  
R=1e-6^2*eye(length(lidar));


while (~feof(F_IMU))
    pr_count=imu_data(1);
    imu=imu_data(2:7);
    u=[imu(1:3); Cbn*imu(4:6)];
    lidar=lidar_data(2:end);

    fprintf('%d\n', pr_count);
        
    A = [1 0 0 dt 0 0 0 0 0 0 0 0 1 0 0 dt 0 0; ...
         0 1 0 0 dt 0 0 0 0 0 0 0 0 1 0 0 dt 0; ...
         0 0 1 0 0 dt 0 0 0 0 0 0 0 0 1 0 0 dt; ...
         0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0 0; ...
         0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1 0; ...
         0 0 0 0 0 1 0 0 0 0 0 0 0 0 0 0 0 1; ...
         0 0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0 0; ...
         0 0 0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0 0; ...
         0 0 0 0 0 0 0 0 1 0 0 dt 0 0 0 0 0 0; ...
         0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0 0; ...
         0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0 0; ...
         0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0 0; ...
         0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0 0; ...
         0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0 0; ...
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0 0; ...
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0 0; ...
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 0; ...
         0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1; ...
         ];

    B = [0.5*dt*dt 0 0 0 0 0; ...
         0 0.5*dt*dt 0 0 0 0; ...
         0 0 0.5*dt*dt 0 0 0; ...
         1 0 0 0 0 0; ...
         0 1 0 0 0 0; ...
         0 0 1 0 0 0; ...
         0 0 0 0.5*dt*dt 0 0; ...
         0 0 0 0 0.5*dt*dt 0; ...
         0 0 0 0 0 0.5*dt*dt; ...
         0 0 0 1 0 0; ...
         0 0 0 0 1 0; ...
         0 0 0 0 0 1; ...
         0 0 0 0 0 0; ...
         0 0 0 0 0 0; ...
         0 0 0 0 0 0; ...
         0 0 0 0 0 0; ...
         0 0 0 0 0 0; ...
         0 0 0 0 0 0; ...
         ];
 
        f = @(x)(A*x + B*u);
        h = @(x)(CalcRayDistances(x(1:3), euler2dcm_v000(x(7:9)) * diag([1 1 -1]), rays, DTM, cellsize)');
        z = lidar;
        
        [x, P] = ekf(f,x,P,h,z,Q,R);
        
        pos = x(1:3);
        vel = x(4:6);
        att = x(7:9);
        Cbn = euler2dcm_v000(att);
        ang = x(10:12);
        

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
if show_only == 0
    return;
end
end
%% Show results

err=readbin_v000(out_err,11);
res=readbin_v000(out_res,10);
tru=readbin_v000(in_mnav,10);

% p = [1:2850, 2880:size(err, 2)];

[X, Y] = meshgrid((1:size(DTM, 2))*cellsize, (1:size(DTM, 1))*cellsize);

% return;
figure;
contour(X, Y, DTM);
hold on;
plot(res(2,:), res(3,:), 'bo');
% figure;
plot(tru(2,1:size(res, 2)),tru(3,1:size(res, 2)),'r.');
grid;
pbaspect([1 1 1]);
daspect([1 1 1]);
title('Recovered and Original Trajectories');
xlabel('X offset (meters)');
ylabel('Y offset (meters)');
legend('Terrain', 'Recovered', 'Original');

figure;
plot(err(2,:));
title('Error Over Time (X)');
xlabel('Frame Number');
ylabel('X offset (meters)');
hold on;
grid;

figure;
plot(err(3,:));
title('Error Over Time (Y)');
xlabel('Frame Number');
ylabel('Y offset (meters)');
hold on;
grid;

figure;
plot(err(4,:));
title('Error Over Time (Z)');
xlabel('Frame Number');
ylabel('Z offset (meters)');
hold on;
grid;

figure;
plot(err(5,:) .* 1e3);
title('Error Over Time (Yaw)');
xlabel('Frame Number');
ylabel('Yaw offset (mrad)');
hold on;
grid;

figure;
plot(err(6,:) .* 1e3);
title('Error Over Time (Pitch)');
xlabel('Frame Number');
ylabel('Pitch offset (mrad)');
hold on;
grid;

figure;
r = err(7,:);
r(r < -6) = r(r < -6) + 2*pi;
r(r > 6) = r(r > 6) + 2*pi;
plot(r .* 1e3);
title('Error Over Time (Roll)');
xlabel('Frame Number');
ylabel('Roll offset (mrad)');
hold on;
grid;

figure;
plot(err(8,:));
title('Err LIDAR');
hold on;
grid;

% figure;
% plot(err(9,:));
% title('Valid LIDAR');
% hold on;
% grid;

if success
    fprintf('Navigation success!\n');
else
    fprintf('Navigation canceled early\n');
end
pos_err = sqrt(sum(err(2:4,:).^2));
att_err = sqrt(sum(err(5:7,:).^2));
fprintf('Avg. position error: %d\n', mean(pos_err));
fprintf('Max  position error: %d\n', max(pos_err));
fprintf('Avg. attitude error: %d\n', mean(att_err));
fprintf('Max  attitude error: %d\n', max(att_err));

fprintf('x: %d\n', mean(abs(err(2,:))));
fprintf('y: %d\n', mean(abs(err(3,:))));
fprintf('z: %d\n', mean(abs(err(4,:))));
fprintf('yaw: %d\n', 180/pi*mean(abs(err(5,:))));
fprintf('pit: %d\n', 180/pi*mean(abs(err(6,:))));
fprintf('rol: %d\n', 180/pi*mean(abs(r)));

fprintf('x: %d\n', max(abs(err(2,:))));
fprintf('y: %d\n', max(abs(err(3,:))));
fprintf('z: %d\n', max(abs(err(4,:))));
fprintf('yaw: %d\n', 180/pi*max(abs(err(5,:))));
fprintf('pit: %d\n', 180/pi*max(abs(err(6,:))));
fprintf('rol: %d\n', 180/pi*max(abs(r)));

x = sort(abs(err(2,:)));
fprintf('x: %d\n', x(floor(length(x)*9/10)));

x = sort(abs(err(3,:)));
fprintf('y: %d\n', x(floor(length(x)*9/10)));

x = sort(abs(err(4,:)));
fprintf('z: %d\n', x(floor(length(x)*9/10)));

x = sort(abs(err(5,:)));
fprintf('yaw: %d\n', 180/pi*x(floor(length(x)*9/10)));

x = sort(abs(err(6,:)));
fprintf('pit: %d\n', 180/pi*x(floor(length(x)*9/10)));

x = sort(abs(r));
fprintf('rol: %d\n', 180/pi*x(floor(length(x)*9/10)));

end

