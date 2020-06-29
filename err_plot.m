PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_3\'
        };
freqs = [ ...
    100 ...
    ];

NAVIGATOR = 'unscented';
ERR_FILENAME = ['err_' NAVIGATOR '.bin'];
RES_FILENAME = ['res_' NAVIGATOR '.bin'];

MIN_SAMPLE_SIZE = 50;

scenario_test_parameters();
    
%IMU
imu_mean = zeros(length(accelerometer_variances_m_sec2), length(gyro_variances_deg_sec2), 2);
imu_legal = zeros(length(accelerometer_variances_m_sec2), length(gyro_variances_deg_sec2));
le = accelerometer_variances_m_sec2;
ae = gyro_variances_deg_sec2;
%DTM
dtm_mean = zeros(length(dtm_errs_m),2);
dtm_legal = zeros(length(dtm_errs_m),1);
%LIDAR
lidar_mean = zeros(length(lidar_errs_percent),2);
lidar_legal = zeros(length(lidar_errs_percent),1);
%LIDAR Scatter
ls = [];
    
for k = 1:length(PATHS)
PATH = PATHS{k};
freq = freqs(k);

% IMU
imu = zeros(length(accelerometer_variances_m_sec2), length(gyro_variances_deg_sec2), 2);
for linear_err = 1:length(accelerometer_variances_m_sec2)
    for angular_err = 1:length(gyro_variances_deg_sec2)
        F_ERR = [PATH sprintf('imu_%.0d_%.0d/%s', accelerometer_variances_m_sec2(linear_err), gyro_variances_deg_sec2(angular_err), ERR_FILENAME)];
        if (linear_err) == 1 && (angular_err == 1)
            F_ERR = [PATH ERR_FILENAME];
        end
        err=readbin_v000(F_ERR,11);
        if size(err, 2) < MIN_SAMPLE_SIZE
            continue
        end
        imu(linear_err,angular_err,1)=mean(sqrt(sum(err(2:4,:).^2)));
        imu(linear_err,angular_err,2)=mean(sqrt(sum(err(5:7,:).^2)));
        imu_legal(linear_err,angular_err)=imu_legal(linear_err,angular_err)+1;
        l = [sqrt(sum(err(2:4,:).^2)); sqrt(sum(err(5:7,:).^2)); abs(err(8,:))];
        ls = [ls l];
    end
end
imu_mean = imu_mean + imu;

% DTM
dtm = zeros(length(dtm_errs_m),2);
for dtm_err = 1:length(dtm_errs_m)
    F_ERR = [PATH sprintf('dtm_%.0d/%s', dtm_errs_m(dtm_err), ERR_FILENAME)];
    err=readbin_v000(F_ERR,11);
    if size(err, 2) < MIN_SAMPLE_SIZE
        continue
    end
    dtm(dtm_err,1)=mean(sqrt(sum(err(2:4,:).^2)));
    dtm(dtm_err,2)=mean(sqrt(sum(err(5:7,:).^2)));
    dtm_legal(dtm_err)=dtm_legal(dtm_err)+1;
    l = [sqrt(sum(err(2:4,:).^2)); sqrt(sum(err(5:7,:).^2)); abs(err(8,:))];
    ls = [ls l];
end
dtm_mean = dtm_mean + dtm;

%LIDAR
lidar = zeros(length(lidar_errs_percent),2);
for lidar_err = 1:length(lidar_errs_percent)
    F_ERR = [PATH sprintf('lidar_%.0d/%s', lidar_errs_percent(lidar_err), ERR_FILENAME)];
    err=readbin_v000(F_ERR,11);
    if size(err, 2) < MIN_SAMPLE_SIZE
        continue
    end
    lidar(lidar_err,1)=mean(sqrt(sum(err(2:4,:).^2)));
    lidar(lidar_err,2)=mean(sqrt(sum(err(5:7,:).^2)));
    lidar_legal(lidar_err)=lidar_legal(lidar_err)+1;
    l = [sqrt(sum(err(2:4,:).^2)); sqrt(sum(err(5:7,:).^2)); abs(err(8,:))];
    ls = [ls l];
end
lidar_mean = lidar_mean + lidar;

end

imu_mean = imu_mean ./ repmat(imu_legal, 1, 1, 2);
dtm_mean = dtm_mean ./ repmat(dtm_legal, 1, 2);
lidar_mean = lidar_mean ./ repmat(lidar_legal, 1, 2);


%% Plot data

%IMU
[X, Y] = meshgrid(le, ae);
figure;
contour(X, Y, imu_mean(:,:,1)');
set(gca, 'xscale', 'log');
set(gca, 'yscale', 'log');
xlabel('Accelerometer Variance (m/sec^2)');
ylabel('Gyro Variance (deg/sec^2)');
legend('Position error (m)');
colorbar();
title('Position Error vs IMU');

figure;
contour(X, Y, imu_mean(:,:,2)');
set(gca, 'xscale', 'log');
set(gca, 'yscale', 'log');
xlabel('Accelerometer Variance (m/sec^2)');
ylabel('Gyro Variance (deg/sec^2)');
legend('Rotation error (deg)');
colorbar();
title('Rotation Error vs IMU');

%DTM
figure;
plot(dtm_errs_m, dtm_mean(:,1));
set(gca, 'xscale', 'log');
xlabel('DTM error (m)');
ylabel('Position error (m)');
title('Position Error vs DTM');

figure;
plot(dtm_errs_m, dtm_mean(:,2));
set(gca, 'xscale', 'log');
xlabel('DTM error (m)');
ylabel('Rotation error (deg)');
title('Rotation Error vs DTM');

%LIDAR
figure;
plot(lidar_errs_percent, lidar_mean(:,1));
set(gca, 'xscale', 'log');
xlabel('LIDAR error (%)');
ylabel('Position error (m)');
title('Position Error vs LIDAR');

figure;
plot(lidar_errs_percent, lidar_mean(:,2));
set(gca, 'xscale', 'log');
xlabel('LIDAR error (%)');
ylabel('Rotation error (deg)');
title('Rotation Error vs LIDAR');

%LIDAR Scatter
% good_points = ls(2,:)<2;

% figure;
% scatter(ls(3,good_points),ls(1,good_points), '.');
% set(gca, 'xscale', 'log');
% set(gca, 'yscale', 'log');
% xlabel('Cost function value');
% ylabel('Position error (m)');
% title('Position Error vs Cost Function Scatter');
% 
% figure;
% scatter(ls(3,good_points),ls(2,good_points), '.');
% set(gca, 'xscale', 'log');
% set(gca, 'yscale', 'log');
% xlabel('Cost function value');
% ylabel('Rotation error (deg)');
% title('Rotation Error vs Cost Function Scatter');
