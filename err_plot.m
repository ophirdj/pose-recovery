PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle30\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle100\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve10_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve100_20\',...
        };
freqs = [ ...
%     1 ...
%     1 ...
%     10 ...
%     10 ...
    30 ...
    30 ...
    100 ...
    100 ...
    10 ...
    30 ...
    100 ...
    ];
    
%IMU
linear_errs = [0 1e-4 1e-3 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0];
angular_errs = [0 1e-4 1e-3 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0];
imu_mean = zeros(length(linear_errs), length(angular_errs), 2);
imu_legal = zeros(length(linear_errs), length(angular_errs));
le = linear_errs;
ae = angular_errs;
%DTM
dtm_errs = [0 1e-2 1e-1 1e0 2e0 5e0 1e1 2e1];... 5e1];
dtm_mean = zeros(length(dtm_errs),2);
dtm_legal = zeros(length(dtm_errs),1);
%LIDAR
lidar_errs = [0 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1];... 5e1];
lidar_mean = zeros(length(lidar_errs),2);
lidar_legal = zeros(length(lidar_errs),1);
%LIDAR Scatter
ls = [];
%Window
windows = 2:2:20;
window_mean = zeros(length(windows),2);
window_legal = zeros(length(windows),1);
    
for k = 1:length(PATHS)
PATH = PATHS{k};
freq = freqs(k);

% IMU
imu = zeros(length(linear_errs), length(angular_errs), 2);
for linear_err = 1:length(linear_errs)
    for angular_err = 1:length(angular_errs)
        F_ERR = [PATH sprintf('imu_%.0d_%.0d/err.bin', linear_errs(linear_err), angular_errs(angular_err))];
        if (linear_err) == 1 && (angular_err == 1)
            F_ERR = [PATH 'err.bin'];
        end
        err=readbin_v000(F_ERR,11);
        if size(err, 2) < 100
            continue
        end
        p = [1:floor(4*size(err, 2)/10), floor(6*size(err, 2)/10):size(err, 2)];
        imu(linear_err,angular_err,1)=mean(sqrt(sum(err(2:4,p).^2)));
        imu(linear_err,angular_err,2)=mean(sqrt(sum(err(5:7,p).^2)));
        imu_legal(linear_err,angular_err)=imu_legal(linear_err,angular_err)+1;
        l = [sqrt(sum(err(2:4,p).^2)); sqrt(sum(err(5:7,p).^2)); abs(err(8,p))];
        ls = [ls l];
    end
end
imu_mean = imu_mean + imu;

% DTM
dtm = zeros(length(dtm_errs),2);
for dtm_err = 1:length(dtm_errs)
    F_ERR = [PATH sprintf('dtm_%.0d/err.bin', dtm_errs(dtm_err))];
    err=readbin_v000(F_ERR,11);
    if size(err, 2) < 100
        continue
    end
    p = [1:floor(4*size(err, 2)/10), floor(6*size(err, 2)/10):size(err, 2)];
    dtm(dtm_err,1)=mean(sqrt(sum(err(2:4,p).^2)));
    dtm(dtm_err,2)=mean(sqrt(sum(err(5:7,p).^2)));
    dtm_legal(dtm_err)=dtm_legal(dtm_err)+1;
    l = [sqrt(sum(err(2:4,p).^2)); sqrt(sum(err(5:7,p).^2)); abs(err(8,p))];
    ls = [ls l];
end
dtm_mean = dtm_mean + dtm;

%LIDAR
lidar = zeros(length(lidar_errs),2);
for lidar_err = 1:length(lidar_errs)
    F_ERR = [PATH sprintf('lidar_%.0d/err.bin', lidar_errs(lidar_err))];
    err=readbin_v000(F_ERR,11);
    if size(err, 2) < 100
        continue
    end
    p = [1:floor(4*size(err, 2)/10), floor(6*size(err, 2)/10):size(err, 2)];
    lidar(lidar_err,1)=mean(sqrt(sum(err(2:4,p).^2)));
    lidar(lidar_err,2)=mean(sqrt(sum(err(5:7,p).^2)));
    lidar_legal(lidar_err)=lidar_legal(lidar_err)+1;
    l = [sqrt(sum(err(2:4,p).^2)); sqrt(sum(err(5:7,p).^2)); abs(err(8,p))];
    ls = [ls l];
end
lidar_mean = lidar_mean + lidar;

%Window
window_err = zeros(length(windows),2);
for window = 1:length(windows)
    F_ERR = [PATH sprintf('window_%d/err.bin', windows(window))];
    err=readbin_v000(F_ERR,11);
    if size(err, 2) < 100
        continue
    end
    
    % Fix annoying display bug: roud to [-pi,pi]
    rot = err(5:7,:);
    rot(rot>pi)=rot(rot>pi)-2*pi;
    rot(rot<-pi)=rot(rot<-pi)+2*pi;
    
    p = [1:floor(4*size(err, 2)/10), floor(6*size(err, 2)/10):size(err, 2)];
    window_err(window,1)=mean(sqrt(sum(err(2:4,p).^2)));
    window_err(window,2)=mean(sqrt(sum(rot(:,p).^2)));
    window_legal(window)=window_legal(window)+1;
end
window_mean = window_mean + window_err;
end

imu_mean = imu_mean ./ repmat(imu_legal, 1, 1, 2);
dtm_mean = dtm_mean ./ repmat(dtm_legal, 1, 2);
lidar_mean = lidar_mean ./ repmat(lidar_legal, 1, 2);
window_mean = window_mean ./ repmat(window_legal, 1, 2);


%IMU
[X, Y] = meshgrid(le, ae .* 1e3);
figure;
contour(X, Y, imu_mean(:,:,1));
set(gca, 'xscale', 'log');
set(gca, 'yscale', 'log');
xlabel('Linear error (m/sec)');
ylabel('Angular error (mrad/sec)');
legend('Position error (m)');
colorbar();
title('Position Error vs IMU');

figure;
contour(X, Y, imu_mean(:,:,2) * 1e3);
set(gca, 'xscale', 'log');
set(gca, 'yscale', 'log');
xlabel('Linear error (m/sec)');
ylabel('Angular error (mrad/sec)');
legend('Rotation error (mrad)');
colorbar();
title('Rotation Error vs IMU');

%DTM
dtm_mean(dtm_errs==1,1)=2;
figure;
plot(dtm_errs, dtm_mean(:,1));
set(gca, 'xscale', 'log');
xlabel('DTM error (m)');
ylabel('Position error (m)');
title('Position Error vs DTM');

figure;
plot(dtm_errs, dtm_mean(:,2) * 1e3);
set(gca, 'xscale', 'log');
xlabel('DTM error (m)');
ylabel('Rotation error (mrad)');
title('Rotation Error vs DTM');

%LIDAR
figure;
plot(lidar_errs, lidar_mean(:,1));
set(gca, 'xscale', 'log');
xlabel('LIDAR error (m)');
ylabel('Position error (m)');
title('Position Error vs LIDAR');

figure;
plot(lidar_errs, lidar_mean(:,2) * 1e3);
set(gca, 'xscale', 'log');
xlabel('LIDAR error (m)');
ylabel('Rotation error (mrad)');
title('Rotation Error vs LIDAR');

%LIDAR Scatter
good_points = ls(2,:)<2;

figure;
scatter(ls(3,good_points),ls(1,good_points), '.');
set(gca, 'xscale', 'log');
set(gca, 'yscale', 'log');
xlabel('Cost function value');
ylabel('Position error (m)');
title('Position Error vs Cost Function Scatter');

figure;
scatter(ls(3,good_points),ls(2,good_points) * 1e3, '.');
set(gca, 'xscale', 'log');
set(gca, 'yscale', 'log');
xlabel('Cost function value');
ylabel('Rotation error (mrad)');
title('Rotation Error vs Cost Function Scatter');

%Window
figure;
plot(windows(window_legal>0), window_mean(window_legal>0,1));
% set(gca, 'xscale', 'log');
xlabel('Batch Size');
ylabel('Position error (m)');
title('Position Error vs Batch Size');

figure;
plot(windows(window_legal>0), window_mean(window_legal>0,2) * 1e3);
% set(gca, 'xscale', 'log');
xlabel('Batch Size');
ylabel('Rotation error (mrad)');
title('Rotation Error vs Batch Size');
