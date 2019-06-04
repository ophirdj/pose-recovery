function [ ] = PlotErrors( dir_name )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here


% TODO: read this from a file
lin_errs = [0 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0];
angular_errs = [0 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0];
dtm_errs = [0 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1];


% Ground Truth
[avg_pos0, max_pos0, avg_att0, max_att0] = error_summary(dir_name);


% IMU Noise
err_imu = zeros(length(lin_errs), length(angular_errs), 4);

for j = 1:length(lin_errs)
    for k = 1:length(angular_errs)
        [avg_pos, max_pos, avg_att, max_att] = error_summary([dir_name sprintf('imu_%.0d_%.0d/', lin_errs(j), angular_errs(k))]);
        err_imu(j, k, :) = [avg_pos, max_pos, avg_att, max_att];
    end
end

[lin, ang] = meshgrid(lin_errs, angular_errs);
figure();
surf(lin, ang, err_imu(:, :, 1));
title('IMU Avg Pos');
xlabel('Linear Noise (m)');
ylabel('Angular noise (rad)');
zlabel('Error (m)');
figure();
surf(lin, ang, err_imu(:, :, 2));
title('IMU Max Pos');
xlabel('Linear Noise (m)');
ylabel('Angular noise (rad)');
zlabel('Error (m)');
figure();
surf(lin, ang, err_imu(:, :, 3));
title('IMU Avg Att');
xlabel('Linear Noise (m)');
ylabel('Angular noise (rad)');
zlabel('Error (rad)');
figure();
surf(lin, ang, err_imu(:, :, 4));
title('IMU Max Att');
xlabel('Linear Noise (m)');
ylabel('Angular noise (rad)');
zlabel('Error (rad)');


% DTM Noise
err_dtm = zeros(length(dtm_errs), 4);

for j = 1:length(dtm_errs)
    [avg_pos, max_pos, avg_att, max_att] = error_summary([dir_name sprintf('dtm_%.0d/', dtm_errs(j))]);
    err_dtm(j, :) = [avg_pos, max_pos, avg_att, max_att];
end

figure();
plot(dtm_errs, err_dtm(:, 1));
title('DTM Avg Pos');
xlabel('DTM Noise (m)');
ylabel('Error (m)');
figure();
plot(dtm_errs, err_dtm(:, 2));
title('DTM Max Pos');
xlabel('DTM Noise (m)');
ylabel('Error (m)');
figure();
plot(dtm_errs, err_dtm(:, 3));
title('DTM Avg Att');
xlabel('DTM Noise (m)');
ylabel('Error (rad)');
figure();
plot(dtm_errs, err_dtm(:, 4));
title('DTM Max Att');
xlabel('DTM Noise (m)');
ylabel('Error (rad)');


end

function [avg_pos, max_pos, avg_att, max_att] = error_summary(dir_name)
    err=readbin_v000([dir_name 'err.bin'],11);
    pos_err = sqrt(sum(err(2:4,:).^2));
    att_err = sqrt(sum(err(5:7,:).^2));
    avg_pos = mean(pos_err);
    max_pos = max(pos_err);
    avg_att = mean(att_err);
    max_att = max(att_err);
end

