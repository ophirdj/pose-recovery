function [] = err_plot_nav(out_err,out_res,in_mnav,in_meta,DTM,cellsize,success)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
err=readbin_v000(out_err,11);
res=readbin_v000(out_res,10);
tru=readbin_v000(in_mnav,10);
meta=readbin_v000(in_meta,1);

freq_Hz = meta(1);
time_series = (1:size(err, 2))/freq_Hz;

RAD2DEG = pi/180;

[X, Y] = meshgrid((1:size(DTM, 2))*cellsize, (1:size(DTM, 1))*cellsize);

figure('Name','Trajectory');
subplot(1,2,1);
contour(X, Y, DTM);
hold on;
plot(res(2,:), res(3,:), 'bo');
plot(tru(2,2:size(res, 2)+1),tru(3,2:size(res, 2)+1),'r.');
grid;
pbaspect([1 1 1]);
daspect([1 1 1]);
title('Recovered and Original Trajectories');
xlabel('X offset (m)');
ylabel('Y offset (m)');
legend('Terrain', 'Recovered', 'Original');

subplot(1,2,2);
contour(X, Y, DTM);
hold on;
quiver(tru(2,2:size(res, 2)+1), tru(3,2:size(res, 2)+1), ...
    res(2,:)-tru(2,2:size(res, 2)+1), res(3,:)-tru(3,2:size(res, 2)+1), 0);
grid;
pbaspect([1 1 1]);
daspect([1 1 1]);
title('Difference between Recovered and Original Trajectories');
xlabel('X offset (m)');
ylabel('Y offset (m)');
legend('Terrain', 'Difference');

figure('Name','Error');
subplot(2,3,1);
plot(time_series, err(2,:));
title('Error Over Time (X)');
xlabel('Time (sec)');
ylabel('X offset (m)');
hold on;
grid;

subplot(2,3,2);
plot(time_series, err(3,:));
title('Error Over Time (Y)');
xlabel('Time (sec)');
ylabel('Y offset (m)');
hold on;
grid;

subplot(2,3,3);
plot(time_series, err(4,:));
title('Error Over Time (Z)');
xlabel('Time (sec)');
ylabel('Z offset (m)');
hold on;
grid;

subplot(2,3,4);
plot(time_series, err(5,:)*RAD2DEG);
title('Error Over Time (Yaw)');
xlabel('Time (sec)');
ylabel('Yaw offset (deg)');
hold on;
grid;

subplot(2,3,5);
plot(time_series, err(6,:)*RAD2DEG);
title('Error Over Time (Pitch)');
xlabel('Time (sec)');
ylabel('Pitch offset (deg)');
hold on;
grid;

subplot(2,3,6);
plot(time_series, asin(sin(err(7,:)))*RAD2DEG);
title('Error Over Time (Roll)');
xlabel('Time (sec)');
ylabel('Roll offset (deg)');
hold on;
grid;

figure('Name','Lidar');
subplot(2,1,1);
plot(time_series, err(8,:));
title('Err LIDAR (All)');
xlabel('Time (sec)');
ylabel('Lidar error (m)');
hold on;
grid;

subplot(2,1,2);
plot(err(1,mod(err(1,:),10)==0)/freq_Hz, err(8,mod(err(1,:),10)==0));
title('Err LIDAR (Measured)');
xlabel('Time (sec)');
ylabel('Lidar error (m)');
hold on;
grid;

%% Print summary

fprintf('Mean Error x: %d\n', mean(abs(err(2,:))));
fprintf('Mean Error y: %d\n', mean(abs(err(3,:))));
fprintf('Mean Error z: %d\n', mean(abs(err(4,:))));
fprintf('Mean Error yaw: %d\n', RAD2DEG*mean(abs(err(5,:))));
fprintf('Mean Error pit: %d\n', RAD2DEG*mean(abs(err(6,:))));
fprintf('Mean Error rol: %d\n', RAD2DEG*mean(abs(asin(sin(err(7,:))))));

x = sort(abs(err(2,:)));
fprintf('90%% Error x: %d\n', x(floor(length(x)*9/10)));

x = sort(abs(err(3,:)));
fprintf('90%% Error y: %d\n', x(floor(length(x)*9/10)));

x = sort(abs(err(4,:)));
fprintf('90%% Error z: %d\n', x(floor(length(x)*9/10)));

x = sort(abs(err(5,:)));
fprintf('90%% Error yaw: %d\n', RAD2DEG*x(floor(length(x)*9/10)));

x = sort(abs(err(6,:)));
fprintf('90%% Error pit: %d\n', RAD2DEG*x(floor(length(x)*9/10)));

x = sort(abs(asin(sin(err(7,:)))));
fprintf('90%% Error rol: %d\n', RAD2DEG*x(floor(length(x)*9/10)));

if success
    fprintf('Navigation success!\n');
else
    fprintf('Navigation canceled early\n');
end

end

