function [] = err_plot_nav(out_err,out_res,in_mnav,DTM,cellsize,success)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
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
plot(tru(2,2:size(res, 2)+1),tru(3,2:size(res, 2)+1),'r.');
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

