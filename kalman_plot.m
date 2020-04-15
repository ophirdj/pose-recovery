function [] = kalman_plot(out_prv)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
prv=readbin_v000(out_prv,30);

x=prv(1:15,:);
C=prv(16:30,:);
C_sqrt=sqrt(C);
rho=C./repmat(std(x,1,1).^2,[size(C,1) 1]);

figure;
title('Accelerometer Bias Over Time');
xlabel('Frame Number');
ylabel('Accelerometer Bias (m/sec)');
hold on;
grid;
plot(x(10:12,:)');
legend('X', 'Y', 'Z');

figure;
title('Gyroscope Drift Over Time');
xlabel('Frame Number');
ylabel('Gyroscope Drift (rad/sec)');
hold on;
grid;
plot(x(13:15,:)');
legend('yaw', 'pitch', 'roll');


figure;
title('Square Root of Covariance (Position)');
xlabel('Frame Number');
ylabel('m');
hold on;
grid;
plot(C_sqrt(1:3,:)');
legend('X', 'Y', 'Z');


figure;
title('Square Root of Covariance (Attitude)');
xlabel('Frame Number');
ylabel('rad');
hold on;
grid;
plot(C_sqrt(7:9,:)');
legend('yaw', 'pitch', 'roll');


figure;
title('Square Root of Covariance (Velocity)');
xlabel('Frame Number');
ylabel('m/sec');
hold on;
grid;
plot(C_sqrt(4:6,:)');
legend('X', 'Y', 'Z');


figure;
title('Square Root of Covariance (Accelerometer Bias)');
xlabel('Frame Number');
ylabel('m/sec');
hold on;
grid;
plot(C_sqrt(10:12,:)');
legend('X', 'Y', 'Z');


figure;
title('Square Root of Covariance (Gyroscope Drift)');
xlabel('Frame Number');
ylabel('rad/sec');
hold on;
grid;
plot(C_sqrt(10:12,:)');
legend('yaw', 'pitch', 'roll');

% figure;
% title('Correlation (Position)');
% xlabel('Frame Number');
% ylabel('Correlation');
% hold on;
% grid;
% plot(rho(1:3,:)');
% legend('X', 'Y', 'Z');
% 
% 
% figure;
% title('Correlation (Attitude)');
% xlabel('Frame Number');
% ylabel('Correlation');
% hold on;
% grid;
% plot(rho(7:9,:)');
% legend('yaw', 'pitch', 'roll');
% 
% 
% figure;
% title('Correlation (Velocity)');
% xlabel('Frame Number');
% ylabel('Correlation');
% hold on;
% grid;
% plot(rho(4:6,:)');
% legend('X', 'Y', 'Z');
% 
% 
% figure;
% title('Correlation (Accelerometer Bias)');
% xlabel('Frame Number');
% ylabel('Correlation');
% hold on;
% grid;
% plot(rho(10:12,:)');
% legend('X', 'Y', 'Z');
% 
% 
% figure;
% title('Correlation (Gyroscope Drift)');
% xlabel('Frame Number');
% ylabel('Correlation');
% hold on;
% grid;
% plot(rho(10:12,:)');
% legend('yaw', 'pitch', 'roll');

end

