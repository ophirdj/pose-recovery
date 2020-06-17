function [] = kalman_plot(out_prv)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
prv=readbin_v000(out_prv,30);

x=prv(1:15,:);
C=prv(16:30,:);
C_sqrt=sqrt(C);
rho=C./repmat(std(x,1,1).^2,[size(C,1) 1]);

figure;
subplot(2,3,1);
title('Position Correction Over Time');
xlabel('Frame Number');
ylabel('Position Correction m');
hold on;
grid;
plot(x(1:3,:)');
legend('X', 'Y', 'Z');

subplot(2,3,2);
title('Attitude Correction Over Time');
xlabel('Frame Number');
ylabel('Attitude Correction rad');
hold on;
grid;
plot(x(7:9,:)');
legend('yaw', 'pitch', 'roll');

subplot(2,3,3);
title('Velocity Correction Over Time');
xlabel('Frame Number');
ylabel('Velocity Correction (m/sec)');
hold on;
grid;
plot(x(4:6,:)');
legend('X', 'Y', 'Z');

subplot(2,3,4);
title('Bias Correction Over Time');
xlabel('Frame Number');
ylabel('Accelerometer Bias Correction (m/sec2)');
hold on;
grid;
plot(x(10:12,:)');
legend('X', 'Y', 'Z');

subplot(2,3,5);
title('Drift Correction Over Time');
xlabel('Frame Number');
ylabel('Gyroscope Drift Correction (rad/sec2)');
hold on;
grid;
plot(x(13:15,:)');
legend('yaw', 'pitch', 'roll');

figure;
subplot(2,3,1);
title('Square Root of Covariance (Position)');
xlabel('Frame Number');
ylabel('m');
hold on;
grid;
plot(C_sqrt(1:3,:)');
legend('X', 'Y', 'Z');

subplot(2,3,2);
title('Square Root of Covariance (Attitude)');
xlabel('Frame Number');
ylabel('mrad');
hold on;
grid;
plot(C_sqrt(7:9,:)');
legend('yaw', 'pitch', 'roll');


subplot(2,3,3);
title('Square Root of Covariance (Velocity)');
xlabel('Frame Number');
ylabel('m/sec');
hold on;
grid;
plot(C_sqrt(4:6,:)');
legend('X', 'Y', 'Z');

subplot(2,3,4);
title('Square Root of Covariance (Bias)');
xlabel('Frame Number');
ylabel('m/sec');
hold on;
grid;
plot(C_sqrt(10:12,:)');
legend('X', 'Y', 'Z');

subplot(2,3,5);
title('Square Root of Covariance (Drift)');
xlabel('Frame Number');
ylabel('mrad/sec');
hold on;
grid;
plot(C_sqrt(10:12,:)');
legend('yaw', 'pitch', 'roll');

figure;
subplot(2,3,1);
title('Correlation (Position)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(rho(1:3,:)');
legend('X', 'Y', 'Z');

subplot(2,3,2);
title('Correlation (Attitude)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(rho(7:9,:)');
legend('yaw', 'pitch', 'roll');

subplot(2,3,3);
title('Correlation (Velocity)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(rho(4:6,:)');
legend('X', 'Y', 'Z');

subplot(2,3,4);
title('Correlation (Bias)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(rho(10:12,:)');
legend('X', 'Y', 'Z');

subplot(2,3,5);
title('Correlation (Drift)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(rho(10:12,:)');
legend('yaw', 'pitch', 'roll');

end

