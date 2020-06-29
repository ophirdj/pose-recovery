function [figs] = kalman_plot(in_prv, in_meta, visible)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
if nargin < 3
    visible = 1;
end

prv=readbin_v000(in_prv,30);

x=prv(1:15,:);
C=prv(16:30,:);
C_sqrt=sqrt(C);
rho=C./repmat(std(x,1,1).^2,[size(C,1) 1]);


meta=readbin_v000(in_meta,1);
freq_Hz = meta(1);
time_series = repmat((1:size(x, 2))/freq_Hz, [3 1])';

RAD2DEG = 180/pi;

if visible
    onoff = 'On';
else
    onoff = 'Off';
end

figs = [];

figs = [figs figure('Name','Correction','doublebuffer',onoff,'Visible',onoff)];
subplot(2,3,1);
title('Position Correction Over Time');
xlabel('Time (sec)');
ylabel('Position Correction (m)');
hold on;
grid;
plot(time_series, cumsum(x(1:3,:))');
legend('X', 'Y', 'Z');

subplot(2,3,2);
title('Attitude Correction Over Time');
xlabel('Time (sec)');
ylabel('Attitude Correction (deg)');
hold on;
grid;
plot(time_series, cumsum(x(7:9,:))'*RAD2DEG);
legend('yaw', 'pitch', 'roll');

subplot(2,3,3);
title('Velocity Correction Over Time');
xlabel('Time (sec)');
ylabel('Velocity Correction (m/sec)');
hold on;
grid;
plot(time_series, cumsum(x(4:6,:))');
legend('X', 'Y', 'Z');

subplot(2,3,4);
title('Bias Over Time');
xlabel('Time (sec)');
ylabel('Accelerometer Bias (m/sec^2)');
hold on;
grid;
plot(time_series, x(10:12,:)');
legend('X', 'Y', 'Z');

subplot(2,3,5);
title('Drift Over Time');
xlabel('Time (sec)');
ylabel('Gyroscope Drift (deg/sec^2)');
hold on;
grid;
plot(time_series, x(13:15,:)'*RAD2DEG);
legend('yaw', 'pitch', 'roll');

figs = [figs figure('Name','Covariance','doublebuffer',onoff,'Visible',onoff)];
subplot(2,3,1);
title('Square Root of Covariance (Position)');
xlabel('Time (sec)');
ylabel('m');
hold on;
grid;
plot(time_series, C_sqrt(1:3,:)');
legend('X', 'Y', 'Z');

subplot(2,3,2);
title('Square Root of Covariance (Attitude)');
xlabel('Time (sec)');
ylabel('deg');
hold on;
grid;
plot(time_series, C_sqrt(7:9,:)'*RAD2DEG);
legend('yaw', 'pitch', 'roll');


subplot(2,3,3);
title('Square Root of Covariance (Velocity)');
xlabel('Time (sec)');
ylabel('m/sec');
hold on;
grid;
plot(time_series, C_sqrt(4:6,:)');
legend('X', 'Y', 'Z');

subplot(2,3,4);
title('Square Root of Covariance (Bias)');
xlabel('Time (sec)');
ylabel('m/sec^2');
hold on;
grid;
plot(time_series, C_sqrt(10:12,:)');
legend('X', 'Y', 'Z');

subplot(2,3,5);
title('Square Root of Covariance (Drift)');
xlabel('Time (sec)');
ylabel('deg/sec^2');
hold on;
grid;
plot(time_series, C_sqrt(10:12,:)'*RAD2DEG);
legend('yaw', 'pitch', 'roll');

figs = [figs figure('Name','Correletaion','doublebuffer',onoff,'Visible',onoff)];
subplot(2,3,1);
title('Correlation (Position)');
xlabel('Time (sec)');
ylabel('Correlation');
hold on;
grid;
plot(time_series, rho(1:3,:)');
legend('X', 'Y', 'Z');

subplot(2,3,2);
title('Correlation (Attitude)');
xlabel('Time (sec)');
ylabel('Correlation');
hold on;
grid;
plot(time_series, rho(7:9,:)');
legend('yaw', 'pitch', 'roll');

subplot(2,3,3);
title('Correlation (Velocity)');
xlabel('Time (sec)');
ylabel('Correlation');
hold on;
grid;
plot(time_series, rho(4:6,:)');
legend('X', 'Y', 'Z');

subplot(2,3,4);
title('Correlation (Bias)');
xlabel('Time (sec)');
ylabel('Correlation');
hold on;
grid;
plot(time_series, rho(10:12,:)');
legend('X', 'Y', 'Z');

subplot(2,3,5);
title('Correlation (Drift)');
xlabel('Time (sec)');
ylabel('Correlation');
hold on;
grid;
plot(time_series, rho(10:12,:)');
legend('yaw', 'pitch', 'roll');

end
