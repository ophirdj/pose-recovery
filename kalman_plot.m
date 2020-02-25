function [] = kalman_plot(out_prv)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
prv=readbin_v000(out_prv,30);

x=prv(1:15,:);
cov=prv(16:30,:);
corr=cov./repmat(std(cov,1,1).^2,[size(cov,1) 1]);

figure;
title('Linear Bias Over Time');
xlabel('Frame Number');
ylabel('Linear Bias (meters)');
hold on;
grid;
plot(x(10:12,:)');
legend('X', 'Y', 'Z');

figure;
title('Angular Bias Over Time');
xlabel('Frame Number');
ylabel('Angular Bias (mrad)');
hold on;
grid;
plot(x(13:15,:)' * 1e3);
legend('yaw', 'pitch', 'roll');


figure;
title('Covariance (Position)');
xlabel('Frame Number');
ylabel('Covariance');
hold on;
grid;
plot(cov(1:3,:)');
legend('X', 'Y', 'Z');


figure;
title('Covariance (Attitude)');
xlabel('Frame Number');
ylabel('Covariance');
hold on;
grid;
plot(cov(7:9,:)');
legend('yaw', 'pitch', 'roll');


figure;
title('Covariance (Velocity)');
xlabel('Frame Number');
ylabel('Covariance');
hold on;
grid;
plot(cov(4:6,:)');
legend('X', 'Y', 'Z');


figure;
title('Covariance (Linear Bias)');
xlabel('Frame Number');
ylabel('Covariance');
hold on;
grid;
plot(cov(10:12,:)');
legend('X', 'Y', 'Z');


figure;
title('Covariance (Angular Bias)');
xlabel('Frame Number');
ylabel('Covariance');
hold on;
grid;
plot(cov(10:12,:)');
legend('X', 'Y', 'Z');

figure;
title('Correlation (Position)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(corr(1:3,:)');
legend('X', 'Y', 'Z');


figure;
title('Correlation (Attitude)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(corr(7:9,:)');
legend('yaw', 'pitch', 'roll');


figure;
title('Correlation (Velocity)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(corr(4:6,:)');
legend('X', 'Y', 'Z');


figure;
title('Correlation (Linear Bias)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(corr(10:12,:)');
legend('X', 'Y', 'Z');


figure;
title('Correlation (Angular Bias)');
xlabel('Frame Number');
ylabel('Correlation');
hold on;
grid;
plot(corr(10:12,:)');
legend('X', 'Y', 'Z');

end

