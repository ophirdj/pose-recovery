clear;
fclose all;
PATH='C:\Users\Ophir\matlab_workspace\trajectories\circle100\';

%% Navigate
% F_IMU=fopen([PATH 'mimu.bin'],'rb'); % Errorless
F_IMU=fopen([PATH 'eimu.bin'],'rb'); % IMU error
F_TRU=fopen([PATH 'mnav.bin'],'rb');
F_RES=fopen([PATH 'res.bin'],'wb');
F_ERR=fopen([PATH 'err.bin'],'wb');

dt=1/100;
pos_g=[0 0 0]';

%Use constant scale for all observations
[Rn, Re, g, sL, cL, WIE_E]=geoparam_v000(pos_g);

%%read the input data from the files
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');

% skip first record (wierd bug - record is not correct)
imu_data=fread(F_IMU,7,'double');
true_val = fread(F_TRU, 10, 'double');


% Assume we know initial position, velocity, and orientation (read it from
% true_val)
pos = true_val(2:4);
vel_n = true_val(5:7);
Cbn = euler2dcm_v000(true_val(8:10));

while (~feof(F_IMU))
        pr_coun=imu_data(1);
        imu=imu_data(2:7);
        
        [Cbn, vel_n, pos]=strapdown_pln_dcm_v000(Cbn, vel_n, pos, imu(1:3), imu(4:6), g, dt, 0);
        
        % Calculate position error
        pos_err=pos-true_val(2:4);
        
        % Calculate attitude error
        true_Cbn = euler2dcm_v000(true_val(8:10));
        att_err = dcm2euler_v000(Cbn' * true_Cbn);
        
        % Write recovered results and errors
        fwrite(F_ERR,[pr_coun;pos_err;att_err;-1;-1;-1;-1],'double');
        fwrite(F_RES,[pr_coun;pos; dcm2euler_v000(Cbn); Cbn'*vel_n],'double');
        
        if any(abs(pos_err)>0.2)
            success = false;
            break;
        end
        
        if any(abs(pos_err)>0.2)
            success = false;
            break;
        end
        
        % Read next records
        imu_data=fread(F_IMU,7,'double');
        true_val = fread(F_TRU, 10, 'double');
end

fclose(F_IMU);
fclose(F_TRU);
fclose(F_RES);
fclose(F_ERR);

%% Show results

err=readbin_v000([PATH 'err.bin'],11);
res=readbin_v000([PATH 'res.bin'],10);
tru=readbin_v000([PATH 'mnav.bin'],10);

% return;
figure;
plot(res(2,:), res(3,:), 'b');
hold on;
% figure;
plot(tru(2,:),tru(3,:),'r');
grid;
pbaspect([1 1 1]);
daspect([1 1 1]);
legend('Recovered', 'Original');

figure;
plot(err(2,:));
title('Err x');
hold on;
grid;

figure;
plot(err(3,:));
title('Err y');
hold on;
grid;

figure;
plot(err(4,:));
title('Err z');
hold on;
grid;

figure;
plot(err(5,:));
title('Err yaw');
hold on;
grid;

figure;
plot(err(6,:));
title('Err pitch');
hold on;
grid;

figure;
plot(err(7,:));
title('Err roll');
hold on;
grid;