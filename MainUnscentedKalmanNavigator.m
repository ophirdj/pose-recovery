addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_1\',...
        };
    
ERR_FILENAME = 'err_unscented.bin';
RES_FILENAME = 'res_unscented.bin';
PRV_FILENAME = 'prv_unscented.bin';

show_only = 2;
sim_len = 12000;

scenarios = {};
scenario_names = {};
logs = {};
ls = {};

accelerometer_bias_m_per_sec2 = [0 0 0]; % X Y Z
gyro_drift_rad_per_sec2 = [0 0 0]*pi/180; % yaw pitch roll

for k = 1:length(PATHS)
    PATH = PATHS{k};
    F_LOG = fopen([PATH 'log.txt'],'w');
    ls{end+1} = F_LOG;
    
    Q = ...
        diag([1e-1 1e-1 1e-2                                         ... % pos
              1e-6 1e-6 1e-6                                         ... % vel (acc variance)
              (0.1*pi/180/60*1e3)^2 (0.1*pi/180/60*1e3)^2 (0.1*pi/180/60*1e3)^2  ... % att
              0 0 0                                                  ... % acc-bias
              0 0 0                                                  ... % gyr-bias
              ]);
                      
    P = ...
    diag([1 1 1                                                      ... % pos
          1e-1 1e-1 1e-1                                             ... % vel
          (0.5*pi/180*1e3)^2 (0.5*pi/180*1e3)^2 (0.5*pi/180*1e3)^2   ... % att
          1e-4 1e-4 1e-4                                             ... % bias-acc
          (10*pi/180/3600*1e3)^2 (10*pi/180/3600*1e3)^2 (10*pi/180/3600*1e3)^2   ... % bias-gyr
          ]);
    
    % Ground Truth
    scenario_names{end+1} = 'Ground Truth';
    logs{end+1} = F_LOG;
    scenarios{end+1} = @()...
    UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [PATH RES_FILENAME], [PATH ERR_FILENAME], [PATH PRV_FILENAME], ...
        DTM, Q, 1e-9, 1e-2, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2, sim_len, show_only);
    
%     % IMU
%     for linear_err = [0 1e-4 5e-4 1e-3 5e-3 1e-2 5e-2 1e-1 5e-1 1e0 5e0]
%         for angular_err = [0 1e-4 5e-4 1e-3 5e-3 1e-2]
%             scenario_names{end+1} = sprintf('%s %.0d %.0d', 'IMU', linear_err, angular_err);
%             logs{end+1} = F_LOG;
%             dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
%             scenarios{end+1} = @()...
%             UnscentedKalmanNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
%         [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
%         DTM, Q, 1e-6, 1e-3, P, sim_len, show_only);
%         end
%     end
%     
%     % Bias & drift
%     for accelerometer_bias = [1e-1] ...[0 1e-2 1e-1 1e0 1e1 1e2]
%         for gyro_drift = [0] ...[0 1e-3 5e-3 1e-2 5e-2 1e-1 5e-1 1e0]
%             scenario_names{end+1} = sprintf('%s %.0d %.0d', 'Bias/drift', accelerometer_bias, gyro_drift);
%             logs{end+1} = F_LOG;
%             dir = [PATH sprintf('bd_%.0d_%.0d/', accelerometer_bias, gyro_drift)];
%             scenarios{end+1} = @()...
%             UnscentedKalmanNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
%         [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
%         DTM, Q, 1e-9, 1e-2, P, sim_len, show_only);
%         end
%     end
%     
%     % DTM
%     for dtm_err = [0 1e-2 1e-1 1e0 2e0 5e0 1e1]
%         scenario_names{end+1} = sprintf('%s %.0d', 'DTM', dtm_err);
%         logs{end+1} = F_LOG;
%         dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
%         scenarios{end+1} = @()...
%         UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%             [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
%             DTM, Q, 1e-6, 1e-3, P, sim_len, show_only);
%     end
%     
%     % LIDAR
%     for lidar_err = [0 1e-4 1e-3 1e-2 5e-2 1e-1 5e-1 1e0 5e0 1e1]
%         scenario_names{end+1} = sprintf('%s %.0d\n', 'LIDAR', lidar_err);
%         logs{end+1} = F_LOG;
%         dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
%         scenarios{end+1} = @()...
%         UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%             [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
%             DTM, Q, 1e-6, 1e-3, P, sim_len, show_only);
%     end
    
end
%%
% parfor j = 1:length(scenarios)
for j = 1:length(scenarios)
    try
        fprintf('%s\n', scenario_names{j});
        if scenarios{j}()
            fprintf('%s SUCCESS\n', scenario_names{j});
        else
            fprintf('%s FAIL\n', scenario_names{j});
        end
%         close all;
    catch e
        fprintf(logs{j}, '%s\n%s\n', scenario_names{j}, getReport(e));
        fprintf('%s\n%s\n', scenario_names{j}, getReport(e));
    end
end

for k = 1:length(ls)
    fclose(ls{k});
end

