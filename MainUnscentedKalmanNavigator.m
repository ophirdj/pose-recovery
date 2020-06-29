addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();
scenario_test_parameters();

PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_1\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_2\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_3\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_4\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_5\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_6\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_7\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_8\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_9\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_10\',...
        };
    
ERR_FILENAME = 'err_unscented.bin';
RES_FILENAME = 'res_unscented.bin';
PRV_FILENAME = 'prv_unscented.bin';

show_only = 0;
sim_len = 0;

scenarios = {};
scenario_names = {};
logs_path = {};
ls = {};
figs_path = {};

accelerometer_bias_m_per_sec2 = [0.1 0.05 -0.1]; % X Y Z
gyro_drift_rad_per_sec2 = [0 0 0]*pi/180; % yaw pitch roll

ini_pos_err_m = [10 20 0];
ini_vel_err_m_sec = [0 0 0];
ini_att_err_rad = [0 0 0]*pi/180;

P = ...
diag([10 10 10                                          ... % pos
      1 1 1                                             ... % vel
      0.5*pi/180 0.5*pi/180 0.5*pi/180                  ... % att
      1e-1 1e-1 1e-1                                    ... % bias-acc
      5*pi/180/3600 5*pi/180/3600 5*pi/180/3600         ... % bias-gyr
      ].^2);

for k = 1:length(PATHS)
    PATH = PATHS{k};
    F_LOG = fopen([PATH 'log.txt'],'w');
    ls{end+1} = F_LOG;
    
    
    % Ground Truth
    scenario_names{end+1} = sprintf('%s%s', PATH, 'Ground Truth');
    figs_path{end+1} = [PATH 'figs/'];
    logs_path{end+1} = F_LOG;
    
    
    Q = ...
        diag([1e-1 1e-1 1e-2                                         ... % pos
              1e-1 1e-1 1e-2                                         ... % vel (acc variance)
              0.1*pi/180/60 0.1*pi/180/60 0.1*pi/180/60              ... % att
              1e-5 1e-5 1e-5                                         ... % acc-bias
              0.1*pi/180/3600 0.1*pi/180/3600 0.1*pi/180/3600        ... % gyr-bias
              ].^2);
    
    scenarios{end+1} = @()...
    UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [PATH RES_FILENAME], [PATH ERR_FILENAME], [PATH PRV_FILENAME], ...
        DTM, Q, 1e-9, 1e-2, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2, ...
        ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
    
    % IMU
    for linear_err = accelerometer_variances_m_sec2
        for angular_err = gyro_variances_deg_sec2
            scenario_names{end+1} = sprintf('%s%s %.0d %.0d', PATH, 'IMU', linear_err, angular_err);
            logs_path{end+1} = F_LOG;
            dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
            figs_path{end+1} = [dir 'figs/'];
            
            Q = ...
            diag([1e-1 + (linear_err/100^2) ... % pos
                  1e-1 + (linear_err/100^2) ...
                  1e-2 + (linear_err/100^2) ...
                  1e-1 + (linear_err/100) ...
                  1e-1 + (linear_err/100) ...
                  1e-2 + (linear_err/100) ... % vel
                  0.1*pi/180/60 + (angular_err*pi/180/100^2) ...
                  0.1*pi/180/60 + (angular_err*pi/180/100^2) ...
                  0.1*pi/180/60 + (angular_err*pi/180/100^2) ... % att
                  1e-5 + linear_err ...
                  1e-5 + linear_err ...
                  1e-5 + linear_err ... % acc-bias
                  0.1*pi/180/3600 + (angular_err*pi/180) ...
                  0.1*pi/180/3600 + (angular_err*pi/180) ...
                  0.1*pi/180/3600 + (angular_err*pi/180) ... % gyro-bias
                  ].^2);
              
              scenarios{end+1} = @()...
                UnscentedKalmanNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
                DTM, Q, 1e-9, 1e-2, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2, ...
                ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
        end
    end
    
    % DTM
    for dtm_err = dtm_errs_m
        scenario_names{end+1} = sprintf('%s%s %.0d', PATH, 'DTM', dtm_err);
        logs_path{end+1} = F_LOG;
        dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
        figs_path{end+1} = [dir 'figs/'];
            
    Q = ...
        diag([1e-1 1e-1 1e-2                                         ... % pos
              1e-1 1e-1 1e-2                                         ... % vel (acc variance)
              0.1*pi/180/60 0.1*pi/180/60 0.1*pi/180/60              ... % att
              1e-5 1e-5 1e-5                                         ... % acc-bias
              0.1*pi/180/3600 0.1*pi/180/3600 0.1*pi/180/3600        ... % gyr-bias
              ].^2);

        scenarios{end+1} = @()...
        UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
            [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
            DTM, Q, 1e-9+dtm_err, 1e-2, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2, ...
            ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
    end
    
    % LIDAR
    for lidar_err = lidar_errs_percent
        scenario_names{end+1} = sprintf('%s%s %.0d', PATH, 'LIDAR', lidar_err);
        logs_path{end+1} = F_LOG;
        dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
        figs_path{end+1} = [dir 'figs/'];
        
        Q = ...
        diag([1e-1 1e-1 1e-2                                         ... % pos
              1e-1 1e-1 1e-2                                         ... % vel (acc variance)
              0.1*pi/180/60 0.1*pi/180/60 0.1*pi/180/60              ... % att
              1e-5 1e-5 1e-5                                         ... % acc-bias
              0.1*pi/180/3600 0.1*pi/180/3600 0.1*pi/180/3600        ... % gyr-bias
              ].^2);
          
        scenarios{end+1} = @()...
        UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
            [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
            DTM, Q, 1e-9+lidar_err*1e3/100, 1e-2, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec2, ...
            ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
    end
    
end
%%
% parfor j = 1:length(scenarios)
for j = 1:length(scenarios)
    try
%         fprintf('%s\n', scenario_names{j});
        [success, steps, figHandles] = scenarios{j}();
        if ~isfolder(figs_path{j})
            mkdir(figs_path{j});
        end
        for n=1:length(figHandles)
            savefig(figHandles(n), [figs_path{j} figHandles(n).Name '.fig'], 'compact');
            saveas(figHandles(n),[figs_path{j} figHandles(n).Name '.png']);
        end
        for n=1:length(figHandles)
            close(figHandles(n));
        end
        if (success)
            fprintf('%s SUCCESS\n', scenario_names{j});
        else
            fprintf('%s FAIL\n', scenario_names{j});
        end
    catch e
        fprintf(logs_path{j}, '%s\n%s\n', scenario_names{j}, getReport(e));
        fprintf('%s\n%s\n', scenario_names{j}, getReport(e));
    end
end

for k = 1:length(ls)
    fclose(ls{k});
end

