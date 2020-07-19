addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();
scenario_test_parameters();

PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_1\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_2\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_3\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_bank_1\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_bank_2\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\constant_bank_3\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_4\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_5\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_6\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_7\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_8\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_9\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\constant_velocity_10\',...
        };
    
ERR_FILENAME = 'err_unscented.bin';
RES_FILENAME = 'res_unscented.bin';
PRV_FILENAME = 'prv_unscented.bin';

show_only = 2;
sim_len = 0;

scenarios = {};
scenario_names = {};
logs_path = {};
ls = {};
figs_path = {};

P = ...
diag([10 10 10                                          ... % pos
      1 1 1                                             ... % vel
      0.5*pi/180 0.5*pi/180 0.5*pi/180                  ... % att
      1e-1 1e-1 1e-1                                    ... % bias-acc
      1*pi/180/3600 1*pi/180/3600 1*pi/180/3600         ... % bias-gyr
      ].^2);
  
Q = ...
diag([5e-1 5e-1 5e-1                                         ... % pos
      1e-1 1e-1 1e-1                                         ... % vel (acc variance)
      0.1*pi/180/60 0.1*pi/180/60 0.1*pi/180/60              ... % att
      1e-5 1e-5 1e-5                                         ... % acc-bias
      1e-3/60*pi/180 1e-3/60*pi/180 1e-3/60*pi/180           ... % gyr-bias
      ].^2);

alpha = 1.5e-1;

MEASUREMENT_NOISE = 1e1;

%% Prepare Scenarios
for k = 1:length(PATHS)
    PATH = PATHS{k};
    F_LOG = fopen([PATH 'log_general.txt'],'w');
    ls{end+1} = F_LOG;
    
    % Ground Truth
    scenario_names{end+1} = sprintf('%s%s', PATH, 'Ground Truth');
    figs_path{end+1} = [PATH 'figs/'];
    logs_path{end+1} = [PATH 'log.txt'];
%     MEASUREMENT_NOISE = 1e-8;
    
    scenarios{end+1} = @()...
    UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [PATH RES_FILENAME], [PATH ERR_FILENAME], [PATH PRV_FILENAME], ...
        DTM, Q, MEASUREMENT_NOISE, alpha, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec, ...
        ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
    
    % IMU
    for linear_err = accelerometer_variances_dt
        for angular_err = gyro_variances_dt
            scenario_names{end+1} = sprintf('%s%s %.0d %.0d', PATH, 'IMU', linear_err, angular_err);
            dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
            logs_path{end+1} = [dir 'log.txt'];
            figs_path{end+1} = [dir 'figs/'];
%             MEASUREMENT_NOISE = 1e-8;
              
              scenarios{end+1} = @()...
                UnscentedKalmanNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
                DTM, Q, MEASUREMENT_NOISE, alpha, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec, ...
                ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
        end
    end
    
    % DTM
    for dtm_err = dtm_errs_m
        scenario_names{end+1} = sprintf('%s%s %.0d', PATH, 'DTM', dtm_err);
        dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
        logs_path{end+1} = [dir 'log.txt'];
        figs_path{end+1} = [dir 'figs/'];
%         MEASUREMENT_NOISE = dtm_err + 1e-8;

        scenarios{end+1} = @()...
        UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
            [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
            DTM, Q, MEASUREMENT_NOISE, alpha, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec, ...
            ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
    end
    
    % LIDAR
    for lidar_err = lidar_errs
        scenario_names{end+1} = sprintf('%s%s %.0d', PATH, 'LIDAR', lidar_err);
        dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
        logs_path{end+1} = [dir 'log.txt'];
        figs_path{end+1} = [dir 'figs/'];
%         MEASUREMENT_NOISE = lidar_err + 1e-8;
        
        scenarios{end+1} = @()...
        UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
            [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...
            DTM, Q, MEASUREMENT_NOISE, alpha, P, accelerometer_bias_m_per_sec2, gyro_drift_rad_per_sec, ...
            ini_pos_err_m, ini_vel_err_m_sec, ini_att_err_rad, sim_len, show_only);
    end
    
end

successes = zeros(1,length(scenarios));

%% Execute Scenarios
parfor j = 1:length(scenarios)
% for j = 1:length(scenarios)
    try
        fprintf('%[BEGIN]\n', scenario_names{j});
        [success, steps, figHandles] = scenarios{j}();
        successes(j) = success;
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
            fprintf('[SUCCESS] %s\n', scenario_names{j});
        else
            fprintf('[FAILED]  %s\n', scenario_names{j});
        end
        fclose(logs_path{j});
    catch e
        try
            fprintf(logs_path{j}, '%s\n%s\n', scenario_names{j}, getReport(e));
        catch
        end
        fprintf('[CRASHED] %s\n%s\n', scenario_names{j}, getReport(e));
    end
end

%% Summary
fprintf('Summary\n');
for j = 1:length(scenarios)
    if successes(j)
        fprintf('[SUCCESS] %s\n', scenario_names{j});
    else
        fprintf('[FAILED]  %s\n', scenario_names{j});
    end
end

for k = 1:length(ls)
    fclose(ls{k});
end
fclose('all');
zip(['C:\Users\Ophir\matlab_workspace\results-' date '.zip'], ...
    'C:\Users\Ophir\matlab_workspace\trajectories\');
