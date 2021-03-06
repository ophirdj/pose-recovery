addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_30_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_100_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Path_2_100_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Circle_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Circle_10_5\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Curve_100_20\',...
        };
    
ERR_FILENAME = 'err_imu_lidar.bin';
RES_FILENAME = 'res_imu_lidar.bin';
PRV_FILENAME = 'prv_imu_lidar.bin';
    
show_only = 0;
sim_len = 600;

scenarios = {};
scenario_names = {};
logs = {};
ls = {};

for k = 1:length(PATHS)
    PATH = PATHS{k};
    F_LOG = fopen([PATH 'log.txt'],'w');
    ls{end+1} = F_LOG;
    window = 3;
    
    % Ground Truth
    scenario_names{end+1} = 'Ground Truth';
    logs{end+1} = F_LOG;
    scenarios{end+1} = @()...
    ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [PATH RES_FILENAME], [PATH ERR_FILENAME], [PATH PRV_FILENAME], ...
        window, DTM, sim_len, show_only);
    
%     % IMU
%     for linear_err = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0]
%         for angular_err = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0]
%             scenario_names{end+1} = sprintf('%s %.0d %.0d', 'IMU', linear_err, angular_err);
%             logs{end+1} = F_LOG;
%             dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
%             scenarios{end+1} = @()...
%             ImuLidarNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
%                 [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...,
%                 window, DTM, sim_len, show_only);
%         end
%     end
%     
%     % DTM
%     for dtm_err = [0 1e-2 1e-1 1e0 2e0 5e0 1e1]
%         scenario_names{end+1} = sprintf('%s %.0d', 'DTM', dtm_err);
%         logs{end+1} = F_LOG;
%         dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
%         scenarios{end+1} = @()...
%         ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%                 [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...,
%                 window, DTM, sim_len, show_only);
%     end
%     
%     % LIDAR
%     for lidar_err = [0 1e-4 1e-3 1e-2 1e-1 1e0 5e0]
%         scenario_names{end+1} = sprintf('%s %.0d\n', 'LIDAR', lidar_err);
%         logs{end+1} = F_LOG;
%         dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
%         scenarios{end+1} = @()...
%         ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%                 [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...,
%                 window, DTM, sim_len, show_only);
%     end

%     
%     %Batch Size
%     for w = 10:-2:2
%         scenario_names{end+1} = sprintf('%s %d\n', 'Batch', w);
%         logs{end+1} = F_LOG;
%         
%         dir_imu = [PATH sprintf('imu_%.0d_%.0d/', 1e-1, 1e-1)];
%         dir_lidar = [PATH sprintf('lidar_%.0d/', 1e-2)];
%         dir_dtm = [PATH sprintf('dtm_%.0d/', 1e-1)];
%         dir = [PATH sprintf('window_%d/', w)];
%         if ~isdir(dir)
%             mkdir(dir);
%         end
%         scenarios{end+1} = @()...
%         ImuLidarNavigator([PATH 'mnav.bin'], [dir_imu 'eimu.bin'], [dir_dtm 'mlidar.bin'], ...
%             [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...,
%                 window, DTM, sim_len, show_only);
%     end
end

for j = 1:length(scenarios)
    try
        fprintf('%s\n', scenario_names{j});
        if scenarios{j}()
            fprintf('%s SUCCESS\n', scenario_names{j});
        else
            fprintf('%s FAIL\n', scenario_names{j});
        end
        close all;
    catch e
        fprintf(logs{j}, '%s\n%s\n', scenario_names{j}, getReport(e));
        fprintf('%s\n%s\n', scenario_names{j}, getReport(e));
    end
end

for k = 1:length(ls)
    fclose(ls{k});
end
