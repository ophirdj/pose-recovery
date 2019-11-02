addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
%         'C:\Users\Internet\matlab_workspace\trajectories\Path_1_30_20\',...
%         'C:\Users\Internet\matlab_workspace\trajectories\Path_1_100_20\',...
%         'C:\Users\Internet\matlab_workspace\trajectories\Path_2_100_20\',...
%         'C:\Users\Internet\matlab_workspace\trajectories\Circle_100_20\',...
        'C:\Users\Internet\matlab_workspace\trajectories\Curve_100_20\',...
        };
    
show_only = 2;
sim_len = 150;

scenarios = {};
scenario_names = {};
logs = {};
ls = {};

for k = 1:length(PATHS)
    PATH = PATHS{k};
    F_LOG = fopen([PATH 'log.txt'],'w');
    ls{end+1} = F_LOG;
    out_err = [PATH 'err.bin'];
    out_res = [PATH 'res.bin'];
    in_mnav = [PATH 'mnav.bin'];
    window = 3;
    
    % Ground Truth
    scenario_names{end+1} = 'Ground Truth';
    logs{end+1} = F_LOG;
    scenarios{end+1} = @()...
    KalmanImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [PATH 'res.bin'], [PATH 'err.bin'], window, DTM, sim_len, show_only);
    
%     % IMU
%     for linear_err = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0]
%         for angular_err = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0]
%             scenario_names{end+1} = sprintf('%s %.0d %.0d', 'IMU', linear_err, angular_err);
%             logs{end+1} = F_LOG;
%             dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
%             scenarios{end+1} = @()...
%             ImuLidarNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
%                 [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
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
%                 [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
%     end
%     
%     % LIDAR
%     for lidar_err = [0 1e-4 1e-3 1e-2 1e-1 1e0 5e0]
%         scenario_names{end+1} = sprintf('%s %.0d\n', 'LIDAR', lidar_err);
%         logs{end+1} = F_LOG;
%         dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
%         scenarios{end+1} = @()...
%         ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%                 [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
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
%             [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
%     end
end

% parfor j = 1:length(scenarios)
for j = 1:length(scenarios)
%     try
        fprintf('%s\n', scenario_names{j});
        scenarios{j}();
%         close all;
%     catch
%         fprintf(logs{j}, '%s\n', scenario_names{j});
%     end
end

for k = 1:length(ls)
    fclose(ls{k});
end
