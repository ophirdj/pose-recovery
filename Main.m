addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle30\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle100\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve10_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve100_20\',...
        };
    
show_only = 0;
sim_len = 150;

for k = 1:length(PATHS)
    PATH = PATHS{k};
    
    F_LOG = fopen([PATH 'log.txt'],'w');

    out_err = [PATH 'err.bin'];
    out_res = [PATH 'res.bin'];
    in_mnav = [PATH 'mnav.bin'];

    window = 3;

    try
    % Ground Truth
    ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [PATH 'res.bin'], [PATH 'err.bin'], window, DTM, sim_len, show_only);
    catch
        fprintf(F_LOG, '%s\n', 'Ground Truth');
    end

    if ~show_only
        close all;
        
        % IMU
        for linear_err = [0 1e-4 1e-3 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0]
            for angular_err = [0 1e-4 1e-3 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0]
                try
                dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
                ImuLidarNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
                    [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
                close all;
                catch
                    fprintf(F_LOG, '%s %.0d %.0d\n', 'IMU', linear_err, angular_err);
                end
            end
        end

        % DTM
        for dtm_err = [0 1e-2 1e-1 1e0 2e0 5e0 1e1 2e1 5e1]
            try
            dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
            ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
                    [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %.0d\n', 'DTM', dtm_err);
            end
        end

        % LIDAR
        for lidar_err = [0 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1]
            try
            dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
            ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
                    [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %.0d\n', 'LIDAR', lidar_err);
            end
        end

        %Batch Size
        for w = 20:-2:2
            try
            dir_imu = [PATH sprintf('imu_%.0d_%.0d/', 1e-1, 1e-1)];
            dir_lidar = [PATH sprintf('lidar_%.0d/', 1e-2)];
            dir_dtm = [PATH sprintf('dtm_%.0d/', 1e-1)];
            dir = [PATH sprintf('window_%d/', w)];
            if ~isdir(dir)
                mkdir(dir);
            end
            ImuLidarNavigator([PATH 'mnav.bin'], [dir_imu 'eimu.bin'], [dir_dtm 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %d\n', 'Batch', w);
            end
        end
    end
    fclose(F_LOG);
end
