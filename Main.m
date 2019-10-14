addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_2_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Circle_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve_100_20\',...
        };
    
show_only = 0;
sim_len = 1000;

gcp;

for k = 1:length(PATHS)
    PATH = PATHS{k};
    
    F_LOG = fopen([PATH 'log.txt'],'w');

    out_err = [PATH 'err.bin'];
    out_res = [PATH 'res.bin'];
    in_mnav = [PATH 'mnav.bin'];

    window = 3;
    
%     for dtm_err = [1e-1]
%         try
%         dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
%         ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%                 [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
% %         close all;
%         catch
%             fprintf(F_LOG, '%s %.0d\n', 'DTM', dtm_err);
%         end
%     end

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
        linear_errs = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0];
        angular_errs = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0];
        [lin, ang] = meshgrid(linear_errs, angular_errs);
        lin = lin(:);
        ang = ang(:);
        parfor j = 1:length(lin)
            try
            linear_err = lin(j);
            angular_err = ang(j);
            dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
            ImuLidarNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %.0d %.0d\n', 'IMU', linear_err, angular_err);
            end
        end

        % DTM
        dtm_errs = [0 1e-2 1e-1 1e0 2e0 5e0 1e1];
        for dtm_err = 1:length(dtm_errs)
            try
            dir = [PATH sprintf('dtm_%.0d/', dtm_errs(dtm_err))];
            ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
                    [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %.0d\n', 'DTM', dtm_errs(dtm_err));
            end
        end

        % LIDAR
        lidar_errs = [0 1e-4 1e-3 1e-2 1e-1 1e0 5e0];
        parfor lidar_err = 1:length(lidar_errs)
            try
            dir = [PATH sprintf('lidar_%.0d/', lidar_errs(lidar_err))];
            ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
                    [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %.0d\n', 'LIDAR', lidar_errs(lidar_err));
            end
        end

        %Batch Size
        windows = 10:-2:2;
        parfor w = 1:length(windows)
            try
            dir_imu = [PATH sprintf('imu_%.0d_%.0d/', 1e-1, 1e-1)];
            dir_lidar = [PATH sprintf('lidar_%.0d/', 1e-2)];
            dir_dtm = [PATH sprintf('dtm_%.0d/', 1e-1)];
            dir = [PATH sprintf('window_%d/', windows(w))];
            if ~isdir(dir)
                mkdir(dir);
            end
            ImuLidarNavigator([PATH 'mnav.bin'], [dir_imu 'eimu.bin'], [dir_dtm 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM, sim_len, show_only);
            close all;
            catch
                fprintf(F_LOG, '%s %d\n', 'Batch', windows(w));
            end
        end
    end
    fclose(F_LOG);
end
