addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve10_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve100_20\',...
        };
    

for k = 1:length(PATHS)
PATH = PATHS{k};

out_err = [PATH 'err.bin'];
out_res = [PATH 'res.bin'];
in_mnav = [PATH 'mnav.bin'];

window = 7;

% Ground Truth
ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
    [PATH 'meta.bin'], [PATH 'res.bin'], [PATH 'err.bin'], window, DTM,0);
close all;

% % IMU
% for linear_err = [0 1e-4 1e-3 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0]
%     for angular_err = [0 1e-4 1e-3 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0]
%         dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
%         ImuLidarNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
%             [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM);
%         close all;
%     end
% end
% 
% % DTM
% for dtm_err = [0 1e-2 1e-1 1e0 2e0 5e0 1e1 2e1 5e1]
%     dir = [PATH sprintf('dtm_%.0d/', dtm_err)];
%     ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%             [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM);
%     close all;
% end
% 
% % LIDAR
% for lidar_err = [0 1e-2 2e-2 5e-2 1e-1 2e-1 5e-1 1e0 2e0 5e0 1e1 2e1 5e1]
%     dir = [PATH sprintf('lidar_%.0d/', lidar_err)];
%     ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [dir 'mlidar.bin'], ...
%             [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM);
%     close all;
% end

%Batch Size
for w = 20:-2:2
    dir = [PATH sprintf('window_%d/', w)];
    if ~isdir(dir)
        mkdir(dir);
    end
    ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
        [PATH 'meta.bin'], [dir 'res.bin'], [dir 'err.bin'], window, DTM);
    close all;
end
end

% tru=readbin_v000([PATH 'mnav.bin'],10);
% 
% range=101:110;
% pos_window = tru(2:4, range);
% delta_pos_window = pos_window(:,2:end) - pos_window(:,1:end-1);
% 
% att_window = tru(8:10, range);
% delta_att_window = att_window(:,2:end) - att_window(:,1:end-1);
% Cbn_window = zeros(3,3,length(range));
% for n = 1:length(range)
%     Cbn_window(:,:,n) = euler2dcm_v000(att_window(:,n));
% end
% 
% F_META = fopen([PATH 'meta.bin'], 'rb');
% freq_Hz = fread(F_META, 1, 'double');
% dt = 1/freq_Hz;
% n_rays = fread(F_META, 1, 'double');
% span_angle = fread(F_META, 1, 'double');
% cellsize = fread(F_META, 1, 'double');
% fclose(F_META);
% 
% lidar_data = readbin_v000([PATH 'mlidar.bin'], 1+n_rays);
% lidar_window = lidar_data(2:end, range);
% 
% n_rays = (n_rays-1) / 2;
% if n_rays == 0
%     alpha = 0;
% else
%     alpha = span_angle / n_rays;
% end
% rays = GenerateRays(alpha, n_rays);
% 
% D = DTM + 1 .* randn(size(DTM));
% 
% err = visualize_error( ...
%     pos_window, Cbn_window, delta_pos_window, delta_att_window, ...
%     rays, lidar_window, D, cellsize, 10);