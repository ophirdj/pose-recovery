PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_2_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Circle_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve_100_20\',...
        };

NAVIGATOR = 'kalman';
% NAVIGATOR = 'imu_lidar';
ERR_FILENAME = ['err_' NAVIGATOR '.bin'];
RES_FILENAME = ['res_' NAVIGATOR '.bin'];
MNAV_FILENAME = 'mnav.bin';

    
%IMU
linear_errs = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0];
angular_errs = [0 1e-4 1e-2 1e-1 1e0 2e0 5e0];
le = linear_errs;
ae = angular_errs;
%DTM
dtm_errs = [0 1e-2 1e-1 1e0 2e0 5e0 1e1];
dtm_mean = zeros(length(dtm_errs),2);
dtm_legal = zeros(length(dtm_errs),1);
%LIDAR
lidar_errs = [0 1e-4 1e-3 1e-2 1e-1 1e0 5e0];
lidar_mean = zeros(length(lidar_errs),2);
lidar_legal = zeros(length(lidar_errs),1);

    
for k = 1:length(PATHS)
PATH = PATHS{k};

% Ground Truth
F_ERR = [PATH ERR_FILENAME];
F_RES = [PATH RES_FILENAME];
F_MNAV = [PATH MNAV_FILENAME];
err_plot_nav(F_ERR,F_RES,F_MNAV,DTM,cellsize,1);
h =  findobj('type','figure');
for n=1:length(h)
    fig_name = get(get(get(h(n), 'CurrentAxes'), 'Title'), 'String');
    fig_file = [PATH NAVIGATOR '_' int2str(n) '_' fig_name];
    saveas(h(n), [fig_file '.png']);
    saveas(h(n), [fig_file '.fig']);
end
close all;

% IMU
for linear_err = 1:length(linear_errs)
    for angular_err = 1:length(angular_errs)
        F_ERR = [PATH sprintf('imu_%.0d_%.0d/%s', linear_errs(linear_err), angular_errs(angular_err), ERR_FILENAME)];
        F_RES = [PATH sprintf('imu_%.0d_%.0d/%s', linear_errs(linear_err), angular_errs(angular_err), RES_FILENAME)];
        F_MNAV = [PATH MNAV_FILENAME];
        err_plot_nav(F_ERR,F_RES,F_MNAV,DTM,cellsize,1);
        if (linear_err) == 1 && (angular_err == 1)
            F_ERR = [PATH ERR_FILENAME];
            F_RES = [PATH RES_FILENAME];
        end
        err_plot_nav(F_ERR,F_RES,F_MNAV,DTM,cellsize,1);
        h =  findobj('type','figure');
        for n=1:length(h)
            fig_name = get(get(get(h(n), 'CurrentAxes'), 'Title'), 'String');
            fig_file = [PATH NAVIGATOR '_' int2str(n) '_' fig_name];
            saveas(h(n), [fig_file '.png']);
            saveas(h(n), [fig_file '.fig']);
        end
        close all;
    end
end

% DTM
dtm = zeros(length(dtm_errs),2);
for dtm_err = 1:length(dtm_errs)
    F_ERR = [PATH sprintf('dtm_%.0d/%s', dtm_errs(dtm_err), ERR_FILENAME)];
    F_RES = [PATH sprintf('dtm_%.0d/%s', dtm_errs(dtm_err), RES_FILENAME)];
    F_MNAV = [PATH MNAV_FILENAME];
    err_plot_nav(F_ERR,F_RES,F_MNAV,DTM,cellsize,1);
    h =  findobj('type','figure');
    for n=1:length(h)
        fig_name = get(get(get(h(n), 'CurrentAxes'), 'Title'), 'String');
        fig_file = [PATH NAVIGATOR '_' int2str(n) '_' fig_name];
        saveas(h(n), [fig_file '.png']);
        saveas(h(n), [fig_file '.fig']);
    end
    close all;
end

%LIDAR
lidar = zeros(length(lidar_errs),2);
for lidar_err = 1:length(lidar_errs)
    F_ERR = [PATH sprintf('lidar_%.0d/%s', lidar_errs(lidar_err), ERR_FILENAME)];
    F_RES = [PATH sprintf('lidar_%.0d/%s', lidar_errs(lidar_err), RES_FILENAME)];
    F_MNAV = [PATH MNAV_FILENAME];
    err_plot_nav(F_ERR,F_RES,F_MNAV,DTM,cellsize,1);
    h =  findobj('type','figure');
    for n=1:length(h)
        fig_name = get(get(get(h(n), 'CurrentAxes'), 'Title'), 'String');
        fig_file = [PATH NAVIGATOR '_' int2str(n) '_' fig_name];
        saveas(h(n), [fig_file '.png']);
        saveas(h(n), [fig_file '.fig']);
    end
    close all;
end

end


