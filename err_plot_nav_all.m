PATHS = {...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_30_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Path_2_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Circle_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Curve_100_20\',...
        };

NAVIGATOR = 'unscented';
ERR_FILENAME = ['err_' NAVIGATOR '.bin'];
RES_FILENAME = ['res_' NAVIGATOR '.bin'];
MNAV_FILENAME = 'mnav.bin';

scenario_test_parameters();
    
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
for linear_err = 1:length(accelerometer_variances_m_sec2)
    for angular_err = 1:length(gyro_variances_deg_sec2)
        F_ERR = [PATH sprintf('imu_%.0d_%.0d/%s', accelerometer_variances_m_sec2(linear_err), gyro_variances_deg_sec2(angular_err), ERR_FILENAME)];
        F_RES = [PATH sprintf('imu_%.0d_%.0d/%s', accelerometer_variances_m_sec2(linear_err), gyro_variances_deg_sec2(angular_err), RES_FILENAME)];
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
for dtm_err = 1:length(dtm_errs_m)
    F_ERR = [PATH sprintf('dtm_%.0d/%s', dtm_errs_m(dtm_err), ERR_FILENAME)];
    F_RES = [PATH sprintf('dtm_%.0d/%s', dtm_errs_m(dtm_err), RES_FILENAME)];
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
lidar = zeros(length(lidar_errs_percent),2);
for lidar_err = 1:length(lidar_errs_percent)
    F_ERR = [PATH sprintf('lidar_%.0d/%s', lidar_errs_percent(lidar_err), ERR_FILENAME)];
    F_RES = [PATH sprintf('lidar_%.0d/%s', lidar_errs_percent(lidar_err), RES_FILENAME)];
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


