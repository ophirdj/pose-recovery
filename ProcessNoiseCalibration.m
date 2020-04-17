ERR_FILENAME = 'err_unscented.bin';
RES_FILENAME = 'res_unscented.bin';
PRV_FILENAME = 'prv_unscented.bin';

show_only = 0;
sim_len = 2000;

PATH = 'C:\Users\Ophir\matlab_workspace\trajectories\Line_100_1\';

pn_max = 1e-2;
c_max = 0;
for pn = 1e-2:1e-2:1
    % IMU
    for linear_err = [1e-4]
        for angular_err = [0]
            dir = [PATH sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
            [s, c] = UnscentedKalmanNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...,
                DTM, pn, 1e-6, 0.77, sim_len, show_only);
        end
    end
    if s
        pn_max = pn;
        c_max = c;
        break;
    end
    
    if c > c_max
        
        pn_max = pn;
        c_max = c;
    end
    
    fprintf('%d  %d\n', pn_max, c_max);
end


    fprintf('END\n');
    fprintf('%d  %d\n', pn_max, c_max);