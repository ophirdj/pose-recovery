ERR_FILENAME = 'err_unscented.bin';
RES_FILENAME = 'res_unscented.bin';
PRV_FILENAME = 'prv_unscented.bin';

show_only = 0;
sim_len = 1000;

PATH = 'C:\Users\Ophir\matlab_workspace\trajectories\Line_100_1\';

ka_max = 0;
c_max = 0;
for ka = 0.01:1e-3:1
    [s, c] = UnscentedKalmanNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
            [PATH 'meta.bin'], [PATH RES_FILENAME], [PATH ERR_FILENAME], [PATH PRV_FILENAME], ...
            window, DTM, sim_len, show_only, ka);
    if s
        k_max = ka;
        c_max = c;
        break;
    end
    
    if c > c_max
        
        k_max = ka;
        c_max = c;
    end
    
    fprintf('%d  %d\n', k_max, c_max);
end


    fprintf('END\n');
    fprintf('%d  %d\n', k_max, c_max);