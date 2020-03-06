addpath(genpath([pwd() '\..\MRepo\']));

dtm_load();

PATHS = {...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_30_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Path_1_100_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Path_2_100_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Circle_100_20\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Circle_10_5\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\Curve_100_20\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\Line_100_20\',...
        };
    
ERR_FILENAME = 'err_unscented.bin';
RES_FILENAME = 'res_unscented.bin';
PRV_FILENAME = 'prv_unscented.bin';

show_only = 0;
sim_len = 600;

scenarios = {};
scenario_names = {};
logs = {};
ls = {};

window = 0;

for k = 1:length(PATHS)
    PATH = PATHS{k};
    F_LOG = fopen([PATH 'log.txt'],'w');
    ls{end+1} = F_LOG;
    
    % DTM
    for n=1:500
        scenario_names{end+1} = sprintf('%s %.0d', 'monte_carlo', n);
        logs{end+1} = F_LOG;
        dir = [PATH sprintf('monte_carlo_%d/', n)];
        scenarios{end+1} = @()...
        UnscentedKalmanNavigator([PATH 'mnav.bin'], [dir 'eimu.bin'], [PATH 'mlidar.bin'], ...
                [PATH 'meta.bin'], [dir RES_FILENAME], [dir ERR_FILENAME], [dir PRV_FILENAME], ...,
                window, DTM, sim_len, show_only);
    end

    
end
%%
parfor j = 1:length(scenarios)
% for j = 1:length(scenarios)
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

