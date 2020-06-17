addpath(genpath([pwd() '\..\MRepo\']));
dir = 'C:\Users\Ophir\matlab_workspace\trajectories\';
dtm_load();

%% constant_velocity
for n=1:10
    while 1
        name = sprintf('constant_velocity_%d', n);
        sim_time_secs = 120;
        ini_pos = [2000 + randi(5000); ...
                   2000 + randi(5000); ...
                   1500 + randi(1000)];
        ini_vel = [5 * rand(); ...
                   5 * rand(); ...
                   1 * rand()];
        if generate_line(ini_pos, ini_vel, sim_time_secs, name, dir, DTM, cellsize)
            fprintf('%s [Done]\n', name);
            break;
        end
    end
end
% %% Circle_100_1
% freq_Hz = 100;
% ray_angles = [-pi/6:pi/180:pi/6 pi/6-pi/180:-pi/180:-pi/6+pi/180];
% ini_pos = [4500 4500 1500]';
% ini_vel = [10 0 0]';
% mot_def = [6 pi/16 0 0 0 5000];
% PATH = [dir 'Circle_100_1\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, ray_angles, freq_Hz, DTM, cellsize, ini_vel)
%     fprintf('Error generating path\n');
% else
%     fprintf('Done %s\n', 'Line_100_1\');
% end


%% Supporting functions
function [success] = generate_line(ini_pos, ini_vel, sim_time_secs, name, dir, DTM, cellsize)
    freq_Hz = 100;
    ray_angles = [-pi/6:pi/180:pi/6 pi/6-pi/180:-pi/180:-pi/6+pi/180];
    mot_def = repmat([3 0 0 0 0 1], [sim_time_secs*freq_Hz 1]);
    PATH = sprintf('%s%s\\', dir, name);
    success = PathGenAll(PATH, mot_def, ini_pos, ray_angles, freq_Hz, DTM, cellsize, ini_vel);
end