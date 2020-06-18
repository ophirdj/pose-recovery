addpath(genpath([pwd() '\..\MRepo\']));
dir = 'C:\Users\Ophir\matlab_workspace\trajectories\';
dtm_load();

% Limit path generation
sim_time_secs = 180;

% Limit area of path generation
margin = 3500;
x_start = margin;
x_end = size(DTM, 2) * cellsize - margin;
y_start = margin;
y_end = size(DTM, 1) * cellsize - margin;
z_start = 2500;
z_end = 3500;

%Velocity variance
V_horiz = 5;
V_vert = 1;

%% constant_velocity
fail_cnt = 0;
for n=1:10
    while 1
        name = sprintf('constant_velocity_%d', n);
        ini_pos = [x_start + randi(x_end-x_start); ...
                   y_start + randi(y_end-y_start); ...
                   z_start + randi(z_end-z_start)];
        vel = [V_horiz * rand(); ...
                   V_horiz * rand(); ...
                   V_vert * rand()];
        if constant_velocity(ini_pos, vel, sim_time_secs, name, dir, DTM, cellsize)
            fprintf('%s [Done]\n', name);
            fail_cnt = 0;
            break;
        else
            fail_cnt = fail_cnt + 1;
            fprintf('%s [FAILED] %d, retrying with different parameters...\n', name, fail_cnt);
        end
    end
end

%% constant_bank
for n=1:10
    while 1
        name = sprintf('constant_bank_%d', n);
        ini_pos = [x_start + randi(x_end-x_start); ...
                   y_start + randi(y_end-y_start); ...
                   z_start + randi(z_end-z_start)];
        ini_vel = [V_horiz * rand(); ...
                   V_horiz * rand(); ...
                   V_vert * rand()];
        bank = (pi/256 + pi/512 * rand()) * power(-1, randi(2));
        if constant_bank(ini_pos, ini_vel, bank, sim_time_secs, name, dir, DTM, cellsize)
            fprintf('%s [Done]\n', name);
            fail_cnt = 0;
            break;
        else
            fail_cnt = fail_cnt + 1;
            fprintf('%s [FAILED] %d, retrying with different parameters...\n', name, fail_cnt);
        end
    end
end

%% Supporting functions
function [success] = constant_velocity(ini_pos, vel, sim_time_secs, name, dir, DTM, cellsize)
    freq_Hz = 100;
    ray_angles = [-pi/6:pi/180:pi/6 pi/6-pi/180:-pi/180:-pi/6+pi/180];
    mot_def = repmat([3 0 0 0 0 1], [sim_time_secs*freq_Hz 1]);
    PATH = sprintf('%s%s\\', dir, name);
    success = PathGenAll(PATH, mot_def, ini_pos, ray_angles, freq_Hz, DTM, cellsize, vel);
end

function [success] = constant_bank(ini_pos, ini_vel, bank_rads, sim_time_secs, name, dir, DTM, cellsize)
    freq_Hz = 100;
    ray_angles = [-pi/6:pi/180:pi/6 pi/6-pi/180:-pi/180:-pi/6+pi/180];
    mot_def = [6 bank_rads 0 0 0 sim_time_secs];
    PATH = sprintf('%s%s\\', dir, name);
    success = PathGenAll(PATH, mot_def, ini_pos, ray_angles, freq_Hz, DTM, cellsize, ini_vel);
end