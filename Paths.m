addpath(genpath([pwd() '\..\MRepo\']));
dir = 'C:\Users\Ophir\matlab_workspace\trajectories\';
dtm_load();

% Limit path generation
sim_time_secs = 360;
NUM_PATHS_PER_CLASS = 3;

% Limit area of path generation
margin = 3500;
x_start = margin;
x_end = size(DTM, 2) * cellsize - margin;
y_start = margin;
y_end = size(DTM, 1) * cellsize - margin;
z_start = 2500;
z_end = 3500;

%Velocity variance
V_horiz_m_sec = 5;
V_vert_m_sec = 1;

%% constant_velocity
fail_cnt = 0;
for n=1:NUM_PATHS_PER_CLASS
    while 1
        name = sprintf('constant_velocity_%d', n);
        ini_pos = random_pos([x_start x_end], [y_start y_end], [z_start z_end]);
        vel = random_vel(V_horiz_m_sec, V_vert_m_sec);
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
fail_cnt = 0;
for n=1:NUM_PATHS_PER_CLASS
    while 1
        name = sprintf('constant_bank_%d', n);
        ini_pos = random_pos([x_start x_end], [y_start y_end], [z_start z_end]);
        ini_vel = random_vel(V_horiz_m_sec, V_vert_m_sec);
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
function [pos] = random_pos(x, y, z)
        pos = [x(1) + randi(x(2)-x(1)); ...
               y(1) + randi(y(2)-y(1)); ...
               z(1) + randi(z(2)-z(1))];
end

function [vel] = random_vel(h, v)
        vel = [h * rand(); ...
               h * rand(); ...
               v * rand()];
end

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