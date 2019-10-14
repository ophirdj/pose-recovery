dir = 'C:\Users\Ophir\matlab_workspace\trajectories\';
dtm_load();

%% Path_1_30_20
freq_Hz = 30;
n_rays = 20;
span_angle = pi/4;
ini_pos = [4500 4500 1000]';
mot_def = [2 0 0 0 100 1;
            6 pi/6 0 0 0 10;
            6 0 0 0 0 1;
            3 0 0 0 100 20;
           ];
PATH = [dir 'Path_1_30_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end

%% Path_1_100_20
freq_Hz = 100;
n_rays = 20;
span_angle = pi/4;
ini_pos = [4500 4500 1000]';
mot_def = [2 0 0 0 100 1;
            6 pi/6 0 0 0 10;
            6 0 0 0 0 1;
            3 0 0 0 100 20;
           ];
PATH = [dir 'Path_1_100_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end

%% Path_2_300_20
% freq_Hz = 30;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [3500 1500 1000]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/6 0 0 0 10;
%             6 0 0 0 0 1;
%             3 0 0 0 100 4;
%             6 -pi/6 0 0 0 8;
%             6 0 0 0 0 1;
%             6 -pi/6 0 0 0 8;
%             6 0 0 0 0 1;
%             6 pi/8 0 0 0 5;
%             6 0 0 0 0 1;
%             3 0 0 0 100 10;
%            ];
% PATH = [dir 'Path_2_30_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end

%% Path_2_100_20
freq_Hz = 100;
n_rays = 20;
span_angle = pi/4;
ini_pos = [3500 1500 1000]';
mot_def = [2 0 0 0 100 1;
            6 pi/6 0 0 0 10;
            6 0 0 0 0 1;
            3 0 0 0 100 4;
            6 -pi/6 0 0 0 8;
            6 0 0 0 0 1;
            6 -pi/6 0 0 0 8;
            6 0 0 0 0 1;
            6 pi/8 0 0 0 5;
            6 0 0 0 0 1;
            3 0 0 0 100 10;
           ];
PATH = [dir 'Path_2_100_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end

%% Circle_30_20
% freq_Hz = 30;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'Circle_30_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end

%% Circle_100_20
freq_Hz = 100;
n_rays = 20;
span_angle = pi/4;
ini_pos = [4500 4500 1500]';
mot_def = [2 0 0 0 100 1;
            6 pi/8 0 0 0 50;
            6 0 0 0 0 1;
           ];
PATH = [dir 'Circle_100_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end

%% Curve_30_20
% freq_Hz = 30;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [3500 1500 1000]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/6 0 0 0 10;
%             6 0 0 0 0 1;
%             3 0 0 0 100 4;
%             6 -pi/6 0 0 0 8;
%             6 0 0 0 0 1;
%             6 -pi/6 0 0 0 8;
%             6 0 0 0 0 1;
%             6 pi/8 0 0 0 5;
%             6 0 0 0 0 1;
%             2 0 0 0 100 10;
%             1 0 0 0 10 10;
%             6 pi/6 0 0 0 10;
%             6 0 0 0 0 1;
%             3 0 0 0 100 4;
%            ];
% PATH = [dir 'Curve_30_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end

%% Curve_100_20
freq_Hz = 100;
n_rays = 20;
span_angle = pi/4;
ini_pos = [3500 1500 1000]';
mot_def = [2 0 0 0 100 1;
            6 pi/6 0 0 0 10;
            6 0 0 0 0 1;
            3 0 0 0 100 4;
            6 -pi/6 0 0 0 8;
            6 0 0 0 0 1;
            6 -pi/6 0 0 0 8;
            6 0 0 0 0 1;
            6 pi/8 0 0 0 5;
            6 0 0 0 0 1;
            2 0 0 0 100 10;
            1 0 0 0 10 10;
            6 pi/6 0 0 0 10;
            6 0 0 0 0 1;
            3 0 0 0 100 4;
           ];
PATH = [dir 'Curve_100_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end
