dir = 'C:\Users\Ophir\matlab_workspace\trajectories\';
dtm_load();
% %% Path 1
% freq_Hz = 10;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [4500 4500 1000]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/6 0 0 0 10;
%             6 0 0 0 0 1;
%             3 0 0 0 100 20;
%            ];
% PATH = [dir '1\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% %% Path 2
% freq_Hz = 100;
% n_rays = 20;
% span_angle = pi/3;
% ini_pos = [6000 7000 1500]';
% mot_def = [2 0 0 0 10 1;
%             2 0 0 0 1 1;
%             6 pi/6 0 0 0 1;
%             6 0 0 0 0 1;
%             3 0 0 0 10 2;
%            ];
% PATH = [dir '2\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% %% Path 3
% freq_Hz = 100;
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
% PATH = [dir '3\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% %% Path 4
% freq_Hz = 10;
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
% PATH = [dir '4\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% PathPlot;
% %% Circle1
% freq_Hz = 1;
% n_rays = 5;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle1\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle10
% freq_Hz = 10;
% n_rays = 5;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle10\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle30
% freq_Hz = 30;
% n_rays = 5;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle30\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle 100
% freq_Hz = 100;
% n_rays = 5;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle100\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle1_20
% freq_Hz = 1;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle1_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle10_20
% freq_Hz = 10;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle10_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle30_20
% freq_Hz = 30;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle30_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% % PathPlot;
% %% Circle 100_20
% freq_Hz = 100;
% n_rays = 20;
% span_angle = pi/4;
% ini_pos = [4500 4500 1500]';
% mot_def = [2 0 0 0 100 1;
%             6 pi/8 0 0 0 50;
%             6 0 0 0 0 1;
%            ];
% PATH = [dir 'circle100_20\'];
% if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
%     fprintf('Error generating path\n');
% end
% PathPlot;
%% Curve10_20
freq_Hz = 10;
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
PATH = [dir 'Curve10_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end
%% Curve30_20
freq_Hz = 30;
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
PATH = [dir 'Curve30_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end
%% Curve100_20
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
PATH = [dir 'Curve100_20\'];
if ~PathGenAll(PATH, mot_def, ini_pos, n_rays, span_angle, freq_Hz, DTM, cellsize);
    fprintf('Error generating path\n');
end
% PathPlot;