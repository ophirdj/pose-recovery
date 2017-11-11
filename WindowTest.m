%% Section - Generate path
% The 'real' pose

scenario_name = 'WindowTestHigh';

% Low Flight
% via_points = [114 171 350; ...
%               145 166 420; ...
%               184 169 400; ...
%               214 207 420; ...
%               237 226 430; ...
%               285 250 500; ...
%               300 276 500; ...
%               266 293 500; ...
%               257 297 500; ...
%               200 298 400; ...
%               168 287 350; ...
%               ];
% 
% rots = eul2rotm([0 pi 0;           ...
%                  0 pi pi;          ...
%                  0 pi pi/2;        ...
%                  pi/6 pi 0;        ...
%                  0 pi 0;           ...
%                  pi/6 pi pi/3;     ...
%                  0 pi pi/3;        ...
%                  pi/6 pi pi;       ...
%                  0 pi 0;           ...
%                  0 pi pi;          ...
%                  0 pi 0;           ...
%                  ], 'ZYZ');

via_points = [114 171 750; ...
              237 226 730; ...
              ];

rots = eul2rotm([0 pi pi/2;           ...
                 0 pi pi/2;           ...
                 ], 'ZYZ');

% Generate and concatenate path parts
Ps = [];
Rs = [];
for n = (1:size(via_points, 1) - 1)
    [temp_Ps, temp_Rs] = linpath(via_points(n, :), via_points(n+1, :), ...
                                 rots(:, :, n), rots(:, :, n+1), 50);
    Ps = cat(2, Ps, temp_Ps);
    Rs = cat(3, Rs, temp_Rs);
end
n_samples = size(Ps, 2);

% Errors
ed = 1e0;
eDTM = 5e0;
ep = 5e0;
er = 1e-1;
ev = 5e0;

% Noisy Path poses
cP0s = zeros(size(Ps, 1), n_samples);
cR0s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
for n = (1:n_samples)
    [cP0s(:, n), cR0s(:, :, n)] = NoisyPose(Ps(:, n), Rs(:, :, n), ep, er);
end

% Noisy "velocity" vectors (displacement from one pose to the next)
cv0s = zeros(size(Ps, 1), n_samples - 1);
for n = (1:n_samples - 1)
    cv0s(:, n) = Ps(:, n) - Ps(:, n + 1) + ev * (2 * rand(3, 1) - 1);
end

noisyDTM = DTM + eDTM * (2 * rand(size(DTM)) - 1);

% Distance readings
n_rays = 20;
ray_angle_openning = pi / 16;
rays = GenerateRays(ray_angle_openning / n_rays, n_rays);

cdistances = zeros(size(rays, 2), n_samples);
for n = (1:n_samples)
    cdistances(:, n) = CalcRayDistances(Ps(:, n), Rs(:, :, n), ...
                                       rays, noisyDTM, cellsize) + ...
                        ed * (2 * rand(1, length(rays)) - 1);
end

%% Section window
mean_err_pos = zeros(1, 19);
mean_err_rot = zeros(1, 19);
windows = (20:-2:2);
for window = windows
%% Section - Preprocess
if window > n_samples
    window = n_samples;
end

% Pad data to fit window size
P0s = [repmat(cP0s(:, 1), 1, window - 1), cP0s];
R0s = cat(3, repmat(cR0s(:, :, 1), 1, 1, window - 1), cR0s);
distances = [repmat(cdistances(:, 1), 1, window - 1), cdistances];
v0s = [zeros(3, window), cv0s];


%% Section - Calculate

n_generated_points = 1;
n_iterations = 10;
lambda = 3.6;

P1s = zeros(size(Ps, 1), n_samples);
R1s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
errors = zeros(n_iterations, n_samples);
Pps = zeros(3, n_iterations, n_samples);
Rrs = zeros(3, 3, n_iterations, n_samples);
window_points = zeros(3, size(P0s, 2) + 1);
window_points(:, 1:window) = P0s(:, 1:window);

% Reconstruct 'real' poses
for n = (1:n_samples)
    [P1s(:, n), R1s(:, :, n), errors(:, n), Pps(:, :, n), Rrs(:, :, :, n)] = ...
        FindPoseMulti(window_points(:, (n:n + window - 1)), R0s(:, :, (n:n + window - 1)), ...
        v0s(:, (n:n + window - 2)), ...
        rays, distances(:, (n:n + window - 1))', DTM, cellsize, ...
        ep, er, n_generated_points, n_iterations, lambda);
    window_points(:, n + window - 1) = P1s(:, n);
    window_points(:, n + window) = P1s(:, n) - v0s(:, n + window - 1);
    
    fprintf('[%d/%d]\n', n, n_samples);
end

%% Section Error analasis
% Create folder to save results
if ~exist([pwd '/Scenario_' scenario_name], 'dir')
    mkdir([pwd '/Scenario_' scenario_name]);
end
out_dir = [pwd '/Scenario_' scenario_name '/Window_' num2str(window)];
mkdir(out_dir);

% Analyze error over time

pos_err = sqrt(sum((P1s - Ps) .^ 2, 1));
rot_err = reshape(sqrt(sum(sum((R1s - Rs) .^ 2, 1), 2)), [1, n_samples]);

figure;
semilogy((1:n_samples), pos_err);
hold on;
semilogy((1:n_samples), rot_err);
title('Error Over Time');
legend('Position Error', 'Rotation Error', 'Location','southeast');
xlabel('Frame number');
ylabel('meters / radians');
hold off;

saveas(gcf,[out_dir '/error.fig']);
saveas(gcf,[out_dir '/error.jpg']);
close(gcf);

% Analyze error distribution

figure;
plot((1:n_samples), sort(pos_err));
hold on;
plot((1:n_samples), sort(rot_err));
title('Error Distribution');
legend('Position Error', 'Rotation Error', 'Location','southeast');
xlabel('Percentage of Frames');
ylabel('meters / radians');
xlim([0 n_samples]);
set(gca, 'XTickLabel',num2str((100/n_samples).*get(gca,'XTick')','%g%%'))
hold off;

saveas(gcf,[out_dir '/error_distribution.fig']);
saveas(gcf,[out_dir '/error_distribution.jpg']);
close(gcf);

% Error histogram
figure;
hist(pos_err);
h = findobj(gca,'Type','patch');
set(h,'FaceColor','b','EdgeColor','w');
title('Position Error Distribution');
legend('Position Error','Location','northeast');
xlabel('Error (meters)');
ylabel('#Frames');

saveas(gcf,[out_dir '/position_error_histogram.fig']);
saveas(gcf,[out_dir '/position_error_histogram.jpg']);
close(gcf);

figure;
hist(rot_err);
h = findobj(gca,'Type','patch');
set(h,'FaceColor','r','EdgeColor','w');
title('Rotation Error Distribution');
legend('Rotation Error','Location','northeast');
xlabel('Error (radians)');
ylabel('#Frames');

saveas(gcf,[out_dir '/rotation_error_histogram.fig']);
saveas(gcf,[out_dir '/rotation_error_histogram.jpg']);
close(gcf);

figure;
semilogy((1:n_samples), sort(pos_err));
hold on;
semilogy((1:n_samples), sort(rot_err));
title('Error Distribution (logscale)');
legend('Position Error', 'Rotation Error', 'Location','southeast');
xlabel('Percentage of Frames');
ylabel('meters / radians');
xlim([0 n_samples]);
set(gca, 'XTickLabel',num2str((100/n_samples).*get(gca,'XTick')','%g%%'))
hold off;

saveas(gcf,[out_dir '/error_distribution_log.fig']);
saveas(gcf,[out_dir '/error_distribution_log.jpg']);
close(gcf);


%% Plot path
vis_DTM = DTM;
% Fill it with whatever that is not so off scale
vis_DTM(vis_DTM==NODATA_value) = 0;

Zs = rotm2zvec(Rs);
Z1s = rotm2zvec(R1s);

figure;
h1 = mesh(vis_DTM);
hold on;
h2 = quiver3(Ps(1, :), Ps(2, :), Ps(3, :), Zs(1, :), Zs(2, :), Zs(3, :));
h3 = quiver3(P1s(1, :), P1s(2, :), P1s(3, :), Z1s(1, :), Z1s(2, :), Z1s(3, :));
legend('DTM', 'Original', 'Recovered');
title('Original and Recovered Paths');
hold off;

saveas(gcf,[out_dir '/path.fig']);
saveas(gcf,[out_dir '/path.jpg']);
close(gcf);

mean_err_pos(window - 1) = sum(pos_err) / length(pos_err);
mean_err_rot(window - 1) = sum(rot_err) / length(rot_err);
%%
end
%%
figure;
semilogy(windows, mean_err_pos(windows - 1));
hold on;
semilogy(windows, mean_err_rot(windows - 1));
title('Mean Error vs Window Size');
legend('Position Error', 'Rotation Error', 'Location','southeast');
xlabel('Window Size');
ylabel('meters / radians');
hold off;

out_dir = [pwd '/Scenario_' scenario_name];
saveas(gcf,[out_dir '/error_vs_window.fig']);
saveas(gcf,[out_dir '/error_vs_window.jpg']);