%% Section - Generate path
% The 'real' pose

% Linear path
% n_samples = 20;
% Ps = [linspace(100, 200, n_samples); linspace(100, 200, n_samples); linspace(400, 450, n_samples)];
% Rs = repmat([1 0 0; 0 -1 0; 0 0 -1], 1, 1, n_samples);

n_samples = 30;

% Quick and Dirty closed path
a = [100 200 400];
b = [200 200 450];
c = [200 300 500];

pa = [linspace(a(1), b(1), n_samples); linspace(a(2), b(2), n_samples); linspace(a(3), b(3), n_samples)];
ra = repmat([1 0 0; 0 -1 0; 0 0 -1], 1, 1, n_samples);
pb = [linspace(b(1), c(1), n_samples); linspace(b(2), c(2), n_samples); linspace(b(3), c(3), n_samples)];
rb = repmat([1 0 0; 0 -1 0; 0 0 -1], 1, 1, n_samples);
pc = [linspace(c(1), a(1), n_samples); linspace(c(2), a(2), n_samples); linspace(c(3), a(3), n_samples)];
rc = repmat([1 0 0; 0 -1 0; 0 0 -1], 1, 1, n_samples);

n_samples = n_samples * 3;
Ps = [pa pb pc];
Rs = cat(3, ra, rb, rc);

% Do that loop 3 times
n_samples = n_samples * 3;
Ps = [Ps Ps Ps];
Rs = cat(3, Rs, Rs, Rs);

% Errors
ed = 1e1;
ep = 3e0;
er = 1e-1;
ev = 1e1;

% Noisy Path poses
P0s = zeros(size(Ps, 1), n_samples);
R0s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
for n = (1:n_samples)
    [P0s(:, n), R0s(:, :, n)] = NoisyPose(Ps(:, n), Rs(:, :, n), ep, er);
end

% Noisy "velocity" vectors (displacement from one pose to the next)
v0s = zeros(size(Ps, 1), n_samples - 1);
for n = (1:n_samples - 1)
    v0s(:, n) = Ps(:, n) - Ps(:, n + 1) + ev * (2 * (rand(3, 1) - 0.5));
end

% Distance readings
n_rays = 20;
ray_angle_openning = pi / 4;
rays = GenerateRays(ray_angle_openning / n_rays, n_rays);

distances = zeros(size(rays, 2), n_samples);
for n = (1:n_samples)
    distances(:, n) = CalcRayDistances(Ps(:, n), Rs(:, :, n), ...
                                       rays, DTM, cellsize) + ...
                        ed * rand(1, length(rays));
end

%% Section - Preprocess

% Window size for multi-pose calculation
window = 10;

if window > n_samples
    window = n_samples;
end

% Pad data to fit window size
P0s = [repmat(P0s(:, 1), 1, window - 1), P0s];
R0s = cat(3, repmat(R0s(:, :, 1), 1, 1, window - 1), R0s);
distances = [repmat(distances(:, 1), 1, window - 1), distances];
v0s = [zeros(3, window - 1), v0s];


%% Section - Calculate

n_generated_points = 1;
n_iterations = 10;
lambda = 3.6;

P1s = zeros(size(Ps, 1), n_samples);
R1s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
errors = zeros(n_iterations, n_samples);
Pps = zeros(3, n_iterations, n_samples);
Rrs = zeros(3, 3, n_iterations, n_samples);
window_points = P0s;

% Reconstruct 'real' poses
for n = (1:n_samples)
    [P1s(:, n), R1s(:, :, n), errors(:, n), Pps(:, :, n), Rrs(:, :, :, n)] = ...
        FindPoseMulti(window_points(:, (n:n + window - 1)), R0s(:, :, (n:n + window - 1)), ...
        v0s(:, (n:n + window - 2)), ...
        rays, distances(:, (n:n + window - 1))', DTM, cellsize, ...
        ep, er, n_generated_points, n_iterations, lambda);
    window_points(:, n + window - 1) = P1s(:, n);
end

%% Section Analyze error over time

pos_err_sqr = sum((P1s - Ps) .^ 2, 1);
rot_err_sqr = reshape(sum(sum((R1s - Rs) .^ 2, 1), 2), [1, n_samples]);

figure;
semilogy((1:n_samples), pos_err_sqr);
hold on;
semilogy((1:n_samples), rot_err_sqr);
title('Square Error over time');
legend('Position Error', 'Rotation Error', 'Location','southeast');
hold off;

%% Section Analyze error distribution

figure;
plot((1:n_samples), sort(pos_err_sqr));
hold on;
plot((1:n_samples), sort(rot_err_sqr));
title('Square Error distribution');
legend('Position Error', 'Rotation Error', 'Location','southeast');
xlim([0 n_samples]);
set(gca, 'XTickLabel',num2str((100/n_samples).*get(gca,'XTick')','%g%%'))
hold off;

figure;
semilogy((1:n_samples), sort(pos_err_sqr));
hold on;
semilogy((1:n_samples), sort(rot_err_sqr));
title('Square Error distribution (logscale)');
legend('Position Error', 'Rotation Error', 'Location','southeast');
xlim([0 n_samples]);
set(gca, 'XTickLabel',num2str((100/n_samples).*get(gca,'XTick')','%g%%'))
hold off;

