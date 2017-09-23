%% Section - Generate path
% The 'real' pose
n_samples = 10;
Ps = [linspace(100, 200, n_samples); linspace(100, 200, n_samples); linspace(400, 300, n_samples)];
Rs = repmat([1 0 0; 0 -1 0; 0 0 -1], 1, 1, n_samples);

% Errors
ed = 5e0;
ep = 1e1;
er = 1e-1;
ev = 2e0;

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
ray_angle_openning = pi / 6;
rays = GenerateRays(ray_angle_openning / n_rays, n_rays);

distances = zeros(size(rays, 2), n_samples);
for n = (1:n_samples)
    distances(:, n) = CalcRayDistances(Ps(:, n), Rs(:, :, n), ...
                                       rays, DTM, cellsize) + ...
                        ed * rand(1, length(rays));
end

%% Section - Preprocess

% Window size for multi-pose calculation
window = 4;

if window > n_samples
    window = n_samples;
end

% Pad data to fit window size
P0s = [repmat(P0s(:, 1), 1, window - 1), P0s];
R0s = cat(3, repmat(R0s(:, :, 1), 1, 1, window - 1), R0s);
distances = [repmat(distances(:, 1), 1, window - 1), distances];
v0s = [zeros(3, window - 1), v0s];


%% Section - Calculate

n_generated_points = 10;
n_iterations = 10;
lambda = 3.6;

P1s = zeros(size(Ps, 1), n_samples);
R1s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
errors = zeros(n_iterations, n_samples);
Pps = zeros(3, n_iterations, n_samples);
Rrs = zeros(3, 3, n_iterations, n_samples);

% Reconstruct 'real' poses
for n = (1:n_samples)
    [P1s(:, n), R1s(:, :, n), errors(:, n), Pps(:, :, n), Rrs(:, :, :, n)] = ...
        FindPoseMulti(P0s(:, (n:n + window - 1)), R0s(:, :, (n:n + window - 1)), ...
        v0s(:, (n:n + window - 2)), ...
        rays, distances(:, (n:n + window - 1))', DTM, cellsize, ...
        ep, er, n_generated_points, n_iterations, lambda);
end


%% Section Analyze error

% Mean model error
semilogy((1:n_iterations), sum(errors, 2) / n_samples);
hold on;
% Mean true position error
p_errors = zeros(1, n_iterations);
for n = (1:n_samples)
    p_errors = p_errors + sum((Pps(:, :, n) - repmat(Ps(:, n), 1, n_iterations)) .^ 2, 1);
end
p_errors = p_errors / n_samples;
semilogy((1:n_iterations), p_errors);
% Mean true rotation error
r_errors = zeros(1, n_iterations);
for n = (1:n_samples)
    r_errors = r_errors + reshape(sum(sum((Rrs(:, :, :, n) - repmat(Rs(:, :, n), 1, 1, n_iterations)) .^ 2, 2), 1), 1, n_iterations);
end
r_errors = r_errors / n_samples;
semilogy((1:n_iterations), r_errors);
title('Square Error per model iteration');
legend('Model Error', 'Position Error', 'Rotation Error', 'Location','northeast');
hold off;