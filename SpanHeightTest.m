output_file = 'error_vs_span_height1.csv';

n_samples = 10;

% Errors
ed = 1e0;
eDTM = 5e0;
ep = 5e0;
er = 1e-1;
dv = 1e-3;

% distance between points
d = 10;

% LIDAR rays
n_rays = 20;
   
window = 10;

heights = (100:50:400);
angles = (1e-1:1e-1:pi/3);

% n_tests = 300;

% for test = (1:n_tests)

% fprintf('test [%d/%d]\n', test, n_tests);
    
% scenario variables
% h = 100 + 200 * rand();
% ray_angle_openning = (pi / 4) * rand();

for h = heights
for ray_angle_openning = angles

n_tests_per_initial_condition = 10;

for test_per_initial_condition = (1:n_tests_per_initial_condition)

fprintf('%d x %d [%d/%d]\n', ...
    h, ...
    ray_angle_openning, ...
    test_per_initial_condition, ...
    n_tests_per_initial_condition);

Ps = zeros(3, n_samples);
Rs = repmat(eul2rotm([pi/6 pi 0]), 1, 1, n_samples);
Ps(1, 1) = randi([200, size(DTM, 1) - 200]);
Ps(2, 1) = randi([200, size(DTM, 2) - 200]);
Ps(3, 1) = h + GetSurfaceHeight(Ps(1, 1), Ps(2, 1), DTM, cellsize);

% Choose a random direction of movement
a = 2 * pi * rand();
for n = (2:n_samples)
    % Slightly modify movement direction
    a = mod(a + (3e-1 * rand()), 2 * pi);
    Ps(1, n) = d * cos(a) + Ps(1, n-1);
    Ps(2, n) = d * sin(a) + Ps(2, n-1);
    Ps(3, n) = h + GetSurfaceHeight(Ps(1, n), Ps(2, n), DTM, cellsize);
end

% Noisy Path poses
P0s = zeros(size(Ps, 1), n_samples);
R0s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
for n = (1:n_samples)
    [P0s(:, n), R0s(:, :, n)] = NoisyPose(Ps(:, n), Rs(:, :, n), ep, er);
end

% Noisy "velocity" vectors (displacement from one pose to the next)
v0s = CalcVelocities(Ps, dv);

noisyDTM = DTM + eDTM * randn(size(DTM));

% Distance readings
rays = GenerateRays(ray_angle_openning / n_rays, n_rays);
distances = zeros(size(rays, 2), n_samples);
for n = (1:n_samples)
    distances(:, n) = CalcRayDistances(Ps(:, n), Rs(:, :, n), ...
                                       rays, DTM, cellsize) + ...
                        ed * randn(1, length(rays));
end


% Pad data to fit window size
P0s = [repmat(P0s(:, 1), 1, window - 1), P0s];
R0s = cat(3, repmat(R0s(:, :, 1), 1, 1, window - 1), R0s);
distances = [repmat(distances(:, 1), 1, window - 1), distances];
v0s = [zeros(3, window), v0s];


%% Section - Calculate

n_generated_points = 1;
n_iterations = 10;
lambda = 3.6;

P1s = zeros(size(Ps, 1), n_samples);
R1s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
window_points = zeros(3, size(P0s, 2) + 1);
window_points(:, 1:window) = P0s(:, 1:window);

% Reconstruct 'real' poses
for n = (1:n_samples)
    [P1s(:, n), R1s(:, :, n), ~, ~, ~] = ...
        FindPoseMulti(window_points(:, (n:n + window - 1)), R0s(:, :, (n:n + window - 1)), ...
        v0s(:, (n:n + window - 2)), ...
        rays, distances(:, (n:n + window - 1))', noisyDTM, cellsize, ...
        ep, er, n_generated_points, n_iterations, lambda);
    window_points(:, n + window - 1) = P1s(:, n);
    window_points(:, n + window) = P1s(:, n) - v0s(:, n + window - 1);
end

pos_err = sqrt(sum((P1s(:, n_samples) - Ps(:, n_samples)) .^ 2, 1));
rot_err = sqrt(sum(sum((R1s(:, :, n_samples) - Rs(:, :, n_samples)) .^ 2, 1), 2));

%%
fid = fopen(output_file, 'a+');
fprintf(fid, '%d,%d,%d,%d\n', [h, ray_angle_openning, pos_err, rot_err]);
fclose(fid);


end

end
end
