%% Section - Generate path
% The 'real' pose

via_points = [114 171 300; ...
              145 166 370; ...
              184 169 350; ...
              214 207 370; ...
              237 226 380; ...
              285 250 450; ...
              300 276 450; ...
              266 293 450; ...
              257 297 450; ...
              200 298 350; ...
              168 287 300];

rots = eul2rotm([0 pi 0;           ...
                 0 pi pi;          ...
                 0 pi pi/2;        ...
                 pi/6 pi 0;        ...
                 0 pi 0;           ...
                 pi/6 pi pi/3;     ...
                 0 pi pi/3;        ...
                 pi/6 pi pi;       ...
                 0 pi 0;           ...
                 0 pi pi;          ...
                 0 pi 0], 'ZYZ');

% Generate and concatenate path parts
Ps = [];
Rs = [];
for n = (1:size(via_points, 1) - 1)
    [temp_Ps, temp_Rs] = linpath(via_points(n, :), via_points(n+1, :), ...
                                 rots(:, :, n), rots(:, :, n+1), 20);
    Ps = cat(2, Ps, temp_Ps);
    Rs = cat(3, Rs, temp_Rs);
end
n_samples = size(Ps, 2);

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


%% Section plot path
vis_DTM = DTM;
% Fill it with whatever that is not so off scale
vis_DTM(vis_DTM==NODATA_value) = 0;

figure;
mesh(vis_DTM);
hold on;
Zs = rotm2zvec(Rs);
quiver3(Ps(1, :), Ps(2, :), Ps(3, :), Zs(1, :), Zs(2, :), Zs(3, :));
hold off;