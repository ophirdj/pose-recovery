%% Section - Generate path
% The 'real' pose
n_samples = 10;
Ps = [linspace(100, 200, n_samples); linspace(100, 200, n_samples); linspace(400, 300, n_samples)];
Rs = repmat([1 0 0; 0 -1 0; 0 0 -1], 1, 1, n_samples);

% Errors
ed = 5e0;
ep = 1e1;
er = 1e-1;

% Noisy Path poses
P0s = zeros(size(Ps, 1), n_samples);
R0s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
for n = (1:n_samples)
    [P0s(:, n), R0s(:, :, n)] = NoisyPose(Ps(:, n), Rs(:, :, n), ep, er);
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

%% Section - Calculate

n_generated_points = 100;
n_iterations = 10;
lambda = 3.6;

% best_lambda = 0;
% best_ep = +inf;
% for lambda = (2:0.2:5)

P1s = zeros(size(Ps, 1), n_samples);
R1s = zeros(size(Rs, 1), size(Rs, 2), n_samples);
errors = zeros(n_iterations, n_samples);
Pps = zeros(3, n_iterations, n_samples);
Rrs = zeros(3, 3, n_iterations, n_samples);

% curr_ep = 0;

% Reconstruct 'real' poses
for n = (1:n_samples)    
    [P1s(:, n), R1s(:, :, n), errors(:, n), Pps(:, :, n), Rrs(:, :, :, n)] = ...
        FindPose(P0s(:, n), R0s(:, :, n), ...
        rays, distances(:, n)', DTM, cellsize, ...
        ep, er, n_generated_points, n_iterations, lambda);
%     if n < size(Rs, 3)
%         P0s(:, n + 1) = P1s(:, n);
%         R0s(:, :, n + 1) = R1s(:, :, n);
%     end
%     curr_ep = curr_ep + sum((Ps(:, 1) - Pps(size(Pps, 2))) .^ 2);
end
% if curr_ep < best_ep
%     best_ep = curr_ep;
%     best_lambda = lambda;
% end
% fprintf('lambda = %f\n', lambda);
% end
% 
% fprintf('best lambda = %f, best_ep = %f\n', best_lambda, best_ep / n_samples);

%% Section Analyze error

% Mean model error
semilogy((1:length(errors)), sum(errors, 2) / n_samples);
hold on;
% Mean true position error
semilogy((1:length(errors)), sum(sum((repmat(Ps(:, 1), 1, size(Pps, 2), n_samples) - Pps) .^ 2), 3) / n_samples);
% Mean true rotation error
semilogy((1:length(errors)), reshape(sum(sum(sum((repmat(Rs(:, :, 1), 1, 1, size(Rrs, 3), n_samples) - Rrs) .^ 2, 2), 1), 4) / n_samples, 1, size(Rrs, 3)));
title('Square Error per model iteration');
legend('Model Error', 'Position Error', 'Rotation Error', 'Location','northeast');
hold off;