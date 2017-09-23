% The 'real' pose
P = [100 200 300]';
R = [1 0 0; 0 -1 0; 0 0 -1];
% Generate readings
rays = GenerateRays(5e-2, 12);
ed = 5e0;
distances = CalcRayDistances(P, R, rays, DTM, cellsize) + ...
    ed * rand(1, length(rays));

% A noisy origin
ep = 5e0;
er = 1e-1;
[P0, R0] = NoisyPose(P, R, ep, er);

% Reconstruct 'real' pose
[P1, R1, errors, Ps, Rs] = FindPose(P0, R0, rays, distances, DTM, cellsize);

% Analyze errors
Perr = zeros(1, length(errors));
Rerr = zeros(1, length(errors));
for n = (1:length(errors))
    Perr(n) = sum((Ps(:, n) - P).^2);
    Rerr(n) = sum(sum((Rs(:, :, n) - R).^2));
end

semilogy((1:length(errors)), errors);
hold;
semilogy((1:length(errors)), Perr);
semilogy((1:length(errors)), Rerr);
title('Square Error vs GD Iterations');
legend('Model Error','Position Error', 'Rotation Error', 'Location','northeast');

% disp 'Start Error'
% sum((P0 - P).^2)
% sum(sum((R0 - R).^2))
% 
% disp 'Final Error'
% sum((P1 - P).^2)
% sum(sum((R1 - R).^2))