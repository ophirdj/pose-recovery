function [ P1, R1, errors, Ps, Rs ] = FindPose( P0, R0, rays, distances, DTM, cellsize, ep, er, n_points, n_iter, lambda )
%FindPose Generate points close to given origin and recalculate original
%pose from them. Return result with minimal model error.
%   P0(3, 1)        - Initial global camera position
%   R0(3, 3)        - Initial global camera rotation
%   rays(3, n)      - Rays vectors
%   distances(1, n) - Rays distances to surface
%   DTM             - DTM
%   cellsize        - DTM resolution (distance between cells)
%   ep              - (Model) Position error (noise)
%   er              - (Model) Rotation error (noise)
%   n_points        - Number of points to generate and test
%   lambda          - [OPTIONAL] Descent max step size (should be > 0)

if nargin < 11
    lambda = 5e0;
end

if nargin < 10
    n_iter = 10;
end

if nargin < 9
    n_points = 100;
end

min_err = +inf;

for n = (1:n_points)
    [P, R] = NoisyPose(P0, R0, ep, er);
    [P1_tmp, R1_tmp, errors_tmp, Ps_tmp, Rs_tmp] = FindPoseSingle(P, R, rays, distances, DTM, cellsize, n_iter, lambda);
    if errors_tmp(length(errors_tmp)) < min_err
        min_err = errors_tmp(length(errors_tmp));
        P1 = P1_tmp;
        R1 = R1_tmp;
        errors = errors_tmp;
        Ps = Ps_tmp;
        Rs = Rs_tmp;
    end    
end
end


function [ P1, R1, errors, Ps, Rs ] = FindPoseSingle( P0, R0, rays, distances, DTM, cellsize, n_iter, lambda )
%FindPoseSingle Calculate position and rotation.
%   P0(3, 1)        - Initial global camera position
%   R0(3, 3)        - Initial global camera rotation
%   rays(3, n)      - Rays vectors
%   distances(1, n) - Rays distances to surface
%   DTM             - DTM
%   cellsize        - DTM resolution (distance between cells)
%   n_iter          - number of iterations to run
%   lambda          - [OPTIONAL] Descent max step size (should be > 0)

v0 = PoseToVec(P0, R0);
[v, errors, vs] = GradientDescent(@(v)Error(v, rays, distances, DTM, cellsize), ...
    v0, n_iter, 1e-6, lambda, 0.8, 2e0, [1 1 1 1 1 1]);
[P1, R1] = VecToPose(v);
Ps = zeros(3, size(vs, 2));
Rs = zeros(3, 3, size(vs, 2));
for n = (1:size(vs, 2))
    [Ps(:, n), Rs(:, :, n)] = VecToPose(vs(:, n)');
end
end


function [ v, errors, vs ] = GradientDescent( f, v0, n_iter, epsilon, lambda, beta, h, m )
%GradientDescent Calculate v that minimizes f by Gradient descent.
%   f         - Function to minimize
%   v0        - Start vector
%   n_iter    - [OPTIONAL] Number of iterations
%   epsilon   - [OPTIONAL] Acceptable error
%   lambda    - [OPTIONAL] Descent max step size (should be > 0)
%   beta      - [OPTIONAL] step size rate (should be in (0, 1))
%   h         - [OPTIONAL] Interval for gradient
%   m         - [OPTIONAL] Mask for v. Gradient will be calculated for x(m==1)

if nargin < 8
    m = ones(1, length(v0));
end

if nargin < 7
    h = 1e1;
end

if nargin < 6
    beta = 0.9;
end

if nargin < 5
    lambda = 1e2;
end

if nargin < 4
    epsilon = 1e-2;
end

if nargin < 3
    n_iter = 30;
end

v = v0;
errors = zeros(1, n_iter);
vs = zeros(length(v0), n_iter);
for n = (1:n_iter)
    errors(n) = f(v);
    vs(:, n) = v;
    if errors(n) < epsilon
        return
    end
    
    g = Gradient(f, v, h, m);
    
    % In case gradient is close to 0
    if any(isnan(g)) || all(abs(g) < repmat(epsilon, 1, length(g)))
        if any(isnan(g))
            fprintf('gradient of is nan\n');
            v;
        end
        return
    end

    % We want only the gradient's direction
    g = g / norm(g);
    
    % Find the "optimal" step size along the gradient that best minimizes f
    min_res = f(v);
    min_v = v;
    step_iter = 10;
    step = lambda;
    for s = (1:step_iter)
        u = v - step * g;
        fu = f(u);
        step = step * beta;
        if fu < min_res
            min_v = u;
            min_res = fu;
        end
    end
    
    v = min_v;
end
end


function g = Gradient( f, x, h, m )
%Gradient Calculate numeric gradient of f(x)
%   f - Function of x
%   x - Argument of f
%   h - Interval for gradient
%   m - [OPTIONAL] Mask for x. Gradient will be calculated for x(m==1)

if nargin < 4
    m = ones(1, length(x));
end

g = zeros(1, length(x));
for n = (1:length(x))
    if ~m(n)
        continue;
    end
    y1 = x;
    y1(n) = y1(n) - h;
    y2 = x;
    y2(n) = y2(n) + h;
    
    g(n) = (f(y2) - f(y1)) / (2 * h);
end

end




function v = PoseToVec( P, R )
%PoseToVec Convert position and rotation to flat vector.
    v = [reshape(P, [1, 3]), 1e2 * rotm2eul(R)];
end

function [ P, R ] = VecToPose( v )
%VecToPose Convert flat vector to position and rotation.
    P = reshape(v(1:3), [3, 1]);
    R = eul2rotm(1e-2 * v(4:6));
end

function err = Error( v, rays, distances, DTM, cellsize )
%Error Calculate sum of square distance errors.
    [P, R] = VecToPose(v);
    
    % Calculate the expected ray distances based on pose and DTM and
    % compare them to real distance readings.
    ds = CalcRayDistances(P, R, rays, DTM, cellsize);
    err = sum((distances - ds) .^ 2);

    % Calculate the absolute points where rays touch the surface based on
    % the real distance readings and compare their height to DTM prediction.
%     Prs = repmat(P, 1, size(rays, 2)) + ((R * rays) .* repmat(distances, 3, 1));
%     err = sum((GetSurfaceHeight(Prs(1, :), Prs(2, :), DTM, cellsize) - Prs(3, :)) .^ 2);
end

