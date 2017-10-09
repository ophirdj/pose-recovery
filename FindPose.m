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
f = @(v)Error(v, rays, distances, DTM, cellsize);
df = @(v)ErrorGradient(v, rays, distances, DTM, cellsize);
[v, errors, vs] = GradientDescent(f, df, ...
    v0, n_iter, 1e-6, lambda, 0.8);
[P1, R1] = VecToPose(v);
Ps = zeros(3, size(vs, 2));
Rs = zeros(3, 3, size(vs, 2));
for n = (1:size(vs, 2))
    [Ps(:, n), Rs(:, :, n)] = VecToPose(vs(:, n)');
end
end


function [ v, errors, vs ] = GradientDescent( f, df, v0, n_iter, epsilon, lambda, beta )
%GradientDescent Calculate v that minimizes f by Gradient descent.
%   f         - Function to minimize
%   df        - Gradient of f
%   v0        - Start vector
%   n_iter    - [OPTIONAL] Number of iterations
%   epsilon   - [OPTIONAL] Acceptable error
%   lambda    - [OPTIONAL] Descent max step size (should be > 0)
%   beta      - [OPTIONAL] step size rate (should be in (0, 1))

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
    
    g = df(v);
    
    % In case gradient is close to 0
    if any(isnan(g)) || all(abs(g) < repmat(epsilon, 1, length(g)))
        if any(isnan(g))
            fprintf('gradient of is nan\n');
            v
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




function v = PoseToVec( P, R )
%PoseToVec Convert position and rotation to flat vector.
    v = [reshape(P, [1, 3]), 1e2 * rotm2eul(R', 'ZYZ')];
end

function [ P, R ] = VecToPose( v )
%VecToPose Convert flat vector to position and rotation.
    P = reshape(v(1:3), [3, 1]);
    R = eul2rotm(1e-2 * v(4:6), 'ZYZ')';
end

function err = Error( v, rays, distances, DTM, cellsize )
%Error Calculate vertical error.
    [P, R] = VecToPose(v);

    % Calculate the absolute points where rays touch the surface based on
    % the real distance readings and compare their height to DTM prediction.
    Prs = repmat(P, 1, size(rays, 2)) + ((R * rays) .* repmat(distances, 3, 1));
    err = sum((GetSurfaceHeight(Prs(1, :), Prs(2, :), DTM, cellsize) - Prs(3, :)) .^ 2);
end


function dv = ErrorGradient( v, rays, distances, DTM, cellsize )
%ErrorGradient Calculate vertical error gradient.

    [P, R] = VecToPose(v);

    % Calculate the absolute points where rays touch the surface based on
    % the real distance readings and compare their height to DTM prediction.
    Prs = repmat(P, 1, size(rays, 2)) + ((R * rays) .* repmat(distances, 3, 1));
    derrs = 2 * (GetSurfaceHeight(Prs(1, :), Prs(2, :), DTM, cellsize) - Prs(3, :));

    % Derivatives of Prs by P and R. Pr(n) = P + R * [0 0 d_n]';
    % Notice we relate only to (x, y) coordinates of Pr(n) and ignore z.
    dPrsdP = [1 0 0; 0 1 0]';
    dPrsdR = [0 0 1 0 0 0 0 0 0; 0 0 0 0 0 1 0 0 0]'; % * distances(n)
    
    dEdP = zeros(3, 1);
    dEdR = zeros(9, 1);
    for n = (1:length(rays))
        h = GetSurfaceHeightGradient(Prs(1, n), Prs(2, n), DTM, cellsize);
        d = distances(n);
        dEdP = dEdP + derrs(n) * ([0 0 1]' - dPrsdP * h);
        dEdR = dEdR + derrs(n) * d * ([0 0 0 0 0 0 0 0 1]' - dPrsdR * h);
    end
    
    % Derive by Euler angles
    a = v(4);
    b = v(5);
    c = v(6);
    
    dRda = [-sin(a)*cos(b)*cos(c)-cos(a)*sin(c), sin(a)*cos(b)*sin(c)-cos(a)*cos(c), -sin(a)*sin(b),...
            cos(a)*cos(b)*cos(c)-sin(a)*sin(c), -cos(a)*cos(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b),...
            0, 0, 0]';
    dRdb = [-cos(a)*sin(b)*cos(c), -cos(a)*sin(b)*sin(c), cos(a)*cos(b),...
            -sin(a)*sin(b)*cos(c), sin(a)*sin(b)*sin(c), sin(a)*cos(b),...
            -cos(b)*cos(c), cos(b)*sin(c), -sin(b)]';
    dRdc = [-cos(a)*cos(b)*sin(c)-sin(a)*cos(c), -cos(a)*cos(b)*cos(c)+sin(a)*sin(c), 0,...
            -sin(a)*cos(b)*sin(c)+cos(a)*cos(c), -sin(a)*cos(b)*cos(c)-cos(a)*sin(c), 0,...
            sin(b)*sin(c), sin(b)*cos(c), 0]';
    
    dEdeul = [dRda, dRdb, dRdc]' * dEdR;
    
    % Result is expected as row vector
    dv = [dEdP', dEdeul'];
end

