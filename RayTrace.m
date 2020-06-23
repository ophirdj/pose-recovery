function [ distance, Q ] = RayTrace( P, ray, DTM, cellsize, epsilon, lambda, beta )
% GETDISTANCETOGROUD Get distance from point P to ground along the ray.
%   P(3, 1)   - 3D global camera position
%   ray(3, 1) - 3D global ray direction
%   DTM       - DTM
%   cellsize  - DTM resolution (distance between cells)
%   epsilon   - [OPTIONAL] Acceptable result error (should be > 0)
%   lambda    - [OPTIONAL] Initial distance for ray trace (should be > 0)
%   beta      - [OPTIONAL] Ray trace extention rate (should be > 1)

if nargin < 7 || beta <= 1
    beta = 1.3;
end

if nargin < 6 || lambda <= 0
    lambda = 1000;
end

if nargin < 5 || epsilon <= 0
    epsilon = 1e-6;
end

% normalize ray;
ray = ray / norm(ray);

% First, check we are above ground level
dist_high = 0;
if P(3) - GetSurfaceHeight(P(1), P(2), DTM, cellsize) <= 0
    distance = 0;
    Q = P;
    return;
end


% Then, trace along the ray to find a place below ground level


Q = P + lambda * ray;
height = Q(3) - GetSurfaceHeight(Q(1), Q(2), DTM, cellsize);


while height > 0
    dist_high = lambda;
    lambda = lambda * beta;
    Q = P + lambda * ray;
    if Q(1) > cellsize && Q(1) < cellsize * size(DTM, 1) && ...
            Q(2) > cellsize && Q(2) < cellsize * size(DTM, 2)
        height = Q(3) - GetSurfaceHeight(Q(1), Q(2), DTM, cellsize);
    else
        distance = dist_high;
        return;
    end
end

dist_low = lambda;
distance = lambda;

% Now we know we intersect with ground somewhere in [dist_low, dist_high]
% Bisect to find intersection
while dist_low - dist_high > epsilon
    distance = (dist_high + dist_low) / 2;
    Q = P + distance * ray;
    height = Q(3) - GetSurfaceHeight(Q(1), Q(2), DTM, cellsize);
    if abs(height) < epsilon
        return
    end
    if height > 0
        dist_high = distance;
    else
        dist_low = distance;
    end
end

end

