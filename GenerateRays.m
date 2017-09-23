function [ rays ] = GenerateRays( alpha, n )
% GenLidarRays Generate a matrix where each column is a ray direction.
% Rays are on the y-z plane in a symmetric span around [0 0 1].
%   alpha   - angle between rays
%   n       - Will generate 2n+1 rays

if n < 0
    rays = [];
    return
end

angles = alpha * (-n:n);
rays = [zeros(1, size(angles, 2)); ...
        sin(angles); ...
        cos(angles)];

end

