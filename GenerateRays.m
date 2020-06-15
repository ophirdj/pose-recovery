function [ rays ] = GenerateRays( ray_angles )
% GenLidarRays Generate a matrix where each column is a ray direction.
% Rays are on the y-z plane with.
%   ray_angles - Array specifying angles of rays relative to Y axis.

angles = ray_angles(:)';

rays = [zeros(1, length(angles)); ...
        sin(angles); ...
        -cos(angles)];

end

