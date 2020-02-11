function [ distances, points_of_intersect, rays_g ] = CalcRayDistances( P, R, rays, DTM, cellsize )
%CalcRayDistances Calculate distances to surface along given rays.
%   P(3, 1)    - Global camera position
%   R(3, 3)    - Global camera rotation
%   rays(3, n) - Rays vectors
%   DTM        - DTM
%   cellsize   - DTM resolution (distance between cells)

% Make sure P is a column vector
if size(P, 1) == 1
    P = P';
end

% Convert to global coordinates
rays_g = R * rays;

distances = zeros(1, size(rays_g, 2));
points_of_intersect = zeros(3, size(rays_g, 2));
for n = 1:size(rays_g, 2)
    [distances(n), points_of_intersect(:,n)] = RayTrace(P, rays_g(:, n), DTM, cellsize);
end

end

