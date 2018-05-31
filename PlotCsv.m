function [ ] = PlotCsv( csv )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

hs = csv(:, 1);
as = csv(:, 2);
eps = csv(:, 3);
ers = csv(:, 4);

heights_unique = sort(unique(hs));
angles_unique = sort(unique(as));

[grid_heights, grid_angles] = meshgrid(linspace(min(heights_unique), max(heights_unique), 25), ...
                                       linspace(min(angles_unique), max(angles_unique), 25));

pos_errors = griddata(hs, as, eps, grid_heights, grid_angles, 'cubic');
rot_errors = griddata(hs, as, ers, grid_heights, grid_angles, 'cubic');


surf(grid_heights, grid_angles, pos_errors);

end

