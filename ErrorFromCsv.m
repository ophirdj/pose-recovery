function [ heights, angles, errors ] = ErrorFromCsv( csv )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

hs = csv(:, 1);
as = csv(:, 2);
eps = csv(:, 3);
ers = csv(:, 4);

heights_unique = sort(unique(hs));
angles_unique = sort(unique(as));

[grid_heights, grid_angles] = meshgrid(linspace(min(heights_unique), max(heights_unique), 20), ...
                                       linspace(min(angles_unique), max(angles_unique), 20));

pos_errors = griddata(hs, as, eps, grid_heights, grid_angles, 'cubic');
rot_errors = griddata(hs, as, ers, grid_heights, grid_angles, 'cubic');

heights = grid_heights;
angles = grid_angles;
errors = cat(3, pos_errors, rot_errors);

% NODATA = 999;
% 
% hs = csv(:, 1);
% as = csv(:, 2);
% es = csv(:, 3:4);
% 
% heights = sort(unique(hs));
% angles = sort(unique(as));
% 
% errors = NODATA .* ones(length(heights), length(angles), 2);
% 
% 
% for h = (1:length(heights))
%     ih = hs==heights(h);
%     for a = (1:length(angles))
%         ia = as==angles(a);
%         errors(h, a, :) = mean(es(ih & ia, :));
%     end
% end

end

