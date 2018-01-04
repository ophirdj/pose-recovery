function [ heights, angles, errors ] = ErrorFromCsv( csv )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
NODATA = 999;

hs = csv(:, 1);
as = csv(:, 2);
es = csv(:, 3:4);

heights = sort(unique(hs));
angles = sort(unique(as));

errors = NODATA .* ones(length(heights), length(angles), 2);


for h = (1:length(heights))
    ih = hs==heights(h);
    for a = (1:length(angles))
        ia = as==angles(a);
        errors(h, a, :) = mean(es(ih & ia, :));
    end
end

end

