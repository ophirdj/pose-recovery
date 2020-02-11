function [ N ] = GetSurfaceNormal( x, y, DTM, cellsize )
%GetSurfaceNormal Get height of surface at point (x, y) or [0 0 1] if point is
%outside the DTM.
% Use first order approximation to determine Normal.
%   x - x pos (can be a vector)
%   y - y pos (can be a vector)
%   DTM - DTM
%   cellsize - DTM resolution (distance between cells)

j_low = floor(x ./ cellsize);
j_high = ceil(x ./ cellsize);
k_low = floor(y ./ cellsize);
k_high = ceil(y ./ cellsize);

% Filter only legal indices
legal = j_low >= 1 & j_high <= size(DTM, 1) & ...
        k_low >= 1 & k_high <= size(DTM, 2);
    
j_low = j_low(legal);
j_high = j_high(legal);
k_low = k_low(legal);
k_high = k_high(legal);

u = repmat([1 0 0]', [1, length(x)]);
v = repmat([0 1 0]', [1, length(x)]);

u(:,legal) = [cellsize * ones(1, sum(legal)); zeros(1, sum(legal)); DTM(j_high + ((k_high - 1) * size(DTM, 1))) - DTM(j_low + ((k_high - 1) * size(DTM, 1)))];
v(:,legal) = [zeros(1, sum(legal)); cellsize * ones(1, sum(legal)); DTM(j_high + ((k_high - 1) * size(DTM, 1))) - DTM(j_high + ((k_low - 1) * size(DTM, 1)))];

N = normc(cross(u, v));

end

