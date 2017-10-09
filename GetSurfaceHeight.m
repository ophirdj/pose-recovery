function [ z ] = GetSurfaceHeight( x, y, DTM, cellsize )
%GetSurfaceHeight Get height of surface at point (x, y).
% Use first order approximation to determine height.
%   x - x pos (can be a vector)
%   y - y pos (can be a vector)
%   DTM - DTM
%   cellsize - DTM resolution (distance between cells)

% Get the 4 positions defining the DTM surface for (x, y)
j_low = floor(x ./ cellsize);
j_high = ceil(x ./ cellsize);
k_low = floor(y ./ cellsize);
k_high = ceil(y ./ cellsize);

% In case we're out of bounds
if any(j_low < 1) || any(j_high > size(DTM, 1)) || ...
        any(k_low < 1) || any(k_high > size(DTM, 2))
    z = 0;
    return;
end

weight_x = (x - (j_low .* cellsize)) ./ cellsize;
weight_y = (y - (k_low .* cellsize)) ./ cellsize;

% First order approximation of surface height

% Support vector operation
% DTM(x + ((y - 1) * size(DTM, 1))) equals to DTM(x, y)
z = (DTM(j_low + ((k_low - 1) * size(DTM, 1)))   .* (1 - weight_x) .* (1 - weight_y) + ...
     DTM(j_low + ((k_high - 1) * size(DTM, 1)))  .* (1 - weight_x) .* (weight_y)     + ...
     DTM(j_high + ((k_low - 1) * size(DTM, 1)))  .* (weight_x)     .* (1 - weight_y) + ...
     DTM(j_high + ((k_high - 1) * size(DTM, 1))) .* (weight_x)     .* (weight_y));
end

