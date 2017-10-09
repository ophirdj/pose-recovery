function [ dz ] = GetSurfaceHeightGradient( x, y, DTM, cellsize )
%GetSurfaceHeightGradient Calculate DTM gradient by P and R.
% @see GetSurfaceHeight
%   x        - x coordinate for gradient
%   y        - y coordinate for gradient
%   DTM      - DTM
%   cellsize - DTM resolution (distance between cells)

% Get the 4 positions defining the DTM surface for (x, y)
% In case we fall exactly on a joint of DTM planes, choose an arbitrary
% plane and calculate according to it.
j_low = floor(x ./ cellsize);
j_high = j_low + 1;
k_low = floor(y ./ cellsize);
k_high = k_low + 1;

% In case we're out of bounds
if any(j_low < 1) || any(j_high > size(DTM, 1)) || ...
        any(k_low < 1) || any(k_high > size(DTM, 2))
    dz = zeros(2, 1);
    return;
end

weight_x = (x - (j_low .* cellsize)) ./ cellsize;
weight_y = (y - (k_low .* cellsize)) ./ cellsize;

% First order approximation of surface height

% Support vector operation
% DTM(x + ((y - 1) * size(DTM, 1))) equals to DTM(x, y)
dx = (DTM(j_low + ((k_low - 1) * size(DTM, 1)))   .* -(1 - weight_y) + ...
      DTM(j_low + ((k_high - 1) * size(DTM, 1)))  .* -(weight_y)     + ...
      DTM(j_high + ((k_low - 1) * size(DTM, 1)))  .* (1 - weight_y)  + ...
      DTM(j_high + ((k_high - 1) * size(DTM, 1))) .* (weight_y));


dy = (DTM(j_low + ((k_low - 1) * size(DTM, 1)))   .* -(1 - weight_x) + ...
      DTM(j_low + ((k_high - 1) * size(DTM, 1)))  .* (1 - weight_x)  + ...
      DTM(j_high + ((k_low - 1) * size(DTM, 1)))  .* -(weight_x)     + ...
      DTM(j_high + ((k_high - 1) * size(DTM, 1))) .* (weight_x));

dz = [dx(:)'; dy(:)'] ./ cellsize;

end


