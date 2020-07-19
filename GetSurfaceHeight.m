function [ z ] = GetSurfaceHeight( x, y, DTM, cellsize )
%GetSurfaceHeight Get height of surface at point (x, y) or 9999 if point is
%outside the DTM.
% Use first order approximation to determine height.
%   x - x pos (can be a vector)
%   y - y pos (can be a vector)
%   DTM - DTM
%   cellsize - DTM resolution (distance between cells)


j_low = floor(x ./ cellsize);
j_high = ceil(x ./ cellsize);
k_low = floor(y ./ cellsize);
k_high = ceil(y ./ cellsize);

weight_x = (x - (j_low .* cellsize)) ./ cellsize;
weight_y = (y - (k_low .* cellsize)) ./ cellsize;

try
z = (DTM(j_low, k_low)   .* (1 - weight_x) .* (1 - weight_y) + ...
     DTM(j_low, k_high)  .* (1 - weight_x) .* (weight_y)     + ...
     DTM(j_high, k_low)  .* (weight_x)     .* (1 - weight_y) + ...
     DTM(j_high, k_high) .* (weight_x)     .* (weight_y));
catch
    z = 9999;
end
 
 
% % Filter only legal indices
% legal = j_low >= 1 & j_high <= size(DTM, 1) & ...
%         k_low >= 1 & k_high <= size(DTM, 2);
% 
% z = 9999*ones(size(x));
%     
% j_low = j_low(legal);
% j_high = j_high(legal);
% k_low = k_low(legal);
% k_high = k_high(legal);
% 
% x_legal = x(legal);
% y_legal = y(legal);
% 
% weight_x = (x_legal - (j_low .* cellsize)) ./ cellsize;
% weight_y = (y_legal - (k_low .* cellsize)) ./ cellsize;
% 
% % First order approximation of surface height
% 
% % Support vector operation
% % DTM(x + ((y - 1) * size(DTM, 1))) equals to DTM(x, y)
% z(legal) = (DTM(j_low + ((k_low - 1) * size(DTM, 1)))   .* (1 - weight_x) .* (1 - weight_y) + ...
%             DTM(j_low + ((k_high - 1) * size(DTM, 1)))  .* (1 - weight_x) .* (weight_y)     + ...
%             DTM(j_high + ((k_low - 1) * size(DTM, 1)))  .* (weight_x)     .* (1 - weight_y) + ...
%             DTM(j_high + ((k_high - 1) * size(DTM, 1))) .* (weight_x)     .* (weight_y));
% 
% % Non vectoric variant
% % z = (DTM(j_low, k_low)   .* (1 - weight_x) .* (1 - weight_y) + ...
% %      DTM(j_low, k_high)  .* (1 - weight_x) .* (weight_y)     + ...
% %      DTM(j_high, k_low)  .* (weight_x)     .* (1 - weight_y) + ...
% %      DTM(j_high, k_high) .* (weight_x)     .* (weight_y));
end

