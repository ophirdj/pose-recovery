function [ zs ] = rotm2zvec( Rs )
%UNTITLED6 Summary of this function goes here
%   Detailed explanation goes here

m = size(Rs, 3);
zs = zeros(3, m);
for n = (1:m)
    % Select 3rd (z) column
    zs(:, n) = Rs(:, 3, n);
end

end

