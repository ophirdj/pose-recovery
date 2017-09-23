function [ M ] = Wedge( x, y, z )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
if nargin == 1
    z = x(3);
    y = x(2);
    x = x(1);
end
M = [0 -z y; z 0 -x; -y x 0];
end

