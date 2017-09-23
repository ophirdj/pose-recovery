function [ q, lambda ] = ProjectToImagePlane( P, Q )
%CalcImageProjection Project point Q unto image plane as defined by P
%   P - 4x4 position and rotation of camera relative to global coordinates
%   Q - 3x1 position in global coordinates

p = P(1:3, 4);
R = P(1:3, 1:3);

lambda = norm(Q - p);
q = inv(R) \ (Q - p) / lambda;
q = q / q(3);

end

