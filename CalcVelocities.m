function [ v0s ] = CalcVelocities( Ps, dv )
%CalcVelocities Calculate velocity vectors between path points with error.
%   E ~ D * N(0, dv), where:
%   E - Error
%   D - Distance between path points (Ps[n] - Ps[n+1]
%   N(e, sigma) - Normal distribution with expectency e and variance
%   sigma^2
%   
%   Params:
%   Ps - path points
%   dv - error variance

v0s = zeros(size(Ps, 1), size(Ps, 2) - 1);
for n = (1:size(Ps, 2) - 1)
    v0s(:, n) = (Ps(:, n) - Ps(:, n + 1)) .* (1 + dv * randn(3, 1));
end

end

