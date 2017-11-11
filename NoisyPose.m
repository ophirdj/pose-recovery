function [ P0, R0 ] = NoisyPose( P, R, ep, er )
%NoisyPose Generate a noisy pose.
%   P  - Position
%   R  - Rotation matrix
%   ep - Position error (noise)
%   er - Rotation error (noise)
P0 = P + ep * (2 * (rand(3, 1) - 0.5));
R0 = R * (eye(3) + Wedge(er * (2 * rand(3, 1) - 1)));

end

