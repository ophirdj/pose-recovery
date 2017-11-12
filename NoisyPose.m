function [ P0, R0 ] = NoisyPose( P, R, ep, er )
%NoisyPose Generate a noisy pose.
%   P  - Position
%   R  - Rotation matrix
%   ep - Position error (noise)
%   er - Rotation error (noise)
P0 = P + ep * randn(3, 1);
R0 = R * (eye(3) + Wedge(er * randn(3, 1)));

end

