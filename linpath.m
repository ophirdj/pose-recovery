function [ Ps, Rs ] = linpath( p_sart, p_end, R_start, R_end, n_samples )
%linpath Generate a linear path from start (inclusive) to end (exclusive)
%using n_samples points.

Ps = [linspace(p_sart(1), p_end(1), n_samples); ...
      linspace(p_sart(2), p_end(2), n_samples); ...
      linspace(p_sart(3), p_end(3), n_samples)];

rot_start = rotm2eul(R_start);
rot_end = rotm2eul(R_end);

rotation_angles = [linspace(rot_start(1), rot_end(1), n_samples); ...
                   linspace(rot_start(2), rot_end(2), n_samples); ...
                   linspace(rot_start(3), rot_end(3), n_samples)]';

Rs = eul2rotm(rotation_angles);

end

