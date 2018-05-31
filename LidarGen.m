function [ ] = LidarGen( in_mnav, out_mlidar, n_rays, span_angle, DTM, cellsize )
%LidarGen Compute LIDAR readings for path given by in_mnav.
%   in_mnav    - Navigation data file name.
%   out_mlidar - Output LIDAR data file name.
%   n_rays     - Number of LIDAR rays = 1 + 2*n_rays
%   span_angle - Angle (radians) of side-most ray from y axis.
%   DTM        - DTM.
%   cellsize   - DTM resolution (meters).

F_TRU=fopen(in_mnav,'rb');
F_LIDAR=fopen(out_mlidar,'wb');
true_val = fread(F_TRU, 10, 'double');

rays = GenerateRays(span_angle / n_rays, n_rays);

while (~feof(F_TRU))
    pr_coun = true_val(1);
    pos = true_val(2:4);
    att = true_val(5:7);
    att(2) = -att(2); % Invert pitch (look "down")
    Cbn = euler2dcm_v000(att);
    
    distances = CalcRayDistances(pos, Cbn, rays, DTM, cellsize);
    
    fwrite(F_LIDAR,[pr_coun;1+2*n_rays;span_angle;distances(:)],'double');
    
    true_val = fread(F_TRU, 10, 'double');
end

fclose(F_TRU);
fclose(F_LIDAR);

end

