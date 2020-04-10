function success = LidarGen( in_mnav, out_mlidar, ray_angles, DTM, cellsize )
%LidarGen Compute LIDAR readings for path given by in_mnav.
%   in_mnav    - Navigation data file name.
%   out_mlidar - Output LIDAR data file name.
%   ray_angles - Array of angles (radians) of rays from y axis.
%                Rays are  periodically cycled through this array.
%   DTM        - DTM.
%   cellsize   - DTM resolution (meters).

F_TRU=fopen(in_mnav,'rb');
F_LIDAR=fopen(out_mlidar,'wb');
true_val = fread(F_TRU, 10, 'double');

rays = GenerateRays(ray_angles);

while (~feof(F_TRU))
    pr_coun = true_val(1);
    pos = true_val(2:4);
    att = true_val(8:10);
    % Look down
    Cbn = [0 1 0; 1 0 0; 0 0 -1] * euler2dcm_v000(att);
    
    rho = CalcRayDistances(pos, Cbn, rays(:,1+mod(pr_coun,size(rays,2))), DTM, cellsize);
    
    if any(rho==inf)
        fprintf('BAD LIDAR\n');
        success = false;
        return;
    end
    
    fwrite(F_LIDAR,[pr_coun;rho],'double');
    
    true_val = fread(F_TRU, 10, 'double');
end

fclose(F_TRU);
fclose(F_LIDAR);

success = true;
end

