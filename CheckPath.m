function isOk = CheckPath( dir_name )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
try
    tru=readbin_v000([dir_name 'mnav.bin'],10);
    mimu=readbin_v000([dir_name 'mimu.bin'],7);
    eimu=readbin_v000([dir_name 'eimu.bin'],7);

    F_LIDAR=fopen([dir_name 'mlidar.bin'],'rb');
    lidar_data=fread(F_LIDAR,3,'double');
    n_rays = lidar_data(2);
    fclose(F_LIDAR);
    lidar=readbin_v000([dir_name 'mlidar.bin'], 3+n_rays);


    isOk = ~any(lidar(:)==inf);

catch
    isOk = false;
end
end

