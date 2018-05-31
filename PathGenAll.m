function [] = PathGenAll( dir_name, mot_def, ini_pos, n_rays, span_angle, DTM, cellsize )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

if ~isdir(dir_name)
    mkdir(dir_name);
end

%% Generate ground truth path and IMU
ini_pva = zeros(3);
freq_Hz=100; % dt = 1 / freq_Hz
out_typ = [1 freq_Hz];
sim_mode = 1;
PathGen_v003(dir_name, ini_pva, mot_def, out_typ, sim_mode);

%%"Fix" nav data to start from ini_pos
nav = readbin_v000([dir_name 'mnav.bin'],10);
F_MNAV = fopen([dir_name 'mnav.bin'], 'wb');
for n=1:size(nav, 2)
    fwrite(F_MNAV, [nav(1, n); nav(2:4, n) + ini_pos(:); nav(5:10, n)], 'double');
end
fclose(F_MNAV);

%% Generate errornous IMU
%%IMU error definitions (in continious time)
%Accelerometers
SenErrDef(1).A=1;%-0.0005;
SenErrDef(1).B=1;%1e-3;
SenErrDef(1).C=1;
SenErrDef(1).D=1;%5e-2;
SenErrDef(1).sP=1;%0.1;
SenErrDef(1).tparam=[];

SenErrDef(2)=SenErrDef(1);
SenErrDef(3)=SenErrDef(1);

%Gyroscopes
SenErrDef(4).A=-0.0003;
SenErrDef(4).B=1e-5;
SenErrDef(4).C=1;
SenErrDef(4).D=1e-2;
SenErrDef(4).sP=1e-2;
SenErrDef(4).tparam=[]; 
SenErrDef(5)=SenErrDef(4);
SenErrDef(6)=SenErrDef(4);

%???
tem_mod.A=1;
tem_mod.B=0;
tem_mod.u=0;

%%Generate erronous imu
AddIMUErr_v000([dir_name 'mimu.bin'], [dir_name 'imu.bin'], [dir_name 'imuerr.bin'], 7, 2:7, SenErrDef, tem_mod, 0);

%%"Fix" imu data to fit the form of mimu.bin
imu = readbin_v000([dir_name 'imu.bin'],8);
F_EIMU = fopen([dir_name 'eimu.bin'], 'wb');
for n=1:size(imu, 2)
    fwrite(F_EIMU, imu([1, 3:8], n), 'double');
end
fclose(F_EIMU);

%% Generate LIDAR
LidarGen([dir_name 'mnav.bin'], [dir_name 'mlidar.bin'], n_rays, span_angle, DTM, cellsize);
end

