function success = PathGenAll( dir_name, mot_def, ini_pos, ray_angles, freq_Hz, DTM, cellsize, vel )
    %UNTITLED Summary of this function goes here
    %   Detailed explanation goes here

    if ~isdir(dir_name)
        mkdir(dir_name);
    end

    % Save metadata
    F_META = fopen([dir_name 'meta.bin'], 'wb');
    fwrite(F_META, freq_Hz, 'double');
    fwrite(F_META, cellsize, 'double');
    fwrite(F_META, length(ray_angles), 'double');
    fwrite(F_META, ray_angles, 'double');

    gen_ground_truth(dir_name, mot_def, ini_pos, freq_Hz, vel);
    
    % Nav length
    fwrite(F_META, size(readbin_v000([dir_name 'mnav.bin'],10), 2), 'double');
    fclose(F_META);
    
    success = LidarGen([dir_name 'mnav.bin'], [dir_name 'mlidar.bin'], ray_angles, DTM, cellsize);

    if ~success
        return
    end
    
    %IMU
    for linear_err = [0 1e-16 1e-14 1e-12 1e-10 1e-8 1e-6 1e-4 1e-2 1e-1 1e0]
        for angular_err = [0 1e-16 1e-14 1e-12 1e-10 1e-8 1e-6 1e-4 1e-2 1e-1 1e0]
            dir = [dir_name sprintf('imu_%.0d_%.0d/', linear_err, angular_err)];
            if ~isdir(dir)
                mkdir(dir);
            elseif isdir(dir)
%                 continue;
            end
            gen_imu_err_v000(dir_name, dir, linear_err, angular_err);
        end
    end

%     %DTM
%     for dtm_err = [0 1e-2 1e-1 1e0 2e0 5e0 1e1]
%         dir = [dir_name sprintf('dtm_%.0d/', dtm_err)];
%         if ~isdir(dir)
%             mkdir(dir);
%         end
%         errDTM = DTM + dtm_err .* randn(size(DTM));
%         success = LidarGen([dir_name 'mnav.bin'], [dir 'mlidar.bin'], n_rays, span_angle, errDTM, cellsize);
%         
%         if ~success
%             return
%         end
%     end
%     
%     %LIDAR
%     lidar = readbin_v000([dir_name 'mlidar.bin'],2+2*n_rays);
%     for lidar_err = [0 1e-4 1e-3 1e-2 1e-1 1e0 5e0]
%         dir = [dir_name sprintf('lidar_%.0d/', lidar_err)];
%         if ~isdir(dir)
%             mkdir(dir);
%         end
%         
%         l = lidar(2:end,:);
%         le = l + lidar_err .* (2.* rand(size(l)) - 1);
%         o = [lidar(1,:);le];
%         
%         F_LIDAR = fopen([dir 'mlidar.bin'], 'wb');
%         for n=1:size(o, 2)
%             fwrite(F_LIDAR, o(:, n), 'double');
%         end
%         fclose(F_LIDAR);
%     end
% 
% 
% %     linear_err = 1e-3;
% %     angular_err = 1e-3;
% %     for n=1:500
% %         dir = [dir_name sprintf('monte_carlo_%d/', n)];
% %         if ~isdir(dir)
% %             mkdir(dir);
% %         elseif isdir(dir)
% %             continue;
% %         end
% %         gen_imu_err(dir_name, dir, linear_err, angular_err);
% %     end
end


function [] = gen_ground_truth(dir_name, mot_def, ini_pos, freq_Hz, vel)
    %% Generate ground truth path and IMU
    ini_pva = zeros(3);
    ini_pva(:,2) = vel;
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
end

function [] = gen_imu_err_v000(nav_dir, dir_name, linear_err, angular_err)
    F_MIMU = fopen([nav_dir 'mimu.bin'], 'rb');
    F_EIMU = fopen([dir_name 'eimu.bin'], 'wb');
    imu_data = fread(F_MIMU,7,'double');
    while (~feof(F_MIMU))
        pr_count = imu_data(1);
        accelerometer = imu_data(2:4) + randn(3,1)*linear_err;
        gyroscope = imu_data(5:7) + randn(3,1)*angular_err;
        fwrite(F_EIMU, [pr_count;accelerometer(:);gyroscope(:)], 'double');
        imu_data = fread(F_MIMU,7,'double');
    end
    fclose(F_MIMU);
    fclose(F_EIMU);
end

function [] = gen_imu_err_v001(nav_dir, dir_name, linear_err, angular_err)
    %% Generate errornous IMU
    %%IMU error definitions (in continious time)
    %Accelerometers
    SenErrDef(1).A=0;...-5e-4;
    SenErrDef(1).B=0;...1e-3;
    SenErrDef(1).C=0;...1;
    SenErrDef(1).D=linear_err;...=5e-2;%error magnitude
    SenErrDef(1).sP=0;...1;
    SenErrDef(1).tparam=[];

    SenErrDef(2)=SenErrDef(1);
    SenErrDef(3)=SenErrDef(1);

    %Gyroscopes
    SenErrDef(4).A=0;...-0.0003;
    SenErrDef(4).B=0;...1e-5;
    SenErrDef(4).C=0;...1;
    SenErrDef(4).D=angular_err;...=1e-2;%error magnitude
    SenErrDef(4).sP=0;...1e-2;
    SenErrDef(4).tparam=[]; 
    SenErrDef(5)=SenErrDef(4);
    SenErrDef(6)=SenErrDef(4);

    %???
    tem_mod.A=1;
    tem_mod.B=0;
    tem_mod.u=0;

    %%Generate erronous imu
    AddIMUErr_v000([nav_dir 'mimu.bin'], [dir_name 'imu.bin'], [dir_name 'imuerr.bin'], 7, 2:7, SenErrDef, tem_mod, 0);

    %%"Fix" imu data to fit the form of mimu.bin
    imu = readbin_v000([dir_name 'imu.bin'],8);
    F_EIMU = fopen([dir_name 'eimu.bin'], 'wb');
    for n=1:size(imu, 2)
        fwrite(F_EIMU, imu([1, 3:8], n), 'double');
    end
    fclose(F_EIMU);
end