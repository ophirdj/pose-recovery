function [ res_pos, res_att ] = visualize_error( ...
    pos_window, Cbn_window, delta_pos_window, delta_att_window, ...
    rays, lidar_window, DTM, cellsize, bound)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
    lambda1 = 0.1;
    lambda2 = 0.2;

    att_window = zeros(size(pos_window));
    for n=1:size(pos_window, 2)
        att_window(:, n) = dcm2euler_v000(Cbn_window(:, :, n));
    end
    
    
    delta_pos = 0.5;
    rho = bound;
    
    res_pos = zeros(2*rho+1, 2*rho+1, 2*rho+1);
    
    for l=1:2*rho+1
        for m=1:2*rho+1
            for k=1:2*rho+1
                p = pos_window(:, end) - delta_pos .* ([l, m, k] - (rho+1))';
                cost = calc_cost([pos_window(:, 1:end-1) p], ...
                                       att_window, ...
                                       delta_pos_window, delta_att_window, ...
                                       rays, lidar_window, DTM, cellsize, ...
                                       lambda1, lambda2);
               res_pos(l,m,k) = sqrt(sum(cost .^ 2));
            end
        end
    end
    
    delta_att = 0.01;
    
    res_att = zeros(2*rho+1, 2*rho+1, 2*rho+1);
    
    for l=1:2*rho+1
        for m=1:2*rho+1
            for k=1:2*rho+1
                a = att_window(:, end) - delta_att .* ([l, m, k] - (rho+1))';
                cost = calc_cost(pos_window, ...
                                       [att_window(:, 1:end-1), a], ...
                                       delta_pos_window, delta_att_window, ...
                                       rays, lidar_window, DTM, cellsize, ...
                                       lambda1, lambda2);
               res_att(l,m,k) = sqrt(sum(cost .^ 2));
            end
        end
    end    
    
    if rho > 0
        pos_grid = (-rho:rho) * delta_pos;
        [PGX, PGY] = meshgrid(pos_grid, pos_grid);
        att_grid = (-rho:rho) * delta_att;
        [AGX, AGY] = meshgrid(att_grid, att_grid);
        
        surf(PGX, PGY, res_pos(:,:,rho+1));
        title('Cost Function Value X-Y');
        xlabel('X');
        ylabel('Y');
        figure;
        surf(PGX, PGY, reshape(res_pos(:,rho+1,:), size(res_pos, 1), size(res_pos, 3)));
        title('Cost Function Value X-Z');
        xlabel('X');
        ylabel('Z');
        figure;
        surf(PGX, PGY, reshape(res_pos(rho+1,:,:), size(res_pos, 2), size(res_pos, 3)));
        title('Cost Function Value Y-Z');
        xlabel('Y');
        ylabel('Z');
        figure;
        surf(AGX, AGY, res_att(:,:,rho+1));
        title('Cost Function Value yaw-pitch');
        xlabel('yaw');
        ylabel('pitch');
        figure;
        surf(AGX, AGY, reshape(res_att(:,rho+1,:), size(res_att, 1), size(res_att, 3)));
        title('Cost Function Value yaw-roll');
        xlabel('yaw');
        ylabel('roll');
        figure;
        surf(AGX, AGY, reshape(res_att(rho+1,:,:), size(res_att, 2), size(res_att, 3)));
        title('Cost Function Value pitch-roll');
        xlabel('pitch');
        ylabel('roll');
    end
end


function [cost] = calc_cost(pos_window, att_window, ...
    delta_pos_window, delta_att_window, ...
    rays, lidar_window, DTM, cellsize, lambda1, lambda2)
    model_err = Pijz_minus_mPij(pos_window, att_window, lidar_window, rays, ...
                                 DTM, cellsize);
    % Normalize and get rid of rays outside the DTM
    model_err = model_err / sum(~isnan(model_err)) * (1 - lambda1 - lambda2);
    model_err(isnan(model_err)) = 0;


    delta_pos_err = lambda1 / size(pos_window, 2) * delta_minus_delta0(pos_window, delta_pos_window);
    delta_att_err = lambda2 / size(att_window, 2) * delta_minus_delta0(att_window, delta_att_window);
    
    cost = [model_err(:); delta_pos_err(:); delta_att_err(:)];
end

function [res] = Pijz_minus_mPij(pos_window, att_window, lidar_window, rays, ...
                                 DTM, cellsize)
    Cbn_window = zeros(3*size(pos_window, 2), 3);
    for n=1:size(pos_window, 2)
        Cbn_window(3*n-2:3*n, :) = euler2dcm_v000(att_window(:, n));
    end
    Pij = calc_Pij(pos_window, Cbn_window, lidar_window, rays);
    Pij = reshape(Pij, [3, size(pos_window, 2) * size(rays, 2)]);
    res = Pij(3,:) - GetSurfaceHeight(Pij(1,:), Pij(2,:), DTM, cellsize);
end

function [Pij] = calc_Pij(pos_window, Cbn_window, lidar_window, rays)
    C = repmat(pos_window(:), [1, size(rays, 2)]);
    RsijRi = (Cbn_window * diag([1 1 -1])) * rays;
    d = zeros(size(C));
    d(1:3:end-2, :) = lidar_window';
    d(2:3:end-1, :) = lidar_window';
    d(3:3:end, :) = lidar_window';
    Pij = C + RsijRi.*d;
end

function [res] = delta_minus_delta0(v, dv0)
    dv = v(:, 2:end) - v(:, 1:end-1);
    res = dv - dv0;
    res = res(:);
end

