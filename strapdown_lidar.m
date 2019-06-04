function [new_pos_window, new_Cbn_window] = strapdown_lidar(...
    pos_window, Cbn_window, delta_pos_window, delta_att_window, ...
    rays, lidar_window, DTM, cellsize, bound)
%UNTITLED5 Summary of this function goes here
%   Detailed explanation goes here
    lambda1 = 0.1;
    lambda2 = 0.2;

    att_window = zeros(size(pos_window));
    for n=1:size(pos_window, 2)
        att_window(:, n) = dcm2euler_v000(Cbn_window(:, :, n));
    end
    
    cost = @(x)calc_cost(reshape(x(1:length(x)/2), [3, length(x) / 6]), ...
                         reshape(x(length(x)/2+1:end), [3, length(x) / 6]), ...
                         delta_pos_window, delta_att_window, ...
                         rays, lidar_window, DTM, cellsize, lambda1, lambda2);

    x0 = [pos_window(:); att_window(:)];
    options = optimoptions('lsqnonlin', 'Display', 'off',...
                           'OptimalityTolerance', 1e-10,...
                           'FunctionTolerance', 1e-10,...
                           'StepTolerance', 1e-10, 'Algorithm', 'levenberg-marquardt');
    lb = [pos_window(:)-repmat(bound, [size(pos_window, 2) 1]); att_window(:)-0];
    ub = [pos_window(:)+repmat(bound, [size(pos_window, 2) 1]); att_window(:)+0];
    resmin = inf;
    xmin = x0;
%     x0 + (rand(size(x0))-0.5).*10.*repmat([bound' 0 0 0]'
    for n=1:1
        [x,resnorm] = lsqnonlin(cost, x0, [], [], options);
    %     fprintf('Square Error: %d (Initial: %d)\n', resnorm, sum(cost(x0).^2));
        if resnorm < resmin
            xmin = x;
            remin = resnorm;
        end
    end
    
    new_pos_window = reshape(xmin(1:length(xmin)/2), [3, length(xmin) / 6]);
    new_att_window = reshape(xmin(length(xmin)/2+1:length(xmin)), [3, length(xmin) / 6]);
    
    new_Cbn_window = zeros(size(Cbn_window));
    for n=1:size(new_Cbn_window, 3)
        new_Cbn_window(:, :, n) = euler2dcm_v000(new_att_window(:, n));
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

