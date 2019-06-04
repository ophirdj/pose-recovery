PATH = 'C:\Users\Ophir\matlab_workspace\trajectories\Curve100_20\';

tru=readbin_v000([PATH 'mnav.bin'],10);
% res=readbin_v000([PATH 'res.bin'],10);

% Thin out result for printing
tru=tru(:,1:25:end);
% res=res(:,1:25:end);

pos_tru=tru(2:4,:);
att_tru=tru(8:10,:);
Cbn_tru = zeros(3*size(att_tru, 2), 3);
for n=1:size(att_tru, 2)
    Cbn_tru(3*n-2:3*n, :) = euler2dcm_v000(att_tru(:, n)) * diag([1 1 -1]);
end
% pos_res=res(2:4,:);
% ori_res=euler2dcmz(res(5:7,:));

[X, Y] = meshgrid((1:size(DTM, 2))*cellsize, (1:size(DTM, 1))*cellsize);

figure;
contour(X, Y, DTM);
hold on;
scatter(pos_tru(1,:),pos_tru(2,:),'.');
% quiver(pos_tru(1,:),pos_tru(2,:),ori_tru(1,:),ori_tru(2,:));
% quiver3(pos_tru(1,:),pos_tru(2,:), pos_tru(3,:),Cbn_tru(1:3:end,3)',Cbn_tru(2:3:end,3)',Cbn_tru(3:3:end,3)');
% quiver(pos_res(1,:),pos_res(2,:),ori_res(1,:),ori_res(2,:));
% quiver3(pos_res(1,:),pos_res(2,:), pos_res(3,:),ori_res(1,:),ori_res(2,:),ori_res(3,:));
grid;
pbaspect([1 1 1]);
daspect([1 1 0.1]);
title('Filght Path');
xlabel('X offset (meters)');
ylabel('Y offset (meters)');
legend('Terrain','Path');