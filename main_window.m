dtm_load();

PATHS = {...
%         'C:\Users\Ophir\matlab_workspace\trajectories\circle1\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\circle10\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\circle30\',...
%         'C:\Users\Ophir\matlab_workspace\trajectories\circle100\',...
        'C:\Users\Ophir\matlab_workspace\trajectories\circle100_20\',...
        };

for k = 1:length(PATHS)

for window = 20:-1:2
PATH = [PATHS{k} sprintf('window_%d/', window)];
if ~isdir(PATH)
    mkdir(PATH);
end
    
out_err = [PATH 'err.bin'];
out_res = [PATH 'res.bin'];
in_mnav = [PATH 'mnav.bin'];

% Ground Truth
ImuLidarNavigator([PATHS{k} 'mnav.bin'], [PATHS{k} 'mimu.bin'], [PATHS{k} 'mlidar.bin'], ...
    [PATHS{k} 'meta.bin'], [PATH 'res.bin'], [PATH 'err.bin'], window, DTM);
close all;

end
end
