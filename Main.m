


dtm_load();

PATH = 'C:\Users\Ophir\matlab_workspace\MRepo\ExamplePaths\SimplePath\';

ImuLidarNavigator([PATH 'mnav.bin'], [PATH 'mimu.bin'], [PATH 'mlidar.bin'], ...
    [PATH 'res.bin'], [PATH 'err.bin'], 10, [0 0 0], 1/100, DTM, cellsize);