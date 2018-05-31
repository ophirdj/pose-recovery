% fileID = fopen('betoren_20161007\dtmz_raw.asc','r');
% DTM = fscanf(fileID,'%f', [520, 600])';
% fclose(fileID);
load('DTM.mat');
cellsize = 25;
NODATA_value = -9999;
