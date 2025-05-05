% Example to construct sdf from an occupancy map and save it to binary file.

close all
clear

occupancy_map_file = 'example_occupancy_map.mat';
sdf_save_file = 'DrawerExample.bin';

% addpath('utils')
%% dataset
SaveSDFFromOccMapFile(occupancy_map_file, sdf_save_file);
