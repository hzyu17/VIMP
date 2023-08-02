%% Generate and save map and field files for 2d point robot

clear all
close all
clc

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath('../../tools/2dpR')


% ========== map0 ========== 
dataset = generate2Ddataset_1('OneObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

origin_x = dataset.origin_x
origin_y = dataset.origin_y
cell_size = dataset.cell_size

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

writematrix(dataset.map, '../../../vimp/maps/2dpR/map0/map_multiobs_map0.csv') 
writematrix(field, '../../../vimp/maps/2dpR/map0/field_multiobs_map0.csv') 

figure
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title("Map0")


% ========== map1 ========== 
dataset = generate2Ddataset_1('MultiObstacleEntropy1');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

origin_x = dataset.origin_x
origin_y = dataset.origin_y
cell_size = dataset.cell_size

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

writematrix(dataset.map, '../../../vimp/maps/2dpR/map1/map_multiobs_map1.csv') 
writematrix(field, '../../../vimp/maps/2dpR/map1/field_multiobs_map1.csv') 

figure
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title("Map1")


% ========== map2 ========== 
dataset = generate2Ddataset_1('MultiObstacleEntropy2');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

origin_x = dataset.origin_x
origin_y = dataset.origin_y
cell_size = dataset.cell_size

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

writematrix(dataset.map, '../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv') 
writematrix(field, '../../../vimp/maps/2dpR/map2/field_multiobs_map2.csv') 

figure
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title("Map2")


% ========== map3 ========== 
dataset = generate2Ddataset_1('MultiObstacleEntropy3');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

origin_x = dataset.origin_x
origin_y = dataset.origin_y
cell_size = dataset.cell_size

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

writematrix(dataset.map, '../../../vimp/maps/2dpR/map3/map_multiobs_map3.csv') 
writematrix(field, '../../../vimp/maps/2dpR/map3/field_multiobs_map3.csv') 

figure
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title("Map3")
