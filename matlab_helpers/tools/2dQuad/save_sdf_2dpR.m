%% Generate and save map and field files for 2d point robot

clear all
close all
clc

addpath('../gtsam_toolbox')
import gtsam.*
import gpmp2.*

disp("========== map0 ========== ")
dataset = generate2Ddataset_1('MultiObstacleDataset_Quad'); % Get the map with 1 to be the obstacle
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

writematrix(dataset.map, '../../../vimp/maps/2dQuad/map_multiobs.csv') 
writematrix(field, '../../../vimp/maps/2dQuad/field_multiobs.csv') 

figure
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title("Map0")
