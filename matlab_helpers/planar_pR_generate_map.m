clear all
clc

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
dataset = generate2Ddataset_1('MultiObstacleEntropy2');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% plot sdf
% figure(2)
% plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field')

plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);

% % save map
% writematrix(dataset.map, '../vimp/data/2d_pR/map_multiobs_entropy.csv') 
% writematrix(field, '../vimp/data/2d_pR/field_multiobs_entropy.csv') 

writematrix(dataset.map, '../vimp/data/2d_pR/map_multiobs_entropy_map2.csv') 
writematrix(field, '../vimp/data/2d_pR/field_multiobs_entropy_map2.csv') 

