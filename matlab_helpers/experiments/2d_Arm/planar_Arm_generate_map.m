clear all
clc

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
dataset = generate2Ddataset_arm('MultiObstacleDatasetArm');
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

plotEvidenceMap2D_arm(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);

%% save map
% writematrix(dataset.map, 'maps/map_one_obs.csv') 
% writematrix(field, 'maps/field_one_obs.csv') 

writematrix(dataset.map, 'maps/map_two_obs.csv') 
writematrix(field, 'maps/field_two_obs.csv') 

