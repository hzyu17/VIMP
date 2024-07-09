clear all
close all
clc

addpath("../../../tools/2d_pR")
import gtsam.*
import gpmp2.*

dataset = generate2Ddataset_1('MultiObstacleLongRangeDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% plot sdf
figure(2)
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title('Map')

%%
% save field
csvwrite("field_long_range.csv", field);

% save map
csvwrite("map_long_range.csv", dataset.map);
