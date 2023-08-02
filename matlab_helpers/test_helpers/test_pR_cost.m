%% test_pR_cost.m
% Author: Hongzhe Yu
% Introduction: help generate the ground truth for debugging the obs cost estimators
% corresponding to the test in vimp/instances/tests/test_pR_cost.cpp

addpath('../tools/2dpR')
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

% debug obs cost and sdf
clear all
clc 
dataset = generate2Ddataset_1('MultiObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

val1 = field(100, 101)
val2 = field(300, 400)

writematrix(dataset.map, '../../vimp/maps/2dpR/map_multiobs.csv') 
writematrix(field, '../../vimp/maps/2dpR/field_multiobs.csv') 

% point robot model
pR = PointRobot(2,1);
spheres_data = [0  0.0  0.0  0.0  1];
nr_body = size(spheres_data, 1);
sphere_vec = BodySphereVector;
sphere_vec.push_back(BodySphere(spheres_data(1,1), spheres_data(1,5), ...
        Point3(spheres_data(1,2:4)')));
pR_model = PointRobotModel(pR, sphere_vec);

% cost factor
cost_sigma = 0.5;
epsilon_dist = 4;

key_pos = symbol('x', 0);
obs_factor = ObstaclePlanarSDFFactorPointRobot(...
            key_pos, pR_model, sdf, cost_sigma, epsilon_dist);

x = [11.3321059864421, 9.16117246531728]';
x_pt = Point2(11.3321059864421, 9.16117246531728);
x1 = [0;0];
signed_dist = sdf.getSignedDistance(x_pt);
err_vec = obs_factor.evaluateError(x)

