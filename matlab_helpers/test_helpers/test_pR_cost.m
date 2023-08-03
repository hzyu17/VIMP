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
dataset = generate2Ddataset_1('MultiObstacleEntropy3');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% ========== signed distance field ========== 
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

val1 = field(100, 101);
val2 = field(300, 400);

writematrix(dataset.map, '../../vimp/maps/2dpR/map0/map_multiobs_map0.csv') 
writematrix(field, '../../vimp/maps/2dpR/map0/field_multiobs_map0.csv') 

% ========== point robot model ========== 
pR = PointRobot(2,1);
spheres_data = [0  0.0  0.0  0.0  1.5];
nr_body = size(spheres_data, 1);
sphere_vec = BodySphereVector;
sphere_vec.push_back(BodySphere(spheres_data(1,1), spheres_data(1,5), ...
        Point3(spheres_data(1,2:4)')));
pR_model = PointRobotModel(pR, sphere_vec);

% ========== cost factor ========== 
cost_sigma = 0.5;
epsilon_dist = 4;

key_pos = symbol('x', 0);
obs_factor = ObstaclePlanarSDFFactorPointRobot(...
            key_pos, pR_model, sdf, cost_sigma, epsilon_dist);

% ========== test on the sdf map: signed distance and err_vec ========== 
x = [11.3321059864421, 9.16117246531728]';
x_pt = Point2(11.3321059864421, 9.16117246531728);
signed_dist = sdf.getSignedDistance(x_pt)
err_vec = obs_factor.evaluateError(x)

% ========== test on the obs factor: cost ========== 
addpath('../GaussHermite')
p_GH = 6;

% collision cost function


% Inputs
Int21_gt = err_vec*err_vec/cost_sigma/10.0


% smaller covariance: Monte-Carlo estimation
% Inputs
prec = eye(2);
cov = inv(prec);

x_mean = [11.3321059864421, 9.16117246531728]';
num_samples = 1000000;
ttl_cost = 0;
for i = 1:num_samples
    i_sample = x_mean + randn(2, 1);
    i_err_vec = obs_factor.evaluateError(i_sample);
    ttl_cost = ttl_cost + i_err_vec * i_err_vec / cost_sigma / 10.0;
end

monte_carlo = ttl_cost / num_samples

%% Test dynamics prior cost
% --------------- fixed cost ---------------
p_GH = 6;
temperature = 10.0;
mu_0 = [11.3333333333333, 9.33333333333333]';
K_0 = eye(2) .* 10000.0;

% Cost fn
x = sym('x', [2,1]);
phi_21 = transpose(x-mu_0) * K_0 * (x-mu_0) / temperature

% Inputs
Int1 = GaussHermiteN(2, phi_21, p_GH, mu_0, K_0)


