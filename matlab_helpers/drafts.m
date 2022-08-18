clear all
clc

% %% draft for debugging
% precision  = csvread("../vimp/precision.csv");
% cov = csvread("../vimp/cov.csv");
% 
% 
% %%
% diff = inv(precision) - cov;
% norm(diff);
% 
% true_cov = inv(precision);
% writematrix(true_cov, "cov_expected.csv");

% %% matrices during the iterations
% 
% for i = 0:6
% %     name = "../vimp/data/debug/Vddmu_"+num2str(i)+".csv";
% %     Vddmu = csvread(name)
% %     name = "../vimp/data/debug/j_Vddmu_"+num2str(i)+".csv"
% %     j_Vddmu = csvread(name)
% %     name = "../vimp/data/debug/dprecision_"+num2str(i)+".csv"
% %     dprecision =  csvread(name)
%     name = "../vimp/data/debug/dmu_"+num2str(i)+".csv";
%     dmu =  csvread(name);
%     name = "../vimp/data/debug/precision_"+num2str(i)+".csv";
%     precision =  csvread(name);
%     name = "../vimp/data/debug/cov_"+num2str(i)+".csv";
%     cov=  csvread(name);
%     aa = 1;
% end

%% test gpmp2 functions
% Load libraries
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
dataset = generate2Ddataset('MultiObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% % plot sdf
% figure(2)
% plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field')

%% debug obs cost and sdf
clear all
clc 
dataset = generate2Ddataset('MultiObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

val1 = field(100, 101)
val2 = field(300, 400)

writematrix(dataset.map, '../vimp/data/2d_pR/map_multiobs.csv') 
writematrix(field, '../vimp/data/2d_pR/field_multiobs.csv') 

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
err = obs_factor.evaluateError(x)

%% debug nan cost
clear all
clc

mean_2_read = csvread("../vimp/data/debug/mean2.csv");
precision_2_read = csvread("../vimp/data/debug/precision_2.csv");
cov_2_read = csvread("../vimp/data/debug/cov_2.csv");
cov_2 = inv(precision_2_read);

precision_2 = inv(cov_2_read);
norm(precision_2 - precision_2_read);
norm(cov_2 - cov_2_read);

% plot the process factor costs
factor_costs = csvread("../vimp/data/2d_pR/factor_costs.csv");
figure
hold on
grid on
for i_iter = 1:size(factor_costs, 2)
    plot(factor_costs(1:end, i_iter), 'LineWidth', 2)
end
ylim([0, 100])
legend({"1","2","3","4","5","6"})


%% Arm plotting
clear all 
clc
import gtsam.*
import gpmp2.*

dataset = generate2Ddataset('OneObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% arm model
arm = generateArm('SimpleTwoLinksArm');

% start and end conf
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';

% plot start / end configuration
figure(1), hold on
plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
title('Layout')
plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
hold off


%% see the psd of the d_precision and precision matrix
precision_0_read = csvread("../vimp/data/debug/precision_0.csv");
precision_1_read = csvread("../vimp/data/debug/precision_1.csv");
dprecision_0_read = csvread("../vimp/data/debug/dprecision_0.csv");

eigs_prec0 = eig(precision_0_read);
eigs_dprec0 = eig(dprecision_0_read);
eigs_prec1 = eig(precision_1_read);


%% purturbation
purturb_precision = csvread("../vimp/data/2d_pR/purturbed.csv");
issymmetric(purturb_precision)
norm(purturb_precision - purturb_precision', 'fro')
eig_purturbation = eig(purturb_precision)


%% statistics of purturbed cost
purturb_stat= csvread("../vimp/data/2d_pR/purturbation_statistics.csv");
final_cost = csvread("../vimp/data/2d_pR/final_cost.csv");
final_cost = final_cost(1);
diff_purturb_stat = purturb_stat - final_cost;

figure
hold on
grid on
plot(diff_purturb_stat)


%% test surf the hinge loss
clear all
clc 
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
dataset = generate2Ddataset('MultiObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

grid_rows = size(dataset.map, 1);
grid_cols = size(dataset.map, 2);
grid_corner_x = dataset.origin_x + (grid_cols-1)*cell_size;
grid_corner_y = dataset.origin_y + (grid_rows-1)*cell_size;
grid_X = dataset.origin_x : cell_size : grid_corner_x;
grid_Y = dataset.origin_y : cell_size : grid_corner_y;

writematrix(grid_X, "../vimp/data/sdf_grid_x.csv");
writematrix(grid_Y, "../vimp/data/sdf_grid_y.csv");
mesh_hingeloss = csvread("../vimp/data/mesh_hingeloss.csv");

mesh_X = repmat(grid_X', 1, size(grid_Y,2));
mesh_Y = repmat(grid_Y, size(grid_X,2), 1);
mesh(mesh_X, mesh_Y, mesh_hingeloss)
