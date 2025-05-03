function output = plot_configuration_obstacles(map)
%PLOT_CONFIGURATION_OBSTACLES Plot configuration space obstacles for a
%2-link arm and a 2-D sdf, using a simple swipping throught the joint
%angles.
% Hongzhe Yu

addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

output = 1;

% sdf_map
dataset = generate2Ddataset('OneObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% arm
arm = generateArm('SimpleTwoLinksArm');

epsilon = 10.0;
cost_sigma = 0.1;

% planar collision factor arm
planar_sdf_arm = ObstaclePlanarSDFFactorArm(symbol('x', 0), arm, sdf, cost_sigma, epsilon);

% swip for joint angles
v_theta1 = [];
v_theta2 = [];
theta_cell = linspace(-3.1415926, 3.1415926, 100);
for theta_1 = theta_cell(1:end)
    for theta_2 = theta_cell(1:end)
        config_ij = [theta_1; theta_2];
        err_vec = planar_sdf_arm.evaluateError(config_ij);
        dist_vec = epsilon - err_vec;
        if min(dist_vec) < 0.0
            v_theta1 = [v_theta1, theta_1];
            v_theta2 = [v_theta2, theta_2];
        end
    end
end

gca;

size_theta = size(v_theta1, 2);

htmlGray = [190 190 190]/255;

for i_pt = 1:size_theta
    scatter(v_theta1(i_pt), v_theta2(i_pt), 15, htmlGray,'fill','Marker', 's');
end

end

