clear all
close all
clc

%% ******************* Read datas ******************
addpath('../../tools')
addpath ('../../tools/WAM/utils')
addpath('../../tools/gtsam_toolbox')
addpath("../../tools/error_ellipse")
addpath("../../../vimp/thirdparty/yaml")

import gtsam.*
import gpmp2.*

% =============================
% sparse GH quadrature results 
% =============================
matlabData = yaml.loadFile("../../../vimp/scripts/hardware_experiment/config/config.yaml");

i_exp = 2;

switch i_exp
    case 1
        prefix = "sparse_gh/case1";
        map_name = "FrankaDeskDataset";
    case 2
        prefix = "../../../vimp/" + matlabData.Planning.saving_prefix;
        % map_name = "FrankaBoxDataset";
        map_name = "FrankaBoxDatasetOffset";
end

%% ******************* Start and goal configurations ******************
% start_confs = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785];
% start_confs = [0.0, -0.785, 0.0, -2.356, 0.0, 0, 0.785];
% start_confs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.157, 0.785];
% start_confs = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];

% end_confs = [0.22, 0.22, 0.7, -1.673, -0.157, 1.8, 1.74];
% end_confs = [-1.73, 1.5, -2.56, -1.65, 1.027, 0.611, 1.911];

start_confs = [
    -0.351, -0.521, 2.436, -1.402, 0.315, 1.783, -1.824;
    -1.691, -1.326, 2.048, -1.781, 1.105, 2.478, -1.340
];

end_confs = [
    -0.602, 0.721, 0.832, -0.908, -0.511, 1.457, 0.811;
    -1.848, 0.970, 0.832, -2.228, 2.033, 2.236, 0.637
];

%% read experiment results
means = csvread([prefix+"/mean.csv"]);
covs = csvread([prefix+"/cov.csv"]);
% precisions = csvread([prefix+"/joint_precision.csv"]);
costs = csvread([prefix+"/cost.csv"]);
factor_costs = csvread([prefix+"/factor_costs.csv"]);
good_means = csvread([prefix+"/good_zk.csv"]);

%% ******************* Define parameters ******************
% ----- parameters -----
[ttl_dim, niters] = size(means);
dim_theta = 14;
dim_conf = 14 / 2;

nsteps = 4;

% niters
niters = length(costs);
for i=niters:-1:1
    if costs(i) ~= 0
        niters=i;
        break
    end
end
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

% ============= clean up the raw mean and covariance data to collectors =============
[vec_means, vec_covs] = get_vec_means_covs(means, covs, niters, nsteps, dim_theta);

%% ******************* Define map dataset ******************
dataset = generate3Ddataset_1(map_name);
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% disp('calculating signed distance field ...');
% field = signedDistanceField3D(dataset.map, dataset.cell_size);
% disp('calculating signed distance field done');

%% ******************* PR2Arm ******************
arm = generatePandaArm('PandaArm');

% start and goal 

start_conf = start_confs(i_exp, 1:end)';
end_conf = end_confs(i_exp, 1:end)';

start_vel = zeros(7,1);
end_vel = zeros(7,1);

% % ============= plot sampled states for n iterations =============
% plotArmSamples3D(arm, vec_means, vec_covs, n_states, niters, nsteps, dataset, start_conf, end_conf);

% ================= plot the final iteration, only mean value ===================
% ----- parameters -----
niters = find_niters(means);
dim_state = 14;
nt = size(means, 1) / dim_state;

% % ----- figure settings -----

% x0 = 50;
% y0 = 50;
% width = 400;
% height = 350;
% figure
% set(gcf,'position',[x0,y0,width,height])
% tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')

% nexttile
% t=title(['Supported state mean values']);
% t.FontSize = 14;

x0 = 1000;
y0 = 500;
width = 1600;
height = 1400;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')

nexttile
t=title(['Iteration ', num2str(niters)]);
t.FontSize = 16;
i_means = means(:, niters);
i_means = reshape(i_means, [dim_state, nt]);
i_covs = covs(:, niters);
i_covs = reshape(i_covs, [dim_state, dim_state, nt]);

hold on 
view(3)
plotMap3D(dataset.corner_idx, origin, cell_size);

[Ux, Uy, Uz] = sphere(20);

for j = 1:nt
    % gradual changing colors
    alpha = (j / nt)^(1.15);
    color = [0, 0, 1, alpha];
    mean = i_means(1:7, j);
    mean_resample = good_means(:, j);

    if ~isequal(mean_resample, mean)
        color = [1, 0, 0, alpha];  % Change color to red if not equal
        plotArm3D(arm.fk_model(), mean_resample, color, 4, true);
        color = [0, 0, 1, alpha/3];
        % centers = arm.sphereCentersMat(mean_resample);
        % plot3(centers(1,:), centers(2,:), centers(3,:), 'g.', 'MarkerSize', 25);
    end
    plotArm3D(arm.fk_model(), mean, color, 4, true);

    % centers = arm.sphereCentersMat(mean_resample);
    % plot3(centers(1,:), centers(2,:), centers(3,:), 'g.', 'MarkerSize', 25);
end

plotArm3D(arm.fk_model(), start_conf, 'r', 6, true);
plotArm3D(arm.fk_model(), end_conf, 'g', 6, true);

centers = arm.sphereCentersMat(start_conf);
center_display = centers';
plot3(centers(1,:), centers(2,:), centers(3,:), 'g.', 'MarkerSize', 25);

for k = 1:size(centers, 2)
    r = arm.sphere_radius(k-1);

    % Scale and translate the unit sphere
    X = centers(1,k) + r * Ux;
    Y = centers(2,k) + r * Uy;
    Z = centers(3,k) + r * Uz;

    % Plot a semi-transparent sphere on the same axes
    surf(X, Y, Z, ...
         'FaceColor', 'r', ...
         'FaceAlpha', 0.2, ...    % 20% transparency
         'EdgeColor', 'none');    % No grid lines
end

centers = arm.sphereCentersMat(end_conf);
center_display = centers';
plot3(centers(1,:), centers(2,:), centers(3,:), 'g.', 'MarkerSize', 25);

for k = 1:size(centers, 2)
    r = arm.sphere_radius(k-1);

    % Scale and translate the unit sphere
    X = centers(1,k) + r * Ux;
    Y = centers(2,k) + r * Uy;
    Z = centers(3,k) + r * Uz;

    % Plot a semi-transparent sphere on the same axes
    surf(X, Y, Z, ...
         'FaceColor', 'r', ...
         'FaceAlpha', 0.2, ...    % 20% transparency
         'EdgeColor', 'none');    % No grid lines
end

% axis equal
hold off

% %% ================= plot costs ===================
% output = plot_costs(costs, factor_costs, precisions, niters, n_states, dim_state);

% %% ==== plot iterations ==== 
% x0 = 500;
% y0 = 500;
% width = 1000;
% height = 550;
% figure
% set(gcf,'position',[x0,y0,width,height])

% tiledlayout(floor(niters/5), 5, 'TileSpacing', 'tight', 'Padding', 'tight')
% for i_iter = 1:2:niters
%     nexttile
%     t=title(['Iteration ', num2str(i_iter)]);
%     t.FontSize = 16;
%     i_means = means(:, i_iter);
%     i_means = reshape(i_means, [dim_state, nt]);
%     i_covs = covs(:, i_iter);
%     i_covs = reshape(i_covs, [dim_state, dim_state, nt]);

%     hold on 
%     view(3)
%     plotMap3D(dataset.corner_idx, origin, cell_size);
%     for j = 1:nt
%         % gradual changing colors
%         alpha = (j / nt)^(1.15);
%         color = [0, 0, 1, alpha];
%         % means
%         plotArm3D(arm.fk_model(), i_means(1:7, j), color, 4, true);
%     end
%     plotArm3D(arm.fk_model(), start_conf, 'r', 6, true);
%     plotArm3D(arm.fk_model(), end_conf, 'g', 6, true);
% end


% %% ==== plot sampled covariance for the supported states seperately ==== 
% x0 = 500;
% y0 = 500;
% width = 600;
% height = 350;
% figure
% set(gcf,'position',[x0,y0,width,height])
% 
% tiledlayout(3, 5, 'TileSpacing', 'tight', 'Padding', 'tight')
% 
% n_samples = 50;
% for j = 1:1:n_states
%     nexttile
%     t = title(['Support State ',num2str(j)]);
%     t.FontSize = 16;
%     hold on 
%     view(3)
%     % plot map
%     plotMap3D(dataset.corner_idx, origin, cell_size);
% 
%     % ------------------- sampling ------------------- 
%     % gradual changing colors
%     color = [0, 0, 1, 0.9];
%     color_sample = [0.0, 0.0, 0.7, 0.04];
% 
%     i_vec_means = vec_means{nsteps};
%     i_vec_covs = vec_covs{nsteps};
%     % mu j
%     mean_j = i_vec_means{j};
%     % cov j
%     cov_j = i_vec_covs{j};
%     rng('default')  % For reproducibility
%     samples = mvnrnd(mean_j, cov_j(1:7, 1:7), n_samples);
%     plotArm3D(arm.fk_model(), mean_j, color, 4, true);
%     for k = 1: size(samples, 1)
%         k_sample = samples(k, 1:end)';
%         plotArm3D(arm.fk_model(), k_sample, color_sample, 3, false);
%     end
% 
% plotArm3D(arm.fk_model(), start_conf, 'r', 4, true);
% plotArm3D(arm.fk_model(), end_conf, 'g', 4, true);
% 
% end
