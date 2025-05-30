clear all
close all
clc

%% ******************* Read datas ******************
addpath('../../tools')
addpath ('../../tools/WAM/utils')
addpath('../../tools/gtsam_toolbox')
addpath("../../tools/error_ellipse");

import gtsam.*
import gpmp2.*

% =================================  
% full grid GH quadrature results 
% ================================= 
% prefix = "case1";

% =============================
% sparse GH quadrature results 
% =============================
i_exp = 2;

switch i_exp
    case 1
        prefix = "sparse_gh/case1";
        map_name = "PR2BookShelfDataset";
    case 2
        prefix = "sparse_gh/case2";
        map_name = "PR2IndustrialDataset";
end

%% ******************* Start and goal configurations ******************
% start_confs = [-1.75, 0.261, -1.57, -0.349, 0.0, -1.05, 1.57];
% end_confs = [0.0, 0.169, 0.0, -0.524, 3.14, -0.349, 0];

start_confs = [-0.524, 0.0, -0.785, -0.785, 3.14, -1.047, 0.785;
            -1.45, 0.0, -2.681, -0.61, 3.14, -0.524, -0.524];

end_confs = [-0.174, 0.785, -0.261, -0.785, 1.57, -0.349, 1.57;
            0.0, 0.171, 0.0, -0.96, 3.14, -0.785, 0.0];

%% read experiment results
means = csvread([prefix+"/mean.csv"]);
covs = csvread([prefix+"/cov.csv"]);
precisions = csvread([prefix+"/joint_precision.csv"]);
costs = csvread([prefix+"/cost.csv"]);
factor_costs = csvread([prefix+"/factor_costs.csv"]);

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

disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

%% ******************* PR2Arm ******************
arm = generateArm('PR2Arm');

% start and goal 
start_conf = start_confs(i_exp, 1:end)';
end_conf = end_confs(i_exp, 1:end)';

start_vel = zeros(7,1);
end_vel = zeros(7,1);

% % ============= plot sampled states for n iterations =============
% plotArmSamples3D(arm, vec_means, vec_covs, n_states, niters, nsteps, dataset, start_conf, end_conf);

%% ================= plot the final iteration, only mean value ===================
% ----- parameters -----
niters = find_niters(means);
dim_state = 14;
nt = size(means, 1) / dim_state;

% ----- figure settings -----

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
for j = 1:nt
    % gradual changing colors
    alpha = (j / nt)^(1.15);
    color = [0, 0, 1, alpha];
    % means
    plotArm3D(arm.fk_model(), i_means(1:7, j), color, 4, true);
end
plotArm3D(arm.fk_model(), start_conf, 'r', 6, true);
plotArm3D(arm.fk_model(), end_conf, 'g', 6, true);
hold off

%% ================= plot costs ===================
output = plot_costs(costs, factor_costs, precisions, niters, n_states, dim_state);

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
