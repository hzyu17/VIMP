%% Plot the results for planar robot in the paper

close all
clear all
clc

import gtsam.*
import gpmp2.*

addpath("../../");
addpath("../../tools");
addpath('../../tools/gtsam_toolbox');
addpath("../../tools/error_ellipse");
        
is_sparse = 1;
save_figure = 0;

%% ============ 
% read map
% ============
sdfmap = csvread("../../../vimp/python/sdf_robot/map/planar/MultiObstacleLongRangeMap.csv");
% sdfmap = csvread("../../../vimp/python/sdf_robot/map/planar/SingleObstacleMap.csv");
dim_state = 6;
dim_theta = 3;


%% ================ 
% plot costs 
% =================
% prefix = ["case2"+"/"];
prefix = ["case2_300_states"+"/"];
means = csvread([prefix + "mean.csv"]);  %n_states*dim_state x niters
joint_precisions = csvread([prefix + "joint_precision.csv"]);  % n_states*dim_state*dim_state x niters
costs = csvread([prefix + "cost.csv"]);  % niters x 1

factor_costs = csvread([prefix + "factor_costs.csv"]);

[ttl_dim, ~] = size(means);
niters = find_niters(means);
n_states = floor(ttl_dim / dim_state);

output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);
% output_costplot = plot_costs_without_entropy(costs, factor_costs, niters, n_states, dim_state);

exportgraphics(gcf, fullfile(prefix, 'output_figure_1.png'), 'Resolution', 75);


%% ================ 
% plot trajectories 
% =================
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
prefix = ["case2_300_states"+"/"];
% prefix = ["case2"+"/"];
% % --- high temperature ---
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precision.csv"]);
    
niters = find_niters(means);

nsteps = 1;
%     [ttl_dim, niters] = size(means);
output = plot_planarPR_oneiter(means, covs, sdfmap, niters, 0.1, -50, -50);

% title(['iter ' num2str(niters)], 'FontSize', 20);


% xlim([-30, 60])
% ylim([-20, 50])

xlim auto
ylim auto

grid on

exportgraphics(gcf, fullfile(prefix, 'output_figure_2.png'), 'Resolution', 75);


% prefix = ["case2_300_states"+"/"];
% norm_difference = csvread([prefix + "norm_differences.csv"]);
% figure
% plot(norm_difference, 'LineWidth', 2)