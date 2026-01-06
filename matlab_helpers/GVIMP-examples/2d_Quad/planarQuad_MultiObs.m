%% Plot the results for planar robot in the paper
close all
clear all
clc
import gtsam.*
import gpmp2.*

%% ******************* Setup paths ******************
vimp_root = setup_vimp();
matlab_helpers = fullfile(vimp_root, 'matlab_helpers');
addpath(matlab_helpers);
addpath(fullfile(matlab_helpers, 'tools'));
addpath(fullfile(matlab_helpers, 'tools', 'gtsam_toolbox'));
addpath(fullfile(matlab_helpers, 'tools', 'error_ellipse'));

is_sparse = 1;
%% ============ 
% read map
% ============
sdfmap = csvread(fullfile(vimp_root, 'vimp', 'python', 'sdf_robot', 'map', 'planar', 'MultiObstacleMap.csv'));
dim_state = 6;
dim_theta = 3;
save_figure = 0;

%% ================ 
% plot costs 
% =================
prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'Multi_Obs', 'Go_around');

means = csvread(fullfile(prefix, 'mean.csv'));
joint_precisions = csvread(fullfile(prefix, 'joint_precision.csv'));
costs = csvread(fullfile(prefix, 'cost.csv'));
factor_costs = csvread(fullfile(prefix, 'factor_costs.csv'));

[ttl_dim, ~] = size(means);
niters = find_niters(means);
n_states = floor(ttl_dim / dim_state);

output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);
% output_costplot = plot_costs_quadrotor(costs, factor_costs, joint_precisions, niters, n_states, dim_state, save_figure);

%% ================ 
% plot trajectories 
% =================
x0 = 800;
y0 = 500;
width = 1290.427199;
% width = 600;
height = 800;
% height = 750;
figure
set(gcf,'position',[x0,y0,width,height])

% 4 Experiments start and goal states
% start_configs = [7.0, -5, 0, 0;
%                  -7, -5, 0, 0;
%                  -7, -5, 0, 0;
%                  13, 14, 0, 0];
% goal_configs = [-10, 17, 0, 0;
%                 8, 18, 0, 0;
%                 13, 10, 0, 0;
%                 -13, 8, 0, 0];

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
nexttile

prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'Multi_Obs');

% % --- high temperature ---
means = csvread(fullfile(prefix, 'mean.csv'));
covs = csvread(fullfile(prefix, 'cov.csv'));
precisions = csvread(fullfile(prefix, 'precision.csv'));
    
niters = find_niters(means);
nsteps = 1;
%     [ttl_dim, niters] = size(means);
output = plot_planarPR_oneiter(means, covs, sdfmap, niters);

% start_i = start_configs(i, :)';
% goal_i = goal_configs(i, :)';
% s1 = scatter(start_i(1), start_i(2), 200, 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 500);
% s2 = scatter(goal_i(1), goal_i(2), 200, 'x', 'MarkerEdgeColor', 'g', 'LineWidth', 500);
% lgd = legend([s1, s2], {'Start', 'Goal'});
% set(lgd, 'FontWeight', 'bold');

xlim([-30, 60])
ylim([-20, 50])
grid on
% axis tight
% axis off
   
% exportgraphics(gcf, strcat('Trajectory2_', num2str(niters), '_150_point.pdf'), 'ContentType', 'image');