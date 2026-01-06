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
save_figure = 0;

%% ============ 
% read map
% ============
% sdfmap = csvread(fullfile(vimp_root, 'vimp', 'python', 'sdf_robot', 'map', 'planar', 'MultiObstacleLongRangeMap.csv'));
sdfmap = csvread(fullfile(vimp_root, 'vimp', 'python', 'sdf_robot', 'map', 'planar', 'SingleObstacleMap.csv'));
dim_state = 6;
dim_theta = 3;

%% ================ 
% plot costs 
% =================
% prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'case2');
prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'case2_300_states');

means = csvread(fullfile(prefix, 'mean.csv'));  %n_states*dim_state x niters
% joint_precisions = csvread(fullfile(prefix, 'joint_precision.csv'));  % n_states*dim_state*dim_state x niters
costs = csvread(fullfile(prefix, 'cost.csv'));  % niters x 1
factor_costs = csvread(fullfile(prefix, 'factor_costs.csv'));

[ttl_dim, ~] = size(means);
niters = find_niters(means);
n_states = floor(ttl_dim / dim_state);

% output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);
output_costplot = plot_costs_without_entropy(costs, factor_costs, niters, n_states, dim_state);
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

prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'case2_300_states');
% prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'case2');

% % --- high temperature ---
means = csvread(fullfile(prefix, 'mean.csv'));
covs = csvread(fullfile(prefix, 'cov.csv'));
precisions = csvread(fullfile(prefix, 'precision.csv'));
    
niters = find_niters(means);
nsteps = 1;
%     [ttl_dim, niters] = size(means);

% output = plot_planarPR_oneiter(means, covs, sdfmap, niters, 0.1, -50, -50);
output = plot_planarPR_oneiter(means, covs, sdfmap, niters);

% title(['iter ' num2str(niters)], 'FontSize', 20);
% xlim([-30, 60])
% ylim([-20, 50])
xlim auto
ylim auto
grid on

exportgraphics(gcf, fullfile(prefix, 'output_figure_2.png'), 'Resolution', 75);

x0 = 2500;
y0 = 500;
width = 1200;
height = 400;
figure
set(gcf,'position',[x0,y0,width,height])

prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Quad', 'case2_300_states');
norm_difference = csvread(fullfile(prefix, 'norm_differences.csv'));

plot(norm_difference, 'LineWidth', 3)
set(gca, 'FontSize', 14);
xlabel('Iterations', 'FontSize', 16);
ylabel('Norm of Difference', 'FontSize', 16);
grid minor

exportgraphics(gcf, fullfile(prefix, 'Difference_Norm.pdf'), 'ContentType', 'vector', 'BackgroundColor', 'none');
% title('Difference Between Result and the Initial Trajectory', ...
%     'FontSize', 16);