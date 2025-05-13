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

%% ============ 
% read map
% ============
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");

dim_state = 4;
dim_theta = 2;

output_path_1 = '/home/zinuo/VIMP/matlab_helpers/ProxKL-examples/2d_pR/output_figure_1.png';
output_path_2 = '/home/zinuo/VIMP/matlab_helpers/ProxKL-examples/2d_pR/output_figure_2.png';


%% ================ 
% plot costs 
% =================

for i = 1:1 % 4 experiments
    if is_sparse
        prefix = ["sparse_gh/map2/case" + num2str(i)+"/"];
    else
        prefix = ["map2/case" + num2str(i)+"/"];
    end
    means = csvread([prefix + "mean.csv"]);
    joint_precisions = csvread([prefix + "joint_precision.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    
    factor_costs = csvread([prefix + "factor_costs.csv"]);
    
    [ttl_dim, ~] = size(means);
    niters = find_niters(means);
    n_states = floor(ttl_dim / dim_state);

    output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);

end

% exportgraphics(gcf, 'output_figure_1.png', 'Resolution', 75);

%% ================ 
% plot trajectories 
% =================
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
% width = 1200;
% height = 900;
figure
set(gcf,'position',[x0,y0,width,height])

% 4 Experiments start and goal states
start_configs = [7.0, -5, 0, 0;
                 -7, -5, 0, 0;
                 -7, -5, 0, 0;
                 13, 14, 0, 0];
goal_configs = [-10, 17, 0, 0;
                8, 18, 0, 0;
                13, 10, 0, 0;
                -13, 8, 0, 0];

tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 1:4 % 4 experiments  
    nexttile
    if is_sparse
        prefix = ["sparse_gh/map2/case" + num2str(i)+"/"];
    else
        prefix = ["map2/case" + num2str(i)+"/"];
    end
    
    % % --- high temperature ---
    means = csvread([prefix + "mean.csv"]);
    covs = csvread([prefix + "cov.csv"]);
    precisions = csvread([prefix + "precision.csv"]);
       
    niters = find_niters(means);
    
    nsteps = 1;
%     [ttl_dim, niters] = size(means);
    output = plot_planarPR_oneiter(means, covs, sdfmap, niters);
    
    start_i = start_configs(i, :)';
    goal_i = goal_configs(i, :)';
    % s1 = scatter(start_i(1), start_i(2), 200, 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 20);
    % s2 = scatter(goal_i(1), goal_i(2), 200, 'x', 'MarkerEdgeColor', 'g', 'LineWidth', 20);
    
    % lgd = legend([s1, s2], {'Start', 'Goal'});
    % set(lgd, 'FontWeight', 'bold');
    
    % axis off

    set(gca, 'FontName', 'Sans Serif', 'FontSize', 28)

    xlim([-17.5, 17.5])
    ylim([-7.5, 22.5])
    box off
    % legend('show', 'Location', 'northwest');
    
    % set(gca, 'XLimMode', 'manual', 'YLimMode', 'manual')
    
end

% exportgraphics(gcf, 'output_figure_2.pg', 'Resolution', 75);

% exportgraphics(gcf, 'Trajectory_1.pdf', 'ContentType', 'vector', 'BackgroundColor', 'none');