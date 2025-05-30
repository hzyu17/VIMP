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
sdfmap = csvread("../../../vimp/python/sdf_robot/map/planar/SingleObstacleMap.csv");

dim_state = 6;
dim_theta = 3;

%% ================ 
% plot costs 
% =================
for i = 2:2 % 4 experiments
    if is_sparse
        prefix = ["case" + num2str(i)+"/"];
    else
        prefix = ["map2/case" + num2str(i)+"/"];
    end
    means = csvread([prefix + "mean.csv"]);  %n_states*dim_state x niters
    joint_precisions = csvread([prefix + "joint_precision.csv"]);  % n_states*dim_state*dim_state x niters
    costs = csvread([prefix + "cost.csv"]);  % niters x 1
    
    factor_costs = csvread([prefix + "factor_costs.csv"]);
    
    [ttl_dim, ~] = size(means);
    niters = find_niters(means);
    n_states = floor(ttl_dim / dim_state);

    output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);
    % output_costplot = plot_costs_quadrotor(costs, factor_costs, joint_precisions, niters, n_states, dim_state, save_figure);

end

%% ================ 
% plot trajectories 
% =================
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
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
for i = 2:2 % 4 experiments  
    nexttile
    if is_sparse
        % prefix = ["case" + num2str(i)+"/"];
        prefix = "Iter_GVIMP/";
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
    
    % start_i = start_configs(i, :)';
    % goal_i = goal_configs(i, :)';
    % s1 = scatter(start_i(1), start_i(2), 200, 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 500);
    % s2 = scatter(goal_i(1), goal_i(2), 200, 'x', 'MarkerEdgeColor', 'g', 'LineWidth', 500);
    
    % lgd = legend([s1, s2], {'Start', 'Goal'});
    % set(lgd, 'FontWeight', 'bold');
    
    xlim([-30, 60])
    ylim([-20, 50])
    
    grid on
    % axis off
    
end


% prefix = ["case2" + "/"];
% norm_difference = csvread([prefix + "norm_differences.csv"]);
% figure
% plot(norm_difference, 'LineWidth', 2)



% exportgraphics(gcf, strcat('Trajectory2_', num2str(niters), '_150_point.pdf'), 'ContentType', 'image');



% figure
% tiledlayout(2, 4, 'TileSpacing', 'tight', 'Padding', 'tight')
% i = 1;
% nexttile
% if is_sparse
%     prefix = ["case" + num2str(i)+"/"];
% else
%     prefix = ["map2/case" + num2str(i)+"/"];
% end

% sample = csvread([prefix + "samples.csv"]);
% control = csvread([prefix + "controls.csv"]);
% means = csvread([prefix + "zk_sdf.csv"]);


% hold on
% plot(sample(1,:))
% plot(means(1,:), "r")
% nexttile
% hold on
% plot(sample(2,:))
% plot(means(2,:), "r")
% nexttile
% hold on
% plot(sample(3,:))
% plot(means(3,:), "r")
% nexttile
% hold on
% plot(sample(4,:))
% plot(means(4,:), "r")
% nexttile
% hold on
% plot(sample(5,:))
% plot(means(5,:), "r")
% nexttile
% hold on
% plot(sample(6,:))
% plot(means(6,:), "r")
% nexttile
% plot(control(1,:))
% nexttile
% plot(control(2,:))
