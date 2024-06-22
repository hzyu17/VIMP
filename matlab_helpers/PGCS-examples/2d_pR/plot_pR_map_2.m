clear all
close all
clc

addpath("../../tools/error_ellipse/");
addpath("../../../matlab_helpers/");
addpath("../../tools/gtsam_toolbox/");

import gtsam.*
import gpmp2.*



%% read map
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");

% plotting
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')

% 4 Experiments start and goal states
start_configs = [7.0, -5, 0, 0;
                 -7, -5, 0, 0;
                 -7, -5, 0, 0;
                 13, 14, 0, 0];
goal_configs = [-10, 17, 0, 0;
                8, 18, 0, 0;
                13, 10, 0, 0;
                -12.0, 7.0, 0, 0];

for i = 1:4 % 4 experiments
    nexttile
    hold on
    prefix = ["map2/case"+num2str(i)+"/"]
    % % --- read means and covariances ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    cov_final = covs(:,50);
    disp("cov_final_RESHAPED")
    cov_final_RESHAPED = reshape(cov_final, [4,4]);
    
    plot_2d_result(sdfmap, means, covs);

    start_i = start_configs(i, :)';
    goal_i = goal_configs(i, :)';
    s1 = scatter(start_i(1), start_i(2), 200, 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 500);
    s2 = scatter(goal_i(1), goal_i(2), 200, 'x', 'MarkerEdgeColor', 'g', 'LineWidth', 500);
    
    lgd = legend([s1, s2], {'Start', 'Goal'});
    set(lgd, 'FontWeight', 'bold');
    
    xlim([-15, 20])
    ylim([-10, 20])
    
    axis off

end
