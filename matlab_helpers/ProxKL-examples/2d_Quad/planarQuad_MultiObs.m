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
sdfmap = csvread("../../../vimp/python/sdf_robot/map/planar/MultiObstacleMap.csv");

dim_state = 6;
dim_theta = 3;

save_figure = 0;

%% ================ 
% plot costs 
% =================

% prefix = "Multi_Obs/Go_through/";
prefix = "Multi_Obs/Go_around/";

means = csvread([prefix + "mean.csv"]);
joint_precisions = csvread([prefix + "joint_precision.csv"]);
costs = csvread([prefix + "cost.csv"]);

factor_costs = csvread([prefix + "factor_costs.csv"]);

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

tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight')

nexttile
prefix = "Multi_Obs/Go_through/";

% % --- high temperature ---
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precision.csv"]);
    
niters = find_niters(means);

nsteps = 1;
output = plot_planarPR_oneiter(means, covs, sdfmap, niters);

xlim([-30, 60])
ylim([-20, 50])

grid on

nexttile
prefix = "Multi_Obs/Go_around/";

% % --- high temperature ---
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precision.csv"]);
    
niters = find_niters(means);

nsteps = 1;
output = plot_planarPR_oneiter(means, covs, sdfmap, niters);

xlim([-30, 60])
ylim([-20, 50])

grid on

% axis tight
% axis off

   

% exportgraphics(gcf, strcat('Trajectory2_', num2str(niters), '_150_point.pdf'), 'ContentType', 'image');
