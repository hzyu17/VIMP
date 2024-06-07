close all
clear all
clc

addpath('../../tools/gtsam_toolbox');
import gtsam.*
import gpmp2.*

addpath("../../tools/error_ellipse");
addpath("../../tools");

is_sparse = 1;

%% ============ 
% read map
% ============
sdfmap = csvread("map2/map_multiobs_map2.csv");

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

dim_state = 4;
dim_theta = 2;

%% ================ 
% plot costs 
% =================
tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 1:4 % 4 experiments
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
    fig = get(groot,'CurrentFigure');
    title(['Experiment ', num2str(i)])
end

%% ================ 
% plot trajectories 
% =================
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
    
    xlim([-15, 20])
    ylim([-10, 20])
    
end
