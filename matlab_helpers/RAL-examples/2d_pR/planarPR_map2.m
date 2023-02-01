clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("map2/map_multiobs_entropy_map2.csv");

v_niters = [18, 10, 18, 18];
v_nsteps = [6, 10, 6, 6];

x0 = 500;
y0 = 500;
width = 600;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 1:4 % 4 experiments
    nexttile
    prefix = ["map2/exp"+num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "mean.csv"]);
    covs = csvread([prefix + "cov.csv"]);
    precisions = csvread([prefix + "precisoin.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    
    factor_costs = csvread([prefix + "factor_costs.csv"]);
%     perturb_stat= csvread([prefix + "perturbation_statistics.csv"]);
%     final_cost = csvread([prefix + "final_cost.csv"]);
    addpath("error_ellipse");
    
    niters = v_niters(i);
    nsteps = v_nsteps(i);
    output = plotPointRobotFinalIteration(means, covs, precisions, costs, factor_costs, sdfmap, niters, nsteps);
end
