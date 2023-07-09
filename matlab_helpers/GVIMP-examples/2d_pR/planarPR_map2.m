clear all
close all
clc
addpath('/home/hyu419/.local/gtsam_toolbox')
addpath("error_ellipse");
addpath("../../../matlab_helpers/");
import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("map2/map_multiobs_map2.csv");

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact')
for i = 1:4 % 4 experiments
    nexttile
    prefix = ["map2/case" + num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "mean.csv"]);
    covs = csvread([prefix + "cov.csv"]);
    precisions = csvread([prefix + "precisoin.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    
    factor_costs = csvread([prefix + "factor_costs.csv"]);
       
    niters = find_niters(means);
    nsteps = 1;
    
    output = plot_planarPR_oneiter(means, covs, sdfmap, niters);
end
