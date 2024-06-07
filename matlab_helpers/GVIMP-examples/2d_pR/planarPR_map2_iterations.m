clear all
close all
clc
addpath('../../tools/gtsam_toolbox');
addpath("../../tools/error_ellipse");
addpath("../../../matlab_helpers/");
import gtsam.*
import gpmp2.*


%% read map
sdfmap = csvread("map2/map_multiobs_map2.csv");

v_niters = [18, 10, 18, 18];
v_nsteps = [6, 10, 6, 6];

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

is_sparse = 1;

i_exp = 2;
if is_sparse
    prefix = ["sparse_gh/map2/case" + num2str(i_exp)+"/"];
else
    prefix = ["map2/case" + num2str(i_exp)+"/"];
end
    
% prefix = ["map2/case" + num2str(i_exp)+"/"];

means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precision.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

niters = find_niters(means);

% tiledlayout(2, floor(niters/5), 'TileSpacing', 'none', 'Padding', 'none')
% for i = 1:8:niters % 
tiledlayout(1, 4, 'TileSpacing', 'none', 'Padding', 'none')
for i = [1, 5, 15, 25]
    nexttile
    output = plot_planarPR_oneiter(means, covs, sdfmap, i);
    title(['Iteration ', num2str(i), ', $\hat{T} = 2$'], 'Interpreter', 'latex')
    axis off
end

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 4, 'TileSpacing', 'none', 'Padding', 'none')
for i = [ 30, 35, 40, 45]
    nexttile
    output = plot_planarPR_oneiter(means, covs, sdfmap, i);
    title(['Iteration ', num2str(i), ', $\hat{T} = 20$'], 'Interpreter', 'latex')
    axis off
end
