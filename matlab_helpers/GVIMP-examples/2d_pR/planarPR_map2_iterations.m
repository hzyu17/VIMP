clear all
close all
clc
addpath('/usr/local/gtsam_toolbox');
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


i_exp = 1;
prefix = ["map2/case" + num2str(i_exp)+"/"];
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

niters = find_niters(means);

tiledlayout(2, floor(niters/2), 'TileSpacing', 'none', 'Padding', 'none')
for i = 1:niters % 4 experiments
    nexttile
    output = plot_planarPR_oneiter(means, covs, sdfmap, i);
end
