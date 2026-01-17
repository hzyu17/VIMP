clear all
close all
clc

vimp_root = setup_vimp();
matlab_helpers = fullfile(vimp_root, 'matlab_helpers');
addpath(matlab_helpers);
addpath(fullfile(matlab_helpers, 'tools'));
addpath(fullfile(matlab_helpers, 'tools', 'gtsam_toolbox'));
addpath(fullfile(matlab_helpers, 'tools', 'error_ellipse'));
addpath(fullfile(matlab_helpers, 'tools', '2dpR'));

import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread(fullfile(vimp_root, 'vimp', 'maps', '2dpR', 'map2', 'map_multiobs_map2.csv'));

v_niters = [18, 10, 18, 18];
v_nsteps = [6, 10, 6, 6];

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

is_sparse = 1;
i_exp = 3;

if is_sparse
    prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_pR', 'sparse_gh', 'map2', ['case' num2str(i_exp)]);
else
    prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_pR', 'map2', ['case' num2str(i_exp)]);
end

means = csvread(fullfile(prefix, 'mean.csv'));
covs = csvread(fullfile(prefix, 'cov.csv'));
precisions = csvread(fullfile(prefix, 'precision.csv'));
costs = csvread(fullfile(prefix, 'cost.csv'));
factor_costs = csvread(fullfile(prefix, 'factor_costs.csv'));

niters = find_niters(means);

% tiledlayout(2, floor(niters/5), 'TileSpacing', 'none', 'Padding', 'none')
% for i = 1:8:niters % 
tiledlayout(1, 4, 'TileSpacing', 'none', 'Padding', 'none')
% for i = [1, 5, 15, 25]
for i = [1, 5, 10, 15]
    nexttile
    output = plot_planarPR_oneiter(means, covs, sdfmap, i);
    title(['Iteration ', num2str(i), ', $\hat{T} = 10$'], 'Interpreter', 'latex', 'FontSize', 20)
    axis off
end

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 4, 'TileSpacing', 'none', 'Padding', 'none')
% for i = [ 30, 35, 40, 45]
for i = [ 20, 25, 30, 35]
    nexttile
    output = plot_planarPR_oneiter(means, covs, sdfmap, i);
    title(['Iteration ', num2str(i), ', $\hat{T} = 30$'], 'Interpreter', 'latex', 'FontSize', 20)
    axis off
end