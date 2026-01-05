%% Plot the results for planar robot in the paper
close all
clear all
clc

import gtsam.*
import gpmp2.*

vimp_root = setup_vimp();
matlab_helpers = fullfile(vimp_root, 'matlab_helpers');
addpath(matlab_helpers);
addpath(fullfile(matlab_helpers, 'tools'));
addpath(fullfile(matlab_helpers, 'tools', 'gtsam_toolbox'));
addpath(fullfile(matlab_helpers, 'tools', 'error_ellipse'));
addpath(fullfile(matlab_helpers, 'tools', '2dpR'));

is_sparse = 1;


%% ============ 
% read map
% ============
sdfmap = csvread(fullfile(vimp_root, 'vimp/maps/2dpR/map2/map_multiobs_map2.csv'));
dim_state = 4;
dim_theta = 2;

% Base directory for GVIMP examples
gvimp_2dpR_dir = fullfile(matlab_helpers, 'GVIMP-examples/2d_pR');


for i = 3:3
    if is_sparse
        prefix = fullfile(gvimp_2dpR_dir, 'sparse_gh/map2', ['case' num2str(i)]);
    else
        prefix = fullfile(gvimp_2dpR_dir, 'map2', ['case' num2str(i)]);
    end
    
    % Check if directory exists
    disp(['Checking prefix: ', prefix]);
    if ~exist(prefix, 'dir')
        disp('ERROR: Directory does not exist!');
        dir(gvimp_2dpR_dir)  % Show what's actually there
        return;
    end
    
    % List files in directory
    disp('Files in directory:');
    dir(prefix)
    
    % Check if mean.csv exists and has data
    mean_file = fullfile(prefix, 'mean.csv');
    if ~exist(mean_file, 'file')
        disp(['ERROR: File not found: ', mean_file]);
        return;
    end
    
    means = csvread(mean_file);
    disp(['means size: ', num2str(size(means))]);
    disp(['means min/max: ', num2str(min(means(:))), ' / ', num2str(max(means(:)))]);
end

% Check sdfmap
disp(['sdfmap size: ', num2str(size(sdfmap))]);
figure;
imagesc(sdfmap);
colorbar;
title('SDF Map');


%% ================ 
% plot costs 
% =================
for i = 3:3 % 4 experiments
    if is_sparse
        prefix = fullfile(gvimp_2dpR_dir, 'sparse_gh/map2', ['case' num2str(i)]);
    else
        prefix = fullfile(gvimp_2dpR_dir, 'map2', ['case' num2str(i)]);
    end
    
    means = csvread(fullfile(prefix, 'mean.csv'));
    joint_precisions = csvread(fullfile(prefix, 'precision.csv'));
    costs = csvread(fullfile(prefix, 'cost.csv'));
    factor_costs = csvread(fullfile(prefix, 'factor_costs.csv'));
    
    [ttl_dim, ~] = size(means);
    niters = find_niters(means);
    n_states = floor(ttl_dim / dim_state);
    output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);
end

%% ================ 
% plot trajectories 
% =================
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 850;

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
locations = {'northeast', 'northeast', 'northeast', 'northwest'};

for i = 1:4 % 4 experiments  
    nexttile
    
    if is_sparse
        prefix = fullfile(gvimp_2dpR_dir, 'sparse_gh/map2', ['case' num2str(i)]);
    else
        prefix = fullfile(gvimp_2dpR_dir, 'map2', ['case' num2str(i)]);
    end
    
    means = csvread(fullfile(prefix, 'mean.csv'));
    covs = csvread(fullfile(prefix, 'cov.csv'));
    precisions = csvread(fullfile(prefix, 'precision.csv'));
    
    niters = find_niters(means);
    nsteps = 1;
    
    output = plot_planarPR_oneiter(means, covs, sdfmap, niters);
    
    start_i = start_configs(i, :)';
    goal_i = goal_configs(i, :)';
    
    s1 = scatter(start_i(1), start_i(2), 300, 'x', 'MarkerEdgeColor', 'r', 'LineWidth', 2.5);
    s1.Annotation.LegendInformation.IconDisplayStyle = 'off';
    s2 = scatter(goal_i(1), goal_i(2), 300, 'x', 'MarkerEdgeColor', 'g', 'LineWidth', 2.5);
    s2.Annotation.LegendInformation.IconDisplayStyle = 'off';
    
    h1 = plot(NaN,NaN,'xr','MarkerSize',15,'LineWidth',2.5);
    h2 = plot(NaN,NaN,'xg','MarkerSize',15,'LineWidth',2.5);
    lgd = legend([h1, h2], {'Start', 'Goal'});
    set(lgd, 'FontWeight', 'bold', 'FontSize', 18, 'Location', locations{i});
    
    xlim([-15, 20])
    ylim([-10, 20])
    axis off
end