clear all
close all
clc
addpath('/usr/local/gtsam_toolbox');
addpath("../../tools/error_ellipse");
addpath("../../../matlab_helpers/");
import gtsam.*
import gpmp2.*

% %%
% cov_sp = csvread("../../../vimp/cov_sp.csv");
% cov_full = csvread("../../../vimp/cov_full.csv");
% 
% num_states = 15;
% state_dim = 4;
% for i_s = 0:num_states-2
%     disp("diff norm i block");
%     start_indx = i_s*state_dim+1
%     end_indx = (i_s+2)*state_dim
%     diff = cov_sp(start_indx:end_indx, start_indx:end_indx) - cov_full(start_indx:end_indx, start_indx:end_indx);
%     diff;
%     norm(diff)
% end

%% ground truth
prefix_gt = ["../../RAL-examples/2d_pR/map2/exp" + num2str(1)+"/"];
% % --- high temperature ---
means_gt = csvread([prefix_gt + "mean_base.csv"]);
covs_gt = csvread([prefix_gt + "cov_base.csv"]);
precisions_gt = csvread([prefix_gt + "precisoin_base.csv"]);
costs_gt = csvread([prefix_gt + "cost_base.csv"]);

factor_costs_gt = csvread([prefix_gt + "factor_costs_base.csv"]);

% first iteration
i_iter = 1;
mean_gt_1 = means_gt(i_iter, 1:end)';
precisions_gt_1 = precisions_gt((i_iter-1)*60+1:i_iter*60, 1:60);
covs_gt_1 = covs_gt((i_iter-1)*60+1:i_iter*60, 1:60);
factor_costs_gt_1 = factor_costs_gt(i_iter, 1:end);

% debugging code 
prefix = ["map2/case" + num2str(1)+"/"];
% % --- high temperature ---
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "joint_cov.csv"]);
precisions = csvread([prefix + "joint_precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

% first iteration
means_1 = means;
precisions_1 = reshape(precisions, 60, 60);
covs_1 = reshape(covs, 60, 60);
factor_costs_1 = factor_costs';

%% read map
sdfmap = csvread("map2/map_multiobs_map2.csv");

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'none', 'Padding', 'none')
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
