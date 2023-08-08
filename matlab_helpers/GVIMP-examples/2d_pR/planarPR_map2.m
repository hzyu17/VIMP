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
    prefix = ["../../RAL-examples/2d_pR/map2/exp" + num2str(1)+"/"];
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
