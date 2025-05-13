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
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");

dim_state = 4;
dim_theta = 2;
time = 3.0;
num_states = 300;

dt = time / (num_states - 1);

% %% ================ 
% % plot costs 
% % =================
% for i = 3:3 % 4 experiments
%     if is_sparse
%         prefix = ["sparse_gh/map2/case" + num2str(i)+"/"];
%     else
%         prefix = ["map2/case" + num2str(i)+"/"];
%     end
%     means = csvread([prefix + "mean.csv"]);
%     joint_precisions = csvread([prefix + "joint_precision.csv"]);
%     costs = csvread([prefix + "cost.csv"]);
    
%     factor_costs = csvread([prefix + "factor_costs.csv"]);
    
%     [ttl_dim, ~] = size(means);
%     niters = find_niters(means);
%     n_states = floor(ttl_dim / dim_state);

%     output_costplot = plot_costs(costs, factor_costs, joint_precisions, niters, n_states, dim_state);

% end

%% ================ 
% plot trajectories 
% =================
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
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

A_matrix = zeros(dim_state, dim_state);
A_matrix(1:dim_theta, dim_theta+1:end) = eye(dim_theta);

B_matrix = [zeros(dim_theta, dim_theta); eye(dim_theta)];

A_discrete = (A_matrix * dt) + eye(dim_state);
B_discrete = B_matrix * dt;

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 1:1 % 4 experiments  
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
    
    % nsteps = 1;
%     [ttl_dim, niters] = size(means);
    % output = plot_planarPR_oneiter(means, covs, sdfmap, niters);

    output = plot_planarPR_oneiter(means(:,end), covs(:,end), sdfmap, 1);
    % output = plot_planarPR_oneiter(means(1:2*dim_state,end), covs(1:2*dim_state*dim_state,end), sdfmap, 1);

    K_list = cell(num_states - 1, 1);

    joint_covariance = csvread([prefix + "joint_cov.csv"]);
    cov = joint_covariance(:,end);
    cov = reshape(cov, [num_states*dim_state, num_states*dim_state]);

    for j = 1:(num_states - 1)
        cov_i = cov((j-1)*dim_state+1:j*dim_state, (j-1)*dim_state+1:j*dim_state);
        cov_i1_i = cov(j*dim_state+1:(j+1)*dim_state, (j-1)*dim_state+1:j*dim_state);
        K_list{j} = inv(B_discrete'* B_discrete) * B_discrete' * (cov_i1_i * inv(cov_i) - A_discrete);
    end

    norm_k = [];
    for j = 1:(num_states-1)
        cov_i = cov((j-1)*dim_state+1:j*dim_state, (j-1)*dim_state+1:j*dim_state);
        cov_i1_i = cov(j*dim_state+1:(j+1)*dim_state, (j-1)*dim_state+1:j*dim_state);

        norm_k = [norm_k, norm(B_discrete*K_list{j} - (cov_i1_i * inv(cov_i) - A_discrete))];
    end

    % disp(norm_k)

    means_control = zeros(dim_state*num_states, 1);
    covs_control = zeros(dim_state*dim_state*num_states, 1);

    means_control(1:dim_state) = means(1:dim_state, end);
    covs_control(1:dim_state*dim_state) = covs(1:dim_state*dim_state, end);

    u_matrix = zeros(dim_theta, num_states);

    for j = 1:(num_states - 1)
        mean_i = means_control(dim_state*(j-1)+1:dim_state*j);
        means_control(dim_state*j+1:dim_state*(j+1)) = (A_discrete + B_discrete * K_list{j})*mean_i;
        cov_i = covs_control(dim_state*dim_state*(j-1)+1:dim_state*dim_state*j);
        cov_i = reshape(cov_i, dim_state, dim_state);
        cov_i1 = (A_discrete + B_discrete * K_list{j}) * cov_i * (A_discrete + B_discrete * K_list{j})' + B_discrete * B_discrete';
        covs_control(dim_state*dim_state*j+1:dim_state*dim_state*(j+1)) = reshape(cov_i1, dim_state*dim_state, 1);
        u_matrix(:,j) = K_list{j} * mean_i;
    end

    

    output = plot_planarPR_oneiter(means_control, covs_control, sdfmap, 1);
    % output = plot_planarPR_oneiter(means_control(1:2*dim_state), covs_control(1:2*dim_state*dim_state), sdfmap, 1);

    figure
    plot(u_matrix(1,:), 'r','LineWidth',2)
    hold on
    plot(u_matrix(2,:), 'b','LineWidth',2)

    
end
