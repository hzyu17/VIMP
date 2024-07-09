%% ================ test data ================
% plot final iteration results
sdfmap = csvread("map1/map_multiobs_map1.csv");
prefix = ["map1/case1/"];

means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "joint_precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

% -------------------- print out the difference --------------------
prefix_gt = ["map1/above_case/"];

means_gt = csvread([prefix_gt + "mean_base.csv"])';
covs_gt = csvread([prefix_gt + "cov_base.csv"]);
precisions_gt = csvread([prefix_gt + "precisoin_base.csv"]);
costs_gt = csvread([prefix_gt + "cost_base.csv"]);
factor_costs_gt = csvread([prefix_gt + "factor_costs_base.csv"]);

for i = 2:10
    % norm diff of precision matrices
    precisions_gt_2 = precisions_gt((i-1)*40+1:i*40, 1:40);
    precisions_2 = reshape(precisions(1:end, i), 40, 40);
    norm_diff_precison_2 = norm(precisions_2 - precisions_gt_2)
    
    % norm diff of mean vectors
    means_gt_2 = means_gt(1:end, i);
    means_2 = means(1:end, i);
    norm_diff_means_2 = norm(means_2 - means_gt_2)
end

% -------------------- plot final iteration ------------------------


%% ====================== case 2 ====================
prefix = ["map1/case2/"];

means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "joint_precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

% -------------------- print out the difference --------------------
prefix_gt = ["map1/below_case/"];

means_gt = csvread([prefix_gt + "mean_base.csv"])';
covs_gt = csvread([prefix_gt + "cov_base.csv"]);
precisions_gt = csvread([prefix_gt + "precisoin_base.csv"]);
costs_gt = csvread([prefix_gt + "cost_base.csv"]);
factor_costs_gt = csvread([prefix_gt + "factor_costs_base.csv"]);

for i = 2:10
    % norm diff of precision matrices
    precisions_gt_2 = precisions_gt((i-1)*40+1:i*40, 1:40);
    precisions_2 = reshape(precisions(1:end, i), 40, 40);
    norm_diff_precison_2 = norm(precisions_2 - precisions_gt_2)
    
    % norm diff of mean vectors
    means_gt_2 = means_gt(1:end, i);
    means_2 = means(1:end, i);
    norm_diff_means_2 = norm(means_2 - means_gt_2)
end

% -------------------- plot final iteration ------------------------

niters = find_niters(means);
nt = 10;
dim_state = 4;

x0 = 500;
y0 = 500;
width = 600;
height = 380;

cell_size = 0.1;
origin_x = -20;
origin_y = -10;

num_figures = 10;
step_size = floor(niters / num_figures);

figure
tiledlayout(2, floor(num_figures/2), 'TileSpacing', 'tight', 'Padding', 'none')
set(gcf,'position',[x0,y0,width,height])

for i_step = 1:step_size:niters
    nexttile
    hold on
    means_final = means(1:end, i_step);
    means_final = reshape(means_final, [4, 10]);
    cov_final = covs(:, i_step);
    cov_final = reshape(cov_final, dim_state, dim_state, nt);
    plot_2d_result(sdfmap, means_final, cov_final);
end

%% ================== map 2 ===================
state_dim = 4;
nt = 15;
sdfmap_2 = csvread("map2/map_multiobs_map2.csv");
% ------------- ground truth ------------- 
prefix_gt = ["../../RAL-examples/2d_pR/map2/exp" + num2str(2)+"/"];
% % --- high temperature ---
means_gt = csvread([prefix_gt + "mean_base.csv"]);
covs_gt = csvread([prefix_gt + "cov_base.csv"]);
precisions_gt = csvread([prefix_gt + "precisoin_base.csv"]);
costs_gt = csvread([prefix_gt + "cost_base.csv"]);

factor_costs_gt = csvread([prefix_gt + "factor_costs_base.csv"]);

% first iteration
i_iter = 6;
mean_gt_1 = means_gt(i_iter, 1:end)';
precisions_gt_1 = precisions_gt((i_iter-1)*60+1:i_iter*60, 1:60);
covs_gt_1 = covs_gt((i_iter-1)*60+1:i_iter*60, 1:60);
factor_costs_gt_1 = factor_costs_gt(i_iter, 1:end);

% debugging code 
prefix = ["map2/case" + num2str(3)+"/"];
% % --- high temperature ---
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "joint_precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

niters = find_niters(means);
num_figures = 10;
step_size = floor(niters / num_figures);

figure
tiledlayout(2, floor(num_figures/2), 'TileSpacing', 'tight', 'Padding', 'none')
set(gcf,'position',[x0,y0,width,height])

for i_step = 1:step_size:niters
    nexttile
    hold on
    means_final = means(1:end, i_step);
    means_final = reshape(means_final, [state_dim, nt]);
    cov_final = covs(:, i_step);
    cov_final = reshape(cov_final, dim_state, dim_state, nt);
    plot_2d_result(sdfmap_2, means_final, cov_final);
end

% % first iteration
% means_1 = means;
% precisions_1 = reshape(precisions, 60, 60);
% covs_1 = reshape(covs, 60, 60);
% factor_costs_1 = factor_costs';
