clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
% sdfmap = csvread("../vimp/data/2d_pR/map_multiobs.csv");
sdfmap = csvread("../vimp/data/2d_pR/map_multiobs_entropy.csv");

%% ******************* Read datas ******************
% --- low temperature ---
means = csvread("../vimp/data/2d_pR/mean_base.csv");
covs = csvread("../vimp/data/2d_pR/cov_base.csv");
precisions = csvread("../vimp/data/2d_pR/precisoin_base.csv");
costs = csvread("../vimp/data/2d_pR/cost_base.csv");

factor_costs = csvread("../vimp/data/2d_pR/factor_costs_base.csv");
perturb_stat= csvread("../vimp/data/2d_pR/perturbation_statistics_base.csv");
final_cost = csvread("../vimp/data/2d_pR/final_cost_base.csv");
addpath("error_ellipse");


% --- high temperature ---
% means = csvread("../vimp/data/2d_pR/mean.csv");
% covs = csvread("../vimp/data/2d_pR/cov.csv");
% precisions = csvread("../vimp/data/2d_pR/precisoin.csv");
% costs = csvread("../vimp/data/2d_pR/cost.csv");
% 
% factor_costs = csvread("../vimp/data/2d_pR/factor_costs.csv");
% perturb_stat= csvread("../vimp/data/2d_pR/perturbation_statistics.csv");
% final_cost = csvread("../vimp/data/2d_pR/final_cost.csv");
% addpath("error_ellipse");

% means = csvread("../vimp/data/checkpoints/2d_pR_0.15/mean.csv");
% covs = csvread("../vimp/data/checkpoints/2d_pR_0.15/cov.csv");
% precisions = csvread("../vimp/data/checkpoints/2d_pR_0.15/precisoin.csv");
% costs = csvread("../vimp/data/checkpoints/2d_pR_0.15/cost.csv");
% sdfmap = csvread("../vimp/data/checkpoints/2d_pR_0.15/map_multiobs.csv");
% factor_costs = csvread("../vimp/data/checkpoints/2d_pR_0.15/factor_costs.csv");
% perturb_stat= csvread("../vimp/data/checkpoints/2d_pR_0.15/purturbation_statistics.csv");
% final_cost = csvread("../vimp/data/checkpoints/2d_pR_0.15/final_cost.csv");
% addpath("error_ellipse");

%%
[niters, ttl_dim] = size(means);
dim_theta = 4;
niters = 10;
nsteps = 8;
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

mesh_hingeloss = csvread("../vimp/data/mesh_hingeloss.csv");

% =========================== load the means and covs on the 2*2 level
% containers for all the steps data
vec_means = cell(niters, 1);
vec_covs = cell(niters, 1);
vec_precisions = cell(niters, 1);

for i_iter = 0: nsteps-1
        % each time step 
        i = i_iter * step_size;
        i_mean = means(i+1, 1:end);
        i_cov = covs(i*ttl_dim+1 : (i+1)*ttl_dim, 1:ttl_dim);
        i_prec = precisions(i*ttl_dim+1 : (i+1)*ttl_dim, 1:ttl_dim);
        i_vec_means_2d = cell(n_states, 1);
        i_vec_covs_2d = cell(n_states, 1);
        vec_precisions{i_iter+1} = i_prec;
        for j = 0:n_states-1
            % each state
            i_vec_means_2d{j+1} = i_mean(j*dim_theta+1 : j*dim_theta+2);
            i_vec_covs_2d{j+1} = i_cov(j*dim_theta +1 : j*dim_theta+2,  j*dim_theta+1 : j*dim_theta+2);
        end
        vec_means{i_iter+1} = i_vec_means_2d;
        vec_covs{i_iter+1} = i_vec_covs_2d;
end

%% plot sdf and means and covs
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
figure
% -------- plot the means -------- 
colors = [255, 0, 0];

cell_size = 0.1;
origin_x = -20;
origin_y = -10;
origin_point2 = Point2(origin_x, origin_y);
field = signedDistanceField2D(sdfmap, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

grid_rows = size(sdfmap, 1);
grid_cols = size(sdfmap, 2);
grid_corner_x = origin_x + (grid_cols-1)*cell_size;
grid_corner_y = origin_y + (grid_rows-1)*cell_size;
grid_X = origin_x : cell_size : grid_corner_x;
grid_Y = origin_y : cell_size : grid_corner_y;

mesh_X = repmat(grid_X', 1, size(grid_Y,2));
mesh_Y = repmat(grid_Y, size(grid_X,2), 1);

tiledlayout(2, floor(nsteps/2), 'TileSpacing', 'tight', 'Padding', 'tight')
for i_iter = 1: nsteps
    nexttile
    title(['Iteration ', num2str(i_iter*step_size)])
    hold on 
   
%     plotSignedDistanceField2D(field, origin_x, origin_y, cell_size);
    plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
%     mesh(mesh_X, mesh_Y, mesh_hingeloss, 'FaceAlpha', 0.5);
    i_vec_means_2d = vec_means{i_iter};
    i_vec_covs_2d = vec_covs{i_iter};
    for j = 1:n_states
        % means
        scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 10, 'r', 'fill');
        % covariance
        error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
    end
end

%% ================ plot the last iteration ================ 
figure
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
nexttile
title(['Iteration ', num2str(nsteps*step_size)])
hold on 

%     plotSignedDistanceField2D(field, origin_x, origin_y, cell_size);
plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
% mesh(mesh_X, mesh_Y, mesh_hingeloss, 'FaceAlpha', 0.5);
i_vec_means_2d = vec_means{nsteps};
i_vec_covs_2d = vec_covs{nsteps};
for j = 1:n_states
    % means
    scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 10, 'r', 'fill');
    % covariance
    error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
end
xlim([-15, 20])
ylim([-15, 20])

%% ================ plot the total costs ================
figure
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none') 
nexttile
title('Total Loss')
grid minor 
hold on
plot(costs, 'LineWidth', 2.0, 'LineStyle', '-.');
scatter(linspace(1, length(costs), length(costs)), costs, 'fill')
xlabel('Iterations','fontweight','bold')
ylabel('V(q)','fontweight','bold')

%% =============== plot cost for each factor ================
fixed_prior_costs = [factor_costs(1:end, 1), factor_costs(1:end, end)];
prior_costs = [];
for i = 1:n_states-1
    prior_costs = [prior_costs, factor_costs(1:end, 1+(i-1)*2+1)];
end
obs_costs = [];
for i = 1:n_states-2
    obs_costs = [obs_costs, factor_costs(1:end, 1+(i-1)*2+2)];
end

% subplot(1, 3, 1)
% hold on
% grid on
% title('Fixed prior costs')
% plot(fixed_prior_costs, 'LineWidth', 1.5)

figure
tiledlayout(1, 3, 'TileSpacing', 'tight', 'Padding', 'tight') 
nexttile

title('Prior cost factors')
hold on
grid minor
plot(prior_costs, 'LineWidth', 1.5, 'LineStyle','-.')
scatter(linspace(1,niters, niters), prior_costs(1:niters, 1:end), 10, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(x_k))','fontweight','bold')

nexttile
title('Collision cost factors')
hold on
grid minor
plot(obs_costs, 'LineWidth', 1.5, 'LineStyle','-.')
scatter(linspace(1,niters, niters), obs_costs(1:niters, 1:end), 10, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(z|x_k))','fontweight','bold')

% --- entropy
entropy_costs = [];
n_dim = size(precisions, 2);
for i = 1:niters
    precision_i  = precisions((i-1)*n_dim+1: i*n_dim, 1:end);
    entropy_costs = [entropy_costs, log(det(precision_i))/2];
end

nexttile
title('Entropy cost factors')
hold on
grid minor
plot(entropy_costs, 'LineWidth', 1.5, 'LineStyle','-.')
scatter(linspace(1,niters, niters), entropy_costs(1:niters), 10, 'filled')
xlabel('Iterations', 'fontweight', 'bold')
ylabel('log(|\Sigma^{-1}|)/2', 'Interpreter', 'tex', 'fontweight', 'bold')
% ylabel('\log(\lvert\Sigma^{-1}\rvert))', 'Interpreter','latex','fontweight','bold')

% verify that the sum of the factored costs is the same as the total cost
sum_fact_costs = sum(factor_costs(1:niters, 1:end), 2);
diff = sum_fact_costs + entropy_costs' - costs(1:niters)

%% statistics of perturbed cost
final_cost = final_cost(1);
diff_purturb_stat = perturb_stat - final_cost;
avg_diff_purturb = sum(diff_purturb_stat) / length(diff_purturb_stat);

figure
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none') 
nexttile
title('Purturbed cost values')
hold on
grid on
plot(diff_purturb_stat, 'c', 'LineWidth', 1.5,'LineStyle', '-.')
plot(linspace(1, length(diff_purturb_stat), length(diff_purturb_stat)), avg_diff_purturb.*ones(length(diff_purturb_stat)), 'r-', 'LineWidth', 1.5)
plot(linspace(0, length(diff_purturb_stat), length(diff_purturb_stat)), zeros(length(diff_purturb_stat)), 'k-', 'LineWidth', 1.5)
scatter(linspace(1, length(diff_purturb_stat), length(diff_purturb_stat)), diff_purturb_stat, 'bo')
legend({'Perturbed cost value', 'Average'})
xlabel('Perturbation index', 'fontweight', 'bold')
ylabel('\delta V', 'fontweight', 'bold')
ylim([-0.01, max(diff_purturb_stat)*1.1])


%%
% field = csvread("../vimp/data/2d_pR/field_multiobs_entropy.csv");
% 
% cell_size = 0.1;
% origin_x = -20;
% origin_y = -10;
% 
% plotSignedDistanceField2D(field, origin_x, origin_y, cell_size);

