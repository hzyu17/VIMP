clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
% sdfmap = csvread("../../../vimp/data/2d_pR/map_multiobs.csv");
% sdfmap = csvread("../../../vimp/data/2d_pR/map_multiobs_entropy.csv");
sdfmap = csvread("../../../vimp/data/2d_pR/map_multiobs_entropy_map2.csv");
% sdfmap = csvread("../../../vimp/data/2d_pR/map_multiobs_entropy_map3.csv");

%% an initilization specially for map3 narrow
% means = csvread("../../../vimp/data/2d_pR/mean_base.csv");
% mean = means(1, 1:end);
% positions = [-3, -8; 
%                         0, -5; 
%                         5, -5; 
%                         10, -5; 
%                         10, 0;
%                         10, 2;
%                         10, 3;
%                         10, 4;
%                         10, 5;
%                         10, 10;
%                         10, 12;
%                         10, 14;
%                         10, 15;
%                         8, 16;
%                         4, 17];
% for i=1:15
%     mean(1, (i-1)*4+1:(i-1)*4+2) = positions(i, 1:end);
% end
% csvwrite("../../../vimp/data/2d_pR/mean_map3_circumvent_base.csv", mean)

%% ******************* Read datas ******************
% --- low temperature ---
% means = csvread("../../../vimp/data/2d_pR/mean_base.csv");
% covs = csvread("../../../vimp/data/2d_pR/cov_base.csv");
% precisions = csvread("../../../vimp/data/2d_pR/precisoin_base.csv");
% costs = csvread("../../../vimp/data/2d_pR/cost_base.csv"); 
% factor_costs = csvread("../../../vimp/data/2d_pR/factor_costs_base.csv");
% 
% perturb_stat= csvread("../../../vimp/data/2d_pR/perturbation_statistics_base.csv");
% final_cost = csvread("../../../vimp/data/2d_pR/final_cost_base.csv");
% addpath("error_ellipse");

% % --- high temperature ---
means = csvread("../../../vimp/data/2d_pR/mean.csv"); 
covs = csvread("../../../vimp/data/2d_pR/cov.csv");
precisions = csvread("../../../vimp/data/2d_pR/precisoin.csv");
costs = csvread("../../../vimp/data/2d_pR/cost.csv");
factor_costs = csvread("../../../vimp/data/2d_pR/factor_costs.csv");

perturb_stat= csvread("../../../vimp/data/2d_pR/perturbation_statistics.csv");
final_cost = csvread("../../../vimp/data/2d_pR/final_cost.csv");
addpath("error_ellipse");

%%
[niters, ttl_dim] = size(means);
dim_theta = 4;
nsteps = 10;
% niters
% niters = length(costs);
niters = 18;
nsteps = 18;
for i=niters:-1:1
    if costs(i) ~= 0
        niters=i;
        break
    end
end
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

mesh_hingeloss = csvread("../../../vimp/data/mesh_hingeloss.csv");

% plot the hinge loss
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
% figure
% tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
% nexttile
% mesh(mesh_X, mesh_Y, mesh_hingeloss, 'FaceAlpha', 0.5);
% title("Hinge Loss")

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
x0 = 50;
y0 = 50;
width = 600;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
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

% mesh_X = repmat(grid_X', 1, size(grid_Y,2));
% mesh_Y = repmat(grid_Y, size(grid_X,2), 1);
% mesh(mesh_X, mesh_Y, mesh_hingeloss, 'FaceAlpha', 0.5);

tiledlayout(2, floor(nsteps/2), 'TileSpacing', 'tight', 'Padding', 'none')
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
        scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 20, 'k', 'fill');
        % covariance
        error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
    end
    xlim([-15, 20])
ylim([-10, 20])
end

%% ================ plot the last iteration ================ 
x0 = 50;
y0 = 50;
width = 400;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
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
    scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 20, 'k', 'fill');
    % covariance
    error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
end
xlim([-15, 20])
ylim([-10, 20])

%% =============== plot cost for each factor and the total cost ================
fixed_prior_costs = [factor_costs(1:end, 1), factor_costs(1:end, end)];
prior_costs = [];
for i = 1:n_states-1
    prior_costs = [prior_costs, factor_costs(1:end, 1+(i-1)*2+1)];
end
obs_costs = [];
for i = 1:n_states-2
    obs_costs = [obs_costs, factor_costs(1:end, 1+(i-1)*2+2)];
end

x0 = 50;
y0 = 50;
width = 1000;
height = 500;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 3, 'TileSpacing', 'tight', 'Padding', 'tight') 
nexttile

title('Prior cost factors')
hold on
grid on
% plot(prior_costs, 'LineWidth', 1.5, 'LineStyle','-.')
plot(prior_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), prior_costs(1:niters, 1:end), 30, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(x_k))','fontweight','bold')

nexttile
title('Collision cost factors')
hold on
grid on
% plot(obs_costs, 'LineWidth', 1.5, 'LineStyle','-.')
plot(obs_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), obs_costs(1:niters, 1:end), 30, 'filled')
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
grid on
% plot(entropy_costs, 'LineWidth', 1.5, 'LineStyle','-.')
plot(entropy_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), entropy_costs(1:niters), 30, 'filled')
xlabel('Iterations', 'fontweight', 'bold')
ylabel('log(|\Sigma^{-1}|)/2', 'Interpreter', 'tex', 'fontweight', 'bold')
% ylabel('\log(\lvert\Sigma^{-1}\rvert))', 'Interpreter','latex','fontweight','bold')

% verify that the sum of the factored costs is the same as the total cost
sum_fact_costs = sum(factor_costs(1:niters, 1:end), 2);
% diff = sum_fact_costs + entropy_costs' - costs(1:niters);

% ================ plot the total costs ================
% tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none') 
nexttile([1 3])
title('Total Loss')
grid on 
hold on
plot(costs(1:niters), 'LineWidth', 2.0);
scatter(linspace(1, niters, niters), costs(1:niters), 30, 'fill')
xlabel('Iterations','fontweight','bold')
ylabel('V(q)','fontweight','bold')
hold off

%% ====== statistics of the cost distributions ======
disp('========== final prior cost ==========')
sum(prior_costs(niters,1:end))

disp('========== final obs_costs cost ==========')
sum(obs_costs(niters, 1:end))

disp(['========== motion planning cost ', num2str(i),  '==========='])
    sum(obs_costs(niters, 1:end)) + sum(prior_costs(niters,1:end))

disp('========== final entropy cost ==========')
entropy_costs(niters)

disp('========== final total cost ==========')
costs(niters)

%% statistics of perturbed cost
% final_cost = final_cost(1);
% diff_purturb_stat = perturb_stat - final_cost;
% avg_diff_purturb = sum(diff_purturb_stat) / length(diff_purturb_stat);
% 
% % === plot the perturbation ===
% 
% nexttile([1 3])
% title('Perturbed cost values')
% hold on
% grid on
% plot(diff_purturb_stat, 'c', 'LineWidth', 1.5,'LineStyle', '-.')
% plot(linspace(1, length(diff_purturb_stat), length(diff_purturb_stat)), avg_diff_purturb.*ones(length(diff_purturb_stat)), 'r-', 'LineWidth', 1.5)
% plot(linspace(0, length(diff_purturb_stat), length(diff_purturb_stat)), zeros(length(diff_purturb_stat)), 'k-', 'LineWidth', 1.5)
% scatter(linspace(1, length(diff_purturb_stat), length(diff_purturb_stat)), diff_purturb_stat, 'bo')
% legend({'Perturbed cost value', 'Average'})
% xlabel('Perturbation index', 'fontweight', 'bold')
% ylabel('\delta V', 'fontweight', 'bold')
% ylim([-0.01, max(diff_purturb_stat)*1.1])

% %% statistics of perturbed cost
% final_cost = final_cost(1);
% diff_purturb_stat = perturb_stat - final_cost;
% avg_diff_purturb = sum(diff_purturb_stat) / length(diff_purturb_stat);
% figure
% tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none') 
% nexttile
% title('Purturbed cost values')
% hold on
% grid on
% plot(diff_purturb_stat, 'c', 'LineWidth', 1.5,'LineStyle', '-.')
% plot(linspace(1, length(diff_purturb_stat), length(diff_purturb_stat)), avg_diff_purturb.*ones(length(diff_purturb_stat)), 'r-', 'LineWidth', 1.5)
% plot(linspace(0, length(diff_purturb_stat), length(diff_purturb_stat)), zeros(length(diff_purturb_stat)), 'k-', 'LineWidth', 1.5)
% scatter(linspace(1, length(diff_purturb_stat), length(diff_purturb_stat)), diff_purturb_stat, 'bo')
% legend({'Perturbed cost value', 'Average'})
% xlabel('Perturbation index', 'fontweight', 'bold')
% ylabel('\delta V', 'fontweight', 'bold')
% ylim([-0.01, max(diff_purturb_stat)*1.1])


%% ==== plot sampled covariance for the states ==== 
x0 = 50;
y0 = 50;
width = 400;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
title(['Iteration ', num2str(nsteps*step_size)])
hold on 
i_vec_means_2d = vec_means{nsteps};
i_vec_covs_2d = vec_covs{nsteps};

plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);

n_samples = 100;
for j = 1:3:n_states
    % gradual changing colors
%     alpha = (j / n_states)^(1.15);
    color = [0, 0, 1, 0.9];
    color_sample = [0.2, 0.2, 0.9, 0.07];
    % mu j
    mean_j = i_vec_means_2d{j}';
    % cov j
    cov_j = i_vec_covs_2d{j};
    % sampling 
    rng('default')  % For reproducibility
    samples = mvnrnd(mean_j, cov_j, n_samples);
    for k = 1: size(samples, 1)
        k_sample = samples(k, 1:end)';
        scatter(k_sample(1), k_sample(2), 10, 'red', 'fill');
    end
    
    % plot mean
    scatter(mean_j(1), mean_j(2), 40, 'k', 'fill');
    
end

hold off

%%
% field = csvread("../../vimp/data/2d_pR/field_multiobs_entropy.csv");
% 
% cell_size = 0.1;
% origin_x = -20;
% origin_y = -10;
% 
% plotSignedDistanceField2D(field, origin_x, origin_y, cell_size);

