clear all
close all
clc

%% ******************* Read datas ******************
addpath("../../tools/error_ellipse");
addpath('../../tools/2dArm');
addpath('../../tools/gtsam_toolbox');

import gtsam.*
import gpmp2.*

map = 1;
exp = 1;

prefix = "map1";
prefix_gpmp2 = "map1";
prefix_gvimp = "map1";
switch map
    case 1
        prefix = "map1";
        sdfmap = csvread("../../../vimp/maps/2dArm/map.csv");
        switch exp
            case 1
                prefix = "map1/case1";
                prefix_gpmp2 = "map1/case1/gpmp2";
                prefix_gvimp = "../../GVIMP-examples/2d_Arm/sparse_gh/map1/case1";
                % boundary conditions
                start_conf = [0, 0]';
                start_vel = [0, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
            case 2
                prefix = "map1/case2";
                prefix_gpmp2 = "map1/case2/gpmp2";
                % boundary conditions
                start_conf = [-pi/2, 0]';
                start_vel = [0.1, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
        end
end

dim_theta = 4;
% =================== read gvimp results ====================
means_gvimp = csvread([prefix_gvimp+"/mean.csv"]);
covs_gvimp = csvread([prefix_gvimp+"/cov.csv"]);
costs_gvimp = csvread([prefix_gvimp+"/cost.csv"]);
[ttl_dim, niters] = size(means_gvimp);
nt_gvimp = floor(ttl_dim/dim_theta);
% n_states = floor(ttl_dim / dim_theta);

% =================== read pgcs results ====================
% means_pcsmp = csvread([prefix+"/zk_sdf.csv"]);
% covs_pcsmp = csvread([prefix+"/Sk_sdf.csv"]);

% % ----- parameters -----
% [ndim, niters_pcs] = size(means_pcsmp);
% covs_pcsmp = reshape(covs_pcsmp, dim_theta, dim_theta, niters_pcs);

means_pcsmp = csvread([prefix + "/mean.csv"]);
covs_pcsmp = csvread([prefix + "/cov.csv"]);
[ttl_dim_pcs, niters_pcs] = size(means_pcsmp);
nt_pcsmp = floor(ttl_dim_pcs/dim_theta);

%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');

%  ------- sdf --------
cell_size = 0.01;
origin_x = -1;
origin_y = -1;
origin_point2 = Point2(origin_x, origin_y);
field = signedDistanceField2D(sdfmap, cell_size);
% save field
sdf = PlanarSDF(origin_point2, cell_size, field);

%% ================== Subplot grid for both methods ==================
iter_step = 5;
iter_step_pcs = 80;

% GVI-MP iterations to plot
iters_gvimp = 1:iter_step:niters;
n_plots_gvimp = length(iters_gvimp);

% PCS-MP iterations to plot
iters_pcsmp = 2:iter_step_pcs:niters_pcs;
n_plots_pcsmp = length(iters_pcsmp);

% Use the same number of plots for fair comparison
n_plots = max(n_plots_gvimp, n_plots_pcsmp);
n_cols = min(5, n_plots);
n_rows = 2;  % Row 1: GVI-MP, Row 2: PCS-MP

% Create figure
figure('Position', [50, 50, 250*n_cols, 500]);
tiledlayout(n_rows, n_cols, 'TileSpacing', 'compact', 'Padding', 'compact');

% --- Row 1: GVI-MP ---
for idx = 1:n_cols
    nexttile
    hold on
    plot_configuration_obstacles()
    
    if idx <= n_plots_gvimp
        iter = iters_gvimp(idx);
        
        % Extract means and covariances for this iteration
        means_iter = means_gvimp(:, iter);
        means_iter = reshape(means_iter, [dim_theta, nt_gvimp]);
        
        covs_iter = covs_gvimp(:, iter);
        covs_iter = reshape(covs_iter, [dim_theta, dim_theta, nt_gvimp]);
        
        % Plot trajectory with uncertainty ellipses
        for i = 1:nt_gvimp
            scatter(means_iter(1, i), means_iter(2, i), 15, 'b', 'fill');
            error_ellipse(covs_iter(1:2, 1:2, i), means_iter(1:2, i), 'style', 'b-');
        end
        
        title(['Iter ' num2str(iter)], 'FontSize', 11);
    end
    
    % Plot start and goal configurations
    scatter(start_conf(1), start_conf(2), 60, 'r', 'fill');
    scatter(end_conf(1), end_conf(2), 60, 'g', 'fill');
    
    if idx == 1
        ylabel('GVI-MP', 'FontSize', 12, 'FontWeight', 'bold');
    end
    hold off
end

% --- Row 2: PCS-MP ---
for idx = 1:n_cols
    nexttile
    hold on
    plot_configuration_obstacles()
    
    if idx <= n_plots_pcsmp
        iter = iters_pcsmp(idx);
        
        % Extract means and covariances for this iteration
        means_iter = means_pcsmp(:, iter);
        means_iter = reshape(means_iter, [dim_theta, nt_pcsmp]);
        
        covs_iter = covs_pcsmp(:, iter);
        covs_iter = reshape(covs_iter, [dim_theta, dim_theta, nt_pcsmp]);
        
        % Plot trajectory with uncertainty ellipses
        for i = 1:nt_pcsmp
            scatter(means_iter(1, i), means_iter(2, i), 15, [0.8500 0.3250 0.0980], 'fill');
            error_ellipse(covs_iter(1:2, 1:2, i), means_iter(1:2, i), 'style', 'r-');
        end
        
        title(['Iter ' num2str(iter)], 'FontSize', 11);
    end
    
    % Plot start and goal configurations
    scatter(start_conf(1), start_conf(2), 60, 'r', 'fill');
    scatter(end_conf(1), end_conf(2), 60, 'g', 'fill');
    
    if idx == 1
        ylabel('PCS-MP', 'FontSize', 12, 'FontWeight', 'bold');
    end
    hold off
end

sgtitle('Convergence Comparison in Configuration Space', 'FontSize', 14, 'FontWeight', 'bold');


%% ================== Overlay plot with color gradient ==================
figure('Position', [50, 50, 1000, 450]);
tiledlayout(1, 2, 'TileSpacing', 'compact', 'Padding', 'compact');

% Colormap for iterations
cmap_gvimp = cool(n_plots_gvimp);
cmap_pcsmp = autumn(n_plots_pcsmp);

% --- GVI-MP ---
nexttile
hold on
plot_configuration_obstacles()

for idx = 1:n_plots_gvimp
    iter = iters_gvimp(idx);
    
    means_iter = means_gvimp(:, iter);
    means_iter = reshape(means_iter, [dim_theta, nt_gvimp]);
    
    plot(means_iter(1, :), means_iter(2, :), '-', 'Color', [cmap_gvimp(idx, :), 0.7], 'LineWidth', 1.5);
    scatter(means_iter(1, :), means_iter(2, :), 15, cmap_gvimp(idx, :), 'fill', 'MarkerFaceAlpha', 0.6);
end

scatter(start_conf(1), start_conf(2), 100, 'r', 'fill');
scatter(end_conf(1), end_conf(2), 100, 'g', 'fill');

colormap(gca, cmap_gvimp);
cb = colorbar;
cb.Ticks = linspace(0, 1, min(5, n_plots_gvimp));
cb.TickLabels = round(linspace(1, niters, min(5, n_plots_gvimp)));
cb.Label.String = 'Iteration';

title('GVI-MP', 'FontSize', 14);
hold off

% --- PCS-MP ---
nexttile
hold on
plot_configuration_obstacles()

for idx = 1:n_plots_pcsmp
    iter = iters_pcsmp(idx);
    
    means_iter = means_pcsmp(:, iter);
    means_iter = reshape(means_iter, [dim_theta, nt_pcsmp]);
    
    plot(means_iter(1, :), means_iter(2, :), '-', 'Color', [cmap_pcsmp(idx, :), 0.7], 'LineWidth', 1.5);
    scatter(means_iter(1, :), means_iter(2, :), 15, cmap_pcsmp(idx, :), 'fill', 'MarkerFaceAlpha', 0.6);
end

scatter(start_conf(1), start_conf(2), 100, 'r', 'fill');
scatter(end_conf(1), end_conf(2), 100, 'g', 'fill');

colormap(gca, cmap_pcsmp);
cb = colorbar;
cb.Ticks = linspace(0, 1, min(5, n_plots_pcsmp));
cb.TickLabels = round(linspace(1, niters_pcs, min(5, n_plots_pcsmp)));
cb.Label.String = 'Iteration';

title('PCS-MP', 'FontSize', 14);
hold off

sgtitle('Trajectory Evolution Comparison', 'FontSize', 14, 'FontWeight', 'bold');


%% ================== Cost convergence comparison ==================
% If cost data is available for both methods
figure('Position', [50, 50, 600, 400]);
hold on

% GVI-MP cost
if exist('costs_gvimp', 'var')
    plot(1:niters, costs_gvimp, 'b-', 'LineWidth', 2, 'DisplayName', 'GVI-MP');
end

% PCS-MP cost (read if available)
costs_pcsmp_file = [prefix+"/cost.csv"];
if isfile(costs_pcsmp_file)
    costs_pcsmp = csvread(costs_pcsmp_file);
    plot(1:niters_pcs, costs_pcsmp, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 2, 'DisplayName', 'PCS-MP');
end

xlabel('Iteration', 'FontSize', 12);
ylabel('Cost', 'FontSize', 12);
title('Cost Convergence', 'FontSize', 14);
legend('Location', 'best');
grid on;
hold off