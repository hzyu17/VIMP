clear all
close all
clc

%% ******************* Read datas ******************
addpath('../../tools/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../../tools/error_ellipse");
addpath('../../tools/2dArm');

map = 1;
exp = 1;

prefix = "map1";
prefix_gpmp2 = "map1";
prefix_gvimp = "map1";
switch map
    case 1
        prefix = "map1";
        sdfmap = csvread("map1/map.csv");
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

    case 2
        sdfmap = csvread("map2/map.csv");
        prefix_gpmp2 = "map2";
        switch exp
            case 1
                prefix = "map2/case1";
                % boundary conditions
                start_conf = [0, 0]';
                start_vel = [0, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
            case 2
                prefix = "map2/case2";
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
means_pgcs = csvread([prefix+"/zk_sdf.csv"]);
covs_pgcs = csvread([prefix+"/Sk_sdf.csv"]);

% ----- parameters -----
[ndim, nt] = size(means_pgcs);
covs_pgcs = reshape(covs_pgcs, dim_theta, dim_theta, nt);

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

%% ================= plot the final iteration ===================
x0 = 50;
y0 = 50;
width = 400;
height = 350;
% ==================== plot GVIMP results ===================
means_gvimp_lastiter = means_gvimp(:,end);
means_gvimp_lastiter = reshape(means_gvimp_lastiter, [dim_theta,nt_gvimp]);
covs_gvimp_lastiter = covs_gvimp(:, end);
covs_gvimp_lastiter = reshape(covs_gvimp_lastiter, [dim_theta, dim_theta, nt_gvimp]);
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
nexttile
% t=title('GVI-MP');
% t.FontSize = 26;

hold on 
plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);
for j = 1:nt_gvimp
    % gradual changing colors
    alpha = (j / nt_gvimp)^(1.15);
    color = [0, 0, 1, alpha];
    % means
    plotPlanarArm1(arm.fk_model(), means_gvimp_lastiter(1:2, j), color, 8, true);
end
plotPlanarArm(arm.fk_model(), start_conf, 'r', 8);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 8);
xlim([-1, 1.5])
ylim([-0.8, 1.5])
hold off
axis off


% ==================== plot GPMP2 results ===================
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
nexttile
% t=title('GPMP2');
% t.FontSize = 26;

hold on 
plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);

% read gpmp2 results
means_gpmp2 = csvread([prefix_gpmp2+"/zt_gpmp2.csv"]);
nt_gpmp2 = size(means_gpmp2, 2);
% plot gpmp2 results
for j = 1:1:nt_gpmp2
    % means
    plotPlanarArm1(arm.fk_model(), means_gpmp2(1:2,j), 'c', 8, true);
end
plotPlanarArm1(arm.fk_model(), start_conf, 'r', 8, true);
plotPlanarArm1(arm.fk_model(), end_conf, 'g', 8, true);
xlim([-1, 1.5])
ylim([-0.8, 1.5])
axis off

% ==================== plot PGCS-MP results ===================
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
nexttile
% t=title('PGCS-MP');
% t.FontSize = 26;

hold on 
plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);

for j = 1:2:nt
    % gradual changing colors
    alpha = (j / nt)^(1.15);
    color = [0, 0, 1, alpha];
    % means
    plotPlanarArm1(arm.fk_model(), means_pgcs(1:2,j), color, 8, true);
end
plotPlanarArm1(arm.fk_model(), start_conf, 'r', 8, true);
plotPlanarArm1(arm.fk_model(), end_conf, 'g', 8, true);
xlim([-1, 1.5])
ylim([-0.8, 1.5])
axis off;
hold off


% ===================================== 
% configuration space trajectory 
% =====================================
% -------------- plot configuration obstacles ----------------

figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
nexttile
hold on

plot_configuration_obstacles()

% read configuration space obstacle mesh
% meshx = csvread("../../../vimp/maps/2dArm/configuration_obs_meshx.csv");
% meshy = csvread("../../../vimp/maps/2dArm/configuration_obs_meshy.csv");
% [meshx, meshy] = meshgrid(v_theta1, v_theta2);
% meshz = ones(size(meshx));

% cell_number = 300;
% configuration_obs = csvread("../../../vimp/maps/2dArm/config_obs.csv");
% origin_x_config = -3.1415926;
% origin_y_config = -3.1415926;
% cell_size_config = 3.1415926*2/cell_number;
% plotEvidenceMap2D_arm(configuration_obs, origin_x_config, origin_y_config, cell_size_config);

% t=title("2-link arm");
t.FontSize = 26;

hold on 

% plot pgcs results
nt = size(means_pgcs, 2);
for i=1:nt
    scatter(means_pgcs(1, i), means_pgcs(2, i), 20, 'k', 'fill');
    error_ellipse(covs_pgcs(1:2,1:2,i), means_pgcs(1:2, i));
end

% plot gvimp results
nt_gvi = size(means_gvimp_lastiter, 2);
for i=1:nt_gvi
    scatter(means_gvimp_lastiter(1, i), means_gvimp_lastiter(2, i), 20, 'k', 'fill');
    error_ellipse(covs_gvimp_lastiter(1:2,1:2,i), means_gvimp_lastiter(1:2, i), 'style', 'b-.');
end

% plot gpmp2 results
for i = 1:1:nt_gpmp2
    scatter(means_gpmp2(1, i), means_gpmp2(2, i), 100, 'd', 'g', 'fill');
end

% plot start and goal conf
scatter(start_conf(1), start_conf(2), 100, 'r', 'fill');
scatter(end_conf(1), end_conf(2), 100, 'g', 'fill');

hold off

%% ================= plot samples ===============
% ------------------ 
% gvi-mp 
% ------------------
n_plots = 10;
stepsize = floor(nt_gvimp/n_plots);

pos_figsample = 1.0e+03 .*[0.2026, 1.3822, 1.0276, 0.1828];

figure
set(gcf,'position',pos_figsample)
tiledlayout(1, floor(n_plots/2), 'TileSpacing', 'none', 'Padding', 'none')

plot_config_samples(sdfmap, arm, means_gvimp_lastiter, covs_gvimp_lastiter, ...
                     1, floor(nt_gvimp/2), stepsize, start_conf, end_conf);

figure
set(gcf,'position',pos_figsample)
tiledlayout(1, floor(n_plots/2), 'TileSpacing', 'none', 'Padding', 'none')

plot_config_samples(sdfmap, arm, means_gvimp_lastiter, covs_gvimp_lastiter, ...
                     floor(nt_gvimp/2)+stepsize, nt_gvimp, stepsize, start_conf, end_conf);

% ------------------ pgcs-mp ------------------
n_plots = 10;
n_samples = 10;

stepsize = floor(nt/n_plots);

figure
set(gcf,'position',pos_figsample)
tiledlayout(1, floor(n_plots/2), 'TileSpacing', 'none', 'Padding', 'none')

plot_config_samples(sdfmap, arm, means_pgcs, covs_pgcs, ...
                     1, floor(nt/2), stepsize, start_conf, end_conf);

figure
set(gcf,'position',pos_figsample)
tiledlayout(1, floor(n_plots/2), 'TileSpacing', 'none', 'Padding', 'none')

plot_config_samples(sdfmap, arm, means_pgcs, covs_pgcs, ...
                     floor(nt/2)+stepsize, nt, stepsize, start_conf, end_conf);

% %% ================= plot costs =================
% costs = csvread([prefix+"/costs.csv"]);
% x0 = 50;
% y0 = 50;
% width = 400;
% height = 350;
% figure
% set(gcf,'position',[x0,y0,width,height])
% tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
% nexttile
% t=title("Total Cost");
% t.FontSize = 26;
% hold on 
% % grid minor
% % plot(costs(1,:), 'LineWidth', 2.5)
% % plot(costs(2,:), 'LineWidth', 2.5)
% plot(costs(3,:), 'LineWidth', 2.5)
% xlabel('Iterations')
% ylabel('Cost')

% %% ==== animated motion plan ==== 
% x0 = 50;
% y0 = 50;
% width = 400;
% height = 350;
% figure(2)
% set(figure(2),'position',[x0,y0,width,height])
% tiledlayout(1, 2, 'TileSpacing', 'none', 'Padding', 'none')
% nexttile
% t=title("Animation");
% t.FontSize = 16;
% 
% % ---------- plot the mean positions ---------- 
% for j = 1:nt
%     hold on 
%     figure(2)
%     plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);
%     plotPlanarArm1(arm.fk_model(), start_conf, 'r', 2, true);
%     plotPlanarArm1(arm.fk_model(), end_conf, 'g', 2, true);
%     % gradual changing colors
% %     alpha = (j / nt)^(1.15);
%     color = [0, 0, 1, 1];
%     % means
%     plotPlanarArm1(arm.fk_model(), means(1:2,j), color, 2, true);
% %     hold off
%     pause(0.01)
% end

% %% ==== plot sampled covariance for the states ==== 
% x0 = 500;
% y0 = 500;
% width = 600;
% height = 350;
% figure
% set(gcf,'position',[x0,y0,width,height])
% 
% tiledlayout(2, 3, 'TileSpacing', 'tight', 'Padding', 'tight')
% 
% n_samples = 50;
% for j = 1:3:ndim
%     j
%     nexttile
%     t = title(['Support State ',num2str(j)]);
%     t.FontSize = 16;
%     hold on 
%     i_vec_means_2d = vec_means{nsteps};
%     i_covs_2d = vec_covs{nsteps};
%     plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);
% 
%     % gradual changing colors
% %     alpha = (j / n_states)^(1.15);
%     color = [0, 0, 1, 0.9];
%     color_sample = [0.0, 0.0, 0.7, 0.02];
%     % mu j
%     mean_j = i_vec_means_2d{j}';
%     % cov j
%     cov_j = i_covs_2d{j};
%     % sampling 
%     rng('default')  % For reproducibility
%     samples = mvnrnd(mean_j, cov_j, n_samples);
%     plotPlanarArm1(arm.fk_model(), i_vec_means_2d{j}', color, 4, true);
%     for k = 1: size(samples, 1)
%         k_sample = samples(k, 1:end)';
%         plotPlanarArm1(arm.fk_model(), k_sample, color_sample, 3, false);
%     end
%     % means
% %     plotPlanarArm1(arm.fk_model(), , color, 2);
% plotPlanarArm1(arm.fk_model(), start_conf, 'r', 3, true);
% plotPlanarArm1(arm.fk_model(), end_conf, 'g', 3, true);
% xlim([-1 1.5])
% ylim([-0.5 1.5])
% end
% 
% % final step
% j = 15;
% nexttile
% t = title(['Support State ',num2str(j)]);
% t.FontSize = 16;
% hold on 
% i_vec_means_2d = vec_means{nsteps};
% i_covs_2d = vec_covs{nsteps};
% plotEvidenceMap2D_arm(sdfmap, origin_x, origin_y, cell_size);
% 
% % gradual changing colors
% %     alpha = (j / n_states)^(1.15);
% color = [0, 0, 1, 0.9];
% color_sample = [0.0, 0.0, 0.7, 0.02];
% % mu j
% mean_j = i_vec_means_2d{j}';
% % cov j
% cov_j = i_covs_2d{j};
% % sampling 
% rng('default')  % For reproducibility
% samples = mvnrnd(mean_j, cov_j, n_samples);
% plotPlanarArm1(arm.fk_model(), i_vec_means_2d{j}', color, 4, true);
% for k = 1: size(samples, 1)
%     k_sample = samples(k, 1:end)';
%     plotPlanarArm1(arm.fk_model(), k_sample, color_sample, 3, false);
% end
% % means
% %     plotPlanarArm1(arm.fk_model(), , color, 2);
% plotPlanarArm1(arm.fk_model(), start_conf, 'r', 3, true);
% plotPlanarArm1(arm.fk_model(), end_conf, 'g', 3, true);
% xlim([-1 1.5])
% ylim([-0.5 1.5])
% 
% hold off

%% create map and save
% dataset = generate2Ddataset('OneObstacleDataset');
% rows = dataset.rows;
% cols = dataset.cols;
% cell_size = dataset.cell_size;
% origin_point2 = Point2(dataset.origin_x, dataset.origin_y);
% csvwrite( '../vimp/data/2d_Arm/map.csv', dataset.map);
