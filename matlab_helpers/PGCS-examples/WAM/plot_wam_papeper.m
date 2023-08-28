clear all
close all
clc

% ******************* dependencies and includes ******************
addpath('/usr/local/gtsam_toolbox')
addpath ('../../tools/WAM/utils')

import gtsam.*
import gpmp2.*

addpath("../../tools");
addpath("../../tools/error_ellipse");
addpath("../../tools/WAM/utils")

% ******************* Define map dataset ******************
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% ******************* WAM Arm and start and end conf ******************
arm = generateArm('WAMArm');

% ================= figure configs and start and goal states ===================
x0 = 50;
y0 = 50;
width = 400;
height = 350;

start_conf = [-0.8,  -1.70, 1.64,  1.29,   1.1,    -0.106,   2.2;
            -0.9,    -1.70,   1.34,  1.19,   0.8,    -0.126,   2.5;
            -1.8,    -1.50,   1.84,  1.29,   1.5,    0.26,     0.2];
end_conf = [-0.0,    0.94,     0,    1.6,    0,     -0.919,     1.55;
          -0.7,     1.35,     1.2,   1.0,   -0.7,    -0.1,      1.2;
          -0.0,     0.6,     -0.5,   0.2,    0.2,    0.8,       1.15];
      
nx = 14;

% ================= data reading and plots ===================
for i_exp = 2:2 
    % ====================================================================================== 
    %                                   read data
    % ====================================================================================== 

    prefix = ["case"+num2str(i_exp)+"/"];
    prefix_gvi = ["../../GVIMP-examples/WAM/case"+num2str(i_exp)+"/"];
    prefix_gpmp2 = ["case"+num2str(i_exp)+"/gpmp2"];
    
    % ------------ read gpmp2 results ------------ 
    gpmp2_confs = csvread([prefix_gpmp2+"/zk_sdf.csv"]);
    [nx_gpmp2, nt_gpmp2] = size(gpmp2_confs);
    
    % ------------  read gvi-mp results ------------ 
    means_gvi = csvread([prefix_gvi + "zk_sdf.csv"]);
    [~, nt_gvi] = size(means_gvi);
    
    covs_gvi = csvread([prefix_gvi + "Sk_sdf.csv"]);
    covs_gvi = reshape(covs_gvi, [14, 14, nt_gvi]);
    
    % ------------  read pgcs-mp results ------------ 
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);

    [nx, nt] = size(means);
    covs = reshape(covs, [nx,nx,nt]);
    
    
    % ====================================================================================== 
    %                                   plot mean trajectories
    % ====================================================================================== 
    
    % --------------------------- plot gpmp2 results ---------------------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    
    nexttile
    grid on
    hold on 
    view(-14.7458, 9.8376);
    plotMap3D(dataset.corner_idx, origin, cell_size);
    
    for j = 1:nt_gpmp2
        % gradual changing colors
        alpha = (j / nt_gpmp2)^(1.15);
        color = [0, 1, 1, alpha];
        % means
        plotArm3D(arm.fk_model(), gpmp2_confs(:, j), color, 4, true);
    end

    plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 6, true);
    for jj=1:5
        plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 6, true);
    end
    axis off;
    xlim([-1, 1.4])
    ylim([-1, 1.0])
    zlim([-0.8, 0.9])
    
    % --------------------------- plot gvi-mp results ---------------------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    
    nexttile
    grid on
    hold on 
    view(-14.7458, 9.8376);
    plotMap3D(dataset.corner_idx, origin, cell_size);
    
    for j = 1:nt_gvi
        % gradual changing colors
        alpha = (j / nt_gvi)^(1.15);
        color = [0.2706, 0.2706, 1, alpha];
        % means
        plotArm3D(arm.fk_model(), means_gvi(1:7, j), color, 4, true);
    end

    plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 6, true);
    for jj=1:5
        plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 6, true);
    end
    axis off;
    xlim([-1, 1.4])
    ylim([-1, 1.0])
    zlim([-0.8, 0.9])

    
    % --------------------------- plot pgcs-mp results ---------------------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    
    nexttile
    grid on
    hold on 
    view(-14.7458, 9.8376);
    plotMap3D(dataset.corner_idx, origin, cell_size);
    for j = 1:nt
        % gradual changing colors
        alpha = (j / nt)^(1.15);
        color = [0, 0, 0.5020, alpha];
        % means
        plotArm3D(arm.fk_model(), means(:, j), color, 4, true);
    end

    plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 6, true);
    for jj=1:5
        plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 6, true);
    end
    axis off;
    xlim([-1, 1.4])
    ylim([-1, 1.0])
    zlim([-0.8, 0.9])
    
    %%
    % ====================================================================================== 
    %                           plot means and samples
    % ====================================================================================== 
    % ------------- pgcs-mp -------------
    n_plots = 10;
    n_rows = 1;
    n_samples = 10;
    
    stepsize = floor(nt/n_plots);
    
    pos_figsample = 1.0e+03 .*[0.2026, 1.3822, 1.0276, 0.1828];
    
    % ---------- first row -----------
    figure
    set(gcf,'position',pos_figsample)
    tiledlayout(1, 5, 'TileSpacing', 'none', 'Padding', 'none')

    plot_samples_WAM(dataset, origin, cell_size, arm, means, covs, 1, floor(nt/2), stepsize, ...
                     start_conf(i_exp,1:end)', end_conf(i_exp,1:end)');
    % ---------- 2nd row -----------
    figure
    set(gcf,'position',pos_figsample)
    tiledlayout(1, 5, 'TileSpacing', 'none', 'Padding', 'none')

    plot_samples_WAM(dataset, origin, cell_size, arm, means, covs, floor(nt/2)+stepsize, nt, stepsize, ...
                     start_conf(i_exp,1:end)', end_conf(i_exp,1:end)');
                 
    % ------------- gvi-mp -------------
    n_plots = 10;
    n_samples = 10;
    stepsize = floor(nt_gvi/n_plots);
    
    % ---------- first row -----------
    figure
    set(gcf,'position',pos_figsample)
    tiledlayout(1, 5, 'TileSpacing', 'none', 'Padding', 'none')

    plot_samples_WAM(dataset, origin, cell_size, arm, means_gvi, covs_gvi, 1, floor(nt_gvi/2), stepsize, ...
                     start_conf(i_exp,1:end)', end_conf(i_exp,1:end)');
    % ---------- 2nd row -----------     
    figure
    set(gcf,'position',pos_figsample)
    tiledlayout(1, 5, 'TileSpacing', 'none', 'Padding', 'none')

    plot_samples_WAM(dataset, origin, cell_size, arm, means_gvi, covs_gvi, floor(nt_gvi/2)+stepsize, nt_gvi, stepsize, ...
                     start_conf(i_exp,1:end)', end_conf(i_exp,1:end)');

%     for j = 1:stepsize:nt_gvi
%         nexttile
%         hold on 
%         view(-14.7458, 9.8376);
%         plotMap3D(dataset.corner_idx, origin, cell_size);
%         % gradual changing colors
%         alpha_samples = 0.3;
%         color = [0, 0, 1, 1];
%         color_samples = [0, 0, 1, alpha_samples];
%         % sample from covariance
%         n_samples = 10;
%         for i_sample = 1:n_samples
%             % mu j
%             mean_j = means_gvi(1:7, j);
%             % cov j
%             cov_j = covs_gvi(1:7, 1:7, j);
% 
%             % means
%             plotArm3D(arm.fk_model(), mean_j, color, 8, true);
% 
%             % sampling 
%             rng('default')  % For reproducibility
%             samples = mvnrnd(mean_j, cov_j, n_samples);
%             for k = 1: size(samples, 1)
%                 k_sample = samples(k, 1:end)';
%                 plotArm3D(arm.fk_model(), k_sample, color_samples, 4, false);
%             end
%         end
%         plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 6, true);
%         plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 6, true);
%         xlim([-1, 1.5])
%         ylim([-0.8, 1.5])
%         hold off
%         axis off
% 
%     end    
    
    % ====================================================================================== 
    %                           plot configuration space marginals
    % ====================================================================================== 
    % ------------- q1 q2 q3 -------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    nexttile
    hold on
    grid on
    plot_3d_result(means(1:3,1:end), covs(1:3,1:3,1:end));
    
    % ------------- gvimp q1 q2 q3 -------------
    plot_3d_result(means_gvi(1:3,1:end), covs_gvi(1:3,1:3,1:end));

    % ------------- gpmp2 q1 q2 q3 -------------
    for i_gpmp2 = 1:nt_gpmp2
        scatter3(gpmp2_confs(1, i_gpmp2), gpmp2_confs(2, i_gpmp2), gpmp2_confs(3, i_gpmp2), 20, 'blue', 'filled','d');
    end
    set(gca,'fontsize',16);
    xlabel('Joint $q_1$','Interpreter','latex'),ylabel('Joint $q_2$','Interpreter','latex');
    zlabel('Joint $q_3$','Interpreter','latex');
    
    % ------------- q4 q5 -------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    nexttile
    grid on
    hold on
    plot_2d_result_no_sdf(means(4:5,1:end), covs(4:5,4:5,1:end));
    
    % ------------- gvimp q4 q5 -------------
    plot_2d_result_no_sdf(means_gvi(4:5,1:end), covs_gvi(4:5,4:5,1:end));
    
    for i_gpmp2 = 1:nt_gpmp2
        scatter(gpmp2_confs(4, i_gpmp2), gpmp2_confs(5, i_gpmp2), 20, 'blue', 'filled','d');
    end

    set(gca,'fontsize',16);
    xlabel('Joint $q_4$','Interpreter','latex'),ylabel('Joint $q_5$','Interpreter','latex');

    % ------------- q6 q7 -------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    nexttile
    grid on
    hold on
    plot_2d_result_no_sdf(means(6:7,1:end), covs(6:7,6:7,1:end));
    
    % ------------- gvimp q4 q5 -------------
    plot_2d_result_no_sdf(means_gvi(6:7,1:end), covs_gvi(6:7,6:7,1:end));
    
    for i_gpmp2 = 1:nt_gpmp2
        scatter(gpmp2_confs(6, i_gpmp2), gpmp2_confs(7, i_gpmp2), 20, 'blue', 'filled','d');
    end

    set(gca,'fontsize',16);
    xlabel('Joint $q_6$','Interpreter','latex'),ylabel('Joint $q_7$','Interpreter','latex');
   
end
