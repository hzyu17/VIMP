addpath('../../tools')
close all
clear all
clc
%% ================ read data ================
sdfmap = csvread("map2/map_multiobs_map2.csv");
prefix = ["map2/case4/"];

means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "joint_precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

niters = find_niters(means);
nt = 15;
dim_state = 4;

x0 = 500;
y0 = 500;
width = 600;
height = 380;

cell_size = 0.1;
origin_x = -20;
origin_y = -10;

num_figures = 10;
niter_lowtemp = 15;
step_size_lowtemp = floor(niter_lowtemp / (num_figures/2));

% --------------------- low temperature planning ---------------------
figure
hold on
t1=tiledlayout(1, floor(num_figures/2), 'TileSpacing', 'none', 'Padding', 'none');
title(t1,'Low temperature planning for collision avoidance','fontweight','bold','fontsize',16)
set(gcf,'position',[x0,y0,width,height])

for i_step = 1:step_size_lowtemp:niter_lowtemp
    nexttile
    hold on
    means_final = means(1:end, i_step);
    means_final = reshape(means_final, [4, nt]);
    cov_final = covs(:, i_step);
    cov_final = reshape(cov_final, dim_state, dim_state, nt);
    plot_2d_result(sdfmap, means_final, cov_final);
end

% ---------- high temperature planning ---------- 
n_figures_hightemp = floor(num_figures/2);
num_iter_hightemp = niters - niter_lowtemp;
stepsize_hightemp = floor(num_iter_hightemp / n_figures_hightemp);
figure
t2=tiledlayout(1, n_figures_hightemp, 'TileSpacing', 'none', 'Padding', 'none');
title(t2,'High temperature for high-entropy robustness','fontweight','bold','fontsize',16)
% tiledlayout(1, floor(num_figures/2), 'TileSpacing', 'none', 'Padding', 'none')
set(gcf,'position',[x0,y0,width,height])
for i_step = niter_lowtemp + 1 : stepsize_hightemp : niters
    nexttile
    hold on
    means_final = means(1:end, i_step);
    means_final = reshape(means_final, [4, nt]);
    cov_final = covs(:, i_step);
    cov_final = reshape(cov_final, dim_state, dim_state, nt);
    plot_2d_result(sdfmap, means_final, cov_final);
end

%% plot ground truth iterations
addpath('../../tools/gtsam_toolbox')
addpath("../../tools/error_ellipse");
import gtsam.*
import gpmp2.*
% tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 4:4 % 4 experiments
%     nexttile
    prefix = ["../../RAL-examples/2d_pR/map2/exp"+num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "mean.csv"]);
    covs = csvread([prefix + "cov.csv"]);
    precisions = csvread([prefix + "precisoin.csv"]);
    costs = csvread([prefix + "cost.csv"]);
    
    factor_costs = csvread([prefix + "factor_costs.csv"]);    
    
    [niters, ttl_dim] = size(means);
    dim_theta = 4;

    step_size = 1;
    n_states = 15;
    
    % plot the hinge loss
    cell_size = 0.1;
    origin_x = -20;
    origin_y = -10;
    origin_point2 = Point2(origin_x, origin_y);
    field = signedDistanceField2D(sdfmap, cell_size);
    sdf = PlanarSDF(origin_point2, cell_size, field);
    
    % containers for all the steps data
    vec_means = cell(niters, 1);
    vec_covs = cell(niters, 1);
    vec_precisions = cell(niters, 1);
    
    for i_iter = 0: niters-1
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
    
    % ================ plot the iterations ================ 
    figure
    tiledlayout(2, floor(niters/2), 'TileSpacing', 'none', 'Padding', 'none')
    for i_step = 1:niters
        nexttile
    %     title(['Iteration ', num2str(nsteps*step_size)])

        hold on 

        %     plotSignedDistanceField2D(field, origin_x, origin_y, cell_size);
        plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
        % mesh(mesh_X, mesh_Y, mesh_hingeloss, 'FaceAlpha', 0.5);
        i_vec_means_2d = vec_means{i_step};
        i_vec_covs_2d = vec_covs{i_step};
        for j = 1:n_states
            % means
            scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 20, 'k', 'fill');
            % covariance
            error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
        end
    end
    
    % xlim([-15, 20])
    ylim([-10, 20])
end