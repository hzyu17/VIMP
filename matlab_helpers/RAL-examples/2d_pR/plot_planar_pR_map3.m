clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("map_narrow/map_multiobs_entropy_map3.csv");

v_niters = [18, 24];
v_nsteps = [6, 6];

figure
tiledlayout(1, 2, 'TileSpacing', 'None', 'Padding', 'None')
for i = 1:2 % 4 experiments
    nexttile
    if i == 1
        title('Go Through Plan')
        prefix = ["map_narrow/shortcut/"];
        means = csvread([prefix + "mean.csv"]);
        covs = csvread([prefix + "cov.csv"]);
        precisions = csvread([prefix + "precisoin.csv"]);
        costs = csvread([prefix + "cost.csv"]);
        factor_costs = csvread([prefix + "factor_costs.csv"]);
         addpath("error_ellipse");

    else
        title('Go Around Plan')
        prefix = ["map_narrow/circumvent/"];
        means = csvread([prefix + "mean.csv"]);
        covs = csvread([prefix + "cov.csv"]);
        precisions = csvread([prefix + "precisoin.csv"]);
        costs = csvread([prefix + "cost.csv"]);
        factor_costs = csvread([prefix + "factor_costs.csv"]);
         addpath("error_ellipse");

    end

    %%
    [niters, ttl_dim] = size(means);

    dim_theta = 4;
    niters = v_niters(i);
    nsteps = v_nsteps(i);
    step_size = floor(niters / nsteps);
    n_states = floor(ttl_dim / dim_theta);

    % final entropy cost
    disp(['========== final entropy cost ', num2str(i),  '==========='])
    n_dim = size(precisions, 2);
    precision_i  = precisions((niters-1)*n_dim+1: niters*n_dim, 1:end);
    entropy_cost = log(det(precision_i))/2

    costs(niters)
        
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
    
    %% ================ plot the last iteration ================ 
%     figure
%     tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
%     nexttile
%     title(['Iteration ', num2str(nsteps*step_size)])
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
    % xlim([-15, 20])
    ylim([-10, 20])
end
