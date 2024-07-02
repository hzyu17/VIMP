% @brief    Comparison of the go-through and the go-around plan.
% @author   Hongzhe Yu
% @date     May 01 2024

clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("map3/map_multiobs_map3.csv");

v_niters = [30, 30];
v_nsteps = [6, 6];
nsteps = 6;

x0 = 500;
y0 = 500;
width = 650;
height = 300;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight') 
tiledlayout(1, 2, 'TileSpacing', 'None', 'Padding', 'None')
for i = 1:2 % 2 experiments
    nexttile
    if i == 1
        t = title('Go Through Plan');
        t.FontSize = 20;
        prefix = ["map3/shortcut/"];
        means = csvread([prefix + "mean.csv"]);
        covs = csvread([prefix + "cov.csv"]);
        precisions = csvread([prefix + "precisoin.csv"]);
        costs = csvread([prefix + "cost.csv"]);
        factor_costs = csvread([prefix + "factor_costs.csv"]);
         addpath("error_ellipse");

    else
        t = title('Go Around Plan');
        t.FontSize = 20;
        prefix = ["map3/circumvent/"];
        means = csvread([prefix + "mean.csv"]);
        covs = csvread([prefix + "cov.csv"]);
        precisions = csvread([prefix + "precisoin.csv"]);
        costs = csvread([prefix + "cost.csv"]);
        factor_costs = csvread([prefix + "factor_costs.csv"]);
         addpath("error_ellipse");

    end

    %%
    [niters, ttl_dim] = size(means);
    dim_conf = 2;
    dim_theta = 2*dim_conf;

    niters = length(costs);
    for ii=niters:-1:1
        if costs(ii) ~= 0
            niters=ii;
            break
        end
    end
    step_size = floor(niters / nsteps);
    n_states = floor(ttl_dim / dim_theta);
    
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
            i_step = i_iter * step_size;
            i_mean = means(i_step+1, 1:end);
            i_cov = covs(i_step*ttl_dim+1 : (i_step+1)*ttl_dim, 1:ttl_dim);
            i_prec = precisions(i_step*ttl_dim+1 : (i_step+1)*ttl_dim, 1:ttl_dim);
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
    hold on 
    plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
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

    %% ====== statistics of the cost distributions ======
    % Factor Order: [fixed_gp_0, lin_gp_1, obs_1, ..., lin_gp_(N-1), obs_(N-1), lin_gp_(N), fixed_gp_(N)] 
    % --- prior
    prior_costs = [];
    prior_costs = [prior_costs, factor_costs(1:end, 1)];
    for iii = 1:n_states-1
        prior_costs = [prior_costs, factor_costs(1:end, 1+(iii-1)*2+1)];
    end
    prior_costs = [prior_costs, factor_costs(1:end, end)];

    
    % --- collision
    obs_costs = [];
    for iii = 1:n_states-2
        obs_costs = [obs_costs, factor_costs(1:end, 1+(iii-1)*2+2)];
    end
    
    % --- entropy
    entropy_costs = [];
    n_dim = size(precisions, 2);
    
    for ii = 1:niters
        precision_i  = precisions((ii-1)*n_dim+1: ii*n_dim, 1:end);
        entropy_costs = [entropy_costs, log(det(precision_i))/2];
    end
    
    disp(['========== prior cost ', num2str(i),  '==========='])
    sum(prior_costs(niters,1:end))
    
    disp(['========== obs cost ', num2str(i),  '==========='])
    sum(obs_costs(niters, 1:end))

    disp(['========== motion planning cost ', num2str(i),  '==========='])
    sum(obs_costs(niters, 1:end)) + sum(prior_costs(niters,1:end))
    
    disp(['========== entropy cost ', num2str(i),  '==========='])
    entropy_costs(niters)
    
    disp(['========== total cost ', num2str(i),  '==========='])
    costs(niters)

end
