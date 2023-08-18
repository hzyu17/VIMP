function output = plotPointRobotFinalIteration(means, covs, precisions, costs, factor_costs, sdfmap, niters, nsteps)
%%
addpath('/home/.hyu419/.local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

[~, ttl_dim] = size(means);
dim_theta = 4;
if nargin == 6
    % niters
    niters = length(costs);
    for i=niters:-1:1
        if costs(i) ~= 0
            niters=i;
            break
        end
    end
    nsteps = 6;
end

 %%
    [niters, ttl_dim] = size(means);
    dim_theta = 4;
    step_size = floor(niters / nsteps);
    n_states = floor(ttl_dim / dim_theta);
       
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

    axis off

output = 1;
end