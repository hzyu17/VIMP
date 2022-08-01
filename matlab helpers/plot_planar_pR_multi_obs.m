% sdf_multi_obs = csvread("../vimp/data/2d_pR/map_multiobs.csv");
% field_multi_obs = csvread("../vimp/data/2d_pR/field_multiobs.csv");
% 
% [nmesh_y, nmesh_x] = size(sdf_multi_obs);
% x_mesh = linspace(0, nmesh_x, nmesh_x);
% y_mesh = linspace(0, nmesh_y, nmesh_y);
% [X,Y] = meshgrid(x_mesh, y_mesh);
% figure
% contourf(X,Y,sdf_multi_obs, 1)
% figure
% contourf(X,Y,field_multi_obs, 20)

%% ******************* Read datas ******************
means = csvread("../vimp/data/2d_pR/mean_multiobs.csv");
covs = csvread("../vimp/data/2d_pR/cov_multiobs.csv");
costs = csvread("../vimp/data/2d_pR/cost_multiobs.csv");
sdfmap = csvread("../vimp/data/2d_pR/map_multiobs.csv");
addpath("error_ellipse");

[niters, ttl_dim] = size(means)
dim_theta = 4;
nsteps = 10;
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

% =========================== load the means and covs on the 2*2 level
% containers for all the steps data
vec_means = cell(niters, 1);
vec_covs = cell(niters, 1);

for i_iter = 0: nsteps-1
        % each step updates
        i = i_iter * step_size;
        i_mean = means(i+1, 1:end);
        i_cov = covs(i*ttl_dim+1 : (i+1)*ttl_dim, 1:ttl_dim);
        i_vec_means_2d = cell(n_states, 1);
        i_vec_covs_2d = cell(n_states, 1);
        for j = 0:n_states-1
            i_vec_means_2d{j+1} = i_mean(j*dim_theta+1 : j*dim_theta+2);
            i_vec_covs_2d{j+1} = i_cov(j*dim_theta +1 : j*dim_theta+2,  j*dim_theta+1 : j*dim_theta+2);
        end
        vec_means{i_iter+1} = i_vec_means_2d;
        vec_covs{i_iter+1} = i_vec_covs_2d;
end

%% plot sdf and means and covs
figure
title("SDF 2D Point Robot")
% -------- plot the means -------- 
colors = [255, 0, 0];

cell_size = 0.1;
origin_x = -20;
origin_y = -10;

for i_iter = 1: nsteps
    subplot(2, floor(nsteps/2), i_iter)
    hold on 
    % -------- plot the sdf mesh contour -------- 
    [nmesh_y, nmesh_x] = size(sdfmap);
    x_mesh = linspace(origin_x, origin_x+nmesh_x*cell_size, nmesh_x);
    y_mesh = linspace(origin_y, origin_y+nmesh_y*cell_size, nmesh_y);
    [X,Y] = meshgrid(x_mesh, y_mesh);
    contourf(X,Y,sdfmap, 1)

    i_vec_means_2d = vec_means{i_iter};
    i_vec_covs_2d = vec_covs{i_iter};
    for j = 1:n_states
        % means
        scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 40, 'r*');
        % covariance
        error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
    end
end

figure 
grid on 
hold on
plot(costs, 'LineWidth', 2.2);
xlabel('iterations')
ylabel('cost')