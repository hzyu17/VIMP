function [vec_means, vec_covs] = get_vec_means_covs(means, covs, niters, nsteps, dim_theta)
% Obtain the vector of mean and covariance values of size of the number of iterations
%   Detailed explanation goes here

% containers for all the steps data
vec_means = cell(niters, 1);
vec_covs = cell(niters, 1);

dim_conf = dim_theta / 2;
[~, ttl_dim] = size(means);
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

for i_iter = 0: nsteps-1
        % each time step 
        i = i_iter * step_size;
        i_mean = means(i+1, 1:end);
        i_cov = covs(i*ttl_dim+1 : (i+1)*ttl_dim, 1:ttl_dim);
        i_vec_means = cell(n_states, 1);
        i_vec_covs = cell(n_states, 1);
        for j = 0:n_states-1
            % each state
            i_vec_means{j+1} = i_mean(j*dim_theta+1 : j*dim_theta+dim_conf);
            i_vec_covs{j+1} = i_cov(j*dim_theta +1 : j*dim_theta+dim_conf,  j*dim_theta+1 : j*dim_theta+dim_conf);
        end
        vec_means{i_iter+1} = i_vec_means;
        vec_covs{i_iter+1} = i_vec_covs;
end
end