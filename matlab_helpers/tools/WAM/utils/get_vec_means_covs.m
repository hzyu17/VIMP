function [vec_means, vec_covs] = get_vec_means_covs(means, covs, niters, nsteps, dim_theta)
% Obtain the vector of mean and covariance values of size of the number of iterations
%   Detailed explanation goes here

% containers for all the steps data
vec_means = cell(niters, 1);
vec_covs = cell(niters, 1);

dim_conf = dim_theta / 2;
[ttl_dim, ~] = size(means);
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

for i_iter = 0: nsteps-1
        % each time step 
        i = i_iter * step_size;
        i_mean = means(:,i+1);
        i_cov = covs(:,i+1);
        i_mean = reshape(i_mean, dim_theta,n_states);
        i_cov = reshape(i_cov, dim_theta,dim_theta, n_states);
        i_vec_means = cell(n_states, 1);
        i_vec_covs = cell(n_states, 1);
        for j = 0:n_states-1
            % each state
            i_vec_means{j+1} = i_mean(1 : dim_conf, j+1);
            cov_i = i_cov(1:dim_conf,1:dim_conf, j+1);
            cov_i = (cov_i + cov_i') / 2;

            i_vec_covs{j+1} = cov_i;
        end
        vec_means{i_iter+1} = i_vec_means;
        vec_covs{i_iter+1} = i_vec_covs;
end
end