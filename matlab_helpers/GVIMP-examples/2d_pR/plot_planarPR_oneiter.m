function output = plot_planarPR_oneiter(means, covs, sdfmap, i_iter)
%% plot one iteration of the optimization process.
% Hongzhe Yu

addpath('../../tools/gtsam_toolbox')
addpath("../../");
addpath("../../tools/2dpR");

import gtsam.*
import gpmp2.*

[ttl_dim, ~] = size(means);
dim_state = 4;
nt = floor(ttl_dim / dim_state);

mean_final_iter = means(:,i_iter);
mean_final_iter = reshape(mean_final_iter, [dim_state,nt]);

cov_final_iter = covs(:,i_iter);
cov_final_iter = reshape(cov_final_iter, [dim_state,dim_state,nt]);

hold on 

plot_2d_result(sdfmap, mean_final_iter, cov_final_iter);
xlim([-15, 20])
ylim([-10, 20])

axis off

output = 1;
end