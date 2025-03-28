function output = plot_planarPR_oneiter(means, covs, sdfmap, i_iter, varargin)
%% plot one iteration of the optimization process.
% Hongzhe Yu

addpath('../../tools/gtsam_toolbox');
addpath("../../");
addpath("../../tools/2dQuad");

import gtsam.*
import gpmp2.*

cell_size = 0.1;
origin_x = -20;
origin_y = -20;

% Update default values based on the number of optional input arguments (order is cell_size, origin_x, origin_y)
if ~isempty(varargin)
    cell_size = varargin{1};
end
if length(varargin) >= 2
    origin_x = varargin{2};
end
if length(varargin) >= 3
    origin_y = varargin{3};
end

[ttl_dim, ~] = size(means);
dim_state = 6;
nt = floor(ttl_dim / dim_state);

mean_final_iter = means(:,i_iter);
mean_final_iter = reshape(mean_final_iter, [dim_state,nt]);

cov_final_iter = covs(:,i_iter);
cov_final_iter = reshape(cov_final_iter, [dim_state,dim_state,nt]);

hold on 

plot_2d_result(sdfmap, mean_final_iter, cov_final_iter, cell_size, origin_x, origin_y);
xlim([-15, 20])
ylim([-10, 20])

% axis off

output = 1;
end