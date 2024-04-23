%% Stochastic obstacles and re-sampling experiment
% Hongzhe Yu

% =================== read gvimp results ====================
means_gvimp = csvread([prefix_gvimp+"/mean.csv"]);
covs_gvimp = csvread([prefix_gvimp+"/cov.csv"]);
costs_gvimp = csvread([prefix_gvimp+"/cost.csv"]);
[ttl_dim, niters] = size(means_gvimp);
nt_gvimp = floor(ttl_dim/dim_theta);
% n_states = floor(ttl_dim / dim_theta);

% =================== read pgcs results ====================
means_pgcs = csvread([prefix+"/zk_sdf.csv"]);
covs_pgcs = csvread([prefix+"/Sk_sdf.csv"]);

% ----- parameters -----
[ndim, nt] = size(means_pgcs);
covs_pgcs = reshape(covs_pgcs, dim_theta, dim_theta, nt);

%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');

%  ------- sdf --------
cell_size = 0.01;
origin_x = -1;
origin_y = -1;
origin_point2 = Point2(origin_x, origin_y);
field = signedDistanceField2D(sdfmap, cell_size);
% save field
sdf = PlanarSDF(origin_point2, cell_size, field);

