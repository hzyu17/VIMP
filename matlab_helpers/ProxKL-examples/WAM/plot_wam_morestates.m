clear all
close all
clc

%% ******************* Read datas ******************
addpath('../../tools')
addpath ('../../tools/WAM/utils')
addpath('../../tools/gtsam_toolbox')
addpath("../../tools/error_ellipse");

import gtsam.*
import gpmp2.*

% =================================  
% full grid GH quadrature results 
% ================================= 
% prefix = "case1";

% =============================
% sparse GH quadrature results 
% =============================
i_exp = 2;

switch i_exp
    case 1
        prefix = "sparse_gh/case3";
    case 2
        prefix = "sparse_gh/case4";
end
% prefix_gpmp = "../../PGCS-examples/WAM/sparse_gh/case1";

%% ******************* Start and goal configurations ******************
start_confs = [-0.8,   -1.70,   1.64,  1.29,   1.1, -0.106,    2.2;
               -0.9,  -1.70,   1.34,  1.19,   0.8,  -0.126,    2.5];
end_confs = [-0.0,    0.94,     0,     1.6,     0,   -0.919,   1.55;
              -0.7,   1.35,    1.2,    1.0,   -0.7,  -0.1,   1.2];

%% read experiment results
means = csvread([prefix+"/mean.csv"]);
% mean_last = csvread([prefix+"/zk_sdf.csv"]);
mean_last = csvread([prefix+"/zk_gpmp2.csv"]);
costs = csvread([prefix+"/cost.csv"]);
factor_costs = csvread([prefix+"/factor_costs.csv"]);

%% ******************* Define parameters ******************
% ----- parameters -----
[ttl_dim, niters] = size(means);
dim_theta = 14;
dim_conf = 14 / 2;

nsteps = 4;

% niters
niters = length(costs);
for i=niters:-1:1
    if costs(i) ~= 0
        niters=i;
        break
    end
end
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);


%% ******************* Define map dataset ******************
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
% origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

%% ******************* WAM Arm ******************
arm = generateArm('WAMArm');

% start and goal 
start_conf = start_confs(i_exp, 1:end)';
end_conf = end_confs(i_exp, 1:end)';

start_vel = zeros(7,1);
end_vel = zeros(7,1);

%% ================= plot the final iteration, only mean value ===================
% ----- parameters -----
niters = find_niters(means);
dim_state = 14;
nt = size(means, 1) / dim_state;

% ----- figure settings -----

% x0 = 50;
% y0 = 50;
% width = 400;
% height = 350;
% figure
% set(gcf,'position',[x0,y0,width,height])
% tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')

% nexttile
% t=title(['Supported state mean values']);
% t.FontSize = 14;

x0 = 50;
y0 = 50;
width = 400;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')

nexttile
% t=title(['Iteration ', num2str(niters)]);
t.FontSize = 16;
i_means = means(:, niters);
i_means = reshape(i_means, [dim_state, nt]);

hold on 
view(3)
plotMap3D(dataset.corner_idx, origin, cell_size);
for j = 1:nt
    % gradual changing colors
    alpha = (j / nt)^(1.15);
    color = [0, 0, 1, alpha];
    % means
    plotArm3D(arm.fk_model(), mean_last(1:7, j), color, 4, true);
end
plotArm3D(arm.fk_model(), start_conf, 'r', 6, true);
plotArm3D(arm.fk_model(), end_conf, 'g', 6, true);
hold off

% %% ================= plot costs ===================
% output = plot_costs(costs, factor_costs, precisions, niters, n_states, dim_state);
