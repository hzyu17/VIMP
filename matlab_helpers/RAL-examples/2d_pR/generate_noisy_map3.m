% @brief    Generate a map narrow with noisy boundaries.
% @author   Hongzhe Yu
% @date     May 07 2024

close all
clear
addpath('/usr/local/gtsam_toolbox')
addpath('../../tools/2d_pR')
%% Load libraries
import gtsam.*
import gpmp2.*

addpath("error_ellipse");

%% dataset
dataset = generate2Ddataset_1('MultiObstacleEntropy3');
dataset_noisy = generate2Ddataset_1('MultiObstacleEntropy3Noisy');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;

% origin
origin_x = -20;
origin_y = -10;
origin_point2 = Point2(origin_x, origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% signed distance field
field_noisy = signedDistanceField2D(dataset_noisy.map, cell_size);
sdf_noisy = PlanarSDF(origin_point2, cell_size, field_noisy);

% plot map
x0 = 500;
y0 = 500;
width = 650;
height = 300;

figure
nexttile
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight') 
hold on

t = title('Noisy measurment');
t.FontSize = 20;
prefix = ["map3/circumvent/"];
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);
addpath("error_ellipse");

 %%
[niters, ttl_dim] = size(means);
dim_conf = 2;
dim_theta = 2*dim_conf;
niters = length(costs);
for i=niters:-1:1
    if costs(i) ~= 0
        niters=i;
        break
    end
end
nsteps = 6;
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

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

% plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
i_vec_means_2d = vec_means{nsteps};
i_vec_covs_2d = vec_covs{nsteps};

hold on
plotEvidenceMap2D_1(dataset.map, origin_x, origin_y, cell_size);
hold on 
plotEvidenceMap2D_noisy(dataset_noisy.map, origin_x, origin_y, cell_size);
hold on 

for j = 1:n_states
    % means
    scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 20, 'k', 'fill');
end

for j = 1:n_states
    % covariance
    error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
end

% xlim([-15, 20])
ylim([-10, 20])
axis off

%% sampling and interpolate from the joint trajectory distribution
numSamples = 1000;
samples = zeros(niters, numSamples, 2);

hold on
for i = 1:niters
    samples(i,:,:) = mvnrnd(i_vec_means_2d{i}', i_vec_covs_2d{i}, numSamples);
    
    scatter(samples(i, :,1), samples(i, :,2), 10, 'b', 'filled');
    title('Samples from a Multivariate Normal Distribution');
    xlabel('X1');
    ylabel('X2');
    axis equal;  % Ensure the scaling of axes is equal
    
end


