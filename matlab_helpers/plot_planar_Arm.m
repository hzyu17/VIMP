clear all
clc

%% ******************* Read datas ******************
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

means = csvread("../vimp/data/2d_Arm/mean.csv");
covs = csvread("../vimp/data/2d_Arm/cov.csv");
precisions = csvread("../vimp/data/2d_Arm/precisoin.csv");
costs = csvread("../vimp/data/2d_Arm/cost.csv");
sdfmap = csvread("../vimp/data/2d_Arm/map.csv");
factor_costs = csvread("../vimp/data/2d_Arm/factor_costs.csv");
addpath("error_ellipse");

% means = csvread("../vimp/data/checkpoints/2d_Arm/mean.csv");
% covs = csvread("../vimp/data/checkpoints/2d_Arm/cov.csv");
% precisions = csvread("../vimp/data/checkpoints/2d_Arm/precisoin.csv");
% costs = csvread("../vimp/data/checkpoints/2d_Arm/cost.csv");
% sdfmap = csvread("../vimp/data/checkpoints/2d_Arm/map.csv");
% factor_costs = csvread("../vimp/data/checkpoints/2d_Arm/factor_costs.csv");
% addpath("error_ellipse");

% ----- parameters -----
[niters, ttl_dim] = size(means);
dim_theta = 4;
% niters = 10;
nsteps = 10;
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);
%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');

%  ------- sdf --------
cell_size = 0.01;
origin_x = -1;
origin_y = -1;
origin_point2 = Point2(origin_x, origin_y);
field = signedDistanceField2D(sdfmap, cell_size);
% save field
% csvwrite("../vimp/data/2d_Arm/field.csv", field);
sdf = PlanarSDF(origin_point2, cell_size, field);

%% ==================  plot sdf and means and covs ================
import gtsam.*
import gpmp2.*
% ----------------------- load the means and covs on the 2*2 level -----------------------
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
            i_vec_covs_2d{j+1} = i_cov(j*dim_theta +1 : j*dim_theta+2,  ...
                                                                j*dim_theta+1 : j*dim_theta+2);
        end
        vec_means{i_iter+1} = i_vec_means_2d;
        vec_covs{i_iter+1} = i_vec_covs_2d;
end

% -------- start and end conf -------- 
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';

figure
tiledlayout(2, floor(nsteps/2), 'TileSpacing', 'tight', 'Padding', 'tight')
for i_iter = 1: nsteps
    nexttile
    
    i_vec_means_2d = vec_means{i_iter};
    i_vec_covs_2d = vec_covs{i_iter};
    for j = 1:n_states
        hold on 
        plotEvidenceMap2D(sdfmap, origin_x, origin_y, cell_size);
        % means
        plotPlanarArm(arm.fk_model(), i_vec_means_2d{j}', 'c', 2);
        pause(0.2), hold off
        % covariance
%         error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
    end
    hold on
    plotEvidenceMap2D(sdfmap, origin_x, origin_y, cell_size);
    % means
    plotPlanarArm(arm.fk_model(), i_vec_means_2d{j}', 'c', 2);
    plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
    plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
    hold off
end

%% ================ plot the total costs ================
figure
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none') 
nexttile
title('Total Loss')
grid minor 
hold on
plot(costs, 'LineWidth', 2.0, 'LineStyle', '-.');
scatter(linspace(1, length(costs), length(costs)), costs, 'fill')
xlabel('Iterations','fontweight','bold')
ylabel('V(q)','fontweight','bold')

%% ================ plot the process factor costs ================
% figure
% tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
% title('Factor costs')
% hold on
% grid on
% nexttile
% for i_iter = 1:size(factor_costs, 2)
%     plot(factor_costs(1:end, i_iter), 'LineWidth', 2)
% end
% legend({"FixedGP0","LinGP1","Obs1","LinGP2","Obs2","FixedGP1"})

%% =============== plot cost for each factor ================
fixed_prior_costs = [factor_costs(1:end, 1), factor_costs(1:end, end)];
prior_costs = [];
for i = 1:n_states-1
    prior_costs = [prior_costs, factor_costs(1:end, 1+(i-1)*2+1)];
end
obs_costs = [];
for i = 1:n_states-2
    obs_costs = [obs_costs, factor_costs(1:end, 1+(i-1)*2+2)];
end

figure
tiledlayout(1, 3, 'TileSpacing', 'tight', 'Padding', 'tight') 
nexttile
% subplot(1, 3, 1)
title('Prior cost factors')
hold on
grid minor
plot(prior_costs, 'LineWidth', 1.5, 'LineStyle','-.')
scatter(linspace(1,niters, niters), prior_costs(1:niters, 1:end), 10, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(x_k))','fontweight','bold')

% subplot(1, 3, 2)
nexttile
title('Collision cost factors')
hold on
grid minor
plot(obs_costs, 'LineWidth', 1.5, 'LineStyle','-.')
scatter(linspace(1,niters, niters), obs_costs(1:niters, 1:end), 10, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(z|x_k))','fontweight','bold')

% --- entropy
entropy_costs = [];
n_dim = size(precisions, 2);
for i = 1:niters
    precision_i  = precisions((i-1)*n_dim+1: i*n_dim, 1:end);
    entropy_costs = [entropy_costs, log(det(precision_i))/2];
end
% subplot(1, 3, 3)
nexttile
title('Entropy cost factors')
hold on
grid minor
plot(entropy_costs, 'LineWidth', 1.5, 'LineStyle','-.')
scatter(linspace(1,niters, niters), entropy_costs(1:niters), 10, 'filled')
xlabel('Iterations', 'fontweight', 'bold')
ylabel('log(|\Sigma^{-1}|)/2', 'Interpreter', 'tex', 'fontweight', 'bold')

%% create map and save
% dataset = generate2Ddataset('OneObstacleDataset');
% rows = dataset.rows;
% cols = dataset.cols;
% cell_size = dataset.cell_size;
% origin_point2 = Point2(dataset.origin_x, dataset.origin_y);
% csvwrite( '../vimp/data/2d_Arm/map.csv', dataset.map);
