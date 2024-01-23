clear all
close all
clc

%% ******************* Read datas ******************
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*
prefix = ["/home/hongzhe/git/VIMP/vimp/data/2d_Arm/"];

means = csvread([prefix + "mean_base.csv"]);
covs = csvread([prefix + "cov_base.csv"]);
precisions = csvread([prefix + "precisoin_base.csv"]);
costs = csvread([prefix + "cost_base.csv"]);
sdfmap = csvread([prefix + "map_two_obs.csv"]);
factor_costs = csvread([prefix + "factor_costs_base.csv"]);
addpath("error_ellipse");

% means = csvread([prefix + "mean.csv"]);
% covs = csvread([prefix + "cov.csv"]);
% precisions = csvread([prefix + "precisoin.csv"]);
% costs = csvread([prefix + "cost.csv"]);
% sdfmap = csvread([prefix + "map_two_obs.csv"]);
% factor_costs = csvread([prefix + "factor_costs.csv"]);
% addpath("error_ellipse");

% ----- parameters -----
[niters, ttl_dim] = size(means);
dim_theta = 4;
% niters
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

%% ==================  plot for paper experiment: using the gradual changing colors ================
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
            i_vec_covs_2d{j+1} = i_cov(j*dim_theta +1 : j*dim_theta+2,  j*dim_theta+1 : j*dim_theta+2);
        end
        vec_means{i_iter+1} = i_vec_means_2d;
        vec_covs{i_iter+1} = i_vec_covs_2d;
end

% -------- start and end conf -------- 
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';

x0 = 500;
y0 = 500;
width = 600;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, floor(nsteps/2), 'TileSpacing', 'tight', 'Padding', 'tight')
for i_iter = 1: nsteps
    nexttile
    title(['Iteration ', num2str(i_iter*step_size)])
    hold on 

    i_vec_means_2d = vec_means{i_iter};
    i_vec_covs_2d = vec_covs{i_iter};
    hold on 
    plotEvidenceMap2D(sdfmap, origin_x, origin_y, cell_size);
    for j = 1:n_states
        % gradual changing colors
        
        alpha = (j / n_states)^(1.15);
        color = [0, 0, 1, alpha];
        % means
        plotPlanarArm1(arm.fk_model(), i_vec_means_2d{j}', color, 2);
%         pause(0.2), hold off
        % covariance
%         error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
    end
%     hold on
%     plotEvidenceMap2D(sdfmap, origin_x, origin_y, cell_size);
%     % means
%     plotPlanarArm(arm.fk_model(), i_vec_means_2d{j}', 'c', 2);
%     plotPlanarArm(arm.fk_model(), start_conf, 'b', 2);
%     plotPlanarArm(arm.fk_model(), end_conf, 'r', 2);
%     hold off
end

%% ================= plot the final iteration ===================
x0 = 50;
y0 = 50;
width = 350;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight') 
i_vec_means_2d = vec_means{nsteps};
i_vec_covs_2d = vec_covs{nsteps};
hold on 
plotEvidenceMap2D(sdfmap, origin_x, origin_y, cell_size);
for j = 1:n_states
    % gradual changing colors
    alpha = (j / n_states)^(1.15);
    color = [0, 0, 1, alpha];
    % means
    plotPlanarArm1(arm.fk_model(), i_vec_means_2d{j}', color, 2);
end
plotPlanarArm(arm.fk_model(), start_conf, 'r', 2);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 2);
hold off

%% ================ plot the total costs ================
figure 
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight') 
grid minor 
hold on
plot(costs, 'LineWidth', 2.2);
title('total Loss')
xlabel('iterations')
ylabel('cost')

%% =============== plot cost for each factor and the total cost ================
fixed_prior_costs = [factor_costs(1:end, 1), factor_costs(1:end, end)];
prior_costs = [];
for i = 1:n_states-1
    prior_costs = [prior_costs, factor_costs(1:end, 1+(i-1)*2+1)];
end
obs_costs = [];
for i = 1:n_states-2
    obs_costs = [obs_costs, factor_costs(1:end, 1+(i-1)*2+2)];
end

x0 = 50;
y0 = 50;
width = 1000;
height = 500;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 3, 'TileSpacing', 'tight', 'Padding', 'tight') 
nexttile

title('Prior cost factors')
hold on
grid on
plot(prior_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), prior_costs(1:niters, 1:end), 30, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(x_k))','fontweight','bold')

nexttile
title('Collision cost factors')
hold on
grid on
plot(obs_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), obs_costs(1:niters, 1:end), 30, 'filled')
xlabel('Iterations','fontweight','bold')
ylabel('-log(p(z|x_k))','fontweight','bold')

% --- entropy
entropy_costs = [];
n_dim = size(precisions, 2);
for i = 1:niters
    precision_i  = precisions((i-1)*n_dim+1: i*n_dim, 1:end);
    entropy_costs = [entropy_costs, log(det(precision_i))/2];
end

nexttile
title('Entropy cost factors')
hold on
grid on
plot(entropy_costs, 'LineWidth', 1.5)
scatter(linspace(1,niters, niters), entropy_costs(1:niters), 30, 'filled')
xlabel('Iterations', 'fontweight', 'bold')
ylabel('log(|\Sigma^{-1}|)/2', 'Interpreter', 'tex', 'fontweight', 'bold')

% verify that the sum of the factored costs is the same as the total cost
sum_fact_costs = sum(factor_costs(1:niters, 1:end), 2);
diff = sum_fact_costs + entropy_costs' - costs(1:niters)

% ================ plot the total costs ================
nexttile([1 3])
title('Total Loss')
grid on 
hold on
plot(costs(1:niters), 'LineWidth', 2.0);
scatter(linspace(1, niters, niters), costs(1:niters), 30, 'fill')
xlabel('Iterations','fontweight','bold')
ylabel('V(q)','fontweight','bold')
hold off

%% create map and save
% dataset = generate2Ddataset('OneObstacleDataset');
% rows = dataset.rows;
% cols = dataset.cols;
% cell_size = dataset.cell_size;
% origin_point2 = Point2(dataset.origin_x, dataset.origin_y);
% csvwrite( '../vimp/data/2d_Arm/map.csv', dataset.map);
