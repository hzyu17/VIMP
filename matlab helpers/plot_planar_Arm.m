clear all
clc

import gtsam.*
import gpmp2.*

%% create map and save
% dataset = generate2Ddataset('OneObstacleDataset');
% rows = dataset.rows;
% cols = dataset.cols;
% cell_size = dataset.cell_size;
% origin_point2 = Point2(dataset.origin_x, dataset.origin_y);
% csvwrite( '../vimp/data/2d_Arm/map.csv', dataset.map);

%% ******************* Read datas ******************
means = csvread("../vimp/data/2d_Arm/mean.csv");
covs = csvread("../vimp/data/2d_Arm/cov.csv");
precisions = csvread("../vimp/data/2d_Arm/precisoin.csv");
costs = csvread("../vimp/data/2d_Arm/cost.csv");
sdfmap = csvread("../vimp/data/2d_Arm/map.csv");
addpath("error_ellipse");
%%
[niters, ttl_dim] = size(means);
dim_theta = 4;
nsteps = 10;
step_size = floor(niters / nsteps);
n_states = floor(ttl_dim / dim_theta);

% =========================== load the means and covs on the 2*2 level
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


%% plot sdf and means and covs
%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');
% -------- plot the means -------- 
colors = [255, 0, 0];

cell_size = 0.01;
origin_x = -1;
origin_y = -1;
origin_point2 = Point2(origin_x, origin_y);
field = signedDistanceField2D(sdfmap, cell_size);
% save field
csvwrite("../vimp/data/2d_Arm/field.csv", field);
sdf = PlanarSDF(origin_point2, cell_size, field);

% -------- start and end conf -------- 
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';

for i_iter = 1: nsteps
    subplot(2, floor(nsteps/2), i_iter)
    
    i_vec_means_2d = vec_means{i_iter};
    i_vec_covs_2d = vec_covs{i_iter};
    for j = 1:n_states
        hold on 
        plotEvidenceMap2D(sdfmap, origin_x, origin_y, cell_size);
        % means
        plotPlanarArm(arm.fk_model(), i_vec_means_2d{j}', 'c', 2);
        pause(0.5), hold off
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

% ================ plot the total costs ================
figure 
grid on 
hold on
plot(costs, 'LineWidth', 2.2);
xlabel('iterations')
ylabel('cost')

% ================ plot the process factor costs ================
factor_costs = csvread("../vimp/data/2d_pR/factor_costs.csv");
figure
hold on
grid on
for i_iter = 1:size(factor_costs, 2)
    plot(factor_costs(1:end, i_iter), 'LineWidth', 2)
end
ylim([0, 100])
legend({"FixedGP0","LinGP1","Obs1","LinGP2","Obs2","FixedGP1"})
