% @brief    Point Robot 2D example, building factor graph in matlab
% @author   Mustafa Mukadam
% @date     July 20, 2016

close all
clear
addpath('/usr/local/gtsam_toolbox')
addpath('../../tools/2dpR')
%% Load libraries
import gtsam.*
import gpmp2.*

%% small dataset
dataset = generate2Ddataset_1('MultiObstacleEntropy3');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% signed distance field
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% % plot sdf
% figure(2)
% plotSignedDistanceField2D(field, dataset.origin_x, dataset.origin_y, dataset.cell_size);
% title('Signed Distance Field')


%% settings
total_time_sec = 2.5;
total_time_step = 14;
total_check_step = 50;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% use GP interpolation
use_GP_inter = false;

% point robot model
pR = PointRobot(2,1);
spheres_data = [0  0.0  0.0  0.0  0.5];
nr_body = size(spheres_data, 1);
sphere_vec = BodySphereVector;
sphere_vec.push_back(BodySphere(spheres_data(1,1), spheres_data(1,5), ...
        Point3(spheres_data(1,2:4)')));
pR_model = PointRobotModel(pR, sphere_vec);

% GP
Qc = eye(2).*0.8;
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% obstacle cost settings
cost_sigma = 0.0055;
epsilon_dist = 0.6;

% prior to start/goal
pose_fix = noiseModel.Isotropic.Sigma(2, 0.0001);
vel_fix = noiseModel.Isotropic.Sigma(2, 0.0001);

% start and end conf
start_conf = [-3, -7]';
start_vel = [0, 0]';
end_conf = [3, 18]';
end_vel = [0, 0]';
avg_vel = (end_conf / total_time_step) / delta_t;

% plot param
pause_time = total_time_sec / total_time_step;

% % plot start / end configuration
% figure(1), hold on
% plotEvidenceMap2D(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
% plotPointRobot2D(pR_model, start_conf);
% plotPointRobot2D(pR_model, end_conf);
% title('Layout')
% hold off

x0 = 500;
y0 = 500;
width = 650;
height = 300;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'tight') 

%% init optimization
graph = NonlinearFactorGraph;
init_values = Values;

%% ================== go around initialization for the GVI ===================
nexttile
t = title('GVI, r = 1.5');
t.FontSize = 20;
prefix = ["map3/shortcut_gpmp2_comparisons/"];
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precisoin.csv"]);
costs = csvread([prefix + "cost.csv"]);
factor_costs = csvread([prefix + "factor_costs.csv"]);

sdfmap = csvread("../../../vimp/maps/2dpR/map3/map_multiobs_map3.csv");
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
    
    cell_size = 0.1;
    origin_x = -20;
    origin_y = -10;
    origin_point2 = Point2(origin_x, origin_y);
    field = signedDistanceField2D(sdfmap, cell_size);
    sdf = PlanarSDF(origin_point2, cell_size, field);
    
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
    hold on 
    plotEvidenceMap2D_1(sdfmap, origin_x, origin_y, cell_size);
    i_vec_means_2d = vec_means{nsteps};
    i_vec_covs_2d = vec_covs{nsteps};
    for j = 1:n_states
        % means
        scatter(i_vec_means_2d{j}(1), i_vec_means_2d{j}(2), 20, 'k', 'fill');
        % covariance
        error_ellipse(i_vec_covs_2d{j}, i_vec_means_2d{j});
    end
    % xlim([-15, 20])
    ylim([-10, 20])

%%  -------------------------------- go around initialization GPMP2 --------------------------------
%% init optimization
graph = NonlinearFactorGraph;
init_values = Values;

means = csvread("../../../vimp/data/vimp/2d_pR/mean_map3_circumvent_base.csv");
for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
    
    % initial values: straght line
    pose = means(i*4+1: i*4+2)';
    vel = avg_vel;
    init_values.insert(key_pos, pose);
    init_values.insert(key_vel, vel);
    
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix));
    elseif i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix));
    end
    
    % GP priors and cost factor
    if i > 0
        key_pos1 = symbol('x', i-1);
        key_pos2 = symbol('x', i);
        key_vel1 = symbol('v', i-1);
        key_vel2 = symbol('v', i);
        graph.add(GaussianProcessPriorLinear(key_pos1, key_vel1, ...
            key_pos2, key_vel2, delta_t, Qc_model));
        
        % cost factor
        graph.add(ObstaclePlanarSDFFactorPointRobot(...
            key_pos, pR_model, sdf, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if use_GP_inter & check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstaclePlanarSDFFactorGPPointRobot( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    pR_model, sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

%% optimize!
use_trustregion_opt = false;

if use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end
tic
optimizer.optimize();
toc
result = optimizer.values();
% result.print('Final results')

%% plot final values
nexttile
hold on
t1 = title('GPMP2, r = 0.5');
t1.FontSize = 20;
% plot world
plotEvidenceMap2D_1(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
for i=0:total_time_step
    % plot arm
    conf = result.atVector(symbol('x', i));
    plotPointRobot2D_1(pR_model, conf);
%     pause(pause_time), hold off
end
ylim([-10, 20])