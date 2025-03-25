% 7DOF WAM arm example, build factor graph in matlab
% @author Jing Dong

close all
clear

% addpath('../../tools')
% addpath ('../../tools/WAM/utils')
% addpath('../../tools/gtsam_toolbox')
% addpath("../../tools/error_ellipse");

addpath('../../tools')
addpath("/usr/local/gtsam_toolbox")

import gtsam.*
import gpmp2.*


% 2 configurations
start_confs = [-1.57, -0.261, -3.14, -1.047, 3.14, -0.785, 0];
end_confs = [0.0, 0.0, 0.0, -0.261, 3.14, -0.261, 0];

for i_exp = 1:1

%% dataset
dataset = generate3Ddataset_1('PR2DeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% arm: WAM arm
arm = generateArm('PR2Arm');

start_conf = start_confs(i_exp,1:end)';
end_conf = end_confs(i_exp,1:end)';
start_vel = zeros(7,1);
end_vel = zeros(7,1);

% plot problem setting
figure, hold on
ttl = ['Problem Settings ', i_exp];
title(ttl)
plotMap3D(dataset.corner_idx, origin, cell_size);
plotRobotModel(arm, start_conf)
plotRobotModel(arm, end_conf)
% plot config
set3DPlotRange(dataset)
grid on, view(3)
hold off


%% settings
total_time_sec = 2;
total_time_step = 750;
total_check_step = 1500;
delta_t = total_time_sec / total_time_step;
check_inter = total_check_step / total_time_step - 1;

% GP
Qc = 1 * eye(7);
Qc_model = noiseModel.Gaussian.Covariance(Qc); 

% algo settings
cost_sigma = 0.02;
epsilon_dist = 0.2;

% noise model
fix_sigma = 0.0001;
pose_fix_model = noiseModel.Isotropic.Sigma(7, fix_sigma);
vel_fix_model = noiseModel.Isotropic.Sigma(7, fix_sigma);

% init sdf
sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

% plot settings
plot_inter_traj = false;
plot_inter = 4;
if plot_inter_traj
    total_plot_step = total_time_step * (plot_inter + 1);
else
    total_plot_step = total_time_step;
end
pause_time = total_time_sec / total_plot_step;


%% initial traj
init_values = initArmTrajStraightLine(start_conf, end_conf, total_time_step);

%% init optimization
graph = NonlinearFactorGraph;
graph_obs = NonlinearFactorGraph;

for i = 0 : total_time_step
    key_pos = symbol('x', i);
    key_vel = symbol('v', i);
   
    % priors
    if i==0
        graph.add(PriorFactorVector(key_pos, start_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, start_vel, vel_fix_model));
    elseif i==total_time_step
        graph.add(PriorFactorVector(key_pos, end_conf, pose_fix_model));
        graph.add(PriorFactorVector(key_vel, end_vel, vel_fix_model));
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
        graph.add(ObstacleSDFFactorArm(...
            key_pos, arm, sdf, cost_sigma, epsilon_dist));
        graph_obs.add(ObstacleSDFFactorArm(...
            key_pos, arm, sdf, cost_sigma, epsilon_dist));
        
        % GP cost factor
        if check_inter > 0
            for j = 1:check_inter
                tau = j * (total_time_sec / total_check_step);
                graph.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
                graph_obs.add(ObstacleSDFFactorGPArm( ...
                    key_pos1, key_vel1, key_pos2, key_vel2, ...
                    arm, sdf, cost_sigma, epsilon_dist, ...
                    Qc_model, delta_t, tau));
            end
        end
    end
end

%% optimize!
use_LM = true;
use_trustregion_opt = false;

if use_LM
    parameters = LevenbergMarquardtParams;
    parameters.setVerbosity('ERROR');
%     parameters.setVerbosityLM('LAMBDA');
    parameters.setlambdaInitial(1000.0);
    optimizer = LevenbergMarquardtOptimizer(graph, init_values, parameters);
elseif use_trustregion_opt
    parameters = DoglegParams;
    parameters.setVerbosity('ERROR');
    optimizer = DoglegOptimizer(graph, init_values, parameters);
else
    parameters = GaussNewtonParams;
    parameters.setVerbosity('ERROR');
    optimizer = GaussNewtonOptimizer(graph, init_values, parameters);
end

fprintf('Initial Error = %d\n', graph.error(init_values))
fprintf('Initial Collision Cost: %d\n', graph_obs.error(init_values))

optimizer.optimize();

result = optimizer.values();
result.print('Final results')

fprintf('Error = %d\n', graph.error(result))
fprintf('Collision Cost End: %d\n', graph_obs.error(result))

%% plot results
if plot_inter_traj
    plot_values = interpolateArmTraj(result, Qc_model, delta_t, plot_inter);
else
    plot_values = result;
end

% collect result configurations
gpmp2_result_confs = zeros(7, total_plot_step+1);
for i=0:total_plot_step
    conf = plot_values.atVector(symbol('x', i));
    gpmp2_result_confs(1:7, i+1) = conf;
end
prefix = ["case"+num2str(i_exp)+"/"];
csvwrite([prefix+"zk_gpmp2.csv"], gpmp2_result_confs);
    

end



