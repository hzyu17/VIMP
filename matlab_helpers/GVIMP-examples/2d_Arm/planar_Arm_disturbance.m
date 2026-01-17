%% Add disturbance in the map to simulate the perception noise
% Hongzhe Yu
close all
clear all
clc
import gtsam.*
import gpmp2.*

%% ******************* Setup paths ******************
vimp_root = setup_vimp();
matlab_helpers = fullfile(vimp_root, 'matlab_helpers');
addpath(matlab_helpers);
addpath(fullfile(matlab_helpers, 'tools'));
addpath(fullfile(matlab_helpers, 'tools', '2dArm'));
addpath(fullfile(matlab_helpers, 'tools', 'gtsam_toolbox'));
addpath(fullfile(matlab_helpers, 'tools', 'error_ellipse'));

dim_state = 4;
%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');
%  ------- sdf --------
cell_size = 0.01;
origin_x = -1;
origin_y = -1;
origin_point2 = Point2(origin_x, origin_y);
x0 = 50;
y0 = 50;
width = 400;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'none')
nexttile
for i_map = 1:1
    % generate noise map
    dataset_noisy = generate2Ddataset_1('OneObstacleDataset');
    cell_size = dataset_noisy.cell_size;
    % signed distance field
    field_noisy = signedDistanceField2D(dataset_noisy.map, cell_size);
    sdfmap_noisy = PlanarSDF(origin_point2, cell_size, field_noisy);    
    
    prefix = fullfile(matlab_helpers, 'GVIMP-examples', '2d_Arm', 'sparse_gh', 'map1', 'case1');
    
    % --- high temperature ---
    means = csvread(fullfile(prefix, 'mean.csv'));
    covs = csvread(fullfile(prefix, 'cov.csv'));
    precisions = csvread(fullfile(prefix, 'precision.csv'));
    [ttl_dim, ~] = size(means);
    niters = find_niters(means);
    n_states = floor(ttl_dim / dim_state);
    
    means_niter = reshape(means(1:end, niters), [4, n_states]);
    covs_niter = reshape(covs(1:end, niters), [4,4,n_states]);
    
    % plot map
    plotEvidenceMap2D_arm(dataset_noisy.map, origin_x, origin_y, cell_size);
    hold on
    
    axis off
    
    % plot start and end configs
    
    start_conf = [0, 0]';
    start_vel = [0, 0]';
    end_conf = [pi/2, 0]';
    end_vel = [0, 0]';
    
    plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
    plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
    
    % collision costs
    epsilon = 10.0;
    cost_sigma = 0.1;
    % planar collision factor arm
    planar_sdf_arm = ObstaclePlanarSDFFactorArm(symbol('x', 0), arm, sdfmap_noisy, cost_sigma, epsilon);
    
    sample_trajectory = means_niter;
    
    for j = 1:n_states
        config_ij = means_niter(1:2, j);
        err_vec = planar_sdf_arm.evaluateError(config_ij);
        dist_vec = epsilon - err_vec;
        
        % gradual changing colors
        alpha = (j / n_states)^(1.15);
        color = [0, 0, 1, alpha];
        
        while min(dist_vec) < 0.0
            config_ij = mvnrnd(means_niter(1:2, j), covs_niter(1:2,1:2, j), 1)';
            err_vec = planar_sdf_arm.evaluateError(config_ij);
            dist_vec = epsilon - err_vec;
        
            disp("collision!")
        end
        
        sample_trajectory(1:2, j) = config_ij;
        
        % plot
        plotPlanarArm1(arm.fk_model(), sample_trajectory(1:2, j), color, 5, true);
        
        % Pause to display the plot
        pause(0.1); 
    
    end
    
    plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
    plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
end