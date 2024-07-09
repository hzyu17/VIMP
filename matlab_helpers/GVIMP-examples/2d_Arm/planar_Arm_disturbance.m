%% Add disturbance in the map to simulate the perception noise
% Hongzhe Yu

close all
clear all
clc

import gtsam.*
import gpmp2.*

addpath("../../");
addpath("../../tools");
addpath("../../tools/2dArm");
addpath('../../tools/gtsam_toolbox');
addpath("../../tools/error_ellipse");

dim_state = 4;

% -------------- gvi-mp results ---------------
prefix = ["sparse_gh/map1/case1/"];
    
% --------------- high temperature ------------------
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precision.csv"]);

[ttl_dim, ~] = size(means);
niters = find_niters(means);
n_states = floor(ttl_dim / dim_state);

means_gvimp_niter = reshape(means(1:end, niters), [4, n_states]);
covs_niter = reshape(covs(1:end, niters), [4,4,n_states]);

%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');

%  ------- sdf --------
cell_size = 0.01;
origin_x = -1;
origin_y = -1;
origin_point2 = Point2(origin_x, origin_y);


%% =========================== GPMP2 optimization ===========================
% map dataset
dataset = generate2Ddataset('OneObstacleDataset');

% start and end conf
start_conf = [0, 0]';
start_vel = [0, 0]';
end_conf = [pi/2, 0]';
end_vel = [0, 0]';

nt_gpmp2 = 40;
means_gpmp2 = optimize_gpmp2(dataset, nt_gpmp2);

x0 = 50;
y0 = 50;
width = 400;
height = 350;

%% ============================ Plot mean trajectories ============================ 
% -------------------- plot GVI-MP mean trajectory --------------------
figure(2)
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'none')

nexttile(1)
hold on
dataset = generate2Ddataset('OneObstacleDataset');

% plot world
plotEvidenceMap2D_arm(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);

for j = 1:n_states
    config_ij = means_gvimp_niter(1:2, j);
    sample_trajectory(1:2, j) = config_ij;

    % gradual changing colors
    alpha = (j / n_states)^(1.15);
    color = [0, 0, 1, alpha];

    % plot
    plotPlanarArm1(arm.fk_model(), sample_trajectory(1:2, j), color, 5, true);

    % Pause to display the plot
    pause(0.1); 

end
    
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('GVI-MP')


nexttile(2)
hold on

% plot world
plotEvidenceMap2D_arm(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);

for ii=0:nt_gpmp2  
    % gradual changing colors
    alpha = (ii / nt_gpmp2)^(1.15);
    color = [0, 0, 1, alpha];

    % plot arm        
    conf = means_gpmp2(:, ii+1);
    plotPlanarArm1(arm.fk_model(), conf, color, 5, true);
    
    % Pause to display the plot
    pause(0.1)
end

plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('GPMP2')


%% ============================== Plot configuration space trajectory ==============================
% -------------- plot configuration obstacles ----------------
figure
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
nexttile
hold on

plot_configuration_obstacles()

% plot gvimp results
nt_gvi = size(means_gvimp_niter, 2);
for i=1:nt_gvi
    scatter(means_gvimp_niter(1, i), means_gvimp_niter(2, i), 20, 'k', 'fill');
    error_ellipse(covs_niter(1:2,1:2,i), means_gvimp_niter(1:2, i), 'style', 'b-.');
end

% plot gpmp2 results
for i = 1:1:nt_gpmp2
    scatter(means_gpmp2(1, i), means_gpmp2(2, i), 100, 'd', 'g', 'fill');
end

% plot start and goal conf
scatter(start_conf(1), start_conf(2), 100, 'r', 'fill');
scatter(end_conf(1), end_conf(2), 100, 'g', 'fill');

hold off

%% ============================ Comparisons ============================ 

gvimp_col = false;
gpmp2_col = false;
cnt_gvimp_col = 0;
cnt_gpmp2_col = 0;
cnt_gpmp2_replan_fail = 0;

% --------------- Sample map with uncertainty ---------------
for sigma_map = 1:6
    
    for i_map = 1:2000

        % ----------------- generate noise map -----------------
        % params 
        dataset_noisy.cols = 300;
        dataset_noisy.rows = 300;
        dataset_noisy.origin_x = -1;
        dataset_noisy.origin_y = -1;
        dataset_noisy.cell_size = 0.01;
        % map
        dataset_noisy.map = zeros(dataset_noisy.rows, dataset_noisy.cols);

        % sample the obstacle location
        randN_1 = randn()*sigma_map;
        randN_2 = randn()*sigma_map;

        % obstacles
        dataset_noisy.map = add_obstacle([190+randN_1, 160+randN_2], [60, 80], dataset_noisy.map);

    %     dataset_noisy = generate2Ddataset_1('OneObstacleDatasetNoisy');
        cell_size = dataset_noisy.cell_size;

        % signed distance field
        field_noisy = signedDistanceField2D(dataset_noisy.map, cell_size);
        sdfmap_noisy = PlanarSDF(origin_point2, cell_size, field_noisy);    

        % collision costs
        epsilon = 10.0;
        cost_sigma = 0.1;

        % ===============================================================
        % planar collision factor arm for computing the collision costs
        % ===============================================================
        planar_sdf_arm = ObstaclePlanarSDFFactorArm(symbol('x', 0), arm, sdfmap_noisy, cost_sigma, epsilon);

%         sample_trajectory = means_gvimp_niter;
%         figure
%         hold on
%         % plot map
%         plotEvidenceMap2D_arm(dataset_noisy.map, origin_x, origin_y, cell_size);
%         plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%         plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
%         axis off
%         title('GVI-MP')
        % ------------------ plot GVI-MP ----------------
        for j = 1:n_states
            config_ij = means_gvimp_niter(1:2, j);
            err_vec = planar_sdf_arm.evaluateError(config_ij);
            dist_vec = epsilon - err_vec;

            if min(dist_vec) < 0.0
    %             disp("GVI-MP collision!")
    %             gvimp_col = gvimp_col + 1;
                gvimp_col = true;
            end
            
%             while min(dist_vec) < 0.0
%                 % re-sampling
%                 config_ij = mvnrnd(means_gvimp_niter(1:2, j), covs_niter(1:2,1:2, j), 1)';
%                 err_vec = planar_sdf_arm.evaluateError(config_ij);
%                 dist_vec = epsilon - err_vec;
%             end
% 
%             sample_trajectory(1:2, j) = config_ij;
%             
%             % gradual changing colors
%             alpha = (j / nt_gpmp2)^(1.15);
%             color = [0, 0, 1, alpha];
%             
%             % plot
%             plotPlanarArm1(arm.fk_model(), sample_trajectory(1:2, j), color, 5, true);
%             
%             % Pause to display the plot
%             pause(0.01); 

        end
    %     
    %     plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
    %     plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
    %     
    %     nexttile(2)
    %     hold on
    %     % plot world
    %     plotEvidenceMap2D(dataset_noisy.map, dataset_noisy.origin_x, dataset_noisy.origin_y, cell_size);
    %     plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
    %     plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
    %     title('GPMP2')

        % ------------------ plot GPMP2 ----------------
        for i=0:nt_gpmp2
            config_ij_gpmp2 = means_gpmp2(1:2, i+1);
            err_vec = planar_sdf_arm.evaluateError(config_ij_gpmp2);
            dist_vec_gpmp2 = epsilon - err_vec;

            if min(dist_vec_gpmp2) < 0.0
    %             disp("GPMP2 collision!")
    %             gpmp2_col = gpmp2_col + 1;
                gpmp2_col = true;
                
            end
            
            % gradual changing colors
    %         alpha = (i / nt_gpmp2)^(1.15);
    %         color = [0, 0, 1, alpha];

    %         % plot arm        
    %         conf = means_gpmp2(:, i+1);
    %         plotPlanarArm1(arm.fk_model(), conf, color, 5, true);
    %         pause(0.01)
        end
    % 
    %     plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
    %     plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);

        if gvimp_col
            cnt_gvimp_col = cnt_gvimp_col+1;
            gvimp_col = false;
        end

        if gpmp2_col
            cnt_gpmp2_col = cnt_gpmp2_col+1;
            gpmp2_col = false;
            
%             % ======================== re-plan gpmp2 ========================
%             means_gpmp2_replan = optimize_gpmp2(dataset);
%             
%             % -------------- check if re-plan succeed --------------
%             for i=0:nt_gpmp2
%                 config_ij_gpmp2 = means_gpmp2_replan(1:2, i+1);
%                 err_vec = planar_sdf_arm.evaluateError(config_ij_gpmp2);
%                 dist_vec_gpmp2 = epsilon - err_vec;
% 
%                 if min(dist_vec_gpmp2) < 0.0
%         %             disp("GPMP2 collision!")
%         %             gpmp2_col = gpmp2_col + 1;
%                     gpmp2_replan_fail = true;
%                 end
%             end
%             
%             if gpmp2_replan_fail
%                 cnt_gpmp2_replan_fail = cnt_gpmp2_replan_fail + 1;
%                 gpmp2_replan_fail = false;
%                 
%                 figure
%                 set(gcf,'position',[x0,y0,width,height])
%                 tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
%                 nexttile
%                 hold on
%                 plotEvidenceMap2D(dataset_noisy.map, dataset_noisy.origin_x, dataset_noisy.origin_y, cell_size);
%                 plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%                 plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
%                 
%                 for ii=0:nt_gpmp2
%                     config_ij_gpmp2 = means_gpmp2_replan(1:2, ii+1);
% 
%                     % gradual changing colors
%                     alpha = (ii / nt_gpmp2)^(1.15);
%                     color = [0, 0, 1, alpha];
% 
%                     % plot arm        
%                     conf = means_gpmp2_replan(:, ii+1);
%                     plotPlanarArm1(arm.fk_model(), conf, color, 5, true);
%                     pause(0.1)
%                 end
% 
%                 plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%                 plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
%             end
            
        end

    end

sigma_map

cnt_gvimp_col
cnt_gpmp2_col
cnt_gpmp2_replan_fail

cnt_gvimp_col = 0;
cnt_gpmp2_col = 0;
cnt_gpmp2_replan_fail = 0;

end

function [map, landmarks] = add_obstacle(position, size, map, landmarks, origin_x, origin_y, cell_size)

half_size_row = floor((size(1)-1)/2);
half_size_col = floor((size(2)-1)/2);

% occupency grid
map(position(1)-half_size_row : position(1)+half_size_row, ...
    position(2)-half_size_col : position(2)+half_size_col) ...
    = ones(2*half_size_row+1, 2*half_size_col+1); 

% landmarks
if nargin == 7
    for x = position(1)-half_size_row-1 : 4 : position(1)+half_size_row-1
        y = position(2)-half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        y = position(2)+half_size_col-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
    
    for y = position(2)-half_size_col+3 : 4 : position(2)+half_size_col-5
        x = position(1)-half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
        x = position(1)+half_size_row-1;
        landmarks = [landmarks; origin_y+y*cell_size, origin_x+x*cell_size];
    end
end

end

function center = get_center(x,y,dataset)

center = [y - dataset.origin_y, x - dataset.origin_x]./dataset.cell_size;

end

function dim = get_dim(w,h,dataset)

dim = [h, w]./dataset.cell_size;

end

