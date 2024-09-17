%% Add disturbance in the map to simulate the perception noise
% Hongzhe Yu

close all
clear all
clc

addpath("../../");
addpath("../../tools");
addpath("../../tools/2dArm");
addpath('../../tools/gtsam_toolbox');
addpath("../../tools/error_ellipse");

dim_state = 4;

%% ======================= gvi-mp results =======================
import gtsam.*
import gpmp2.*

prefix = ["sparse_gh/map1/case1/"];
    
% --------------- high temperature ------------------
means = csvread([prefix + "mean.csv"]);
covs = csvread([prefix + "cov.csv"]);
precisions = csvread([prefix + "precision.csv"]);

[ttl_dim, ~] = size(means);
niters = find_niters(means);
n_states = floor(ttl_dim / dim_state);

means_gvimp_niter = reshape(means(1:end, niters), [4, n_states]);
covs_gvimp_niter = reshape(covs(1:end, niters), [4,4,n_states]);


% =================== read pcs-mp results ====================
prefix_pcs = "../../PGCS-examples/2d_Arm/map1/case1";
means_pcsmp = csvread([prefix_pcs+"/zk_sdf.csv"]);
covs_pcsmp = csvread([prefix_pcs+"/Sk_sdf.csv"]);

% ----- parameters -----
[ndim, nt_pcs] = size(means_pcsmp);
covs_pcsmp = reshape(covs_pcsmp, 4, 4, nt_pcs);

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

nt_gpmp2 = 50;
means_gpmp2 = optimize_gpmp2(dataset, nt_gpmp2);

csvwrite('zt_gpmp2.csv', means_gpmp2);

x0 = 50;
y0 = 50;
width = 400;
height = 350;

%% ============================ Plot mean trajectories ============================ 
% -------------------- plot GVI-MP mean trajectory --------------------
figure(2)
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 3, 'TileSpacing', 'tight', 'Padding', 'none')

nexttile(1)
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
    pause(0.05)
end

plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('GPMP2')

nexttile(2)
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
    pause(0.05); 

end
    
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('GVI-MP')

% ------------------------ pcs-mp results ------------------------
nexttile(3)
hold on
dataset = generate2Ddataset('OneObstacleDataset');

% plot world
plotEvidenceMap2D_arm(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);

for j = 1:nt_pcs
    config_ij = means_pcsmp(1:2, j);
    sample_trajectory(1:2, j) = config_ij;

    % gradual changing colors
    alpha = (j / nt_pcs)^(1.15);
    color = [0, 0, 1, alpha];

    % plot
    plotPlanarArm1(arm.fk_model(), sample_trajectory(1:2, j), color, 5, true);

    % Pause to display the plot
    pause(0.05); 

end
    
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('PCS-MP')

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
    error_ellipse(covs_gvimp_niter(1:2,1:2,i), means_gvimp_niter(1:2, i), 'style', 'b-.');
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
pcsmp_col = false;
gpmp2_replan_fail = false;

v_cnt_gvimp_col = [];
v_cnt_pcsmp_col = [];
v_cnt_gpmp2_col = [];
v_cnt_gpmp2_replan_fail = [];

cnt_gpmp2_col = 0;
cnt_gvimp_col = 0;
cnt_pcsmp_col = 0;
cnt_gpmp2_replan_fail = 0;


min_dist_gvimp_ever = 0;
worst_case_map = [];
found_new_worstcase = false;
worst_case_gvimp_sample_trj = means_gvimp_niter;

% fix random seed
rng(123);

% --------------- Sample map with uncertainty ---------------
for sigma_map = 1:3
    
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

        % ------------------ check collision GVI-MP ----------------
        gvimp_sample_trj = means_gvimp_niter(1:2, 1:end);
        for j = 1:n_states
            config_ij_gvimp = means_gvimp_niter(1:2, j);
            err_vec_gvimp = planar_sdf_arm.evaluateError(config_ij_gvimp);
            dist_vec_gvimp = epsilon - err_vec_gvimp;
            
            % ------------------- collision -----------------
            if min(dist_vec_gvimp) < 0.0
                gvimp_col = true;
                
                % record the worst case
                if min(dist_vec_gvimp) < min_dist_gvimp_ever
                   found_new_worstcase = true;
                   
                    min_dist_gvimp_ever = min(dist_vec_gvimp);
                    worst_case_map = dataset_noisy;
                    
                    
                    % ------------ re-sample -------------
                    dist_vec_gvimp_resampled = dist_vec_gvimp;
                    while min(dist_vec_gvimp_resampled) < 0.1
                        config_ij_gvimp_resample = config_ij_gvimp + sqrtm(covs_gvimp_niter(1:2, 1:2, j))*randn(2, 1);
                        err_vec_gvimp_resample = planar_sdf_arm.evaluateError(config_ij_gvimp_resample);
                        dist_vec_gvimp_resampled = epsilon - err_vec_gvimp_resample;
                    end

                    % ------------ update the sampled trajectory ------------
                    gvimp_sample_trj(1:2, j) = config_ij_gvimp_resample;
                end
            end
            
        end
        
        if found_new_worstcase
            found_new_worstcase = false;
            worst_case_gvimp_sample_trj = gvimp_sample_trj;
        end
        
%         % ------------ plot the original and the resampled traj ------------
%         if gvimp_col
%             
%             figure(3)
%             set(gcf,'position',[x0,y0,width,height])
%             tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'none')
% 
%             % ----------------- The original plan ----------------- 
%             nexttile(1)
%             hold on
% 
%             % plot world
%             plotEvidenceMap2D_arm(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%             plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%             plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
% 
%             for ii=1:n_states  
%                 % gradual changing colors
%                 alpha = (ii / n_states)^(1.15);
%                 color = [0, 0, 1, alpha];
% 
%                 % plot arm        
%                 conf = means_gvimp_niter(:, ii);
%                 plotPlanarArm1(arm.fk_model(), conf, color, 5, true);
% 
%                 % Pause to display the plot
%                 pause(0.05)
%             end
% 
%             plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%             plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
%             title('GVI-MP original plan')
% 
%             % ----------------- The replan ----------------- 
%             nexttile(2)
%             hold on
% 
%             % plot world
%             plotEvidenceMap2D_arm(dataset.map, dataset.origin_x, dataset.origin_y, cell_size);
%             plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%             plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
% 
%             for ii=1:n_states  
%                 % gradual changing colors
%                 alpha = (ii / n_states)^(1.15);
%                 color = [0, 0, 1, alpha];
% 
%                 % plot arm        
%                 conf = gvimp_sample_trj(:, ii);
%                 plotPlanarArm1(arm.fk_model(), conf, color, 5, true);
% 
%                 % Pause to display the plot
%                 pause(0.05)
%             end
% 
%             plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
%             plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
%             title('GVI-MP re-sampled plan')
%         end
        
    % ------------------ check collision PCS-MP ----------------
        for j = 1:nt_pcs
            config_ij_pcs = means_pcsmp(1:2, j);
            err_vec_pcs = planar_sdf_arm.evaluateError(config_ij_pcs);
            dist_vec_pcs = epsilon - err_vec_pcs;

            if min(dist_vec_pcs) < 0.0
                pcsmp_col = true;
            end
           
        end

        % ------------------ check collision GPMP2 ----------------
        for i=0:nt_gpmp2
            config_ij_gpmp2 = means_gpmp2(1:2, i+1);
            err_vec_gpmp2 = planar_sdf_arm.evaluateError(config_ij_gpmp2);
            dist_vec_gpmp2 = epsilon - err_vec_gpmp2;

            if min(dist_vec_gpmp2) < 0.0
                gpmp2_col = true;
            end
        end

        if gvimp_col
            cnt_gvimp_col = cnt_gvimp_col+1;
            gvimp_col = false;
        end
        
        if pcsmp_col
            cnt_pcsmp_col = cnt_pcsmp_col+1;
            pcsmp_col = false;
        end

        if gpmp2_col
            cnt_gpmp2_col = cnt_gpmp2_col+1;
            gpmp2_col = false;
            
            % ======================== re-plan gpmp2 ========================
            means_gpmp2_replan = optimize_gpmp2(dataset_noisy, nt_gpmp2);
            
            % -------------- check if re-plan succeed --------------
            for i=0:nt_gpmp2
                config_ij_gpmp2_replan = means_gpmp2_replan(1:2, i+1);
                err_vec_replan = planar_sdf_arm.evaluateError(config_ij_gpmp2_replan);
                dist_vec_gpmp2_replan = epsilon - err_vec_replan;

                if min(dist_vec_gpmp2_replan) < 0.0
                    gpmp2_replan_fail = true;
                end
            end

        end
        
        if gpmp2_replan_fail
            cnt_gpmp2_replan_fail = cnt_gpmp2_replan_fail + 1;
            gpmp2_replan_fail = false;
        end

    end

disp("======= sigma_map =======")
sigma_map

disp("cnt_gpmp2_col")
v_cnt_gpmp2_col = [v_cnt_gpmp2_col, cnt_gpmp2_col / 2000];

disp("cnt_gvimp_col")
v_cnt_gvimp_col = [v_cnt_gvimp_col, cnt_gvimp_col / 2000];

disp("cnt_pcsmp_col")
v_cnt_pcsmp_col = [v_cnt_pcsmp_col, cnt_pcsmp_col / 2000];

disp("cnt_gpmp2_replan_fail")
v_cnt_gpmp2_replan_fail = [v_cnt_gpmp2_replan_fail, cnt_gpmp2_replan_fail / cnt_gpmp2_col];

cnt_gvimp_col = 0;
cnt_gpmp2_col = 0;
cnt_pcsmp_col = 0;
cnt_gpmp2_replan_fail = 0;

end


%% ========================== plot the original and the resampled traj ==========================
import gtsam.*
import gpmp2.*

figure(3)
set(gcf,'position',[x0,y0,width,height])
tiledlayout(1, 2, 'TileSpacing', 'tight', 'Padding', 'none')

% ----------------- The original plan ----------------- 
nexttile(1)
hold on

% plot world
plotEvidenceMap2D_arm(worst_case_map.map, worst_case_map.origin_x, worst_case_map.origin_y, cell_size);
arm = generateArm('SimpleTwoLinksArm');
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);

for ii=1:n_states  
    % gradual changing colors
    alpha = (ii / n_states)^(1.15);
    color = [0, 0, 1, alpha];

    % plot arm        
    conf = means_gvimp_niter(:, ii);
    plotPlanarArm1(arm.fk_model(), conf, color, 5, true);

    % Pause to display the plot
    pause(0.05)
end

plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('GVI-MP original plan')

% ----------------- The replan ----------------- 
nexttile(2)
hold on

% plot world
plotEvidenceMap2D_arm(worst_case_map.map, worst_case_map.origin_x, worst_case_map.origin_y, cell_size);
plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);

for ii=1:n_states  
    % gradual changing colors
    alpha = (ii / n_states)^(1.15);
    color = [0, 0, 1, alpha];

    % plot arm        
    conf = worst_case_gvimp_sample_trj(:, ii);
    plotPlanarArm1(arm.fk_model(), conf, color, 5, true);

    % Pause to display the plot
    pause(0.05)
end

plotPlanarArm(arm.fk_model(), start_conf, 'r', 5);
plotPlanarArm(arm.fk_model(), end_conf, 'g', 5);
title('GVI-MP re-sampled plan')


%% ============================= Helper functions =============================
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

