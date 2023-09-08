clear all
close all
clc
addpath('../../tools/gtsam_toolbox')
addpath('../../tools/2dpR')
% addpath('/home/zchen927/Downloads/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../../tools/error_ellipse");
addpath("../../../matlab_helpers/");

%% read map
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");

% % hyper parameters
% i_exp = 1;
% eps = 0.01;
% eps_map = 0.6;
% speed = 0.23;
% nt = 50;
% sig0 = 0.001;
% sigT = 0.001;
% eta = 1e-1;
% stop_err = 1e-5;
% max_iter = 50;
% cost_sigma = 1.5e5;
% 
% args = [num2str(i_exp), ' ', num2str(eps), ' ', num2str(eps_map), ' ', num2str(speed), ' ', num2str(nt), ' ', num2str(sig0), ' ', num2str(sigT), ' ', ...
%             num2str(eta), ' ', num2str(stop_err), ' ', num2str(max_iter), ' ', num2str(cost_sigma)];
%     
% command = ['/home/hongzhe/git/VIMP/vimp/build/pgcs_PlanarPRModel', ' ', args];
% num_iter = system(command);

% plotting
x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;

figure
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact')

% modify number of experiments accordingly
for i = 1:20
    % nexttile
    hold on
    % prefix = ["map2/case"+num2str(i)+"/"]
    % prefix = ["/home/zchen927/Documents/VIMP/vimp/save/case"+num2str(i)]
    % prefix = ["/home/zchen927/Documents/VIMP/vimp/save/BRM_test/exp"+num2str(i)]
    prefix = ["/home/czy/Documents/VIMP_CZY/VIMP/vimp/save/BRM_test/exp_demo"+num2str(i)]
    % % --- read means and covariances ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    cov_final = covs(:,end);
    disp("cov_final_RESHAPED")
    cov_final_RESHAPED = reshape(cov_final, [4,4])
    
    plot_2d_result(sdfmap, means, covs);

%     % --- read baselines ---
%     means_prm = csvread([prefix + "prm_5000.csv"]);
%     means_rrt = csvread([prefix + "RRTstar_3000.csv"]);
% 
%     % --- plot baselines ---
%     [~, nt_prm] = size(means_prm);
%     [~, nt_rrt] = size(means_rrt);
%     
%     for i_pt = 1:nt_prm-1
%         plot([means_prm(1, i_pt), means_prm(1, i_pt+1)], ...
%             [means_prm(2, i_pt), means_prm(2, i_pt+1)], 'LineWidth', 3.0, 'Color', 'b');
%     end
%     
%     hold on
% 
%     for i_pt = 1:nt_rrt-1
%         plot([means_rrt(1, i_pt), means_rrt(1, i_pt+1)], ...
%             [means_rrt(2, i_pt), means_rrt(2, i_pt+1)], 'LineWidth', 3.0, 'Color', 'g');
%     end
    
%     if i==3
%         xlim([-20, 25]);
%         ylim([-15, 22]);
%     end
    axis off ; 

end
