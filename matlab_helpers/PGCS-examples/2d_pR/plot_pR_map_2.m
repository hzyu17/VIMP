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

% read map
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

plotColors.lightBlue = [0.301 0.745 0.933];
plotColors.blue = [0.15 0.25 0.8];
plotColors.green = [0.85 0.325 0.098];
plotColors.red = [0.9, 0 ,0];

args = {'LineStyle', '-', ...
        'LineWidth',0.7, ...
        'Color', plotColors.lightBlue};

% NOTE: change the number of experiments
for i = 1:92
    % nexttile
    hold on
    % prefix = ["map2/case"+num2str(i)+"/"]
    % prefix = ["/home/zchen927/Documents/VIMP/vimp/save/case"+num2str(i)]
    prefix = ["/home/zchen927/Documents/VIMP/vimp/save/BRM_test/exp"+num2str(i)];
    prefix = ["/home/czy/Documents/VIMP_CZY/VIMP/vimp/save/BRM_30nodes_v1_20/exp"+num2str(i)];
    % prefix = ["C:\Users\CZY-Yoga\Documents\Code\VIMP\vimp\save\BRM_test\exp"+num2str(i)]
    % % --- read means and covariances ---
    disp([prefix + "zk_sdf.csv"])
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    cov_final = covs(:,end);
    cov_final_RESHAPED = reshape(cov_final, [4,4]);
    
    plot_2d_result(sdfmap, means, covs, 3, args);

    axis off ; 

end

%%
xlim([-20 20]); ylim([-10, 20]);
saveas(gcf, '~/Pictures/MP_Paper/CSBRM_comp/BRM_path_30nodes_v1_20.png')
saveas(gcf, '~/Pictures/MP_Paper/CSBRM_comp/BRM_path_30nodes_v1_20.pdf')
