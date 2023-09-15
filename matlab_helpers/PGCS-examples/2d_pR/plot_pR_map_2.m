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

% RUNNING GUIDE:
% change map loaded position
% change data position
% set num_exp from number of experiments
% set ylim and xlim according to transformed coordinates

% read map
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");
% sdfmap = csvread("../../../vimp/maps/2dpR/map4/map_multiobs_map4.csv");
% sdfmap = csvread("../../../vimp/maps/2dpR/map5/map_multiobs_map5.csv");
sdfmap = csvread("../../../vimp/maps/2dpR/map6/map_multiobs_map6.csv");

% NOTE: change the number of experiments
% 148 for 40 nodes
num_exp = 1; % 250 for 80 nodes, 92 for 30 nodes, 62, 24
mean_all = zeros(4,50,num_exp); %50 originallay
cov_all  = zeros(16,50,num_exp);
for i = 1:num_exp
    % nexttile
    hold on
    % prefix = ["map2/case"+num2str(i)+"/"]
    % prefix = ["/home/zchen927/Documents/VIMP/vimp/save/case"+num2str(i)]
    prefix = ["../../../vimp/save/BRM_test/exp"+num2str(i)];
    prefix = ["../../../vimp/save/BRM_80nodes_v1/exp"+num2str(i)];
    prefix = ["../../../vimp/save/BRM_map5_40nodes_v2/exp"+num2str(i)];
    prefix = ["../2d_dIntegrator/map2/casetest/"];
    prefix = ["/home/czy/Documents/VIMP_CZY/VIMP/vimp/save/BRM_map6_100nodes_v1_0915/exp"+num2str(i)];
    prefix = ["/home/czy/Documents/BRM_map6_1000nodes_v1/exp"+num2str(i)];


    prefix = ["/home/zchen927/Documents/VIMP/vimp/save/BRM_map6_300nodes_v2/exp"+num2str(i)];
    % prefix = ["/home/czy/Documents/VIMP_CZY/VIMP/vimp/save/BRM_30nodes_v1_50/exp"+num2str(i)];
    % prefix = ["C:\Users\CZY-Yoga\Documents\Code\VIMP\vimp\save\BRM_test\exp"+num2str(i)]
    % % --- read means and covariances ---
    disp([prefix + "zk_sdf.csv"])
    mean_all(:,:,i) = csvread([prefix + "zk_sdf.csv"]);
    cov_all(:,:,i) = csvread([prefix + "Sk_sdf.csv"]);
end

%%  Plotting
% figure
tiledlayout(2, 2, 'TileSpacing', 'compact', 'Padding', 'compact')

plotColors.lightBlue = [0.301 0.745 0.933 0.0]; % transparent 0.6
plotColors.blue = [0.15 0.25 0.8];
plotColors.green = [0.85 0.325 0.098];
plotColors.red = [0.9, 0 ,0];
plotColors.purple = [0.4940 0.1840 0.5560];
plotColors.brown  = [0.8500 0.3250 0.0980];

args = {'LineStyle', '-', ...
        'LineWidth',0.7, ...
        'Color', plotColors.lightBlue};
hold on

plotpath = true;
if plotpath == true
% plot path
for i = 1:num_exp
    plot_2d_result(sdfmap, mean_all(:,:,i), cov_all(:,:,i), 15, args);
    % axis off ;
end
end

plotColors.lightBlue = [0.301 0.745 0.933 0.8];  % transparent 1
args = {'LineStyle', '-', ...
        'LineWidth',1.2, ...
        'Color', plotColors.lightBlue};
% hightlight the sampled points
for i=1:num_exp
    % plot_2d_result(sdfmap, mean_all(:,1,i), cov_all(:,1,i), 3, args);
    % plot(mean_all(1,1,i), mean_all(2,1,i), '.', 'MarkerSize', 6, 'Color', 'k'); %25 orignally
    axis off;
end

xlim([-20 20]); ylim([-10, 20]); % demo map2
% xlim([-20, 70]); ylim([-10, 62]); % map 4
xlim([-20,26.4]); ylim([-10,30]);
xlim([-20,31]); ylim([-10,35]);

%% save files
% saveas(gcf, '~/Pictures/MP_Paper/CSBRM_comp/BRM_path_10nodes_v1_50iters.png')
% saveas(gcf, '~/Pictures/MP_Paper/CSBRM_comp/BRM_path_10nodes_v1_50iters.pdf')
