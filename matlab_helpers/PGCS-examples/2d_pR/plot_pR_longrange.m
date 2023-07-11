clear all
close all
clc
addpath('/home/hyu419/.local/gtsam_toolbox')
addpath("../error_ellipse");
addpath("../../../matlab_helpers/");
import gtsam.*
import gpmp2.*

%% read map

addpath("../")

x0 = 800;
y0 = 800;
width = 550;
height = 350;
figure
set(gcf,'position',[x0,y0,width,height])
start_pos = [-5; -5];
goal_pos = [55.0; 60.0];

tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
sdfmap = csvread(["long_range/map_long_range.csv"]);
for i = 1:1 % 4 experiments
    nexttile
    hold on
    prefix = ["long_range/case"+num2str(i)+"/"];
    % % --- read means and covs ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    % --- read baseline means ---
%     means_prm = csvread([prefix + "prm_5000_35.csv"]);
%     means_rrt = csvread([prefix + "RRTstar_5000.csv"]);
    
    plot_2d_result(sdfmap, means, covs);

    % --- plot baselines ---
%     [~, nt_prm] = size(means_prm);
%     [~, nt_rrt] = size(means_rrt);
    
%     for i_pt = 1:nt_prm-1
%         scatter(means_prm(1, i_pt), means_prm(2, i_pt), 'blue', 'filled');
%         plot([means_prm(1, i_pt), means_prm(1, i_pt+1)], ...
%             [means_prm(2, i_pt), means_prm(2, i_pt+1)], 'LineWidth', 3.0, 'Color', 'b');
%     end
    
    hold on

%     for i_pt = 1:nt_rrt-1
%         scatter(means_rrt(1, i_pt), means_rrt(2, i_pt), 'green', 'filled');
%         plot([means_rrt(1, i_pt), means_rrt(1, i_pt+1)], ...
%             [means_rrt(2, i_pt), means_rrt(2, i_pt+1)], 'LineWidth', 3.0, 'Color', 'g');
%     end

    xlim([-10, 65])
    ylim([-10, 70])
    axis off ; 
end
