clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
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

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 1:1 % 4 experiments
    sdfmap = csvread(["long_range/case"+num2str(i)+"/"+"map_long_range.csv"]);
    nexttile
    hold on
    prefix = ["long_range/case"+num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    disp("final time covariance")
    covs(1:2,1:2,end)
   
    addpath("../error_ellipse");
    addpath("../../../matlab_helpers/");
    
    plot_2d_result(sdfmap, means, covs);

%     scatter(start_pos(1), start_pos(2), 200, "red", 'filled');
%     scatter(goal_pos(1), goal_pos(2), 200, "green", 'filled');

    xlim([-10, 65])
    ylim([-10, 70])
    axis off ; 
end
