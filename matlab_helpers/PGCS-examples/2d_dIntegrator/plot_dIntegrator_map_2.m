clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("../../RAL-examples/2d_pR/map2/map_multiobs_map2.csv");
addpath("../")

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')
for i = 1:4 % 4 experiments
    nexttile
    hold on
    prefix = ["map2/case"+num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    disp("final time cov")
    covs(1:2, 1:2, end)

    addpath("../error_ellipse");
    addpath("../../../matlab_helpers/");
    
    plot_2d_result(sdfmap, means, covs);

    axis off ; 
end
