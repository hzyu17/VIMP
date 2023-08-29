clear all
close all
clc
addpath('../../tools/gtsam_toolbox')

addpath("../../tools/error_ellipse");
addpath("../../");

import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("../../../vimp/maps/2dpR/map2/map_multiobs_map2.csv");
addpath("../")

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(2, 2, 'TileSpacing', 'tight', 'Padding', 'tight')

for i = 1:4 % 4 experiments
%     x0 = 500;
%     y0 = 500;
%     width = 1290.427199;
%     height = 800;
%     figure
%     set(gcf,'position',[x0,y0,width,height])
%     tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')

    nexttile
    hold on
    prefix = ["map2/case"+num2str(i)+"/"];
    % % --- high temperature ---
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);
    
    plot_2d_result(sdfmap, means, covs);
%     if i==3
        xlim([-20, 25]);
        ylim([-15, 22]);
%     end
    axis off ; 

end
