% plot the comparison of GVI versus PGCS in the same figure
clear all
close all
clc
addpath('/usr/local/gtsam_toolbox')
import gtsam.*
import gpmp2.*

%% read map
sdfmap = csvread("../../RAL-examples/2d_pR/map2/map_multiobs_map2.csv");
addpath("../")
addpath("../error_ellipse");
addpath("../../../matlab_helpers/");

x0 = 500;
y0 = 500;
width = 1290.427199;
height = 800;
figure
set(gcf,'position',[x0,y0,width,height])

tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')

for i = 4:4 % 4 experiments
    nexttile
    hold on
    prefix_pgcs = ["map2/case"+num2str(i)+"/"];
    % % --- read means and covariances ---
    means = csvread([prefix_pgcs + "zk_sdf.csv"]);
    covs = csvread([prefix_pgcs + "Sk_sdf.csv"]);
    
    plot_2d_result(sdfmap, means, covs);


end
