clear all
close all
clc

%% ******************* Setup paths ******************
vimp_root = setup_vimp();
matlab_helpers = fullfile(vimp_root, 'matlab_helpers');
addpath(matlab_helpers);
addpath(fullfile(matlab_helpers, 'tools'));
addpath(fullfile(matlab_helpers, 'tools', 'gtsam_toolbox'));
addpath(fullfile(matlab_helpers, 'tools', 'error_ellipse'));

import gtsam.*
import gpmp2.*

%% generate SDF
% dataset
dataset = generate3Ddataset_1('3dPRMap2');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% init sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
    size(field, 2), size(field, 3));
for z = 1:size(field, 3)
    sdf.initFieldData(z-1, field(:,:,z)');
end

%% plot 3D SDF
x0 = 550;
y0 = 550;
width = 600;
height = 550;

for i = 1:4 % 4 experiments
    i
    figure(i)
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'tight', 'Padding', 'tight')
    t.FontSize = 16;
    nexttile
    grid on
    hold on 
    
    prefix = fullfile(matlab_helpers, 'PGCS-examples', '3d_pR', 'map2', ['case' num2str(i)]);
    
    % % --- high temperature ---
    means = csvread(fullfile(prefix, 'zk_sdf.csv'));
    covs = csvread(fullfile(prefix, 'Sk_sdf.csv'));
    
    terminal_cov = reshape(covs, [6,6,50]);
    terminal_cov = terminal_cov(1:end, 1:end, end);
    diff_terminal_cov = norm(terminal_cov - 0.04.*eye(6), 'fro')
       
    plotMap3D(dataset.corner_idx, origin, cell_size);
    plot_3d_result(means, covs);
    
    % plot the goal position and cov
    xlim([-15, 40])
    ylim([-20, 40])
    zlim([-10, 40])
    
    % camera angle
    v = [-25 -15 10];
    [caz,cel] = view(v);
    set(gca,'fontsize',16);
    xlabel('Position $x$','Interpreter','latex'),ylabel('Position $y$','Interpreter','latex');
    zlabel('Position $z$','Interpreter','latex');
    axis off;
end