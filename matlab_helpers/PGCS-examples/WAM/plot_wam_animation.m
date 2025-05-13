clear all
close all
clc

% ******************* dependencies and includes ******************
addpath('../../tools/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../../tools");
addpath("../../tools/error_ellipse");
addpath("../../tools/WAM/utils")

% ******************* Define map dataset ******************
dataset = generate3Ddataset_1('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% ******************* WAM Arm and start and end conf ******************
arm = generateArm('WAMArm');

% ================= figure configs and start and goal states ===================
x0 = 50;
y0 = 50;
width = 400;
height = 350;

start_conf = [-0.8,  -1.70, 1.64,  1.29,   1.1,    -0.106,   2.2;
            -0.9,    -1.70,   1.34,  1.19,   0.8,    -0.126,   2.5;
            -1.8,    -1.50,   1.84,  1.29,   1.5,    0.26,     0.2];
end_conf = [-0.0,    0.94,     0,    1.6,    0,     -0.919,     1.55;
          -0.7,     1.35,     1.2,   1.0,   -0.7,    -0.1,      1.2;
          -0.0,     0.6,     -0.5,   0.2,    0.2,    0.8,       1.15];
      
nx = 14;

% ================= data reading and plots ===================
% experiments
for i_exp = 1:1 

    % ====================================================================================== 
    %                                   read data
    % ====================================================================================== 

    prefix = ["case"+num2str(i_exp)+"/"];
    prefix_gvi = ["../../GVIMP-examples/WAM/sparse_gh/case"+num2str(i_exp)+"/"];
    prefix_gpmp2 = ["case"+num2str(i_exp)+"/gpmp2"];
    
    % ------------ read gpmp2 results ------------ 
    gpmp2_confs = csvread([prefix_gpmp2+"/zk_sdf.csv"]);
    [nx_gpmp2, nt_gpmp2] = size(gpmp2_confs);
    
    % ------------  read gvi-mp results ------------ 
    means_gvi = csvread([prefix_gvi + "zk_sdf.csv"]);
    [~, nt_gvi] = size(means_gvi);
    
    covs_gvi = csvread([prefix_gvi + "Sk_sdf.csv"]);
    covs_gvi = reshape(covs_gvi, [14, 14, nt_gvi]);
    
    % ------------  read pgcs-mp results ------------ 
    means = csvread([prefix + "zk_sdf.csv"]);
    covs = csvread([prefix + "Sk_sdf.csv"]);

    [nx, nt] = size(means);
    covs = reshape(covs, [nx,nx,nt]);
    
    
    % ====================================================================================== 
    %                                   plot mean trajectories
    % ====================================================================================== 
    
%     % --------------------------- plot gpmp2 results ---------------------------
%     figure
%     set(gcf,'position',[x0,y0,width,height])
%     tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
%     
%     nexttile
%     grid on
%     hold on 
%     view(-14.7458, 9.8376);
%     plotMap3D(dataset.corner_idx, origin, cell_size);
%     
%     plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 8, true);
%     for jj=1:5
%         plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 8, true);
%     end
%     axis off;
%     xlim([-1, 1.4])
%     ylim([-1, 1.0])
%     zlim([-0.8, 0.9])
%     
%     for j = 1:nt_gpmp2
%         % gradual changing colors
%         alpha = (j / nt_gpmp2)^(1.15);
%         color = [0, 1, 1, alpha];
%         % means
%         plotArm3D(arm.fk_model(), gpmp2_confs(:, j), color, 6, true);
%         pause(0.5)
%     end
    
%     % --------------------------- plot gvi-mp results ---------------------------
%     figure
%     set(gcf,'position',[x0,y0,width,height])
%     tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
%     
%     nexttile
%     grid on
%     hold on 
%     view(-14.7458, 9.8376);
%     plotMap3D(dataset.corner_idx, origin, cell_size);
%     
%     % start and goal
%     plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 8, true);
%     for jj=1:5
%         plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 8, true);
%     end
%     axis off;
%     xlim([-1, 1.4])
%     ylim([-1, 1.0])
%     zlim([-0.8, 0.9])
%     
%     % trajectories
%     for j = 1:nt_gvi
%         % gradual changing colors
%         alpha = (j / nt_gvi)^(1.15);
%         color = [0.2706, 0.2706, 1, alpha];
%         % means
%         plotArm3D(arm.fk_model(), means_gvi(1:7, j), color, 6, true);
%         pause(0.5)
%     end
    
    % --------------------------- plot pgcs-mp results ---------------------------
    figure
    set(gcf,'position',[x0,y0,width,height])
    tiledlayout(1, 1, 'TileSpacing', 'none', 'Padding', 'none')
    
    nexttile
    grid on
    hold on 
    view(-14.7458, 9.8376);
    plotMap3D(dataset.corner_idx, origin, cell_size);
    
    plotArm3D(arm.fk_model(), start_conf(i_exp,1:end)', 'r', 6, true);
    for jj=1:5
        plotArm3D(arm.fk_model(), end_conf(i_exp,1:end)', 'g', 6, true);
    end
    axis off;
    xlim([-1, 1.4])
    ylim([-1, 1.0])
    zlim([-0.8, 0.9])
    
    for j = 1:nt
        % gradual changing colors
        alpha = (j / nt)^(1.15);
        color = [0, 0, 0.5020, alpha];
        % means
        plotArm3D(arm.fk_model(), means(:, j), color, 4, true);
        pause(0.5)
    end

end
