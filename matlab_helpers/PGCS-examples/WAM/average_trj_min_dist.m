clear all
close all
clc

%% ******************* Read datas ******************
% ******************* dependencies and includes ******************
addpath('/usr/local/gtsam_toolbox')
addpath ('/home/hongzhe/git/VIMP/matlab_helpers/experiments/WAM/utils')

import gtsam.*
import gpmp2.*

addpath("../../tools");
addpath("../../tools/error_ellipse");
addpath("../../tools/WAM/utils")

%% dataset
dataset = generate3Ddataset('WAMDeskDataset');
origin = [dataset.origin_x, dataset.origin_y, dataset.origin_z];
origin_point3 = Point3(origin');
cell_size = dataset.cell_size;

% sdf
disp('calculating signed distance field ...');
field = signedDistanceField3D(dataset.map, dataset.cell_size);
disp('calculating signed distance field done');

% ******************* WAM Arm and start and end conf ******************
arm = generateArm('WAMArm');
for i_exp = 1:3 % 3 experiments
    disp(["=========== experiment " + i_exp + " =============="]);
    % ====================================================================================== 
    %                                   read data
    % ====================================================================================== 

    prefix = ["case"+num2str(i_exp)+"/"];
    prefix_gvi = ["../../GVIMP-examples/WAM/case"+num2str(i_exp)+"/"];
    prefix_gpmp2 = ["case"+num2str(i_exp)+"/gpmp2"];
    
    % ------------ read gpmp2 results ------------ 
    means_gpmp2 = csvread([prefix_gpmp2+"/zk_sdf.csv"]);
    [nx_gpmp2, nt_gpmp2] = size(means_gpmp2);
    
    % ------------  read gvi-mp results ------------ 
    means_gvimp = csvread([prefix_gvi + "zk_sdf.csv"]);
    [~, nt_gvimp] = size(means_gvimp);
    covs_gvimp = csvread([prefix_gvi + "Sk_sdf.csv"]);
    
    covs_gvimp = reshape(covs_gvimp, [14, 14, nt_gvimp]);
    
    [nx, nt_gvimp] = size(means_gvimp);

    % ------------  read pgcs-mp results ------------ 
    means_pgcs = csvread([prefix + "zk_sdf.csv"]);
    covs_pgcs = csvread([prefix + "Sk_sdf.csv"]);

    [nx, nt_pgcs] = size(means_pgcs);
    covs_pgcs = reshape(covs_pgcs, [nx,nx,nt_pgcs]);
    
    
    % ------- signed distance field -------
    % init sdf
    sdf = SignedDistanceField(origin_point3, cell_size, size(field, 1), ...
        size(field, 2), size(field, 3));
    for z = 1:size(field, 3)
        sdf.initFieldData(z-1, field(:,:,z)');
    end

    % ------- obstacle factor -------
    cost_sigma = 0.0;
    epsilon = 10;
    radius = 0.06;

    planar_sdf_arm = ObstacleSDFFactorArm(symbol('x', 0), arm, sdf, cost_sigma, epsilon);

    % ------- compute average min trj distance -------
    % ------- gpmp2 -------
    avg_dist_gpmp2 = 0;
    min_dist_gpmp2 = 100;
    for i = 1:nt_gpmp2
        conf_i = means_gpmp2(1:7, i);
        err_vec = planar_sdf_arm.evaluateError(conf_i);
        dist_vec = epsilon - err_vec + radius;
        if min(dist_vec) < min_dist_gpmp2
            min_dist_gpmp2 = min(dist_vec);
        end
        avg_dist_gpmp2 = avg_dist_gpmp2 + min(dist_vec);
    end


    avg_dist_gvimp = 0;
    min_dist_gvimp = 10.0;
    for i = 1:nt_gvimp
        conf_i = means_gvimp(1:7, i);
        err_vec = planar_sdf_arm.evaluateError(conf_i);
        dist_vec = epsilon - err_vec + radius;
        if min(dist_vec) < min_dist_gvimp;
            min_dist_gvimp = min(dist_vec);
        end
        avg_dist_gvimp = avg_dist_gvimp + min(dist_vec);
    end


    avg_dist_pgcs = 0;
    min_dist_pgcs = 10.0;
    for i = 1:nt_pgcs
        conf_i = means_pgcs(1:7, i);
        err_vec = planar_sdf_arm.evaluateError(conf_i);
        dist_vec = epsilon - err_vec + radius;
        if min(dist_vec) < min_dist_pgcs;
            min_dist_pgcs = min(dist_vec);
        end
        avg_dist_pgcs = avg_dist_pgcs + min(dist_vec);
    end

    disp('------------- avg min distance ---------------')
    avg_dist_gpmp2 = avg_dist_gpmp2 / nt_gpmp2
    avg_dist_gvimp = avg_dist_gvimp / nt_gvimp
    avg_dist_pgcs = avg_dist_pgcs / nt_pgcs

    disp('------------- min distance ---------------')
    min_dist_gpmp2
    min_dist_gvimp
    min_dist_pgcs

end


