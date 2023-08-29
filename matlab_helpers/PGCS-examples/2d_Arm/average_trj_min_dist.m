clear all
close all
clc

%% ******************* Read datas ******************
addpath('../../tools/gtsam_toolbox')
import gtsam.*
import gpmp2.*

addpath("../../tools/error_ellipse");
addpath('../../tools/2dArm');

map = 1;
exp = 1;

prefix = "map1";
prefix_gpmp2 = "map1";
prefix_gvimp = "map1";
switch map
    case 1
        prefix = "map1";
        sdfmap = csvread("map1/map.csv");
        switch exp
            case 1
                prefix = "map1/case1";
                prefix_gpmp2 = "map1/case1/gpmp2";
                prefix_gvimp = "../../GVIMP-examples/2d_Arm/map1/case1";
                % boundary conditions
                start_conf = [0, 0]';
                start_vel = [0, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
            case 2
                prefix = "map1/case2";
                prefix_gpmp2 = "map1/case2/gpmp2";
                % boundary conditions
                start_conf = [-pi/2, 0]';
                start_vel = [0.1, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
        end

    case 2
        sdfmap = csvread("map2/map.csv");
        prefix_gpmp2 = "map2";
        switch exp
            case 1
                prefix = "map2/case1";
                % boundary conditions
                start_conf = [0, 0]';
                start_vel = [0, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
            case 2
                prefix = "map2/case2";
                % boundary conditions
                start_conf = [-pi/2, 0]';
                start_vel = [0.1, 0]';
                end_conf = [pi/2, 0]';
                end_vel = [0, 0]';
        end
end

dim_theta = 4;

% =================== read gvimp results ====================
means_gvimp = csvread([prefix_gvimp+"/zk_sdf.csv"]);
covs_gvimp = csvread([prefix_gvimp+"/cov.csv"]);
nt_gvimp = size(means_gvimp, 2);

% =================== read pgcs results ====================
means_pgcs = csvread([prefix+"/zk_sdf.csv"]);
covs_pgcs = csvread([prefix+"/Sk_sdf.csv"]);

% =================== read gpmp2 results ====================
means_gpmp2 = csvread([prefix_gpmp2+"/zt_gpmp2.csv"]);
nt_gpmp2 = size(means_gpmp2, 2);

% ----- parameters -----
[ndim, nt_pgcs] = size(means_pgcs);
covs_pgcs = reshape(covs_pgcs, dim_theta, dim_theta, nt_pgcs);

%  ------- arm --------
arm = generateArm('SimpleTwoLinksArm');

%  ------- sdf --------
dataset = generate2Ddataset('OneObstacleDataset');
rows = dataset.rows;
cols = dataset.cols;
cell_size = dataset.cell_size;
origin_point2 = Point2(dataset.origin_x, dataset.origin_y);

% ------- signed distance field -------
field = signedDistanceField2D(dataset.map, cell_size);
sdf = PlanarSDF(origin_point2, cell_size, field);

% ------- obstacle factor -------
cost_sigma = 0.0;
epsilon = 10;

planar_sdf_arm = ObstaclePlanarSDFFactorArm(symbol('x', 0), arm, sdf, cost_sigma, epsilon);
        
% ------- compute average min trj distance -------
% ------- gpmp2 -------
avg_dist_gpmp2 = 0;
min_dist_gpmp2 = 100;
for i = 1:nt_gpmp2
    conf_i = means_gpmp2(1:2, i);
    err_vec = planar_sdf_arm.evaluateError(conf_i);
    dist_vec = epsilon - err_vec;
    if min(dist_vec) < min_dist_gpmp2
        min_dist_gpmp2 = min(dist_vec);
    end
    avg_dist_gpmp2 = avg_dist_gpmp2 + min(dist_vec);
end


avg_dist_gvimp = 0;
min_dist_gvimp = 10.0;
for i = 1:nt_gvimp
    conf_i = means_gvimp(1:2, i);
    err_vec = planar_sdf_arm.evaluateError(conf_i);
    dist_vec = epsilon - err_vec;
    if min(dist_vec) < min_dist_gvimp
        min_dist_gvimp = min(dist_vec);
    end
    avg_dist_gvimp = avg_dist_gvimp + min(dist_vec);
end


avg_dist_pgcs = 0;
min_dist_pgcs = 10.0;
for i = 1:nt_pgcs
    conf_i = means_pgcs(1:2, i);
    err_vec = planar_sdf_arm.evaluateError(conf_i);
    dist_vec = epsilon - err_vec;
    if min(dist_vec) < min_dist_pgcs
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
