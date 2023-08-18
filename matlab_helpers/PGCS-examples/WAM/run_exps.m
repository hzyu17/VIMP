clear all
close all
clc

%% ******************* Read datas ******************
addpath('/usr/local/gtsam_toolbox')
addpath ('/home/hongzhe/git/VIMP/matlab_helpers/experiments/WAM/utils')
import gtsam.*
import gpmp2.*

addpath("..//PGCS-examples");
addpath("../../tools/error_ellipse");

replan = true;
exp=1;
switch exp
    case 1
        prefix = "case1";
        start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
        end_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';
    case 2
        prefix = "case2";
        start_conf = [-0.9,-1.70,1.34,1.19,1.1,-0.126,1.2]';
        end_conf = [ -0.7,1.1,0.1,1.0,0,-0.619,1.75]';
    case 3
        prefix = "case3";
        start_conf = [-1.8,-1.50,1.84,1.29,1.5,0.26,0.2]';
        end_conf = [ 0.0, 0.6, -0.5, 0.2, 0.2, 0.8, 1.15]';
end

% ===================== experiments ===================
% -------------------------- run experiment -----------------------------
i_exp = 2;
eps = 0.01;
eps_map = 0.6;
radius = 0.1;
speed = 0.23;
nt = 50;
sig0 = 0.001;
sigT = 0.001;
eta = 1e-6;
stop_err = 1e-5;
max_iter = 50;
cost_sigma = 1.5e5;
backtrack_ratio = 0.5;
max_n_backtracking = 8;

args = [num2str(i_exp), ' ', num2str(eps), ' ', num2str(eps_map), ' ',... 
    num2str(radius), ' ', num2str(speed), ' ', num2str(nt), ' ', num2str(sig0), ...
    ' ', num2str(sigT), ' ', num2str(eta), ' ', num2str(stop_err), ' ', num2str(max_iter), ' ', ...
    num2str(cost_sigma), num2str(backtrack_ratio), ' ', num2str(max_n_backtracking)];
    
command = ['/home/hongzhe/git/VIMP/vimp/build/pgcs_WAMArm', ' ', args];
num_iter = system(command);

%%
% -------------------------- analyze results --------------------------
start_vel = zeros(7,1);
end_vel = zeros(7,1);

means = csvread([prefix+"/zk_sdf.csv"]);
covs = csvread([prefix+"/Sk_sdf.csv"]);
[nx, nt] = size(means);
covs = reshape(covs, [nx, nx, nt]);