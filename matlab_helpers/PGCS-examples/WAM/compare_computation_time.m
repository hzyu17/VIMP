clear all
close all
clc

%% Compute the computation time and compare between gpmp2, gvimp, and pgcsmp.
% =========================================================================
%                                  PGCS-MP
% =========================================================================
% ------------- parameters -------------
eps = 0.001;
eps_map = 0.18;
radius = 0.0;
total_time = 9.0;
nt = 50;
sig0 = 0.001;
sigT = 0.001;
step_size = 1e-4;
stop_err = 1e-5;
max_iter = 50;
map_name = 'map_bookshelf';
sig_obs = 8e2;
backtrack_ratio = 0.5;
max_n_backtracking = 8;
sdf_file = '/home/hzyu/git/VIMP/vimp/maps/WAM/WAMDeskDataset.bin';

executable = '/home/hzyu/git/VIMP/vimp/build/src/pgcs/pgcs_WAMArm';

%% ========================= 1st experiment =========================
i_exp = 1;
args = [num2str(i_exp), ' ', num2str(eps), ' ', num2str(eps_map), ' ',... 
    num2str(radius), ' ', num2str(total_time), ' ', num2str(nt), ' ', num2str(sig0), ...
    ' ', num2str(sigT), ' ', num2str(step_size), ' ', num2str(stop_err), ' ', num2str(max_iter), ' ', ...
    num2str(sig_obs), ' ', num2str(backtrack_ratio), ' ', num2str(max_n_backtracking), ' ', map_name, ' ', sdf_file];

% ================== optimize and count time ======================
profile on
for i = 1:50
    execute_one_exp(executable, args);
end
profile viewer
% =================================================================

%% ========================= 2nd experiment =========================
i_exp = 2;
args = [num2str(i_exp), ' ', num2str(eps), ' ', num2str(eps_map), ' ',... 
    num2str(radius), ' ', num2str(total_time), ' ', num2str(nt), ' ', num2str(sig0), ...
    ' ', num2str(sigT), ' ', num2str(step_size), ' ', num2str(stop_err), ' ', num2str(max_iter), ' ', ...
    num2str(sig_obs), ' ', num2str(backtrack_ratio), ' ', num2str(max_n_backtracking), ' ', map_name, ' ', sdf_file];

% ================== optimize and count time ======================
profile on
for i = 1:50
    execute_one_exp(executable, args);
end
profile viewer
% =================================================================

%% ========================================================================
%                                  GVI-MP
% =========================================================================
clear all
close all
clc
% ------------- parameters -------------
total_time = 4.0;
n_states = 20;
coeff_Qc = 1.0;
sig_obs = 0.02;
eps_sdf = 0.2;
radius = 0.0;
step_size = 0.7;
init_precision_factor = 1e4;
boundary_penalties = 1e4;
temperature = 0.01;
high_temperature = 0.5;
low_temp_iterations = 20;
stop_err = 1e-5;
num_iter = 30;
max_n_backtracking = 20;
sdf_file = '/home/hzyu/git/VIMP/vimp/maps/WAM/WAMDeskDataset.bin';

executable = '/home/hzyu/git/VIMP/build/src/gvimp/gvi_WAMArm';

%% ================= 1st experiment =================
i_exp = 1;
args = [num2str(i_exp), ' ', num2str(total_time), ' ', num2str(n_states), ' ',... 
        num2str(coeff_Qc), ' ', num2str(sig_obs), ' ', num2str(eps_sdf), ' ', num2str(radius), ...
        ' ', num2str(step_size), ' ', num2str(init_precision_factor), ' ', num2str(boundary_penalties), ...
        ' ', num2str(temperature), ' ', num2str(high_temperature), ...
        ' ', num2str(low_temp_iterations), ' ', num2str(stop_err), ' ', num2str(num_iter), ...
        ' ', num2str(max_n_backtracking), ' ', num2str(sdf_file)];

% ================== optimize and count time ======================
profile on
for i = 1:50
    execute_one_exp(executable, args);
end
profile viewer
% =================================================================

%% ================= 2nd experiment =================
i_exp = 2;
args = [num2str(i_exp), ' ', num2str(total_time), ' ', num2str(n_states), ' ',... 
        num2str(coeff_Qc), ' ', num2str(sig_obs), ' ', num2str(eps_sdf), ' ', num2str(radius), ...
        ' ', num2str(step_size), ' ', num2str(init_precision_factor), ' ', num2str(boundary_penalties), ...
        ' ', num2str(temperature), ' ', num2str(high_temperature), ...
        ' ', num2str(low_temp_iterations), ' ', num2str(stop_err), ' ', num2str(num_iter), ...
        ' ', num2str(max_n_backtracking), ' ', num2str(sdf_file)];

% ================== optimize and count time ======================
profile on
for i = 1:50
    execute_one_exp(executable, args);
end
profile viewer
% =================================================================

%% ========================================================================
%                                  GPMP2
% =========================================================================
clear all 
close all
clc

start_conf = [-0.8,-1.70,1.64,1.29,1.1,-0.106,2.2]';
end_conf = [-0.0,0.94,0,1.6,0,-0.919,1.55]';

sdf_file = '/home/hzyu/git/VIMP/vimp/maps/WAM/WAMDeskDataset.bin';

% ================== optimize and count time ======================
profile on
for i=1:50
    execute_one_exp_gpmp2(sdf_file, start_conf, end_conf);
end
profile viewer
% =================================================================

%% ================= 2nd experiment =================
clear all 
close all
clc

start_conf = [-0.9,-1.70,1.34,1.19,0.8,-0.126,2.5]';
end_conf = [-0.7,1.35,1.2,1.0,-0.7,-0.1,1.2]';

sdf_file = '/home/hzyu/git/VIMP/vimp/maps/WAM/WAMDeskDataset.bin';

profile on
for i=1:50
    execute_one_exp_gpmp2(sdf_file, start_conf, end_conf);
end
profile viewer


%% ============================= sampling time ============================
% ---------------------- GVI-MP ------------------------------           
% ----------------------- reading data -----------------------
clear all
close all
clc

i_exp = 2;

prefix_gvi = ["../../GVIMP-examples/WAM/case"+num2str(i_exp)+"/"];

% ------------  read gvi-mp results ------------ 
means_gvi = csvread([prefix_gvi + "zk_sdf.csv"]);
[~, nt_gvi] = size(means_gvi);

covs_gvi = csvread([prefix_gvi + "Sk_sdf.csv"]);
covs_gvi = reshape(covs_gvi, [14, 14, nt_gvi]);

% ------------- sampling --------------
n_samples = 1000;

% mu j
mean_j = means_gvi(1:7, nt_gvi);
% cov j
cov_j = covs_gvi(1:7, 1:7, nt_gvi);

profile on
% sampling 
rng('default')  % For reproducibility
samples = mvnrnd(mean_j, cov_j, n_samples);
profile viewer

%% ---------------------- PGCS-MP ------------------------  
% ----------------------- reading data -----------------------
clear all
close all
clc

i_exp = 1;

prefix_pgcs = ["case"+num2str(i_exp)+"/"];

% ------------  read gvi-mp results ------------ 
means_pgcs = csvread([prefix_pgcs + "zk_sdf.csv"]);
[~, nt_pgcs] = size(means_pgcs);

covs_pgcs = csvread([prefix_pgcs + "Sk_sdf.csv"]);
covs_pgcs = reshape(covs_pgcs, [14, 14, nt_pgcs]);

% ------------- sampling --------------
n_samples = 1000;

% mu j
mean_j = means_pgcs(1:7, nt_pgcs);
% cov j
cov_j = covs_pgcs(1:7, 1:7, nt_pgcs);

profile on
% sampling 
rng('default')  % For reproducibility
samples = mvnrnd(mean_j, cov_j, n_samples);
profile viewer

%% ==================== closed-from prior time V.S. G-H estimation ==================== 
% ------------------------- Arm 2 --------------------------
profile on
for i = 1:50
    executable = '/home/hzyu/git/VIMP/build/src/gvimp/gvi_Arm2_prior_factors';
    execute_one_exp(executable, '');
end
profile viewer

%% ------------------------- WAM Arm --------------------------
profile on
for i = 1:50
    executable = '/home/hzyu/git/VIMP/build/src/gvimp/gvi_wam_prior_factors';
    execute_one_exp(executable, '');
end
profile viewer